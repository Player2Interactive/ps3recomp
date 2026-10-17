/*
 * ps3recomp - PPU <-> SPU lock-line coherence
 *
 * See spu_coherency.h for what this is for. The state here is deliberately
 * plain: a bit per 128-byte line, a small table of the SPU contexts that have
 * ever reserved one, and the lock-line spinlock. Everything that WRITES it runs
 * with the lock-line lock held (the SPU's GETLLAR, the PPU's coherent store),
 * so the read-modify-write on a shared bitmap byte is serialized. Only
 * spu_coh_is_reserved reads without the lock, which is the entire point: it is
 * on every PPU store.
 */

#include "spu_coherency.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
 * The lock-line lock
 *
 * Moved here from spu_channels.c, where it was a file-static and so could
 * serialize SPU against SPU but not SPU against PPU -- which left the PPU free
 * to write a line in the middle of a PUTLLC's compare-and-commit. Same
 * spinlock, same semantics, now shared. _InterlockedExchange is a clang-cl/MSVC
 * intrinsic (no runtime library symbol needed); elsewhere use the C11
 * equivalent, which lowers to the same LL/SC or lock-xchg on every target.
 * ===========================================================================*/
#if defined(_MSC_VER)
#include <intrin.h>
static volatile long g_lockline = 0;
void spu_lockline_lock(void)   { while (_InterlockedExchange(&g_lockline, 1)) { } }
void spu_lockline_unlock(void) { _InterlockedExchange(&g_lockline, 0); }
#else
#include <stdatomic.h>
static atomic_flag g_lockline = ATOMIC_FLAG_INIT;
void spu_lockline_lock(void)   { while (atomic_flag_test_and_set_explicit(&g_lockline, memory_order_acquire)) { } }
void spu_lockline_unlock(void) { atomic_flag_clear_explicit(&g_lockline, memory_order_release); }
#endif

/* ===========================================================================
 * The reserved-line bitmap
 *
 * One bit per 128-byte line over the whole 32-bit guest address space: 4 MiB of
 * zero-initialized BSS, of which only the pages around addresses an SPU has
 * actually reserved are ever touched. The reference implementation scopes its
 * bitmap to the one window a single title's SPURS structures live in; this
 * runtime hands out guest memory from several disjoint regions (the ELF image,
 * the sys_memory window at 0x40000000, the raw-SPU local-store windows), and a
 * line outside a hardcoded window would be silently non-coherent, which is the
 * failure this whole file exists to remove.
 * ===========================================================================*/
#define SPU_COH_LINE_SHIFT  7                                  /* 128-byte lines */
#define SPU_COH_LINES       (1u << (32 - SPU_COH_LINE_SHIFT))   /* 2^25 lines */
#define SPU_COH_BITMAP_SZ   (SPU_COH_LINES / 8)                 /* 4 MiB */

static unsigned char s_coh_bitmap[SPU_COH_BITMAP_SZ];

/* Zero until the first GETLLAR anywhere. Keeps the PPU fast path off the
 * bitmap entirely for a title that runs no SPU code, and keeps every existing
 * test that stores through vm_write* paying one compare against a hot global.
 * Written once, under the lock; read unlocked, where a stale zero costs at most
 * one missed event on the very first reservation of the run. */
static int s_coh_armed;

/* The SPU contexts that have reserved a line. Sized to match the MFC engine
 * registry in spu_channels.c, which is the ceiling on live SPU contexts. */
#define SPU_COH_MAX_CTX 8
static spu_context* s_coh_ctxs[SPU_COH_MAX_CTX];

unsigned long g_spu_lr_raise = 0;

extern uint8_t* vm_base;

static uint32_t ovl12_load_be32(uint32_t ea)
{
    const uint8_t* p;
    if (!vm_base) return 0;
    p = vm_base + ea;
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
         | ((uint32_t)p[2] << 8) | p[3];
}

/* liveA 00FC7B00+lane<<7, liveB 00FC7D80+lane<<7. slot 0=A, 1=B. */
static int ovl12_occupancy_lane(uint32_t ram, uint32_t* line_off, int* slot)
{
    if (ram >= 0x00FC7B00u && ram < 0x00FC7D00u) {
        uint32_t off = ram - 0x00FC7B00u;
        if (line_off) *line_off = off & 0x7Fu;
        if (slot) *slot = 0;
        return (int)(off >> 7);
    }
    if (ram >= 0x00FC7D80u && ram < 0x00FC7F80u) {
        uint32_t off = ram - 0x00FC7D80u;
        if (line_off) *line_off = off & 0x7Fu;
        if (slot) *slot = 1;
        return (int)(off >> 7);
    }
    return -1;
}

static int ovl12_ea_is_occupancy(uint32_t ea)
{
    uint32_t ram = ovl12_io_to_ram(ea);
    return ovl12_occupancy_lane(ram ? ram : ea, 0, 0) >= 0;
}

static int ovl12_lane_pending(int lane)
{
    uint32_t a, b;
    if (lane < 0 || lane > 3 || !vm_base) return 0;
    a = ovl12_load_be32(0x00FC7B00u + ((uint32_t)lane << 7));
    b = ovl12_load_be32(0x00FC7D80u + ((uint32_t)lane << 7));
    return a != b;
}

static int ovl12_is_overlay_idle(const spu_context* ctx)
{
    uint32_t pc;
    if (!ctx || ctx->resident_ovl != 12) return 0;
    pc = (uint32_t)ctx->pc & SPU_LS_MASK;
    return pc >= 0x3F000u && pc < 0x40000u;
}

static int ovl12_is_ice_parked(const spu_context* ctx)
{
    if (!ctx || !ctx->resv_valid) return 0;
    return (ctx->resv_ea & ~15u) == 0x00F3CF00u;
}

/* 00696614 cell 00F7B488+4 holds the Great Clock handle only after
 * 005F9C48 packs. 00B5D900 is also posted during boot, walker+3C is
 * 9 during FE, and the doorbell is 0x180 in 0055BF88 / 0 at the wrap
 * hang -- none of those uniquely mark leftover wrap. */
static int ovl12_steal_allowed(void)
{
    if (!vm_base) return 0;
    return ovl12_load_be32(0x00F7B48Cu) != 0;
}

/* Lowest overlay-idle matching SPU steals the lowest Ice-parked pending lane. */
static int ovl12_find_steal_peer(const spu_context* self)
{
    int best_idle, peer;
    int i;
    if (!self) return -1;
    best_idle = (int)self->spu_id;
    if (best_idle < 0 || best_idle > 5 || ovl12_lane_pending(best_idle))
        best_idle = 99;
    peer = -1;
    for (i = 0; i < SPU_COH_MAX_CTX; i++) {
        spu_context* c = s_coh_ctxs[i];
        int id;
        if (!c || c == self) continue;
        id = (int)c->spu_id;
        if (ovl12_is_overlay_idle(c) && id >= 0 && id <= 5
            && !ovl12_lane_pending(id) && id < best_idle)
            best_idle = id;
        if (ovl12_is_ice_parked(c) && ovl12_lane_pending(id)
            && (peer < 0 || id < peer))
            peer = id;
    }
    if ((int)self->spu_id != best_idle || peer < 0)
        return -1;
    return peer;
}

uint32_t ovl12_occupancy_remap(const spu_context* ctx, uint32_t ea)
{
    uint32_t ram = ovl12_io_to_ram(ea);
    uint32_t base = ram ? ram : ea;
    uint32_t line_off = 0;
    int slot = 0;
    int lane;
    int peer;
    uint32_t out;

    lane = ovl12_occupancy_lane(base, &line_off, &slot);
    if (lane < 0 || lane > 3)
        return ea;

    out = ram ? ram : ea;
    if (!ctx || !ovl12_is_overlay_idle(ctx))
        return out;
    if (lane != (int)ctx->spu_id)
        return out;
    if (ovl12_lane_pending(lane))
        return out;
    if (!ovl12_steal_allowed())
        return out;
    peer = ovl12_find_steal_peer(ctx);
    if (peer < 0 || peer == lane)
        return out;

    out = (slot ? 0x00FC7D80u : 0x00FC7B00u)
        + ((uint32_t)peer << 7) + line_off;
    {
        static unsigned n;
        if (++n <= 24u || (n % 64u) == 0u) {
            fprintf(stderr,
                    "[ovl12-steal] spu=%u pc=0x%05X lane%u ea=0x%08X -> peer%u "
                    "0x%08X A=0x%08X B=0x%08X\n",
                    ctx->spu_id, (uint32_t)ctx->pc & SPU_LS_MASK,
                    (unsigned)lane, ea, (unsigned)peer, out,
                    ovl12_load_be32(0x00FC7B00u + ((uint32_t)peer << 7)),
                    ovl12_load_be32(0x00FC7D80u + ((uint32_t)peer << 7)));
            fflush(stderr);
        }
    }
    return out;
}

void ovl12_occupancy_sync_liveA(spu_context* ctx, uint32_t liveB_ram)
{
    uint32_t off = 0;
    int slot = 0;
    int lane;
    uint32_t liveA;
    if (!ctx || !ctx->ls || !vm_base) return;
    lane = ovl12_occupancy_lane(liveB_ram, &off, &slot);
    if (slot != 1 || lane < 0 || lane > 3) return;
    liveA = 0x00FC7B00u + ((uint32_t)lane << 7);
    memcpy(&ctx->ls[0x3FE00], vm_base + liveA, 128);
}

void spu_coh_reserve(spu_context* ctx, uint32_t ea)
{
    uint32_t line = ea >> SPU_COH_LINE_SHIFT;
    s_coh_bitmap[line >> 3] |= (unsigned char)(1u << (line & 7));
    /* Overlay-12 GETLLAR is on IO 20020880; packer stw liveB at RAM twin
     * 00FC7D80. Mark the twin so VM_WRITE_COH notifies this reservation. */
    {
        uint32_t twin = ovl12_io_to_ram(ea & ~127u);
        uint32_t steal;
        if (twin) {
            uint32_t tline = twin >> SPU_COH_LINE_SHIFT;
            s_coh_bitmap[tline >> 3] |= (unsigned char)(1u << (tline & 7));
        }
        steal = ctx ? (ovl12_occupancy_remap(ctx, ea) & ~127u) : 0u;
        if (steal && steal != (ea & ~127u) && steal != twin) {
            uint32_t tline = steal >> SPU_COH_LINE_SHIFT;
            s_coh_bitmap[tline >> 3] |= (unsigned char)(1u << (tline & 7));
        }
    }
    s_coh_armed = 1;

    if (!ctx) return;
    for (int i = 0; i < SPU_COH_MAX_CTX; i++) {
        if (s_coh_ctxs[i] == ctx) return;
        if (s_coh_ctxs[i] == NULL) { s_coh_ctxs[i] = ctx; return; }
    }
    /* More live SPU contexts than the registry holds. The ones already in it
     * still get their events; this one would silently never wake, so say so. */
    { static int warned = 0;
      if (!warned) { warned = 1;
          fprintf(stderr, "[spu-coh] more than %d reserving SPU contexts -- "
                          "spu=0x%X will not receive lock-line events\n",
                  SPU_COH_MAX_CTX, ctx->spu_id);
          fflush(stderr); } }
}

int spu_coh_is_reserved(uint32_t addr)
{
    if (!s_coh_armed) return 0;
    uint32_t line = addr >> SPU_COH_LINE_SHIFT;
    return (s_coh_bitmap[line >> 3] >> (line & 7)) & 1u;
}

void spu_coh_notify_write(uint32_t ea)
{
    uint32_t line = ea & ~127u;
    uint32_t extra = ovl12_ram_to_io(line);
    if (!extra)
        extra = ovl12_io_to_ram(line);
    for (int i = 0; i < SPU_COH_MAX_CTX; i++) {
        spu_context* c = s_coh_ctxs[i];
        if (!c) continue;
        if (!c->resv_valid) continue;
        uint32_t rline = c->resv_ea & ~127u;
        if (rline != line && (!extra || rline != extra))
            continue;
        c->event_status |= SPU_EVENT_LR;
        c->resv_valid = 0;          /* reservation lost, PUTLLC must fail */
        g_spu_lr_raise++;
        /* The event_status store has to be visible before the wake. The
         * waiter re-polls its predicate on a timeout as well, so a
         * straggling store costs latency and not the wakeup itself. */
        spu_ch_wake(c);
    }
    /* Packer posts occupancy on an Ice-parked lane. Overlay-idle SPUs wait
     * GETLLAR of their own matching IO line, which is not that twin, so the
     * alias notify above misses them. Wake overlay-idle occupancy waiters
     * so they re-GET and steal the pending lane. Do not wake Ice GETLLAR. */
    if (ovl12_steal_allowed()
        && (ovl12_ea_is_occupancy(line)
            || (extra && ovl12_ea_is_occupancy(extra)))) {
        for (int i = 0; i < SPU_COH_MAX_CTX; i++) {
            spu_context* c = s_coh_ctxs[i];
            uint32_t rline;
            if (!c || !c->resv_valid) continue;
            if (!ovl12_is_overlay_idle(c)) continue;
            rline = c->resv_ea & ~127u;
            if (rline == line || (extra && rline == extra))
                continue;
            c->event_status |= SPU_EVENT_LR;
            c->resv_valid = 0;
            g_spu_lr_raise++;
            spu_ch_wake(c);
        }
    }
}

#ifdef __cplusplus
}
#endif
