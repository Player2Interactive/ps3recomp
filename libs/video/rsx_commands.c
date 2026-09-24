/*
 * ps3recomp - RSX Command Buffer Processor
 *
 * Parses NV47xx GPU methods from the command buffer and updates RSX state.
 * Dispatches to the registered graphics backend for actual rendering.
 *
 * Command buffer format (NV47xx FIFO):
 *   Each command is a 32-bit header followed by N data words.
 *   Header format: [31:29] type | [28:18] count | [17:13] subchannel | [12:2] method | [1:0] flags
 *
 *   Type 0 (increasing): method, method+4, method+8, ... for each data word
 *   Type 2 (non-increasing): same method repeated for each data word
 *   Type 1 (jump): jump to address in data
 *   Type 3 (call/return): call/return from subroutine
 */

#include "rsx_commands.h"
#include "cellGcmSys.h"
#include "ps3emu/milestone.h"   /* ps3_ms -- boot milestone log */
#include <stdio.h>
#include <stdlib.h>   /* getenv -- an implicit decl returns int, truncating the pointer */
#include <string.h>
#include "../../runtime/memory/vm.h"    /* VM_HLE_INJECT_BASE */

/* ---------------------------------------------------------------------------
 * Global backend
 * -----------------------------------------------------------------------*/

static rsx_backend* s_backend = NULL;

/* Last NV406E_SET_REFERENCE value seen in the FIFO. The title writes a
 * reference then spins on the GCM control register's `ref` field until the RSX
 * reports it as reached; cellGcmSys mirrors this into the guest control
 * register after each drain so those waits (cellGcmFinish / wait-label) unblock. */
u32 g_rsx_last_reference = 0;

void rsx_set_backend(rsx_backend* backend)
{
    s_backend = backend;
}

rsx_backend* rsx_get_backend(void)
{
    return s_backend;
}

/* Dump-only: 640x640 clip or any high AOFFSET (offscreen RT, not display
 * 0x0 / 0x398000). Do not use as a compositor trigger. */
static int rsx_hi_rt(const rsx_state* state)
{
    u32 off = state->surface_color_offset[0];
    return (off >= 0x08000000u) ||
           (state->surface_clip_w == 640 && state->surface_clip_h == 640);
}

/* ---------------------------------------------------------------------------
 * State initialization
 * -----------------------------------------------------------------------*/

void rsx_state_init(rsx_state* state)
{
    memset(state, 0, sizeof(rsx_state));
    /* RSX reset value for the constant vertex attributes is (0,0,0,1); a zeroed
     * struct would leave w at 0. */
    for (int _i = 0; _i < RSX_MAX_VERTEX_ATTRIBS; _i++)
        state->vertex_data4f[_i][3] = 1.0f;

    /* Default viewport */
    state->viewport_w = 1280;
    state->viewport_h = 720;
    state->clip_min = 0.0f;
    state->clip_max = 1.0f;

    /* Default scissor */
    state->scissor_w = 4096;
    state->scissor_h = 4096;

    /* Default depth */
    state->depth_func = 1; /* LESS */
    state->depth_mask = 1;

    /* Default cull */
    state->cull_face = 1; /* BACK */
    state->front_face = 0; /* CW */

    /* Default color mask: all channels writable (A|R|G|B) */
    state->color_mask = 0x01010101;

    /* Default stencil: sensible initial values */
    state->stencil_func = 0x0207; /* ALWAYS */
    state->stencil_ref = 0;
    state->stencil_mask = 0xFF;
    state->stencil_op_fail = 0x1E00;  /* KEEP */
    state->stencil_op_zfail = 0x1E00; /* KEEP */
    state->stencil_op_zpass = 0x1E00; /* KEEP */

    /* Default alpha test */
    state->alpha_func = 0x0207; /* ALWAYS */
    state->alpha_ref = 0;

    /* Default shader control: 32-bit colour exports (r0..) -- matches every
     * title that never programs the register. */
    state->shader_control = CELL_GCM_SHADER_CONTROL_32_BITS_EXPORTS;

    /* Mark everything dirty */
    state->surface_dirty = 1;
    state->viewport_dirty = 1;
    state->blend_dirty = 1;
    state->depth_dirty = 1;
    state->stencil_dirty = 1;
    state->texture_dirty = 1;
    state->vertex_dirty = 1;
    state->color_mask_dirty = 1;
    state->alpha_dirty = 1;
    state->shader_dirty = 1;
}

/* NV4097 inline-array / array-element state. Release builds used to drop
 * these as unknown (the unknown log was NDEBUG-only), so a blit/quad that
 * the guest submitted between SET_BEGIN_END never reached the backend. */
#define RSX_INLINE_MAX  4096
static u32 s_inline_words[RSX_INLINE_MAX];
static u32 s_inline_n;
static u32 s_elem_idx[RSX_INLINE_MAX];
static u32 s_elem_n;
/* High local-memory scratch: past display tiles and the 640x640 RT. */
#define RSX_INLINE_SCRATCH  0x0F000000u

static u32 rsx_attrib_packed_bytes(const rsx_vertex_attrib* a)
{
    u32 n = a->size;
    if (!n) return 0;
    switch (a->type) {
    case 2: return n * 4u;                         /* float32 */
    case 3: case 6: return (n * 2u + 3u) & ~3u;    /* half    */
    case 1: case 5: return (n * 2u + 3u) & ~3u;    /* s16     */
    case 4: case 7: return (n + 3u) & ~3u;         /* u8      */
    default: return n * 4u;
    }
}

static void rsx_flush_begin_end(rsx_state* state)
{
    extern u8* vm_base;
    extern u32 cellGcmResolveLocated(int local, u32 offset);
    if (!s_backend) { s_inline_n = 0; s_elem_n = 0; return; }
    if (s_inline_n) {
        u32 packed = 0;
        int any = 0;
        for (int i = 0; i < RSX_MAX_VERTEX_ATTRIBS; i++)
            if (state->vertex_attribs[i].enabled) {
                packed += rsx_attrib_packed_bytes(&state->vertex_attribs[i]);
                any = 1;
            }
        if (!packed) packed = 16;
        u32 nbytes = s_inline_n * 4u;
        u32 count = packed ? nbytes / packed : 0;
        if (!count) { s_inline_n = 0; s_elem_n = 0; return; }
        u32 ea = cellGcmResolveLocated(1, RSX_INLINE_SCRATCH);
        { static int n = 0;
          if (n++ < 16)
              fprintf(stderr, "[RSX] INLINE_ARRAY prim=%u words=%u packed=%u count=%u surf=0x%08X clip=%ux%u%c",
                      state->primitive_type, s_inline_n, packed, count,
                      state->surface_color_offset[0],
                      state->surface_clip_w, state->surface_clip_h, 10); }
        if (vm_base && ea != 0xFFFFFFFFu) {
            u8* p = vm_base + ea;
            for (u32 i = 0; i < s_inline_n; i++) {
                u32 w = s_inline_words[i];
                p[i*4+0] = (u8)(w >> 24); p[i*4+1] = (u8)(w >> 16);
                p[i*4+2] = (u8)(w >> 8);  p[i*4+3] = (u8)w;
            }
            rsx_vertex_attrib saved[RSX_MAX_VERTEX_ATTRIBS];
            memcpy(saved, state->vertex_attribs, sizeof(saved));
            if (!any) {
                state->vertex_attribs[0].enabled = 1;
                state->vertex_attribs[0].type = 2;
                state->vertex_attribs[0].size = 4;
                state->vertex_attribs[0].stride = 16;
                state->vertex_attribs[0].offset = RSX_INLINE_SCRATCH;
            } else {
                u32 off = 0;
                for (int i = 0; i < RSX_MAX_VERTEX_ATTRIBS; i++) {
                    if (!state->vertex_attribs[i].enabled) continue;
                    u32 b = rsx_attrib_packed_bytes(&state->vertex_attribs[i]);
                    state->vertex_attribs[i].offset = RSX_INLINE_SCRATCH + off;
                    state->vertex_attribs[i].stride = packed;
                    off += b;
                }
            }
            if (s_backend->set_vertex_attribs)
                s_backend->set_vertex_attribs(s_backend->userdata, state);
            if (s_backend->draw_arrays)
                s_backend->draw_arrays(s_backend->userdata,
                                        state->primitive_type, 0, count);
            memcpy(state->vertex_attribs, saved, sizeof(saved));
        } else if (ea == 0xFFFFFFFFu) {
            { static int n = 0; if (n++ < 4)
                fprintf(stderr, "[RSX] INLINE_ARRAY scratch resolve failed off=0x%08X%c",
                        RSX_INLINE_SCRATCH, 10); }
        }
        s_inline_n = 0;
    } else if (s_elem_n) {
        u32 ea = cellGcmResolveLocated(1, RSX_INLINE_SCRATCH);
        { static int n = 0;
          if (n++ < 16)
              fprintf(stderr, "[RSX] ARRAY_ELEMENT prim=%u count=%u surf=0x%08X clip=%ux%u%c",
                      state->primitive_type, s_elem_n,
                      state->surface_color_offset[0],
                      state->surface_clip_w, state->surface_clip_h, 10); }
        if (vm_base && ea != 0xFFFFFFFFu && s_backend->draw_indexed) {
            u8* p = vm_base + ea;
            for (u32 i = 0; i < s_elem_n; i++) {
                u32 w = s_elem_idx[i];
                p[i*4+0] = (u8)(w >> 24); p[i*4+1] = (u8)(w >> 16);
                p[i*4+2] = (u8)(w >> 8);  p[i*4+3] = (u8)w;
            }
            u32 save_off = state->index_array_offset;
            u32 save_dma = state->index_array_dma;
            state->index_array_offset = RSX_INLINE_SCRATCH;
            state->index_array_dma = 0; /* local, u32 indices */
            s_backend->draw_indexed(s_backend->userdata,
                                    state->primitive_type, 0, s_elem_n);
            state->index_array_offset = save_off;
            state->index_array_dma = save_dma;
        }
        s_elem_n = 0;
    }
}

static int process_surface_method(rsx_state* state, u32 method, u32 data)
{
    switch (method) {
    case NV4097_SET_SURFACE_FORMAT:
        state->surface_format = data;
        state->surface_dirty = 1;
        return 0;
    case NV4097_SET_SURFACE_CLIP_HORIZONTAL:
        state->surface_clip_x = data & 0xFFFF;
        state->surface_clip_w = (data >> 16) & 0xFFFF;
        state->surface_dirty = 1;
        return 0;
    case NV4097_SET_SURFACE_CLIP_VERTICAL:
        state->surface_clip_y = data & 0xFFFF;
        state->surface_clip_h = (data >> 16) & 0xFFFF;
        state->surface_dirty = 1;
        return 0;
    case NV4097_SET_SURFACE_COLOR_AOFFSET:
        state->surface_color_offset[0] = data;
        { static unsigned _seen[32]; static int _n = 0;
          static int after = 0, rec = 0; static u32 last = 0xFFFFFFFFu;
          int f = 0; for (int k = 0; k < _n; k++) if (_seen[k] == data) f = 1;
          if (data >= 0x08000000u ||
              (state->surface_clip_w == 640 && state->surface_clip_h == 640))
              after = 1;
          if (!f && _n < 32) { _seen[_n++] = data;
              fprintf(stderr, "[SURF] SET_SURFACE_COLOR_AOFFSET=0x%08X clip=%ux%u fmt=0x%X tgt=0x%X pitchA=%u zeta=0x%08X%c",
                      data, state->surface_clip_w, state->surface_clip_h,
                      state->surface_format, state->color_target,
                      state->surface_color_pitch[0], state->surface_zeta_offset, 10); }
          else if (after && data != last && rec < 24) {
              rec++; last = data;
              fprintf(stderr, "[SURF] SET_SURFACE_COLOR_AOFFSET=0x%08X clip=%ux%u fmt=0x%X tgt=0x%X pitchA=%u zeta=0x%08X (post-hi-rt)%c",
                      data, state->surface_clip_w, state->surface_clip_h,
                      state->surface_format, state->color_target,
                      state->surface_color_pitch[0], state->surface_zeta_offset, 10); }
          last = data; }
        state->surface_dirty = 1;
        return 0;
    case NV4097_SET_SURFACE_COLOR_BOFFSET:
        state->surface_color_offset[1] = data;
        state->surface_dirty = 1;
        return 0;
    case NV4097_SET_SURFACE_COLOR_COFFSET:
        state->surface_color_offset[2] = data;
        state->surface_dirty = 1;
        return 0;
    case NV4097_SET_SURFACE_COLOR_DOFFSET:
        state->surface_color_offset[3] = data;
        state->surface_dirty = 1;
        return 0;
    case NV4097_SET_SURFACE_ZETA_OFFSET:
        state->surface_zeta_offset = data;
        state->surface_dirty = 1;
        return 0;
    case NV4097_SET_SURFACE_COLOR_TARGET:
        { static int _ct=0; if (_ct++ < 24 && getenv("RTT_DUMP"))
            fprintf(stderr, "[RSXCT] color_target=0x%X offA=0x%X offB=0x%X offC=0x%X offD=0x%X%s",
                    data, state->surface_color_offset[0], state->surface_color_offset[1],
                    state->surface_color_offset[2], state->surface_color_offset[3], "\n"); }
        state->color_target = data;
        state->surface_dirty = 1;
        return 0;
    case NV4097_SET_SURFACE_PITCH_A:
        state->surface_color_pitch[0] = data;
        return 0;
    case NV4097_SET_SURFACE_PITCH_B:
        state->surface_color_pitch[1] = data;
        return 0;
    case NV4097_SET_SURFACE_PITCH_C:
        state->surface_color_pitch[2] = data;
        return 0;
    case NV4097_SET_SURFACE_PITCH_D:
        state->surface_color_pitch[3] = data;
        return 0;
    case NV4097_SET_SURFACE_PITCH_Z:
        state->surface_zeta_pitch = data;
        return 0;
    default:
        return -1;
    }
}

/* ---------------------------------------------------------------------------
 * Texture method processing
 *
 * Texture registers are laid out in 16 units, each spanning 0x20 bytes:
 *   Unit 0: 0x1A00..0x1A1C
 *   Unit 1: 0x1A20..0x1A3C
 *   ...
 *   Unit N: 0x1A00 + N*0x20 .. 0x1A1C + N*0x20
 * -----------------------------------------------------------------------*/

static int process_texture_method(rsx_state* state, u32 method, u32 data)
{
    /* Compute texture unit index and register offset within the unit */
    u32 base = method - NV4097_SET_TEXTURE_OFFSET; /* 0x1A00 */
    u32 unit = base / 0x20;
    u32 reg  = base % 0x20;

    if (unit >= RSX_MAX_TEXTURES)
        return -1;

    rsx_texture_state* tex = &state->textures[unit];

    switch (reg) {
    case 0x00: /* TEXTURE_OFFSET */
        tex->offset = data;
        { static int after = 0, n = 0;
          if (data >= 0x08000000u || rsx_hi_rt(state)) after = 1;
          if ((data >= 0x08000000u || after) && n < 16) {
              n++;
              fprintf(stderr, "[TEXOFF] unit=%u off=0x%08X rect=%ux%u fmt=0x%X surf=0x%08X%c",
                      unit, data, tex->image_rect >> 16, tex->image_rect & 0xFFFF,
                      tex->format, state->surface_color_offset[0], 10); } }
        break;
    case 0x04: /* TEXTURE_FORMAT */
        tex->format = data;
        break;
    case 0x08: /* TEXTURE_ADDRESS (wrap S/T/R) */
        tex->address = data;
        break;
    case 0x0C: /* TEXTURE_CONTROL0 (enable, min/max LOD, max aniso) */
        tex->control0 = data;
        break;
    case 0x10: /* TEXTURE_CONTROL1 (remap) */
        tex->control1 = data;
        break;
    case 0x14: /* TEXTURE_FILTER (bias, min/mag filter) */
        tex->filter = data;
        break;
    case 0x18: /* TEXTURE_IMAGE_RECT (width << 16 | height) */
        tex->image_rect = data;
        break;
    case 0x1C: /* TEXTURE_BORDER_COLOR */
        tex->border_color = data;
        break;
    default:
        return -1;
    }

    tex->dirty = 1;
    state->texture_dirty = 1;
    return 0;
}

/* Vertex texture unit block: 0x0900 + unit*0x20, eight words. Same order as a
 * fragment unit's except at +0x10, which is CONTROL3 (the row pitch) rather
 * than CONTROL1 (the component crossbar) -- a vertex unit has no crossbar. */
static int process_vertex_texture_method(rsx_state* state, u32 method, u32 data)
{
    u32 base = method - NV4097_SET_VERTEX_TEXTURE_OFFSET;
    u32 unit = base / 0x20;
    u32 reg  = base % 0x20;

    if (unit >= RSX_MAX_VERTEX_TEXTURES)
        return -1;

    rsx_texture_state* tex = &state->vertex_textures[unit];

    switch (reg) {
    case 0x00: tex->offset       = data; break;
    case 0x04: tex->format       = data; break;
    case 0x08: tex->address      = data; break;
    case 0x0C: tex->control0     = data; break;
    case 0x10: tex->control3     = data; break;   /* CONTROL1's slot */
    case 0x14: tex->filter       = data; break;
    case 0x18: tex->image_rect   = data; break;
    case 0x1C: tex->border_color = data; break;
    default:
        return -1;
    }

    tex->dirty = 1;
    state->texture_dirty = 1;
    return 0;
}

/* SET_TEXTURE_CONTROL3: 0x1840 + unit*4, one word per unit rather than a slot
 * in the 0x20-byte unit block. */
static int process_texture_control3(rsx_state* state, u32 method, u32 data)
{
    u32 unit = (method - NV4097_SET_TEXTURE_CONTROL3) / 4;
    if (unit >= RSX_MAX_TEXTURES)
        return -1;
    state->textures[unit].control3 = data;
    state->textures[unit].dirty = 1;
    state->texture_dirty = 1;
    return 0;
}

/* ---------------------------------------------------------------------------
 * Vertex attribute method processing
 *
 * FORMAT registers: 0x1740 + attrib*4  (16 attributes)
 * OFFSET registers: 0x1680 + attrib*4  (16 attributes)
 * -----------------------------------------------------------------------*/

static int process_vertex_attrib_method(rsx_state* state, u32 method, u32 data)
{
    if (method >= NV4097_SET_VERTEX_DATA_ARRAY_FORMAT &&
        method < NV4097_SET_VERTEX_DATA_ARRAY_FORMAT + RSX_MAX_VERTEX_ATTRIBS * 4) {
        u32 index = (method - NV4097_SET_VERTEX_DATA_ARRAY_FORMAT) / 4;
        rsx_vertex_attrib* attr = &state->vertex_attribs[index];

        /*
         * Format register layout:
         *   [3:0]   type   (float, half, ubyte, short, etc.)
         *   [7:4]   size   (number of components: 1-4)
         *   [15:8]  stride (bytes between consecutive elements)
         *   [16]    enable
         */
        attr->type      = data & 0xF;
        attr->size      = (data >> 4) & 0xF;
        attr->stride    = (data >> 8) & 0xFF;
        attr->frequency = (data >> 16) & 0xFFFF;   /* instancing divisor */
        /* SIZE (the component count) is what enables the attribute: NV4097 uses
         * size == 0 to mean "this array is off". Keying off `type` instead marked
         * every unused slot enabled-with-zero-components, because PSGL writes a
         * bare type (e.g. 0x00000002 = float32, size 0, stride 0) into the slots
         * it is NOT using -- so the input layout was built from 16 attributes of
         * which most were degenerate, and the draws rasterized nothing. */
        attr->enabled   = (attr->size != 0);
        attr->format    = data;
        state->vertex_dirty = 1;
        return 0;
    }

    if (method >= NV4097_SET_VERTEX_DATA_ARRAY_OFFSET &&
        method < NV4097_SET_VERTEX_DATA_ARRAY_OFFSET + RSX_MAX_VERTEX_ATTRIBS * 4) {
        u32 index = (method - NV4097_SET_VERTEX_DATA_ARRAY_OFFSET) / 4;
        state->vertex_attribs[index].offset = data;
        state->vertex_dirty = 1;
        return 0;
    }

    return -1;
}

/* ---------------------------------------------------------------------------
 * Main method dispatch
 * -----------------------------------------------------------------------*/

int rsx_process_method(rsx_state* state, u32 method, u32 data)
{
    /* RSX_TRACE=<N>: trace the first N methods (bare "1" keeps the old 250).
     * The fixed 250 was spent entirely on boot-time setup, so the methods around
     * the first real draw -- exactly the ones worth seeing -- were never traced. */
    { static int _rt=-1; if(_rt<0){ const char* e=getenv("RSX_TRACE");
        _rt = e ? (atoi(e) > 1 ? atoi(e) : 250) : 0; }
      if(_rt){ static int _m=0; if(_m++<_rt) fprintf(stderr,"[rsxm] method=0x%04X data=0x%08X\n", method, data); } }
    /* Back-end write label / semaphore (cellGcmSetWriteBackEndLabel): the RSX
     * writes a value to a report/label the CPU polls for CPU<->RSX sync (double
     * buffering). Real hardware DOES this; without it the game's frame-fence
     * loop (func_0006E6E0 polling label 0x41 @ 0x03000410) spins forever and
     * never reaches render. NV4097_SET_SEMAPHORE_OFFSET(0x1D6C)=offset (index*0x10
     * into the GCM label window @0x03000000); NV4097_BACK_END_WRITE_SEMAPHORE_
     * RELEASE(0x1D70)=value. Also NV406E semaphore release (0x0010 offset/0x0014
     * value) for the sub-channel path. */
    /* NV4097_SET_FREQUENCY_DIVIDER_OPERATION: per-attribute modulo/divide mask
     * for the frequency divisor (instancing). Captured for read_vp_vertex. */
    if (method == 0x00001FC0) { state->frequency_divider_op = data; return 0; }

    /* NV4097 / NV406E semaphore → guest label window (GetLabelAddress). */
    if (gcm_sema_apply(method, data))
        return 0;
    if ((method >= 0x200 && method <= 0x23C) ||
        (method >= 0x280 && method <= 0x28C))
        return process_surface_method(state, method, data);

    /* Texture methods: 0x1A00..0x1A00 + 16*0x20 - 1 */
    if (method >= 0x1A00 && method < 0x1A00 + RSX_MAX_TEXTURES * 0x20)
        return process_texture_method(state, method, data);

    /* Texture CONTROL3: 0x1840..0x187C */
    if (method >= NV4097_SET_TEXTURE_CONTROL3 &&
        method < NV4097_SET_TEXTURE_CONTROL3 + RSX_MAX_TEXTURES * 4)
        return process_texture_control3(state, method, data);

    /* Vertex texture units: 0x0900..0x097C */
    if (method >= NV4097_SET_VERTEX_TEXTURE_OFFSET &&
        method < NV4097_SET_VERTEX_TEXTURE_OFFSET + RSX_MAX_VERTEX_TEXTURES * 0x20)
        return process_vertex_texture_method(state, method, data);

    /* Vertex attribute FORMAT: 0x1740..0x177C */
    if (method >= 0x1740 && method < 0x1740 + RSX_MAX_VERTEX_ATTRIBS * 4)
        return process_vertex_attrib_method(state, method, data);

    /* Vertex attribute OFFSET: 0x1680..0x16BC */
    if (method >= 0x1680 && method < 0x1680 + RSX_MAX_VERTEX_ATTRIBS * 4)
        return process_vertex_attrib_method(state, method, data);

    /* Viewport */
    /* Viewport transform: window = ndc*scale + offset. The z lane is the
     * GL->[0,1] depth remap (offset.z=0.5/scale.z=0.5); without honoring it,
     * GL-convention projections (SDK gcm samples) get their near-camera
     * geometry clipped by D3D's 0<=z<=w rule (gcm/cube: missing polygons). */
    if (method >= 0x0A20 && method < 0x0A30) {
        u32 f = data; float v; memcpy(&v, &f, 4);
        state->viewport_offset[(method - 0x0A20) >> 2] = v;
        return 0;
    }
    if (method >= 0x0A30 && method < 0x0A40) {
        u32 f = data; float v; memcpy(&v, &f, 4);
        state->viewport_scale[(method - 0x0A30) >> 2] = v;
        return 0;
    }

    if (method == NV4097_SET_VIEWPORT_HORIZONTAL) {
        state->viewport_x = data & 0xFFFF;
        state->viewport_w = (data >> 16) & 0xFFFF;
        state->viewport_dirty = 1;
        return 0;
    }
    if (method == NV4097_SET_VIEWPORT_VERTICAL) {
        state->viewport_y = data & 0xFFFF;
        state->viewport_h = (data >> 16) & 0xFFFF;
        state->viewport_dirty = 1;
        return 0;
    }

    /* Color mask */
    if (method == NV4097_SET_COLOR_MASK) {
        state->color_mask = data;
        state->color_mask_dirty = 1;
        return 0;
    }

    /* Alpha test */
    if (method == NV4097_SET_ALPHA_TEST_ENABLE) {
        state->alpha_test_enable = data ? 1 : 0;
        state->alpha_dirty = 1;
        return 0;
    }
    if (method == NV4097_SET_ALPHA_FUNC) {
        state->alpha_func = data;
        state->alpha_dirty = 1;
        return 0;
    }
    if (method == NV4097_SET_ALPHA_REF) {
        state->alpha_ref = data;
        state->alpha_dirty = 1;
        return 0;
    }

    /* Clear */
    if (method == NV4097_SET_COLOR_CLEAR_VALUE) {
        state->color_clear_value = data;
        return 0;
    }
    if (method == NV4097_SET_ZSTENCIL_CLEAR_VALUE) {
        state->zstencil_clear_value = data;
        return 0;
    }
    if (method == NV4097_CLEAR_SURFACE) {
        { static int _c=0; if (_c++ < 12)
            fprintf(stderr, "[RSX] CLEAR_SURFACE mask=0x%X color=0x%08X surf=0x%08X clip=%ux%u tgt=0x%X\n",
                    data, state->color_clear_value, state->surface_color_offset[0],
                    state->surface_clip_w, state->surface_clip_h, state->color_target); }
        if (s_backend && s_backend->clear) {
            float depth = (float)(state->zstencil_clear_value >> 8) / (float)0xFFFFFF;
            u8 stencil = state->zstencil_clear_value & 0xFF;
            s_backend->clear(s_backend->userdata, data,
                           state->color_clear_value, depth, stencil);
        }
        return 0;
    }

    /* Blend */
    if (method == NV4097_SET_BLEND_ENABLE) {
        state->blend_enable = data ? 1 : 0;
        state->blend_dirty = 1;
        return 0;
    }
    if (method == NV4097_SET_BLEND_FUNC_SFACTOR) {
        state->blend_sfactor = data;
        state->blend_dirty = 1;
        return 0;
    }
    if (method == NV4097_SET_BLEND_FUNC_DFACTOR) {
        state->blend_dfactor = data;
        state->blend_dirty = 1;
        return 0;
    }
    if (method == NV4097_SET_BLEND_EQUATION) {
        state->blend_equation = data;
        state->blend_dirty = 1;
        return 0;
    }

    /* Depth */
    if (method == NV4097_SET_DEPTH_TEST_ENABLE) {
        state->depth_test_enable = data ? 1 : 0;
        state->depth_dirty = 1;
        return 0;
    }
    if (method == NV4097_SET_DEPTH_FUNC) {
        state->depth_func = data;
        state->depth_dirty = 1;
        return 0;
    }
    if (method == NV4097_SET_DEPTH_MASK) {
        state->depth_mask = data ? 1 : 0;
        state->depth_dirty = 1;
        return 0;
    }

    /* Stencil */
    if (method == NV4097_SET_STENCIL_TEST_ENABLE) {
        state->stencil_test_enable = data ? 1 : 0;
        state->stencil_dirty = 1;
        return 0;
    }
    if (method == NV4097_SET_STENCIL_FUNC) {
        state->stencil_func = data;
        state->stencil_dirty = 1;
        return 0;
    }
    if (method == NV4097_SET_STENCIL_FUNC_REF) {
        state->stencil_ref = data;
        state->stencil_dirty = 1;
        return 0;
    }
    if (method == NV4097_SET_STENCIL_FUNC_MASK) {
        state->stencil_mask = data;
        state->stencil_dirty = 1;
        return 0;
    }
    if (method == NV4097_SET_STENCIL_OP_FAIL) {
        state->stencil_op_fail = data;
        state->stencil_dirty = 1;
        return 0;
    }
    if (method == NV4097_SET_STENCIL_OP_ZFAIL) {
        state->stencil_op_zfail = data;
        state->stencil_dirty = 1;
        return 0;
    }
    if (method == NV4097_SET_STENCIL_OP_ZPASS) {
        state->stencil_op_zpass = data;
        state->stencil_dirty = 1;
        return 0;
    }

    /* Culling */
    if (method == NV4097_SET_CULL_FACE_ENABLE) {
        state->cull_face_enable = data ? 1 : 0;
        return 0;
    }
    if (method == NV4097_SET_CULL_FACE) {
        state->cull_face = data;
        return 0;
    }
    if (method == NV4097_SET_FRONT_FACE) {
        state->front_face = data;
        return 0;
    }

    /* Shader programs */
    if (method == NV4097_SET_SHADER_PROGRAM) {
        /*
         * Fragment program address register:
         *   [1:0]  location (0 = local, 1 = main)
         *   [31:2] offset (4-byte aligned)
         */
        state->shader_program = data;
        state->fragment_program_addr = data & ~0x3u;
        state->shader_dirty = 1;
        return 0;
    }
    if (method == NV4097_SET_TRANSFORM_PROGRAM_LOAD) {
        /* Vertex program load slot — instruction index (each = 16 bytes).
         * Following NV4097_SET_TRANSFORM_PROGRAM words fill from here. */
        state->transform_program_load = data;
        state->vp_ucode_write = data * 16;
        state->shader_dirty = 1;
        return 0;
    }

    if (method == NV4097_SET_TRANSFORM_PROGRAM_START) {
        if (state->transform_program_start != data) state->vp_dirty = 1;
        state->transform_program_start = data;
        state->shader_dirty = 1;
        return 0;
    }

    /* NV4097_SET_TRANSFORM_PROGRAM[0..31] — a run of 32-bit vertex-program
     * microcode words appended at the current write cursor. Capture them so the
     * backend can decompile the real VP (4 words = one NV40 instruction). */
    if (method >= NV4097_SET_TRANSFORM_PROGRAM &&
        method <  NV4097_SET_TRANSFORM_PROGRAM + 32 * 4) {
        u32 w = state->vp_ucode_write;
        if (w + 4 <= sizeof(state->vp_ucode)) {
            /* data is host-endian already; store as little-endian bytes. */
            state->vp_ucode[w+0] = (u8)(data);
            state->vp_ucode[w+1] = (u8)(data >> 8);
            state->vp_ucode[w+2] = (u8)(data >> 16);
            state->vp_ucode[w+3] = (u8)(data >> 24);
            state->vp_ucode_write = w + 4;
            if (w + 4 > state->vp_ucode_bytes) state->vp_ucode_bytes = w + 4;
            state->vp_dirty = 1;
        }
        return 0;
    }
    if (method == NV4097_SET_VERTEX_ATTRIB_OUTPUT_MASK) {
        state->vertex_attrib_output_mask = data;
        return 0;
    }
    /* NV4097_SET_VERTEX_DATA4F_M (0x1C00): the constant/"current" value for each
     * vertex attribute, 16 attributes x 4 floats. The hardware feeds this to
     * every vertex when the attribute's ARRAY is disabled -- it is not zero.
     *
     * Rubber Ducky leaves attribute 3 (diffuse colour) disabled and sets it here
     * instead; its duck shader computes (lighting * col0) * texture + spec, so a
     * zero col0 multiplies the texture away and the ducks rendered as a dim grey
     * specular term only -- present in the framebuffer, invisible on screen.
     * SET_VERTEX_DATA2F/4UB/2S/4S are the other encodings of the same register
     * file; add them if a title needs them. */
    if (method >= 0x1C00u && method < 0x1C00u + RSX_MAX_VERTEX_ATTRIBS * 16u) {
        u32 idx  = (method - 0x1C00u) >> 4;         /* attribute */
        u32 lane = ((method - 0x1C00u) >> 2) & 3;   /* x/y/z/w   */
        float f; memcpy(&f, &data, 4);
        state->vertex_data4f[idx][lane] = f;
        /* Immediate-mode emit: NV4097 issues a vertex when the last component
         * of position (attrib 0) is written inside BEGIN/END and the array is
         * disabled. Without this, SET_VERTEX_DATA4F_M quads never reach draw. */
        if (state->in_begin_end && idx == 0 && lane == 3 &&
            !state->vertex_attribs[0].enabled &&
            s_inline_n + 4u <= RSX_INLINE_MAX) {
            for (int k = 0; k < 4; k++) {
                u32 w; memcpy(&w, &state->vertex_data4f[0][k], 4);
                s_inline_words[s_inline_n++] = w;
            }
        }
        { static int _d = -1; if (_d < 0) _d = getenv("VDATA_DBG") ? 1 : 0;
          static int _n = 0;
          if (_d && _n < 400) { _n++;
            fprintf(stderr, "[VDATA4F] attr=%u lane=%u = %.4f%c", idx, lane, f, 10); } }
        return 0;
    }

    if (method == NV4097_SET_TRANSFORM_CONSTANT_LOAD) {
        { static int _n=0; if (getenv("LOAD_DBG") && _n++ < 200)
            fprintf(stderr, "[LOAD] transform_constant_load = %u\n", data); }
        state->transform_constant_load = data;
        return 0;
    }

    /* NV4097_SET_TRANSFORM_CONSTANT[0..63] — up to 64 dwords (16 vec4s) per
     * command. Each register slot writes to a lane of one vertex constant
     * vec4: vec_index = LOAD + (reg_offset/4), lane = reg_offset%4.
     * The data arrives as a host-endian u32; reinterpret the bits as float
     * because the game's intent is "these 32 bits are a float". The hardware
     * does NOT auto-advance LOAD between commands — games re-issue
     * SET_TRANSFORM_CONSTANT_LOAD before each block. */
    if (method >= NV4097_SET_TRANSFORM_CONSTANT &&
        method <  NV4097_SET_TRANSFORM_CONSTANT + 64 * 4) {
        u32 reg_offset = (method - NV4097_SET_TRANSFORM_CONSTANT) / 4;
        u32 slot = state->transform_constant_load + (reg_offset >> 2);
        u32 lane = reg_offset & 3;
        if (slot < RSX_MAX_VERTEX_CONSTANTS) {
            float f;
            memcpy(&f, &data, 4);
            { static int _en=-1; if(_en<0){const char*e=getenv("TCONST_DBG");_en=e?1:0;}
              static int _n=0;
              static int _max=-1; if(_max<0){const char*m=getenv("TCONST_MAX");_max=m?atoi(m):64;}
              int _hit = _en && (getenv("TCONST_ALL") ? (_n<_max) : (slot>=12 && slot<=30 && _n<400));
              if(_hit){ _n++; fprintf(stderr,"[TCONST] load=%u slot=%u lane=%u = %.4f\n", state->transform_constant_load, slot, lane, f); } }
            state->vertex_constants[slot][lane] = f;
            { static int _sq=0; if (getenv("SEQ_DBG") && slot==256 && lane==0 && _sq++ < 500)
                fprintf(stderr, "[SEQ] upload c256.x=%.4f (load=%u)\n", f, state->transform_constant_load); }
            if (!state->vertex_constants_dirty) {
                state->vertex_constants_lo = slot;
                state->vertex_constants_hi = slot;
                state->vertex_constants_dirty = 1;
            } else {
                if (slot < state->vertex_constants_lo) state->vertex_constants_lo = slot;
                if (slot > state->vertex_constants_hi) state->vertex_constants_hi = slot;
            }
        }
        return 0;
    }

    /* Draw */
    if (method == NV4097_SET_BEGIN_END) {
        if (data != 0) {
            s_inline_n = 0;
            s_elem_n = 0;
            state->primitive_type = data;
            state->in_begin_end = 1;
            state->begin_epoch++;
            { static int after = 0, n = 0;
              if (rsx_hi_rt(state)) after = 1;
              if (after && n++ < 16)
                  fprintf(stderr, "[RSX] BEGIN_END prim=%u surf=0x%08X clip=%ux%u%c",
                          data, state->surface_color_offset[0],
                          state->surface_clip_w, state->surface_clip_h, 10); }

            /* Flush dirty state to backend before drawing */
            if (s_backend) {
                if (state->surface_dirty && s_backend->set_render_target)
                    s_backend->set_render_target(s_backend->userdata, state);
                if (state->viewport_dirty && s_backend->set_viewport)
                    s_backend->set_viewport(s_backend->userdata, state);
                if (state->blend_dirty && s_backend->set_blend)
                    s_backend->set_blend(s_backend->userdata, state);
                if ((state->depth_dirty || state->stencil_dirty) && s_backend->set_depth_stencil)
                    s_backend->set_depth_stencil(s_backend->userdata, state);
                if (state->color_mask_dirty && s_backend->set_color_mask)
                    s_backend->set_color_mask(s_backend->userdata, state);
                if (state->alpha_dirty && s_backend->set_alpha_test)
                    s_backend->set_alpha_test(s_backend->userdata, state);
                if (state->shader_dirty && s_backend->set_shader)
                    s_backend->set_shader(s_backend->userdata, state);
                if (state->vertex_dirty && s_backend->set_vertex_attribs)
                    s_backend->set_vertex_attribs(s_backend->userdata, state);

                /* Bind dirty textures */
                if (state->texture_dirty && s_backend->bind_texture) {
                    for (u32 i = 0; i < RSX_MAX_TEXTURES; i++) {
                        if (state->textures[i].dirty) {
                            s_backend->bind_texture(s_backend->userdata, i, &state->textures[i]);
                            state->textures[i].dirty = 0;
                        }
                    }
                }

                state->surface_dirty = 0;
                state->viewport_dirty = 0;
                state->blend_dirty = 0;
                state->depth_dirty = 0;
                state->stencil_dirty = 0;
                state->color_mask_dirty = 0;
                state->alpha_dirty = 0;
                state->shader_dirty = 0;
                state->vertex_dirty = 0;
                state->texture_dirty = 0;
            }
        } else {
            rsx_flush_begin_end(state);
            state->in_begin_end = 0;
        }
        return 0;
    }

    if (method == NV4097_INLINE_ARRAY) {
        if (s_inline_n < RSX_INLINE_MAX)
            s_inline_words[s_inline_n++] = data;
        return 0;
    }
    if (method == NV4097_ARRAY_ELEMENT32) {
        if (s_elem_n < RSX_INLINE_MAX)
            s_elem_idx[s_elem_n++] = data;
        return 0;
    }
    if (method == NV4097_ARRAY_ELEMENT16) {
        if (s_elem_n < RSX_INLINE_MAX)
            s_elem_idx[s_elem_n++] = data & 0xFFFFu;
        if (s_elem_n < RSX_INLINE_MAX)
            s_elem_idx[s_elem_n++] = data >> 16;
        return 0;
    }

    if (method == NV4097_DRAW_ARRAYS) {
        u32 first = data & 0xFFFFFF;
        u32 count = ((data >> 24) & 0xFF) + 1;
        { static int _d=0; static int after=0, rec=0;
          if (rsx_hi_rt(state)) after = 1;
          if (_d++ < 32 || (after && rec++ < 24))
            fprintf(stderr, "[RSX] DRAW_ARRAYS prim=%u first=%u count=%u surf=0x%08X clip=%ux%u tgt=0x%X%s\n",
                    state->primitive_type, first, count, state->surface_color_offset[0],
                    state->surface_clip_w, state->surface_clip_h, state->color_target,
                    after && _d > 32 ? " (post-hi-rt)" : ""); }
        { static int _sq=0; if (getenv("SEQ_DBG") && _sq++ < 500)
            fprintf(stderr, "[SEQ] DRAW surf0=0x%X c256.x=%.4f c257.y=%.4f\n",
                    state->surface_color_offset[0], state->vertex_constants[256][0],
                    state->vertex_constants[257][1]); }
        /* MVPDBG: dump every non-zero vertex-constant slot the FIRST time a
         * G-buffer draw (surf0=0xCC0000) is dispatched -- the true MVP the GPU
         * will use, read live (no snapshot/parity indirection). */
        if (getenv("MVPDBG") && state->surface_color_offset[0] == 0xCC0000) {
            static int _once = 0;
            if (!_once) { _once = 1;
                for (int s = 0; s < RSX_MAX_VERTEX_CONSTANTS; s++) {
                    float* v = state->vertex_constants[s];
                    if (v[0]||v[1]||v[2]||v[3])
                        fprintf(stderr, "[MVP] c[%3d] = %10.4f %10.4f %10.4f %10.4f\n",
                                s, v[0], v[1], v[2], v[3]);
                }
            }
        }
        ps3_ms("rsx:draw_arrays");
        if (s_backend && s_backend->draw_arrays)
            s_backend->draw_arrays(s_backend->userdata, state->primitive_type, first, count);
        return 0;
    }

    if (method == NV4097_SET_SHADER_CONTROL) {
        { static int _sc=0; if (_sc++ < 12 && getenv("RTT_DUMP"))
            fprintf(stderr, "[RSXSC] shader_control=0x%X\n", data); }
        state->shader_control = data;
        state->shader_dirty = 1;
        return 0;
    }
    if (method == NV4097_SET_INDEX_ARRAY_ADDRESS) {
        state->index_array_offset = data;
        return 0;
    }
    if (method == NV4097_SET_INDEX_ARRAY_DMA) {
        state->index_array_dma = data;
        return 0;
    }
    if (method == NV4097_DRAW_INDEX_ARRAY) {
        /* [23:0] first index, [31:24] count-1 (same packing as DRAW_ARRAYS). */
        u32 first = data & 0xFFFFFF;
        u32 count = ((data >> 24) & 0xFF) + 1;
        { static int _d=0; static int after=0, rec=0;
          if (rsx_hi_rt(state)) after = 1;
          if (_d++ < 8 || (after && rec++ < 16))
            fprintf(stderr, "[RSX] DRAW_INDEX_ARRAY prim=%u first=%u count=%u idxoff=0x%X dma=0x%X surf=0x%08X%s\n",
                    state->primitive_type, first, count, state->index_array_offset,
                    state->index_array_dma, state->surface_color_offset[0],
                    after && _d > 8 ? " (post-hi-rt)" : ""); }
        ps3_ms("rsx:draw_indexed");
        if (s_backend && s_backend->draw_indexed)
            s_backend->draw_indexed(s_backend->userdata, state->primitive_type,
                                    first, count);
        return 0;
    }

    /* Scissor */
    if (method == NV4097_SET_SCISSOR_HORIZONTAL) {
        state->scissor_x = data & 0xFFFF;
        state->scissor_w = (data >> 16) & 0xFFFF;
        return 0;
    }
    if (method == NV4097_SET_SCISSOR_VERTICAL) {
        state->scissor_y = data & 0xFFFF;
        state->scissor_h = (data >> 16) & 0xFFFF;
        return 0;
    }

    /* NV406E_SET_REFERENCE (0x0050): the title writes a reference value it then
     * spins on (cellGcmFinish / cellGcmSetWaitLabel). Record the latest so the
     * GCM control register can report completion back to the guest and unblock
     * the spin. Class 0 (software) method, no subchannel state needed here. */
    if (method == 0x0050) {
        g_rsx_last_reference = data;
        return 0;
    }

    /* GCM_FLIP_HEAD (0xE920/0xE924): immediate display flip from the FIFO.
     * The draw engine handles the actual present; suppress the unknown log. */
    if (method == 0xE920 || method == 0xE924) return 0;

    /* Unique unknown methods in every build. Release used to swallow these,
     * so a 2D blit on the 3D subchannel (or INLINE_ARRAY before it was
     * decoded) left no trace. */
    { static u8 seen[2048];
      u32 mi = (method >> 2) & 2047;
      if (!seen[mi]) {
          seen[mi] = 1;
          fprintf(stderr, "[RSX] unknown method 0x%04X = 0x%08X surf=0x%08X%c",
                  method, data, state->surface_color_offset[0], 10);
      } }

    return -1;
}

/* ---------------------------------------------------------------------------
 * Command buffer parsing
 * -----------------------------------------------------------------------*/

int rsx_process_command_buffer(rsx_state* state, const u32* buf, u32 size)
{
    int methods_processed = 0;
    u32 pos = 0;
    u32 count = size / 4; /* size in dwords */

    { static int _c=0; if (count && _c++ < 16) fprintf(stderr, "[RSX] process_cmd_buffer words=%u first_hdr=0x%08X\n", count, buf[0]); }
    while (pos < count) {
        u32 header = buf[pos++];
        u32 type = (header >> 29) & 0x7;

        if (type == 0 || type == 2) {
            /* Increasing or non-increasing method */
            u32 method = (header >> 2) & 0x7FF;
            method <<= 2; /* method addresses are dword-aligned */
            u32 num_data = (header >> 18) & 0x7FF;
            int increasing = (type == 0);

            for (u32 i = 0; i < num_data && pos < count; i++) {
                u32 data = buf[pos++];
                u32 m = increasing ? (method + i * 4) : method;
                rsx_process_method(state, m, data);
                methods_processed++;
            }
        } else if (type == 1) {
            /* Jump — change command buffer read position */
            /* In recomp context, this is handled by the caller */
            break;
        } else {
            /* Unknown type, skip */
            break;
        }
    }

    return methods_processed;
}
