/* spu_interp.h — SPU interpreter: the correctness floor for the SPU recompiler.
 *
 * Statically lifted functions are the fast path. Computed/indirect branches
 * (`bi $reg`) and code that only exists at runtime — DMA'd overlays (LBP),
 * self-modified code (spu_0004's `stqd; sync; bi $3` trampoline at LS 0xC0) —
 * have no static bytes to lift. On a lifted-function-table miss we interpret
 * from live local store instead of falling off a cliff.
 *
 * The interpreter shares the EXACT spu_context (gpr, ls, channels) the lifted
 * code uses and calls the SAME spu_<mnemonic> helpers, so control and state
 * cross the boundary transparently.
 */
#ifndef SPU_INTERP_H
#define SPU_INTERP_H

#include "spu_context.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A lifted SPU function: void f(spu_context*). Matches the lifter's signature. */
typedef void (*spu_lifted_fn)(spu_context*);

/* Registered lifted-function lookup by local-store entry address for the image
 * currently selected in ctx->image_id. Weak: the default returns NULL (pure
 * interpretation). The lifted image's generated spu_function_table registration
 * overrides this so computed branches reach compiled code. */
spu_lifted_fn spu_lifted_lookup(const spu_context* ctx, uint32_t lsa);

/* Interpret from `start_lsa` until the SPU stops (`stop`/`stopd`/halt) or a
 * host-fatal decode. Handles all control flow internally. Returns the stop
 * code (0 if none). ctx->pc holds the address that stopped execution.
 *
 * When g_spu_interp_until is non-zero, skip lifted rejoin (DMA'd extra PIC
 * shares LSAs with leftover lifts) and return 0 as soon as pc hits that
 * address — the bisl link / call return. */
extern uint32_t g_spu_interp_until;
extern uint64_t g_spu_interp_steps;
uint32_t spu_interp_run(spu_context* ctx, uint32_t start_lsa);

/* CollNarrow extra-PIC bisl: interpret live DMA'd bytes until `until` (the
 * planted link), then restore r0/r1/r2 and r80-r127. A nested helper that
 * returns to that link skips the PIC trampoline epilogue and leaves r1 ~416
 * bytes low, so the caller's stack reload of r91 (mailbox EA for the 0xF5E8
 * PUT of 16 bytes to +0x10) reads the wrong slot. LS writes still stand. */
uint32_t spu_interp_run_keep_nv(spu_context* ctx, uint32_t start_lsa, uint32_t until);

/* The single dispatch point every computed branch routes through. If `target`
 * is a registered lifted entry, runs it; otherwise interprets from `target`.
 * The lifter emits `bi $reg` / unresolved indirect branches as a call here. */
void spu_dispatch(spu_context* ctx, uint32_t target);

#ifdef __cplusplus
}
#endif
#endif /* SPU_INTERP_H */
