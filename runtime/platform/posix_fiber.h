/*
 * ps3recomp - cooperative fiber contexts on plain pthreads
 *
 * cellFiber switches PPU fibers with Windows fibers on Windows and the
 * ucontext family (getcontext / makecontext / swapcontext / setcontext)
 * everywhere else. Bionic ships <ucontext.h> with the ucontext_t type and
 * none of the four functions, so on Android the same four operations are
 * provided here on top of pthreads instead.
 *
 * Model. Every context is a parked thread and exactly one of them holds the
 * run token at any time. A "switch" hands the token to the target and blocks
 * on the caller's own token, so the cooperative, one-runs-at-a-time contract
 * of ucontext is preserved: no two fibers ever execute concurrently, and a
 * fiber resumes exactly where it blocked, on its own stack. The scheduler
 * context is whichever thread called getcontext on it (the guest thread that
 * called cellFiberPpuInitialize).
 *
 * What this is NOT: a same-thread stack switch. The fiber body runs on a
 * different host thread than the scheduler, so anything the runtime keys on
 * the host thread identity (PPU_THREAD_LOCAL state in the boot scaffold,
 * t_current_thread in win32_compat.c, GetCurrentThreadId) sees the fiber
 * thread, not the guest thread that created the fiber. cellFiber is a
 * cooperative scheduler inside one PPU thread on the console, so the guest
 * cannot observe the difference; the runtime's own per-thread bookkeeping can.
 * A hand-written arm64 context switch (save x19-x30, sp, d8-d15) is the
 * upgrade path if that ever matters; this file exists so the runtime library
 * links and boots on Bionic without any assembly.
 *
 * Cost: one host thread per live fiber (CELL_FIBER_MAX_FIBERS = 64) and a
 * futex round trip per switch instead of a register save/restore.
 *
 * Header-only and static: cellFiber.c is the only consumer, and the
 * tests/fiber_switch harness compiles that file standalone.
 */
#ifndef PS3RECOMP_POSIX_FIBER_H
#define PS3RECOMP_POSIX_FIBER_H

#ifndef _WIN32

#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <stddef.h>
#include <string.h>

/* Fibers run lifted PPU code, which nests one host frame per guest call, so a
 * guest-sized stack (64 KB default) is far too small for the host thread. */
#ifndef PS3_FIBER_MIN_HOST_STACK
#  define PS3_FIBER_MIN_HOST_STACK (1u << 20)
#endif

typedef void (*ps3_fiber_entry)(int, int);

typedef struct ps3_fiber_ctx {
    pthread_mutex_t  m;
    pthread_cond_t   c;
    int              go;          /* run token: 1 while this context may run */
    int              inited;      /* m/c are live */
    int              has_thread;  /* a worker was created (joinable) */
    int              cancel;      /* wake the worker only to let it exit */
    int              finished;    /* the worker has left its body */
    pthread_t        thread;      /* owner: the worker, or the getcontext caller */
    ps3_fiber_entry  fn;
    int              a0, a1;
} ps3_fiber_ctx;

/* Park until this context holds the run token. A context being destroyed is
 * woken with `cancel` set; its thread leaves right here rather than returning
 * into a fiber body nobody is scheduling any more -- the pthread equivalent
 * of DeleteFiber / free(stack) on a suspended fiber. */
static inline void ps3_fiber__wait_token(ps3_fiber_ctx* c)
{
    pthread_mutex_lock(&c->m);
    while (!c->go) pthread_cond_wait(&c->c, &c->m);
    c->go = 0;
    int cancel = c->cancel;
    pthread_mutex_unlock(&c->m);
    if (cancel)
        pthread_exit(NULL);
}

static inline void ps3_fiber__give_token(ps3_fiber_ctx* c)
{
    pthread_mutex_lock(&c->m);
    c->go = 1;
    pthread_cond_signal(&c->c);
    pthread_mutex_unlock(&c->m);
}

/* getcontext: (re)initialise a context owned by the calling thread. For the
 * scheduler this is the whole story; a fiber slot goes through makecontext
 * next, which replaces the owner with the worker it creates. */
static inline int ps3_fiber_getcontext(ps3_fiber_ctx* c)
{
    if (c->inited) {
        pthread_cond_destroy(&c->c);
        pthread_mutex_destroy(&c->m);
    }
    memset(c, 0, sizeof *c);
    int rc = pthread_mutex_init(&c->m, NULL);
    if (rc != 0) { errno = rc; return -1; }
    rc = pthread_cond_init(&c->c, NULL);
    if (rc != 0) { pthread_mutex_destroy(&c->m); errno = rc; return -1; }
    c->inited = 1;
    c->thread = pthread_self();
    return 0;
}

static inline void* ps3_fiber__worker(void* p)
{
    ps3_fiber_ctx* c = (ps3_fiber_ctx*)p;
    ps3_fiber__wait_token(c);          /* exits the thread if cancelled */
    if (c->fn)
        c->fn(c->a0, c->a1);
    /* Reached only if the body returns (cellFiber's trampoline never does: it
     * ends in setcontext, which exits this thread). Nothing to hand the token
     * to here -- ucontext would follow uc_link -- so just mark it. */
    c->finished = 1;
    return NULL;
}

/* makecontext: bind an entry and start its worker, parked until the first
 * switch. `stack_size` is the guest's request; the host thread gets at least
 * PS3_FIBER_MIN_HOST_STACK. Returns 0, or -1 with errno set. */
static inline int ps3_fiber_makecontext(ps3_fiber_ctx* c, size_t stack_size,
                                        ps3_fiber_entry fn, int a0, int a1)
{
    if (!c->inited || c->has_thread) { errno = EINVAL; return -1; }
    c->fn = fn; c->a0 = a0; c->a1 = a1;
    c->go = 0; c->cancel = 0; c->finished = 0;

    size_t sz = stack_size;
    if (sz < PS3_FIBER_MIN_HOST_STACK) sz = PS3_FIBER_MIN_HOST_STACK;
#ifdef PTHREAD_STACK_MIN
    if (sz < (size_t)PTHREAD_STACK_MIN) sz = (size_t)PTHREAD_STACK_MIN;
#endif
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, sz);
    int rc = pthread_create(&c->thread, &attr, ps3_fiber__worker, c);
    pthread_attr_destroy(&attr);
    if (rc != 0) { errno = rc; return -1; }
    c->has_thread = 1;
    return 0;
}

/* swapcontext: hand the run token to `to`, park until somebody hands it back
 * to `from`. Must be called by the thread that owns `from` -- which is the
 * only way cellFiber calls it. Switching to the context the caller already
 * owns is a no-op (swapcontext(x, x) is a self-resume there too). */
static inline int ps3_fiber_swapcontext(ps3_fiber_ctx* from, ps3_fiber_ctx* to)
{
    if (!from->inited || !to->inited) { errno = EINVAL; return -1; }
    if (pthread_equal(to->thread, pthread_self()))
        return 0;
    ps3_fiber__give_token(to);
    ps3_fiber__wait_token(from);
    return 0;
}

/* setcontext: hand the token to `to` and never come back. The calling worker
 * exits; its slot is reaped by ps3_fiber_destroy. Called only from a fiber
 * (cellFiberPpuExitFiber and the end of the trampoline), never from the
 * scheduler thread -- if it were, exiting that thread would be wrong, so a
 * scheduler-owned caller just yields the token and returns. */
static inline int ps3_fiber_setcontext(ps3_fiber_ctx* to)
{
    if (!to->inited) { errno = EINVAL; return -1; }
    if (pthread_equal(to->thread, pthread_self()))
        return 0;
    ps3_fiber__give_token(to);
    pthread_exit(NULL);
    return 0; /* unreachable */
}

/* Tear a context down. A worker that never ran, or is parked mid-body, is
 * woken with `cancel` set so it can leave; either way it is joined, so the
 * stack is gone when this returns. Safe on a zeroed (never initialised)
 * context and on the scheduler context. Must not be called by the worker on
 * its own context. */
static inline void ps3_fiber_destroy(ps3_fiber_ctx* c)
{
    if (!c->inited) return;
    if (c->has_thread) {
        if (!pthread_equal(c->thread, pthread_self())) {
            pthread_mutex_lock(&c->m);
            c->cancel = 1;
            c->go = 1;
            pthread_cond_signal(&c->c);
            pthread_mutex_unlock(&c->m);
            pthread_join(c->thread, NULL);
        } else {
            /* Cannot join ourselves; leave the thread to exit on its own. */
            pthread_detach(c->thread);
        }
        c->has_thread = 0;
    }
    pthread_cond_destroy(&c->c);
    pthread_mutex_destroy(&c->m);
    c->inited = 0;
}

#endif /* !_WIN32 */
#endif /* PS3RECOMP_POSIX_FIBER_H */
