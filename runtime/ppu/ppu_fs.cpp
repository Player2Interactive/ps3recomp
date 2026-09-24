/*
 * ps3recomp - cellFs VFS (sys_fs HLE)
 *
 * Backs the game's file I/O with the real host filesystem so it can load its
 * data/config/assets. Guest PS3 paths (/dev_bdvd/..., /app_home/..., /dev_hdd0,
 * etc.) are translated to a host root (the game directory that contains
 * PS3_GAME), set by the boot harness from the EBOOT path (or $PS3_VFS_ROOT).
 *
 * Only the calls the boot actually imports are implemented:
 *   cellFsOpen/Close/Read/Write/Lseek/Stat/Fstat/Opendir/Readdir/Closedir/
 *   Mkdir/Rmdir/Unlink/Rename/Fsync/GetFreeSize, plus Truncate/Ftruncate/
 *   GetBlockSize/FGetBlockSize/AioWrite/AllocateFileAreaWithoutZeroFill/
 *   WithInitialData/ChangeFileSizeWithoutAllocation so a longer FIOS overlay
 *   (1.75 GiB cache.dat) does not fall through to the toolkit fd table.
 *
 * Context-aware HLE: guest pointers (path, buffers, out params) are read/written
 * through vm_base in big-endian.
 */
#include "ppu_recomp.h"      /* ppu_context */
#include "../../libs/filesystem/edat.h"
#include "ps3emu/nid.h"      /* ps3_compute_nid */
#include "ps3emu/guest_call.h" /* ps3_invoke_guest: AIO completion is a guest OPD */
#include "sdata_decrypt.h"   /* SDATA/EDAT (NPD) decryption for cellFsSdataOpen */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include "../platform/win32_dirent.h"   /* <dirent.h>, or a Win32 stand-in */
#ifdef _WIN32
#include <winioctl.h>    /* FSCTL_SET_SPARSE for AllocateFileArea */
#endif

/* Guest-resolving host backtrace (ppu_loader.cpp). Must be declared at file scope:
 * `extern "C"` inside a function body is ill-formed in C++. */
extern "C" void ydkj_host_bt(const char* tag);
#include <fcntl.h>
#include <errno.h>
#ifdef _WIN32
#include <io.h>          /* open/close on MinGW */
#include <direct.h>
#define HOST_MKDIR(p) _mkdir(p)
#else
#include <unistd.h>
#define HOST_MKDIR(p) mkdir((p), 0777)
#endif
#ifndef O_BINARY
#define O_BINARY 0       /* POSIX has no text/binary distinction */
#endif
#ifdef _WIN32
#  define HOST_FSEEK64(f,off,wh) _fseeki64((f), (__int64)(off), (wh))
#  define HOST_FTELL64(f)        _ftelli64(f)
#else
#  define HOST_FSEEK64(f,off,wh) fseeko((f), (off_t)(off), (wh))
#  define HOST_FTELL64(f)        ftello(f)
#endif

extern "C" uint8_t* vm_base;
extern "C" void ppu_guest_caller(char* out, size_t n);
extern "C" uint32_t ppu_vm_size;
extern "C" void     ps3_hle_register_ctx(uint32_t nid, const char* name, void (*fn)(ppu_context*));
extern "C" void     vm_write32(uint64_t a, uint32_t v);
extern "C" void     vm_write64(uint64_t a, uint64_t v);
extern "C" uint64_t cellfs_host_free_bytes(const char* host_path);
#include "../platform/win32_backtrace.h"   /* RtlCaptureStackBackTrace / GetModuleHandleA on POSIX */

/* Host root that the PS3 mount points map into (dir containing PS3_GAME). */
extern "C" const char* ppu_vfs_root = ".";

/* CELL_FS return / mode / flag constants. */
#define CELL_OK              0
#define CELL_FS_ENOENT       (-2147418106)   /* 0x80010006 -- was -2147418090, which is 0x80010016 = CELL_ENOTCONN, not ENOENT */
#define CELL_FS_EISDIR       (-2147418094)   /* 0x80010012 */
#define CELL_FS_ENOSPC       (-2147418077)   /* 0x80010023 */
#define CELL_FS_EINVAL       (-2147418110)   /* 0x80010002 */
#define CELL_FS_EFAULT       (-2147418099)   /* 0x8001000D */
#define CELL_FS_EIO          (-2147418111)
#define CELL_FS_S_IFDIR      0x4000u
#define CELL_FS_S_IFREG      0x8000u
#define CELL_FS_O_RDONLY     0
#define CELL_FS_O_WRONLY     1
#define CELL_FS_O_RDWR       2
/* SDK / cellFs.h are octal (000100 CREAT, 001000 TRUNC). Hex 0x200 was TRUNC
 * mislabelled as CREAT, so FIOS O_RDWR|O_CREAT (0x42) never set O_CREAT and
 * st_blksize 0x200 made overlay I/O 512 B instead of 4 KiB. */
#define CELL_FS_O_CREAT      000100
#define CELL_FS_O_EXCL       000200
#define CELL_FS_O_TRUNC      001000
#define CELL_FS_O_APPEND     002000
#define CELL_FS_EEXIST       (-2147418092)   /* 0x80010014 */
#define CELL_FS_SEEK_SET     0
#define CELL_FS_SEEK_CUR     1
#define CELL_FS_SEEK_END     2
#define CELL_FS_TYPE_DIR     1
#define CELL_FS_TYPE_REG     2

/* ---- guest memory string helpers ---- */
static void guest_strcpy(char* dst, uint32_t gaddr, size_t cap)
{
    size_t i = 0;
    for (; i < cap - 1; i++) {
        if (ppu_vm_size && gaddr + i >= ppu_vm_size) break;
        char c = (char)vm_base[gaddr + i];
        if (!c) break;
        dst[i] = c;
    }
    dst[i] = 0;
}

/* Defined in runtime/syscalls/sys_fs.c -- shared by all three path
 * translators. (extern "C" is ill-formed at block scope.) */
extern "C" void ps3_vfs_ps3game_fallback(char* path, size_t cap);

static int host_mkdir_p(const char* path);

/* Host directory that /dev_hdd1 maps into (the cellSysCacheMount partition).
 * FIOS overlay paths like //cache.idx become /dev_hdd1/cache/<id>/cache.idx
 * after SysCacheMount; dropping the mount prefix onto the disc dump made
 * that look for <VFS>/cache/<id>/cache.idx and FIOS logged err=-129. */
static void hdd1_host_root(char* out, size_t cap)
{
    const char* e1 = getenv("PS3_HDD1_ROOT");
    if (e1 && *e1) {
        snprintf(out, cap, "%s", e1);
    } else {
        const char* e0 = getenv("PS3_HDD0_ROOT");
        if (e0 && *e0) {
            snprintf(out, cap, "%s/syscache", e0);
        } else {
            struct stat st;
            if (stat("game/hdd0", &st) == 0 && (st.st_mode & S_IFMT) == S_IFDIR)
                snprintf(out, cap, "game/hdd0/syscache");
            else
                snprintf(out, cap, "%s/hdd0/syscache", ppu_vfs_root ? ppu_vfs_root : ".");
        }
    }
    for (char* p = out; *p; p++) if (*p == '\\') *p = '/';
}

static void hdd1_prepare(const char* guest, const char* hpath)
{
    char parent[1100];
    snprintf(parent, sizeof parent, "%s", hpath);
    char* slash = strrchr(parent, '/');
    if (slash && slash != parent) {
        *slash = 0;
        host_mkdir_p(parent);
    }
    const char* base = strrchr(guest, '/');
    base = base ? base + 1 : guest;
    /* FIOS 1.3 overlay: cache.idx is the index, cache.dat the payload. Both
     * are absent on a freshly formatted syscache; seed empty files so
     * openCacheFile is not CELL_ENOENT (-129). */
    if (strcmp(base, "cache.idx") != 0 && strcmp(base, "cache.dat") != 0)
        return;
    struct stat st;
    if (stat(hpath, &st) == 0)
        return;
    FILE* sf = fopen(hpath, "wb");
    if (sf) {
        fclose(sf);
        fprintf(stderr, "[fs] seeded empty syscache '%s' -> '%s'\n", guest, hpath);
    }
}

/* Translate a guest path to a host path under ppu_vfs_root. Known PS3 mount
 * prefixes are stripped; the rest is appended to the root. */
static void host_path(char* out, size_t cap, const char* guest)
{
    const char* rel = guest;
    /* /dev_hdd0 overlays the installed game-update dir (a disc title patched to
     * e.g. v1.30 runs the update's EBOOT and reads its patchN.farc from
     * /dev_hdd0/game/<title>/), mirroring the PS3/RPCS3 layout: base data on
     * /dev_bdvd (disc), update data on /dev_hdd0. Set PS3_HDD0_ROOT to the host
     * dir that /dev_hdd0 maps into (the one containing game/<title>/). */
    static const char* hdd0_root = nullptr; static int hdd0_init = 0;
    if (!hdd0_init) { hdd0_root = getenv("PS3_HDD0_ROOT"); hdd0_init = 1; }
    if (hdd0_root && strncmp(guest, "/dev_hdd0/", 10) == 0) {
        snprintf(out, cap, "%s/%s", hdd0_root, guest + 10);
        for (char* p = out; *p; p++) if (*p == '\\') *p = '/';
        return;
    }

    /* /dev_hdd1 is the title syscache (cellSysCacheMount), not disc data. */
    if (strncmp(guest, "/dev_hdd1/", 10) == 0 || strcmp(guest, "/dev_hdd1") == 0) {
        char root[1024];
        hdd1_host_root(root, sizeof root);
        const char* rest = (guest[9] == '/') ? guest + 10 : "";
        if (*rest)
            snprintf(out, cap, "%s/%s", root, rest);
        else
            snprintf(out, cap, "%s", root);
        for (char* p = out; *p; p++) if (*p == '\\') *p = '/';
        static int logged = 0;
        if (!logged) {
            logged = 1;
            fprintf(stderr, "[fs] /dev_hdd1 -> '%s'\n", root);
        }
        hdd1_prepare(guest, out);
        return;
    }

    /* /dev_flash is FIRMWARE, not game data — mapping it into the game root (the
     * old behaviour) makes every firmware lookup miss. YDKJ's FMOD asks for
     * /dev_flash/sys/external/flashMP3.pic and, on a miss, prints "mp3 failed to
     * load MP3 codec (are you using the correct flash?)" and then crashes.
     * Serve it from a real dev_flash tree instead ($PS3_DEV_FLASH, else RPCS3's). */
    if (strncmp(guest, "/dev_flash/", 11) == 0) {
        const char* fw = getenv("PS3_DEV_FLASH");
        if (!fw || !*fw) fw = "D:/recomp/tools/rpcs3/dev_flash";
        snprintf(out, cap, "%s/%s", fw, guest + 11);
        for (char* p = out; *p; p++) if (*p == '\\') *p = '/';
        return;
    }

    static const char* mounts[] = {
        "/dev_bdvd/", "/app_home/", "/dev_hdd0/", "/dev_hdd1/",
        "/dev_flash/", "/host_root/", "/dev_usb000/", "/dev_usb/"
    };
    for (size_t i = 0; i < sizeof(mounts)/sizeof(mounts[0]); i++) {
        size_t n = strlen(mounts[i]);
        if (strncmp(guest, mounts[i], n) == 0) { rel = guest + n; break; }
        /* Every mount above ends in '/', so a BARE device root never matched
         * its own mount: "/dev_bdvd" is not a prefix-match for "/dev_bdvd/".
         * It fell through to the strip-leading-slash case and resolved to
         * <root>/dev_bdvd, which does not exist -- so a title that opendirs or
         * stats the device to check the medium is present reads it as "no
         * disc". YDKJ does exactly that, 27 times a boot, and consequently
         * loaded none of its Scaleform UI or FMOD banks: no logos, no legal
         * screen, just a clear colour.
         *
         * libs/filesystem/cellFs.c already carries this fix (Tokyo Jungle hit
         * the same wall and retried forever); ppu_fs is the other half of the
         * split filesystem and never got it. Map the bare root to the mount
         * directory itself. */
        if (n > 1 && strncmp(guest, mounts[i], n - 1) == 0 && guest[n - 1] == '\0') {
            rel = guest + n - 1;   /* points at the NUL -> <root>/ */
            break;
        }
    }
    if (rel == guest && guest[0] == '/') rel = guest + 1;   /* strip leading '/' */
    snprintf(out, cap, "%s/%s", ppu_vfs_root, rel);
    for (char* p = out; *p; p++) if (*p == '\\') *p = '/';
    ps3_vfs_ps3game_fallback(out, cap);
}

/* ---- fd / dir handle tables ---- */
#define FS_MAX 256
static FILE* g_files[FS_MAX];
static uint8_t g_fd_usm[FS_MAX];
static char    g_fd_path[FS_MAX][192];   /* guest path per fd (diagnostics) */   /* 1 if this fd is an open .usm movie (read tracker) */
static DIR*  g_dirs[FS_MAX];
static char  g_dir_path[FS_MAX][1024];   /* host path per open dir (for readdir stat) */

static int fd_alloc_file(FILE* f)
{
    for (int i = 3; i < FS_MAX; i++) if (!g_files[i] && !g_dirs[i]) { g_files[i] = f; return i; }
    return -1;
}
static int fd_alloc_dir(DIR* d)
{
    for (int i = 3; i < FS_MAX; i++) if (!g_files[i] && !g_dirs[i]) { g_dirs[i] = d; return i; }
    return -1;
}

/* ---- handlers ---- */
static void cellFsOpen(ppu_context* ctx)
{
    char gpath[1024], hpath[1100];
    guest_strcpy(gpath, (uint32_t)ctx->gpr[3], sizeof gpath);
    uint32_t flags  = (uint32_t)ctx->gpr[4];
    uint32_t fd_ptr = (uint32_t)ctx->gpr[5];
    host_path(hpath, sizeof hpath, gpath);

    /* NPDRM: see the note in sys_fs.c -- an EDAT is decrypted once into a cache
     * file and that is opened in its place. */
    { char dec_path[1200];
      const char* use = edat_resolve(hpath, dec_path, sizeof dec_path);
      if (use != hpath) snprintf(hpath, sizeof hpath, "%s", use); }

    /* fopen() mode strings can't express the PS3/POSIX open semantics (e.g.
     * O_WRONLY without create+truncate, or O_CREAT without O_TRUNC), so build
     * real open() flags from the access mode (low 2 bits) + the modifiers, then
     * wrap the fd in a FILE* with fdopen() (which neither creates nor truncates
     * -- that was already decided by open()). */
    int acc = flags & 0x3;
    int oflags = (acc == CELL_FS_O_RDWR)   ? O_RDWR
               : (acc == CELL_FS_O_WRONLY) ? O_WRONLY : O_RDONLY;
    if (flags & CELL_FS_O_CREAT)  oflags |= O_CREAT;
    if (flags & CELL_FS_O_EXCL)   oflags |= O_EXCL;
    if (flags & CELL_FS_O_TRUNC)  oflags |= O_TRUNC;
    if (flags & CELL_FS_O_APPEND) oflags |= O_APPEND;

    /* A bare device root ("/dev_bdvd") resolves to a DIRECTORY, and open() on a
     * directory fails on Windows. Real cellFsOpen answers EISDIR there, and a
     * title reads that as "the medium is mounted" -- YDKJ probes the disc this
     * way before loading its Scaleform UI and FMOD banks, so the ENOENT it got
     * instead read as "no disc" and it drew nothing but a clear colour.
     * S_IFMT/S_IFDIR rather than S_ISDIR: MSVC's CRT does not define the macro. */
    {
        struct stat _dst;
        if (stat(hpath, &_dst) == 0 && (_dst.st_mode & S_IFMT) == S_IFDIR) {
            ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EISDIR;
            return;
        }
    }
    int hfd = open(hpath, oflags | O_BINARY, 0666);
    if (hfd < 0) {
        fprintf(stderr, "[fs] open FAIL '%s' -> '%s'\n", gpath, hpath);
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_ENOENT; return;
    }
    const char* fmode = (acc == CELL_FS_O_RDWR)
                        ? ((flags & CELL_FS_O_APPEND) ? "ab+" : "rb+")
                        : (acc == CELL_FS_O_WRONLY)
                          ? ((flags & CELL_FS_O_APPEND) ? "ab" : "wb")
                          : "rb";
    FILE* f = fdopen(hfd, fmode);
    if (!f) {
        close(hfd);
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO; return;
    }
    int fd = fd_alloc_file(f);
    if (fd < 0) { fclose(f); ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO; return; }
    if (fd_ptr) vm_write32(fd_ptr, (uint32_t)fd);
    g_fd_usm[fd] = (strstr(gpath, ".usm") != nullptr) ? 1 : 0;
    /* FS_CALLER=1: name the guest function that opened each file. A title whose
     * streaming layer opens a file and never reads it (You Don't Know Jack's
     * movies) is diagnosed from the opener: its sibling read path is the one
     * that is not running. */
    if (fd >= 0 && fd < FS_MAX) { strncpy(g_fd_path[fd], gpath, sizeof g_fd_path[fd]-1); g_fd_path[fd][sizeof g_fd_path[fd]-1]=0; }
    { char who[64] = "?";
      if (getenv("FS_CALLER")) { ppu_guest_caller(who, sizeof who);
          fprintf(stderr, "[fs] open '%s' -> fd %d  (opened by %s)\n", gpath, fd, who);
          /* FS_STACK=<substr>: every open funnels through one guest wrapper, so
           * one caller level says nothing about WHICH subsystem wanted the file.
           * Dump the guest call chain for the opens that matter. */
          { const char* want = getenv("FS_STACK");
            if (want && *want && strstr(gpath, want)) ydkj_host_bt("fs-open");
          } }
      else fprintf(stderr, "[fs] open '%s' flags=0x%X -> fd %d\n", gpath, flags, fd); }
    if (getenv("PS3_FSLOG_BT") && strstr(gpath, ".usm")) {
        /* Resolve to GUEST functions (raw host RVAs are useless here): this tells us
         * which criMv/criFs function opened the movie, so the reader-attach path
         * (stream+0x10, never wired => movie opened but never read) can be found. */
        fprintf(stderr,"[USMBT] open '%s' -> fd %d\n", gpath, fd); fflush(stderr);
        ydkj_host_bt("usm-open");
    }
    ctx->gpr[3] = CELL_OK;
}

/* cellFsSdataOpen(path, flags, fd*, arg*, size): opens a PS3 SDATA/EDAT
 * encrypted-data file and returns an fd whose reads yield the DECRYPTED
 * plaintext. LBP 1.30 stores its SPU job code modules inside patch.sdat (an
 * FSHb container, zlib-compressed C0DEC0DE modules) and opens it via this call;
 * leaving it faked (default CELL_OK, no handle) meant the container was never
 * read -> zero job modules loaded -> every SPU physics job dispatched into an
 * empty code buffer -> the loading freeze.
 *
 * Our run env's PS3_HDD0_ROOT (RPCS3 dev_hdd0) already holds patch.sdat in
 * DECRYPTED form (magic "FSHb", not "NPD"), so we just open it read-only --
 * no EDAT decryption needed. (A title supplying a still-encrypted NPD .sdat
 * would need the SDATA block cipher; not required here.) The license `arg`
 * (gpr[6]) and its size (gpr[7]) are ignored. */
static void cellFsSdataOpen(ppu_context* ctx)
{
    char gpath[1024], hpath[1100];
    guest_strcpy(gpath, (uint32_t)ctx->gpr[3], sizeof gpath);
    uint32_t fd_ptr = (uint32_t)ctx->gpr[5];
    host_path(hpath, sizeof hpath, gpath);

    /* Same NPDRM step cellFsOpen does: an encrypted container is decrypted once
     * into a cache file and that is opened in its place. This used to be a
     * second, weaker decryptor local to this function (sdata_decrypt below),
     * which handled the self-keyed SDATA form only and refused every real EDAT
     * -- so a title whose data file was EDAT rather than SDATA got "success
     * without a handle" here while the very same file opened fine through
     * cellFsOpen. One decryptor, reached from both doors. */
    { char dec_path[1200];
      const char* use = edat_resolve(hpath, dec_path, sizeof dec_path);
      if (use != hpath) snprintf(hpath, sizeof hpath, "%s", use); }

    int hfd = open(hpath, O_RDONLY | O_BINARY, 0666);
    if (hfd < 0) {
        fprintf(stderr, "[fs] SdataOpen FAIL '%s' -> '%s'\n", gpath, hpath);
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_ENOENT; return;
    }
    FILE* f = fdopen(hfd, "rb");
    if (!f) { close(hfd); ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO; return; }
    /* If the file IS still NPD-encrypted, we can't serve it plaintext here --
     * warn loudly rather than hand back ciphertext the title will choke on.
     * Use portable fread/fseek (POSIX read()/ssize_t aren't available under the
     * exe's compiler). */
    unsigned char magic[4] = {0};
    size_t got = fread(magic, 1, 4, f);
    fseek(f, 0, SEEK_SET);
    if (got == 4 && magic[0]=='N' && magic[1]=='P' && magic[2]=='D') {
        /* NPD-encrypted SDATA: decrypt it in full and serve the plaintext from
         * an anonymous tmpfile so the title's reads see the real container. */
        fseek(f, 0, SEEK_END); long enc_sz = ftell(f); fseek(f, 0, SEEK_SET);
        uint8_t* enc = (enc_sz > 0) ? (uint8_t*)malloc((size_t)enc_sz) : nullptr;
        size_t rd = enc ? fread(enc, 1, (size_t)enc_sz, f) : 0;
        fclose(f);
        size_t dec_sz = 0;
        uint8_t* dec = (enc && rd == (size_t)enc_sz) ? sdata_decrypt(enc, rd, &dec_sz) : nullptr;
        free(enc);
        if (!dec) {
            fprintf(stderr, "[fs] SdataOpen '%s' NPD decrypt FAILED (unsupported "
                    "EDAT/needs license); returning success without a handle\n", gpath);
            ctx->gpr[3] = CELL_OK; return;   /* don't feed the title ciphertext */
        }
        char dmagic[4] = {0};
        if (dec_sz >= 4) memcpy(dmagic, dec, 4);
        FILE* tf = tmpfile();
        if (!tf || fwrite(dec, 1, dec_sz, tf) != dec_sz) {
            if (tf) fclose(tf); free(dec);
            ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO; return;
        }
        free(dec);
        rewind(tf);
        int fd = fd_alloc_file(tf);
        if (fd < 0) { fclose(tf); ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO; return; }
        if (fd_ptr) vm_write32(fd_ptr, (uint32_t)fd);
        fprintf(stderr, "[fs] SdataOpen '%s' -> fd %d (NPD decrypted, 0x%zX bytes, magic '%c%c%c%c')\n",
                gpath, fd, dec_sz,
                dmagic[0]?dmagic[0]:'?', dmagic[1]?dmagic[1]:'?',
                dmagic[2]?dmagic[2]:'?', dmagic[3]?dmagic[3]:'?');
        ctx->gpr[3] = CELL_OK;
        return;
    }
    int fd = fd_alloc_file(f);
    if (fd < 0) { fclose(f); ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO; return; }
    if (fd_ptr) vm_write32(fd_ptr, (uint32_t)fd);
    fprintf(stderr, "[fs] SdataOpen '%s' -> fd %d (magic %c%c%c%c)\n", gpath, fd,
            magic[0]?magic[0]:'?', magic[1]?magic[1]:'?', magic[2]?magic[2]:'?', magic[3]?magic[3]:'?');
    ctx->gpr[3] = CELL_OK;
}

static void cellFsClose(ppu_context* ctx)
{
    int fd = (int)(uint32_t)ctx->gpr[3];
    if (getenv("FS_CALLER")) { char w[64]="?"; ppu_guest_caller(w,sizeof w);
        fprintf(stderr, "[fs] close fd=%d  (by %s)\n", fd, w); }
    if (fd >= 0 && fd < FS_MAX && g_files[fd]) {
        if (g_fd_usm[fd] && getenv("PS3_FSLOG_READS")) fprintf(stderr, "[USMRD] CLOSE usm fd=%d\n", fd);
        fclose(g_files[fd]); g_files[fd] = nullptr; g_fd_usm[fd] = 0;
    }
    ctx->gpr[3] = CELL_OK;
}

/* Pre-fault a guest range so the kernel can write into it.
 *
 * The flat VM is MEM_RESERVEd and each page is committed on FIRST ACCESS by a
 * vectored exception handler. That covers CPU access from lifted code, but
 * fread/fwrite move data through the kernel, and a kernel write to a reserved
 * page does not raise a user-mode exception -- the I/O just fails, returning 0
 * with ferror set. Twisted Metal hit this on every read whose destination the
 * guest had not touched yet: read 1 of a file succeeded, read 2 into a fresh
 * page came back n=0 eof=0 err=1, and the title logged its own
 * "Short read ... Possible reasons include disc eject" and gave up.
 *
 * Touch each page read-then-write so the fault happens here, in user mode,
 * where the handler can commit it. Read-then-write rather than a plain store so
 * nothing already in the buffer is disturbed. */
static inline void fs_prefault(uint32_t buf, uint64_t len)
{
    if (!vm_base || !len) return;
    volatile uint8_t* p = (volatile uint8_t*)(vm_base + buf);
    for (uint64_t o = 0; o < len; o += 0x1000) p[o] = p[o];
    p[len - 1] = p[len - 1];
}

static void cellFsRead(ppu_context* ctx)
{
    int fd          = (int)(uint32_t)ctx->gpr[3];
    uint32_t buf    = (uint32_t)ctx->gpr[4];
    uint64_t nbytes = ctx->gpr[5];
    uint32_t nread_ptr = (uint32_t)ctx->gpr[6];
    if (fd < 0 || fd >= FS_MAX || !g_files[fd]) { ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO; return; }
    uint64_t raw_nbytes = ctx->gpr[5];
    long fpos_before = ftell(g_files[fd]);
    if (ppu_vm_size && (uint64_t)buf + nbytes > ppu_vm_size) nbytes = ppu_vm_size - buf;
    fs_prefault(buf, nbytes);
    size_t n = fread(vm_base + buf, 1, (size_t)nbytes, g_files[fd]);   /* raw bytes, no swap */
    /* PS3_FSLOG_EOF=<fd>,<bytes>: report end-of-file past <bytes> on one descriptor.
     * The intro cinematic is 28 seconds and this port renders it at a few frames
     * a second, so it cannot be watched to its end -- truncating the stream makes
     * the demuxer see EOF and the movie finish, which is what advances the title
     * to whatever follows the intro. Deliberately a testing knob, not a fix. */
    { static int efd = -2; static long elim = 0;
      if (efd == -2) { const char* e = getenv("PS3_FSLOG_EOF");
          if (e) { efd = atoi(e); const char* c = strchr(e, 44); elim = c ? atol(c + 1) : 0; }
          else efd = -1; }
      if (efd >= 0 && fd == efd && elim > 0 && fpos_before >= elim) {
          static int once = 0;
          if (!once++) fprintf(stderr, "[fs] PS3_FSLOG_EOF: fd=%d truncated at %ld bytes\n", fd, elim);
          n = 0;
      } }
    if (g_fd_usm[fd] && getenv("PS3_FSLOG_READS")) fprintf(stderr, "[USMRD] READ usm fd=%d nbytes=%llu -> %zu magic=%02X%02X%02X%02X pos=%ld lr=0x%08X\n", fd, (unsigned long long)nbytes, n, vm_base[buf], vm_base[buf+1], vm_base[buf+2], vm_base[buf+3], fpos_before, (uint32_t)ctx->lr);
    /* PS3_FSLOG_READS=<fd>: log every read on one descriptor. PS3_FSLOG caps at 20
     * lines and they are all spent before a movie ever opens, so it cannot
     * answer "is the streamer reading the .avi". */
    { static int wfd = -2;
      if (wfd == -2) { const char* e = getenv("PS3_FSLOG_READS"); wfd = e ? atoi(e) : -1; }
      if (wfd >= 0 && fd == wfd)
          fprintf(stderr, "[fsread] fd=%d want=%llu got=%zu pos=%ld\n",
                  fd, (unsigned long long)nbytes, n, fpos_before); }
    if (getenv("PS3_FSLOG")) { static int _fd=0; if(_fd++<20) fprintf(stderr,"[FSDBG] fd=%d raw_nbytes=0x%llX clamped=0x%llX buf=0x%08X fpos_before=%ld n=%zu eof=%d err=%d\n", fd,(unsigned long long)raw_nbytes,(unsigned long long)nbytes,buf,fpos_before,n,feof(g_files[fd]),ferror(g_files[fd])); }
#ifdef _WIN32
    if (getenv("PS3_FSLOG") && buf==0 && raw_nbytes>0x10000) { static int _b=0; if(_b++<2){ void* fr[30]; unsigned short nn=RtlCaptureStackBackTrace(0,30,fr,0); uintptr_t mb=(uintptr_t)GetModuleHandleA(0); fprintf(stderr,"[FSBT] null-buf read caller rvas:"); for(unsigned short i=0;i<nn&&i<16;i++) fprintf(stderr," %llX",(unsigned long long)((uintptr_t)fr[i]-mb)); fprintf(stderr,"\n"); } }
#endif
    /* Per-fd totals, not just the first 50 lines. The flat cap made "this file is
     * opened and never read" unfalsifiable: reads on a later-opened fd fall off
     * the end of the log and look identical to reads that never happen.
     * FS_READ_ALL=1 logs every read; otherwise a per-fd first-read line plus a
     * periodic summary is enough to tell the two apart. */
    { static uint64_t tot=0; static int _n=0;
      static uint64_t per_fd[64]; static uint32_t cnt_fd[64];
      tot+=n;
      if (fd>=0 && fd<64) { per_fd[fd]+=n; cnt_fd[fd]++; }
      int first_for_fd = (fd>=0 && fd<64 && cnt_fd[fd]==1);
      if(_n++<50 || first_for_fd || getenv("FS_READ_ALL"))
        fprintf(stderr,"[fs] read fd=%d nbytes=%llu -> %zu (magic=%02X%02X%02X%02X, total=%llu)%s\n",
                fd,(unsigned long long)nbytes,n,vm_base[buf],vm_base[buf+1],vm_base[buf+2],vm_base[buf+3],
                (unsigned long long)tot, first_for_fd?"  <= FIRST READ ON THIS FD":"");
      if (getenv("FS_READ_PATH") && fd>=0 && fd<64 && g_fd_path[fd][0])
          fprintf(stderr, "        from %s\n", g_fd_path[fd]);
      if ((_n % 2000)==0) { fprintf(stderr,"[fs] read summary after %d reads:",_n);
          for (int i=0;i<64;i++) if (cnt_fd[i]) fprintf(stderr," fd%d=%ux/%lluB",i,cnt_fd[i],(unsigned long long)per_fd[i]);
          fprintf(stderr,"\n"); } }
    if (getenv("PS3_FSLOG_TOC") && nbytes >= 50000) {  /* data.toc read -> who parses it? */
        fprintf(stderr, "[TOC] data.toc read into buf=0x%08X n=%zu; lr=0x%08llX; guest-stack RAs:\n", buf, n, (unsigned long long)ctx->lr);
        uint32_t sp = (uint32_t)ctx->gpr[1];
        for (uint32_t i = 0; i < 128 && sp + i*4 + 4 <= ppu_vm_size; i++) {
            uint32_t a = sp + i*4; uint32_t w = (vm_base[a]<<24)|(vm_base[a+1]<<16)|(vm_base[a+2]<<8)|vm_base[a+3];
            if (w >= 0x10000u && w < 0x600000u) fprintf(stderr, "[TOC]   ra 0x%08X (@sp+0x%X)\n", w, i*4);
        }
    }
    if (nread_ptr) vm_write64(nread_ptr, n);
    ctx->gpr[3] = CELL_OK;
}

static int host_fileno_of(FILE* f)
{
#ifdef _WIN32
    return _fileno(f);
#else
    return fileno(f);
#endif
}

static int64_t cell_fs_from_errno(void);

#ifdef _WIN32
/* Grow/shrink via SetEndOfFile. Mark sparse before a grow so a 1.75 GiB
 * overlay does not zero-fill; ERROR_DISK_FULL -> ENOSPC. */
static int host_set_end_of_file(HANDLE h, uint64_t size)
{
    LARGE_INTEGER cur;
    if (!GetFileSizeEx(h, &cur)) {
        errno = EIO;
        return -1;
    }
    if ((uint64_t)cur.QuadPart < size) {
        DWORD ign = 0;
        DeviceIoControl(h, FSCTL_SET_SPARSE, NULL, 0, NULL, 0, &ign, NULL);
    }
    LARGE_INTEGER li;
    li.QuadPart = (LONGLONG)size;
    if (!SetFilePointerEx(h, li, NULL, FILE_BEGIN) || !SetEndOfFile(h)) {
        DWORD err = GetLastError();
        errno = (err == ERROR_DISK_FULL || err == ERROR_HANDLE_DISK_FULL) ? ENOSPC : EIO;
        return -1;
    }
    return 0;
}
#endif

static int host_ftruncate_size(FILE* f, uint64_t size)
{
    fflush(f);
    int hfd = host_fileno_of(f);
    if (hfd < 0)
        return -1;
#ifdef _WIN32
    HANDLE h = (HANDLE)_get_osfhandle(hfd);
    if (h == INVALID_HANDLE_VALUE || !h)
        return -1;
    int64_t saved = (int64_t)HOST_FTELL64(f);
    int r = host_set_end_of_file(h, size);
    HOST_FSEEK64(f, saved, SEEK_SET);
    return r;
#else
    return ftruncate(hfd, (off_t)size);
#endif
}

/* cellFsAllocateFileAreaWithoutZeroFill: extend to `size` without shrinking
 * and without writing zeros. CELL_ENOENT if the path is missing. */
static int64_t host_allocate_path(const char* hpath, uint64_t size)
{
#ifdef _WIN32
    struct __stat64 st;
    if (_stat64(hpath, &st) != 0)
        return CELL_FS_ENOENT;
#else
    struct stat st;
    if (stat(hpath, &st) != 0)
        return CELL_FS_ENOENT;
#endif
    uint64_t cur = (uint64_t)st.st_size;
    if (cur >= size)
        return CELL_OK;
    uint64_t need = size - cur;
    uint64_t freeb = cellfs_host_free_bytes(hpath);
    if (freeb > 0 && freeb < need)
        return CELL_FS_ENOSPC;
#ifdef _WIN32
    HANDLE h = CreateFileA(hpath, GENERIC_READ | GENERIC_WRITE,
                           FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                           NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (h == INVALID_HANDLE_VALUE) {
        DWORD e = GetLastError();
        if (e == ERROR_FILE_NOT_FOUND || e == ERROR_PATH_NOT_FOUND)
            return CELL_FS_ENOENT;
        if (e == ERROR_DISK_FULL || e == ERROR_HANDLE_DISK_FULL)
            return CELL_FS_ENOSPC;
        return CELL_FS_EIO;
    }
    int r = host_set_end_of_file(h, size);
    CloseHandle(h);
    return r == 0 ? CELL_OK : cell_fs_from_errno();
#else
    if (truncate(hpath, (off_t)size) != 0)
        return cell_fs_from_errno();
    return CELL_OK;
#endif
}

static int64_t host_allocate_fp(FILE* f, uint64_t size)
{
    fflush(f);
#ifdef _WIN32
    int hfd = host_fileno_of(f);
    if (hfd < 0)
        return CELL_FS_EIO;
    HANDLE h = (HANDLE)_get_osfhandle(hfd);
    if (h == INVALID_HANDLE_VALUE || !h)
        return CELL_FS_EIO;
    LARGE_INTEGER cur;
    if (!GetFileSizeEx(h, &cur))
        return CELL_FS_EIO;
    if ((uint64_t)cur.QuadPart >= size)
        return CELL_OK;
    int64_t saved = (int64_t)HOST_FTELL64(f);
    int r = host_set_end_of_file(h, size);
    HOST_FSEEK64(f, saved, SEEK_SET);
    return r == 0 ? CELL_OK : cell_fs_from_errno();
#else
    int hfd = host_fileno_of(f);
    if (hfd < 0)
        return CELL_FS_EIO;
    struct stat st;
    if (fstat(hfd, &st) != 0)
        return CELL_FS_EIO;
    if ((uint64_t)st.st_size >= size)
        return CELL_OK;
    if (ftruncate(hfd, (off_t)size) != 0)
        return cell_fs_from_errno();
    return CELL_OK;
#endif
}

static int64_t host_enospc_if_grow(const char* hpath, uint64_t cur, uint64_t size)
{
    if (cur >= size)
        return CELL_OK;
    uint64_t freeb = cellfs_host_free_bytes(hpath);
    if (freeb > 0 && freeb < size - cur)
        return CELL_FS_ENOSPC;
    return CELL_OK;
}

/* cellFsChangeFileSizeWithoutAllocation: set exact size (grow sparse or shrink).
 * create=0 is ENOENT if missing; create=1 is OPEN_ALWAYS for WithInitialData. */
static int64_t host_set_size_path(const char* hpath, uint64_t size, int create)
{
#ifdef _WIN32
    struct __stat64 st;
    int exists = (_stat64(hpath, &st) == 0);
    if (!exists && !create)
        return CELL_FS_ENOENT;
    uint64_t cur = exists ? (uint64_t)st.st_size : 0;
    int64_t spc = host_enospc_if_grow(hpath, cur, size);
    if (spc != CELL_OK)
        return spc;
    HANDLE h = CreateFileA(hpath, GENERIC_READ | GENERIC_WRITE,
                           FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                           NULL, create ? OPEN_ALWAYS : OPEN_EXISTING,
                           FILE_ATTRIBUTE_NORMAL, NULL);
    if (h == INVALID_HANDLE_VALUE) {
        DWORD e = GetLastError();
        if (e == ERROR_FILE_NOT_FOUND || e == ERROR_PATH_NOT_FOUND)
            return CELL_FS_ENOENT;
        if (e == ERROR_DISK_FULL || e == ERROR_HANDLE_DISK_FULL)
            return CELL_FS_ENOSPC;
        return CELL_FS_EIO;
    }
    int r = host_set_end_of_file(h, size);
    CloseHandle(h);
    return r == 0 ? CELL_OK : cell_fs_from_errno();
#else
    struct stat st;
    int exists = (stat(hpath, &st) == 0);
    if (!exists && !create)
        return CELL_FS_ENOENT;
    uint64_t cur = exists ? (uint64_t)st.st_size : 0;
    if (!exists) {
        int fd = open(hpath, O_RDWR | O_CREAT, 0666);
        if (fd < 0)
            return cell_fs_from_errno();
        close(fd);
        cur = 0;
    }
    int64_t spc = host_enospc_if_grow(hpath, cur, size);
    if (spc != CELL_OK)
        return spc;
    if (truncate(hpath, (off_t)size) != 0)
        return cell_fs_from_errno();
    return CELL_OK;
#endif
}

static int64_t host_write_at_path(const char* hpath, uint32_t buf_ea, uint64_t off, uint64_t len)
{
    if (!len)
        return CELL_OK;
    if (!buf_ea)
        return CELL_FS_EFAULT;
    if (ppu_vm_size && (uint64_t)buf_ea + len > ppu_vm_size)
        return CELL_FS_EFAULT;
    fs_prefault(buf_ea, len);
    FILE* f = fopen(hpath, "rb+");
    if (!f)
        return CELL_FS_EIO;
    if (HOST_FSEEK64(f, (int64_t)off, SEEK_SET) != 0) {
        fclose(f);
        return CELL_FS_EIO;
    }
    size_t n = fwrite(vm_base + buf_ea, 1, (size_t)len, f);
    fflush(f);
    fclose(f);
    if (n < (size_t)len)
        return cell_fs_from_errno();
    return CELL_OK;
}

static int64_t host_write_at_fp(FILE* f, uint32_t buf_ea, uint64_t off, uint64_t len)
{
    if (!len)
        return CELL_OK;
    if (!buf_ea)
        return CELL_FS_EFAULT;
    if (ppu_vm_size && (uint64_t)buf_ea + len > ppu_vm_size)
        return CELL_FS_EFAULT;
    fs_prefault(buf_ea, len);
    int64_t saved = (int64_t)HOST_FTELL64(f);
    if (HOST_FSEEK64(f, (int64_t)off, SEEK_SET) != 0)
        return CELL_FS_EIO;
    size_t n = fwrite(vm_base + buf_ea, 1, (size_t)len, f);
    fflush(f);
    HOST_FSEEK64(f, saved, SEEK_SET);
    if (n < (size_t)len)
        return cell_fs_from_errno();
    return CELL_OK;
}

/* POSIX rename replaces an existing dest; Win32 rename() does not. */
static int host_rename_replace(const char* from, const char* to)
{
    if (rename(from, to) == 0)
        return 0;
#ifdef _WIN32
    if (remove(to) == 0 || errno == ENOENT)
        return rename(from, to);
#endif
    return -1;
}

static int64_t cell_fs_from_errno(void)
{
    if (errno == ENOSPC)
        return CELL_FS_ENOSPC;
#ifdef EDQUOT
    if (errno == EDQUOT)
        return CELL_FS_ENOSPC;
#endif
    return CELL_FS_EIO;
}

static void cellFsWrite(ppu_context* ctx)
{
    int fd          = (int)(uint32_t)ctx->gpr[3];
    uint32_t buf    = (uint32_t)ctx->gpr[4];
    uint64_t nbytes = ctx->gpr[5];
    uint32_t nwr_ptr = (uint32_t)ctx->gpr[6];
    if (fd < 0 || fd >= FS_MAX || !g_files[fd]) { ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO; return; }
    fs_prefault(buf, nbytes);   /* kernel READS the buffer; same reserved-page trap */
    size_t n = 0;
    while (n < (size_t)nbytes) {
        size_t chunk = fwrite(vm_base + buf + n, 1, (size_t)nbytes - n, g_files[fd]);
        if (chunk == 0)
            break;
        n += chunk;
    }
    /* POSIX/Win CRT: a later lseek/read on the same FILE* needs the write flushed. */
    fflush(g_files[fd]);
    {
        static int nlog = 0;
        const char* gp = (fd >= 0 && fd < FS_MAX) ? g_fd_path[fd] : "";
        int cache = gp && strstr(gp, "cache.");
        if (nlog < 16 || (cache && nlog < 32))
            fprintf(stderr, "[fs] write fd=%d '%s' nbytes=%llu -> %zu%s\n",
                    fd, gp && *gp ? gp : "?", (unsigned long long)nbytes, n,
                    (n < (size_t)nbytes) ? "  (SHORT)" : "");
        nlog++;
    }
    /* DIAGNOSTIC (PS3_FSLOG_BT=1): dump the guest back-chain when the game logs the
     * render-config failure, to locate setScreenRenderTargetInternal & the config obj. */
    if (getenv("PS3_FSLOG_BT") && buf && nbytes > 0 && nbytes < 4096 && vm_base) {
        char tmp[256]; uint32_t nn = (uint32_t)(nbytes < 255 ? nbytes : 255);
        memcpy(tmp, vm_base + buf, nn); tmp[nn] = 0;
        if (strstr(tmp,"config") || strstr(tmp,"Config") || strstr(tmp,"Mystery") ||
            strstr(tmp,"downsample") || strstr(tmp,"RenderTarget")) {
            uint32_t sp = (uint32_t)ctx->gpr[1];
            fprintf(stderr, "[cfgbt] write \"%.60s\" lr=0x%08X sp=0x%08X\n", tmp, (uint32_t)ctx->lr, sp);
            for (int i = 0; i < 24 && sp && sp < 0x10000000u; i++) {
                uint32_t nsp; memcpy(&nsp, vm_base + sp, 4);
                nsp = ((nsp>>24)&0xFF)|((nsp>>8)&0xFF00)|((nsp<<8)&0xFF0000)|((nsp<<24)&0xFF000000);
                if (nsp <= sp || nsp >= 0x10000000u) break;
                uint32_t lr; memcpy(&lr, vm_base + nsp + 0x10, 4);
                lr = ((lr>>24)&0xFF)|((lr>>8)&0xFF00)|((lr<<8)&0xFF0000)|((lr<<24)&0xFF000000);
                fprintf(stderr, "[cfgbt]   #%d lr=0x%08X\n", i, lr);
                sp = nsp;
            }
            fflush(stderr);
        }
    }
    if (nwr_ptr) vm_write64(nwr_ptr, n);
    if (n < (size_t)nbytes) {
        ctx->gpr[3] = (uint64_t)(int64_t)cell_fs_from_errno();
        return;
    }
    ctx->gpr[3] = CELL_OK;
}

static void cellFsLseek(ppu_context* ctx)
{
    int fd        = (int)(uint32_t)ctx->gpr[3];
    int64_t off   = (int64_t)ctx->gpr[4];
    uint32_t wh   = (uint32_t)ctx->gpr[5];
    uint32_t pos_ptr = (uint32_t)ctx->gpr[6];
    if (fd < 0 || fd >= FS_MAX || !g_files[fd]) { ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO; return; }
    if (getenv("FS_CALLER")) { char w[64]="?"; ppu_guest_caller(w,sizeof w);
        fprintf(stderr, "[fs] lseek fd=%d off=%lld whence=%u  (by %s)\n", fd, (long long)off, wh, w); }
    int worigin = (wh == CELL_FS_SEEK_END) ? SEEK_END : (wh == CELL_FS_SEEK_CUR) ? SEEK_CUR : SEEK_SET;
    if (HOST_FSEEK64(g_files[fd], off, worigin) != 0) {
        const char* gp = g_fd_path[fd];
        fprintf(stderr, "[fs] lseek FAIL fd=%d '%s' off=%lld whence=%u errno=%d\n",
                fd, gp[0] ? gp : "?", (long long)off, wh, errno);
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO;
        return;
    }
    int64_t p = (int64_t)HOST_FTELL64(g_files[fd]);
    {
        static int nlog = 0;
        const char* gp = g_fd_path[fd];
        int cache = gp[0] && strstr(gp, "cache.");
        int large = (off >= (1ll << 20) || off <= -(1ll << 20) || p >= (1ll << 20));
        if ((cache && (nlog < 16 || large)) || getenv("FS_CALLER"))
            fprintf(stderr, "[fs] lseek fd=%d '%s' off=%lld whence=%u -> %lld\n",
                    fd, gp[0] ? gp : "?", (long long)off, wh, (long long)p);
        if (cache) nlog++;
    }
    if (pos_ptr) vm_write64(pos_ptr, (uint64_t)p);
    ctx->gpr[3] = CELL_OK;
}

/* CellFsStat is 0x34 (52) bytes, 4-byte aligned -- the s64/u64 members are
 * be_t<...,4> so there is NO 4-byte pad after gid (verified vs RPCS3:
 * CHECK_SIZE_ALIGN(CellFsStat, 52, 4)). Laying it out 8-byte-aligned (0x38,
 * pad@0x0C) shifts size/blksize +4 and overruns the struct by 4 bytes -- games
 * that embed a CellFsStat inside a larger object (e.g. Dantelion's
 * DLFileDeviceStream, stat@obj+0xD8) then have the trailing blksize clobber the
 * field right after the stat (the fd at obj+0x10c), which later fails lseek.
 * Layout: mode@0 uid@4 gid@8 atime@0x0C mtime@0x14 ctime@0x1C size@0x24 blksize@0x2C. */
static void write_stat(uint32_t sb, uint32_t mode, uint64_t size)
{
    vm_write32(sb + 0x00, mode);
    vm_write32(sb + 0x04, 0);            /* uid */
    vm_write32(sb + 0x08, 0);            /* gid */
    vm_write64(sb + 0x0C, 0);            /* atime */
    vm_write64(sb + 0x14, 0);            /* mtime */
    vm_write64(sb + 0x1C, 0);            /* ctime */
    vm_write64(sb + 0x24, size);         /* size */
    /* Match GetFreeSize / cellFs.c / sys_fs.c. 0x200 (512) made FIOS treat an
     * existing overlay as 512-byte media and scan cache.idx one sector per
     * open+close (~8k ops / 40s) instead of 4 KiB / 64 KiB like first format. */
    vm_write64(sb + 0x2C, 4096);         /* blksize */
}

#define FS_STAT_RING 8
static struct {
    char path[96];
    int32_t rc;
} s_stat_ring[FS_STAT_RING];
static int s_stat_ring_n = 0;
static void note_stat(const char* p, int32_t rc)
{
    int i = s_stat_ring_n % FS_STAT_RING;
    strncpy(s_stat_ring[i].path, p ? p : "", sizeof s_stat_ring[i].path - 1);
    s_stat_ring[i].path[sizeof s_stat_ring[i].path - 1] = 0;
    s_stat_ring[i].rc = rc;
    s_stat_ring_n++;
}
extern "C" void ppu_fs_dump_last_stat(void)
{
    int n = s_stat_ring_n;
    int start = n > FS_STAT_RING ? n - FS_STAT_RING : 0;
    int k;
    fprintf(stderr, "[ice-fstat] n=%d\n", n);
    for (k = start; k < n; k++) {
        int i = k % FS_STAT_RING;
        fprintf(stderr, "[ice-fstat] [%d] rc=0x%08X '%s'\n",
                k, (uint32_t)s_stat_ring[i].rc, s_stat_ring[i].path);
    }
    fflush(stderr);
}

static void cellFsStat(ppu_context* ctx)
{
    char gpath[1024], hpath[1100];
    guest_strcpy(gpath, (uint32_t)ctx->gpr[3], sizeof gpath);
    uint32_t sb = (uint32_t)ctx->gpr[4];
    host_path(hpath, sizeof hpath, gpath);
    struct stat st;
    uint32_t lr = (uint32_t)ctx->lr;
    int leave_stat = (lr == 0x004C55F4u || lr == 0x004C5630u || lr == 0x004C5728u ||
                      lr == 0x004C5740u);
    if (leave_stat) {
        fprintf(stderr, "[leave-fe] FsStat lr=0x%08X path='%s' host='%s'\n",
                lr, gpath, hpath);
        fflush(stderr);
    }
    if (stat(hpath, &st) != 0) {
        /* Leave-FE note: do NOT invent an hdd0 USRDIR/packed dir or Stat-OK
         * overlay here. C5558 treats packed-present + savedata-missing as
         * inconsistent install and returns 1 → 004C3B48(1) → msg 0xEE
         * ("Game data has become corrupted"). Real leave needs C5558==0 so
         * 004C8898 falls through 004C8D70 → 004C3B48(0) → 00937A90(0x8CB). */
        if (getenv("PS3_FSLOG")) fprintf(stderr, "[fs] stat '%s' -> ENOENT\n", gpath);
        note_stat(gpath, (int32_t)CELL_FS_ENOENT);
        if (leave_stat) {
            fprintf(stderr, "[leave-fe] FsStat FAIL lr=0x%08X path='%s' -> ENOENT\n",
                    lr, gpath);
            fflush(stderr);
        }
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_ENOENT; return;
    }
    if (getenv("PS3_FSLOG")) fprintf(stderr, "[fs] stat '%s' -> OK (size=%lld)\n", gpath, (long long)st.st_size);
    note_stat(gpath, CELL_OK);
    if (leave_stat) {
        fprintf(stderr, "[leave-fe] FsStat OK lr=0x%08X path='%s' size=%lld\n",
                lr, gpath, (long long)st.st_size);
        fflush(stderr);
    }
    uint32_t mode = (st.st_mode & S_IFDIR) ? (CELL_FS_S_IFDIR | 0x1FF)
                                           : (CELL_FS_S_IFREG | 0x1B6);
    if (sb) write_stat(sb, mode, (uint64_t)st.st_size);
    {
        static int nstat = 0;
        if (nstat < 16 && (strstr(gpath, "cache.") || getenv("PS3_FSLOG"))) {
            fprintf(stderr, "[fs] stat '%s' -> size=%lld blksize=4096\n",
                    gpath, (long long)st.st_size);
            nstat++;
        }
    }
    ctx->gpr[3] = CELL_OK;
}

static void cellFsFstat(ppu_context* ctx)
{
    int fd      = (int)(uint32_t)ctx->gpr[3];
    uint32_t sb = (uint32_t)ctx->gpr[4];
    if (fd < 0 || fd >= FS_MAX || !g_files[fd]) { ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO; return; }
    if (getenv("FS_CALLER")) { char w[64]="?"; ppu_guest_caller(w,sizeof w);
        fprintf(stderr, "[fs] fstat fd=%d  (by %s)\n", fd, w); }
    fflush(g_files[fd]);
    int64_t sz = 0;
#ifdef _WIN32
    {
        int hfd = _fileno(g_files[fd]);
        struct __stat64 st;
        if (hfd >= 0 && _fstat64(hfd, &st) == 0)
            sz = (int64_t)st.st_size;
        else {
            int64_t cur = (int64_t)HOST_FTELL64(g_files[fd]);
            HOST_FSEEK64(g_files[fd], 0, SEEK_END);
            sz = (int64_t)HOST_FTELL64(g_files[fd]);
            HOST_FSEEK64(g_files[fd], cur, SEEK_SET);
        }
    }
#else
    {
        int hfd = fileno(g_files[fd]);
        struct stat st;
        if (hfd >= 0 && fstat(hfd, &st) == 0)
            sz = (int64_t)st.st_size;
        else {
            int64_t cur = (int64_t)HOST_FTELL64(g_files[fd]);
            HOST_FSEEK64(g_files[fd], 0, SEEK_END);
            sz = (int64_t)HOST_FTELL64(g_files[fd]);
            HOST_FSEEK64(g_files[fd], cur, SEEK_SET);
        }
    }
#endif
    /* FS_FSTAT_CAP=<bytes>: DIAGNOSTIC. Report a smaller size than the file
     * has. You Don't Know Jack reads a 100 KB archive whole but only probes
     * (open/fstat/close) its 500 KB and 2.4 MB ones; if that branch is driven
     * by the size it just asked for, capping it makes the title take the read
     * path. Answers whether the split is size-based -- nothing more. */
    { const char* cap = getenv("FS_FSTAT_CAP");
      const char* only = getenv("FS_FSTAT_CAP_PATH");
      if (cap && only && *only && !(g_fd_path[fd][0] && strstr(g_fd_path[fd], only))) cap = 0;
      if (cap) { int64_t c = strtoll(cap, 0, 0);
          if (c > 0 && sz > c) {
              fprintf(stderr, "[fs] fstat fd=%d size %lld -> capped %lld (FS_FSTAT_CAP)\n",
                      fd, (long long)sz, (long long)c);
              sz = c; } } }
    if (sb) write_stat(sb, CELL_FS_S_IFREG | 0x1B6, (uint64_t)sz);
    {
        static int nlog = 0;
        const char* gp = (fd >= 0 && fd < FS_MAX) ? g_fd_path[fd] : "";
        int cache = gp && strstr(gp, "cache.");
        if (nlog < 12 || (cache && nlog < 24))
            fprintf(stderr, "[fs] fstat fd=%d '%s' size=%lld blksize=4096\n",
                    fd, gp && *gp ? gp : "?", (long long)sz);
        nlog++;
    }
    if (getenv("PS3_FSLOG")) { static int _n=0; if(_n++<12) fprintf(stderr,"[FSDBG] cellFsFstat(fd=%d) -> size=0x%llX\n",fd,(unsigned long long)sz); }
    ctx->gpr[3] = CELL_OK;
}

static void cellFsOpendir(ppu_context* ctx)
{
    char gpath[1024], hpath[1100];
    guest_strcpy(gpath, (uint32_t)ctx->gpr[3], sizeof gpath);
    uint32_t fd_ptr = (uint32_t)ctx->gpr[4];
    host_path(hpath, sizeof hpath, gpath);
    DIR* d = opendir(hpath);
    if (getenv("PS3_FSLOG")) fprintf(stderr, "[fs] opendir '%s' -> %s\n", gpath, d ? "OK" : "ENOENT");
    if (!d) { ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_ENOENT; return; }
    int fd = fd_alloc_dir(d);
    if (fd < 0) { closedir(d); ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO; return; }
    strncpy(g_dir_path[fd], hpath, sizeof g_dir_path[fd] - 1);
    if (fd_ptr) vm_write32(fd_ptr, (uint32_t)fd);
    ctx->gpr[3] = CELL_OK;
}

/* CellFsDirent: d_type(1) d_namlen(1) d_name[256]; total 0x102. */
static void cellFsReaddir(ppu_context* ctx)
{
    int fd          = (int)(uint32_t)ctx->gpr[3];
    uint32_t dirent = (uint32_t)ctx->gpr[4];
    uint32_t nread_ptr = (uint32_t)ctx->gpr[5];
    if (fd < 0 || fd >= FS_MAX || !g_dirs[fd]) { ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO; return; }
    struct dirent* e = readdir(g_dirs[fd]);
    if (!e) { if (nread_ptr) vm_write64(nread_ptr, 0); ctx->gpr[3] = CELL_OK; return; }
    char full[1300]; struct stat st;
    snprintf(full, sizeof full, "%s/%s", g_dir_path[fd], e->d_name);
    uint8_t type = (stat(full, &st) == 0 && (st.st_mode & S_IFDIR))
                   ? CELL_FS_TYPE_DIR : CELL_FS_TYPE_REG;
    size_t nl = strlen(e->d_name); if (nl > 255) nl = 255;
    vm_base[dirent + 0] = type;
    vm_base[dirent + 1] = (uint8_t)nl;
    for (size_t i = 0; i < nl; i++) vm_base[dirent + 2 + i] = (uint8_t)e->d_name[i];
    vm_base[dirent + 2 + nl] = 0;
    if (nread_ptr) vm_write64(nread_ptr, 0x102);
    ctx->gpr[3] = CELL_OK;
}

static void cellFsClosedir(ppu_context* ctx)
{
    int fd = (int)(uint32_t)ctx->gpr[3];
    if (fd >= 0 && fd < FS_MAX && g_dirs[fd]) { closedir(g_dirs[fd]); g_dirs[fd] = nullptr; }
    ctx->gpr[3] = CELL_OK;
}

/* Create `path` and any missing parent dirs on the host. Returns 0 on
 * success or if the directory already exists. */
static int host_mkdir_p(const char* path)
{
    char tmp[1100];
    snprintf(tmp, sizeof tmp, "%s", path);
    size_t len = strlen(tmp);
    while (len > 1 && (tmp[len-1] == '/' || tmp[len-1] == '\\')) tmp[--len] = 0;
    for (char* p = tmp + 1; *p; p++) {
        if (*p == '/' || *p == '\\') { char c = *p; *p = 0; HOST_MKDIR(tmp); *p = c; }
    }
    int r = HOST_MKDIR(tmp);
    if (r != 0) { struct stat st; if (stat(tmp, &st) == 0 && (st.st_mode & S_IFDIR)) r = 0; }
    return r;
}

/* cellFsMkdir(path, mode): create the guest directory on the host. A no-op stub
 * here silently breaks games that create then poll a dir (e.g. LBP's boot waits
 * for /dev_hdd0/game/<title>/USRDIR to appear). Create parents too. */
static void cellFsMkdir(ppu_context* ctx)
{
    char gpath[1024], hpath[1100];
    guest_strcpy(gpath, (uint32_t)ctx->gpr[3], sizeof gpath);
    host_path(hpath, sizeof hpath, gpath);
    int r = host_mkdir_p(hpath);
    if (getenv("PS3_FSLOG")) fprintf(stderr, "[fs] mkdir '%s' -> '%s' (%s)\n", gpath, hpath, r == 0 ? "OK" : "FAIL");
    ctx->gpr[3] = CELL_OK;   /* existing dir is benign for boot */
}
static void cellFsRmdir(ppu_context* ctx)
{
    char gpath[1024], hpath[1100];
    guest_strcpy(gpath, (uint32_t)ctx->gpr[3], sizeof gpath);
    host_path(hpath, sizeof hpath, gpath);
#ifdef _WIN32
    int r = _rmdir(hpath);
#else
    int r = rmdir(hpath);
#endif
    if (getenv("PS3_FSLOG"))
        fprintf(stderr, "[fs] rmdir '%s' -> '%s' (%s)\n", gpath, hpath, r == 0 ? "OK" : strerror(errno));
    ctx->gpr[3] = CELL_OK;
}
static void cellFsUnlink(ppu_context* ctx)
{
    char gpath[1024], hpath[1100];
    guest_strcpy(gpath, (uint32_t)ctx->gpr[3], sizeof gpath);
    host_path(hpath, sizeof hpath, gpath);
    int r = remove(hpath);
    fprintf(stderr, "[fs] unlink '%s' -> '%s' (%s)\n", gpath, hpath, r == 0 ? "OK" : strerror(errno));
    /* Missing file is success: FIOS may unlink before recreating overlay files. */
    ctx->gpr[3] = CELL_OK;
}
static void cellFsRename(ppu_context* ctx)
{
    char from[1024], to[1024], hfrom[1100], hto[1100];
    guest_strcpy(from, (uint32_t)ctx->gpr[3], sizeof from);
    guest_strcpy(to,   (uint32_t)ctx->gpr[4], sizeof to);
    host_path(hfrom, sizeof hfrom, from);
    host_path(hto,   sizeof hto,   to);
    int r = host_rename_replace(hfrom, hto);
    fprintf(stderr, "[fs] rename '%s' -> '%s' (%s)\n", from, to, r == 0 ? "OK" : strerror(errno));
    ctx->gpr[3] = (r == 0) ? CELL_OK : (uint64_t)(int64_t)CELL_FS_ENOENT;
}
static void cellFsFsync(ppu_context* ctx)
{
    int fd = (int)(uint32_t)ctx->gpr[3];
    if (fd >= 0 && fd < FS_MAX && g_files[fd]) {
        fflush(g_files[fd]);
#ifdef _WIN32
        int hfd = host_fileno_of(g_files[fd]);
        if (hfd >= 0) _commit(hfd);
#else
        int hfd = host_fileno_of(g_files[fd]);
        if (hfd >= 0) fsync(hfd);
#endif
    }
    ctx->gpr[3] = CELL_OK;
}
static void cellFsTruncate(ppu_context* ctx)
{
    char gpath[1024], hpath[1100];
    guest_strcpy(gpath, (uint32_t)ctx->gpr[3], sizeof gpath);
    uint64_t size = ctx->gpr[4];
    host_path(hpath, sizeof hpath, gpath);
#ifdef _WIN32
    FILE* f = fopen(hpath, "rb+");
    int r = -1;
    if (f) {
        r = host_ftruncate_size(f, size);
        fclose(f);
    }
#else
    int r = truncate(hpath, (off_t)size);
#endif
    fprintf(stderr, "[fs] truncate '%s' size=%llu (%s)\n",
            gpath, (unsigned long long)size, r == 0 ? "OK" : strerror(errno));
    ctx->gpr[3] = (r == 0) ? CELL_OK : (uint64_t)(int64_t)cell_fs_from_errno();
}
static void cellFsFtruncate(ppu_context* ctx)
{
    int fd = (int)(uint32_t)ctx->gpr[3];
    uint64_t size = ctx->gpr[4];
    if (fd < 0 || fd >= FS_MAX || !g_files[fd]) {
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO;
        return;
    }
    int r = host_ftruncate_size(g_files[fd], size);
    const char* gp = g_fd_path[fd];
    fprintf(stderr, "[fs] ftruncate fd=%d '%s' size=%llu (%s)\n",
            fd, gp[0] ? gp : "?", (unsigned long long)size, r == 0 ? "OK" : strerror(errno));
    ctx->gpr[3] = (r == 0) ? CELL_OK : (uint64_t)(int64_t)cell_fs_from_errno();
}
static void cellFsGetBlockSize(ppu_context* ctx)
{
    char gpath[1024];
    guest_strcpy(gpath, (uint32_t)ctx->gpr[3], sizeof gpath);
    uint32_t sector_ptr = (uint32_t)ctx->gpr[4];
    uint32_t block_ptr  = (uint32_t)ctx->gpr[5];
    if (sector_ptr) vm_write64(sector_ptr, 512);
    if (block_ptr)  vm_write64(block_ptr, 4096);
    fprintf(stderr, "[fs] GetBlockSize '%s' sector=512 block=4096\n", gpath);
    ctx->gpr[3] = CELL_OK;
}
static void cellFsFGetBlockSize(ppu_context* ctx)
{
    int fd = (int)(uint32_t)ctx->gpr[3];
    uint32_t sector_ptr = (uint32_t)ctx->gpr[4];
    uint32_t block_ptr  = (uint32_t)ctx->gpr[5];
    if (fd < 0 || fd >= FS_MAX || !g_files[fd]) {
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO;
        return;
    }
    if (sector_ptr) vm_write64(sector_ptr, 512);
    if (block_ptr)  vm_write64(block_ptr, 4096);
    fprintf(stderr, "[fs] FGetBlockSize fd=%d sector=512 block=4096\n", fd);
    ctx->gpr[3] = CELL_OK;
}
/* NID 0x7A0329A1. ABI: int cellFsAllocateFileAreaWithoutZeroFill(const char *path, uint64_t size).
 * Extends an existing file to `size` without zero-fill (sparse SetEndOfFile /
 * ftruncate). Does not shrink. CELL_ENOSPC if the volume cannot hold the grow. */
static void cellFsAllocateFileAreaWithoutZeroFill(ppu_context* ctx)
{
    uint32_t path_ea = (uint32_t)ctx->gpr[3];
    uint64_t size = ctx->gpr[4];
    if (!path_ea) {
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EFAULT;
        return;
    }
    char gpath[1024], hpath[1100];
    guest_strcpy(gpath, path_ea, sizeof gpath);
    if (!gpath[0]) {
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EINVAL;
        return;
    }
    host_path(hpath, sizeof hpath, gpath);
    int64_t rc = host_allocate_path(hpath, size);
    fprintf(stderr, "[fs] AllocateFileAreaWithoutZeroFill '%s' size=%llu (%s)\n",
            gpath, (unsigned long long)size,
            rc == CELL_OK ? "OK" : (rc == (int64_t)CELL_FS_ENOSPC ? "ENOSPC" : "FAIL"));
    ctx->gpr[3] = (uint64_t)rc;
}

/* NID 0x2CF1296B. ABI: int cellFsAllocateFileAreaByFdWithoutZeroFill(int fd, uint64_t size).
 * Same grow as the path form, on the ppu_fs fd table (not the toolkit copy). */
static void cellFsAllocateFileAreaByFdWithoutZeroFill(ppu_context* ctx)
{
    int fd = (int)(uint32_t)ctx->gpr[3];
    uint64_t size = ctx->gpr[4];
    if (fd < 0 || fd >= FS_MAX || !g_files[fd]) {
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO;
        return;
    }
    const char* gp = g_fd_path[fd];
    if (gp[0]) {
        char hpath[1100];
        host_path(hpath, sizeof hpath, gp);
        uint64_t cur = 0;
#ifdef _WIN32
        int hfd = host_fileno_of(g_files[fd]);
        struct __stat64 st;
        if (hfd >= 0 && _fstat64(hfd, &st) == 0)
            cur = (uint64_t)st.st_size;
#else
        int hfd = host_fileno_of(g_files[fd]);
        struct stat st;
        if (hfd >= 0 && fstat(hfd, &st) == 0)
            cur = (uint64_t)st.st_size;
#endif
        if (cur < size) {
            uint64_t freeb = cellfs_host_free_bytes(hpath);
            if (freeb > 0 && freeb < size - cur) {
                fprintf(stderr, "[fs] AllocateFileAreaByFdWithoutZeroFill fd=%d '%s' size=%llu (ENOSPC)\n",
                        fd, gp, (unsigned long long)size);
                ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_ENOSPC;
                return;
            }
        }
    }
    int64_t rc = host_allocate_fp(g_files[fd], size);
    fprintf(stderr, "[fs] AllocateFileAreaByFdWithoutZeroFill fd=%d '%s' size=%llu (%s)\n",
            fd, gp[0] ? gp : "?", (unsigned long long)size,
            rc == CELL_OK ? "OK" : (rc == (int64_t)CELL_FS_ENOSPC ? "ENOSPC" : "FAIL"));
    ctx->gpr[3] = (uint64_t)rc;
}

/* NID 0x103B8632. ABI: int cellFsAllocateFileAreaWithInitialData(
 *   const char *path, uint64_t initialDataSize, const void *initialData,
 *   uint64_t initialDataFileOffset, uint64_t allocatedSize).
 * Creates if missing, writes `initialData` at `initialDataFileOffset`, then
 * sparse-extends to `allocatedSize`. Does not refuse an existing file (FIOS
 * overlay seed is an empty cache.dat). */
static void cellFsAllocateFileAreaWithInitialData(ppu_context* ctx)
{
    uint32_t path_ea = (uint32_t)ctx->gpr[3];
    uint64_t data_size = ctx->gpr[4];
    uint32_t buf_ea = (uint32_t)ctx->gpr[5];
    uint64_t data_off = ctx->gpr[6];
    uint64_t alloc_size = ctx->gpr[7];
    if (!path_ea) {
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EFAULT;
        return;
    }
    if (data_off > alloc_size || data_size > alloc_size - data_off) {
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EINVAL;
        return;
    }
    char gpath[1024], hpath[1100];
    guest_strcpy(gpath, path_ea, sizeof gpath);
    if (!gpath[0]) {
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EINVAL;
        return;
    }
    host_path(hpath, sizeof hpath, gpath);
    int64_t rc = host_set_size_path(hpath, alloc_size, 1);
    if (rc == CELL_OK)
        rc = host_write_at_path(hpath, buf_ea, data_off, data_size);
    fprintf(stderr, "[fs] AllocateFileAreaWithInitialData '%s' off=%llu dlen=%llu size=%llu (%s)\n",
            gpath, (unsigned long long)data_off, (unsigned long long)data_size,
            (unsigned long long)alloc_size,
            rc == CELL_OK ? "OK" : (rc == (int64_t)CELL_FS_ENOSPC ? "ENOSPC" : "FAIL"));
    ctx->gpr[3] = (uint64_t)rc;
}

/* NID 0x3394F037. ABI: int cellFsAllocateFileAreaByFdWithInitialData(
 *   int fd, uint64_t initialDataSize, const void *initialData,
 *   uint64_t initialDataFileOffset, uint64_t allocatedSize). */
static void cellFsAllocateFileAreaByFdWithInitialData(ppu_context* ctx)
{
    int fd = (int)(uint32_t)ctx->gpr[3];
    uint64_t data_size = ctx->gpr[4];
    uint32_t buf_ea = (uint32_t)ctx->gpr[5];
    uint64_t data_off = ctx->gpr[6];
    uint64_t alloc_size = ctx->gpr[7];
    if (fd < 0 || fd >= FS_MAX || !g_files[fd]) {
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO;
        return;
    }
    if (data_off > alloc_size || data_size > alloc_size - data_off) {
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EINVAL;
        return;
    }
    const char* gp = g_fd_path[fd];
    if (gp[0]) {
        char hpath[1100];
        host_path(hpath, sizeof hpath, gp);
#ifdef _WIN32
        int hfd = host_fileno_of(g_files[fd]);
        struct __stat64 st;
        uint64_t cur = (hfd >= 0 && _fstat64(hfd, &st) == 0) ? (uint64_t)st.st_size : 0;
#else
        int hfd = host_fileno_of(g_files[fd]);
        struct stat st;
        uint64_t cur = (hfd >= 0 && fstat(hfd, &st) == 0) ? (uint64_t)st.st_size : 0;
#endif
        int64_t spc = host_enospc_if_grow(hpath, cur, alloc_size);
        if (spc != CELL_OK) {
            fprintf(stderr, "[fs] AllocateFileAreaByFdWithInitialData fd=%d '%s' size=%llu (ENOSPC)\n",
                    fd, gp, (unsigned long long)alloc_size);
            ctx->gpr[3] = (uint64_t)spc;
            return;
        }
    }
    int r = host_ftruncate_size(g_files[fd], alloc_size);
    int64_t rc = (r == 0) ? CELL_OK : cell_fs_from_errno();
    if (rc == CELL_OK)
        rc = host_write_at_fp(g_files[fd], buf_ea, data_off, data_size);
    fprintf(stderr, "[fs] AllocateFileAreaByFdWithInitialData fd=%d '%s' off=%llu dlen=%llu size=%llu (%s)\n",
            fd, gp[0] ? gp : "?", (unsigned long long)data_off,
            (unsigned long long)data_size, (unsigned long long)alloc_size,
            rc == CELL_OK ? "OK" : (rc == (int64_t)CELL_FS_ENOSPC ? "ENOSPC" : "FAIL"));
    ctx->gpr[3] = (uint64_t)rc;
}

/* NID 0x606F9F42. ABI: int cellFsChangeFileSizeWithoutAllocation(const char *path, uint64_t newSize).
 * Sets the existing file to `newSize` without zero-fill (sparse grow or shrink). */
static void cellFsChangeFileSizeWithoutAllocation(ppu_context* ctx)
{
    uint32_t path_ea = (uint32_t)ctx->gpr[3];
    uint64_t size = ctx->gpr[4];
    if (!path_ea) {
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EFAULT;
        return;
    }
    char gpath[1024], hpath[1100];
    guest_strcpy(gpath, path_ea, sizeof gpath);
    if (!gpath[0]) {
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EINVAL;
        return;
    }
    host_path(hpath, sizeof hpath, gpath);
    int64_t rc = host_set_size_path(hpath, size, 0);
    fprintf(stderr, "[fs] ChangeFileSizeWithoutAllocation '%s' size=%llu (%s)\n",
            gpath, (unsigned long long)size,
            rc == CELL_OK ? "OK" : (rc == (int64_t)CELL_FS_ENOSPC ? "ENOSPC" : "FAIL"));
    ctx->gpr[3] = (uint64_t)rc;
}

/* NID 0xE15939C3. ABI: int cellFsChangeFileSizeByFdWithoutAllocation(int fd, uint64_t newSize). */
static void cellFsChangeFileSizeByFdWithoutAllocation(ppu_context* ctx)
{
    int fd = (int)(uint32_t)ctx->gpr[3];
    uint64_t size = ctx->gpr[4];
    if (fd < 0 || fd >= FS_MAX || !g_files[fd]) {
        ctx->gpr[3] = (uint64_t)(int64_t)CELL_FS_EIO;
        return;
    }
    const char* gp = g_fd_path[fd];
    if (gp[0]) {
        char hpath[1100];
        host_path(hpath, sizeof hpath, gp);
#ifdef _WIN32
        int hfd = host_fileno_of(g_files[fd]);
        struct __stat64 st;
        uint64_t cur = (hfd >= 0 && _fstat64(hfd, &st) == 0) ? (uint64_t)st.st_size : 0;
#else
        int hfd = host_fileno_of(g_files[fd]);
        struct stat st;
        uint64_t cur = (hfd >= 0 && fstat(hfd, &st) == 0) ? (uint64_t)st.st_size : 0;
#endif
        int64_t spc = host_enospc_if_grow(hpath, cur, size);
        if (spc != CELL_OK) {
            fprintf(stderr, "[fs] ChangeFileSizeByFdWithoutAllocation fd=%d '%s' size=%llu (ENOSPC)\n",
                    fd, gp, (unsigned long long)size);
            ctx->gpr[3] = (uint64_t)spc;
            return;
        }
    }
    int r = host_ftruncate_size(g_files[fd], size);
    int64_t rc = (r == 0) ? CELL_OK : cell_fs_from_errno();
    fprintf(stderr, "[fs] ChangeFileSizeByFdWithoutAllocation fd=%d '%s' size=%llu (%s)\n",
            fd, gp[0] ? gp : "?", (unsigned long long)size,
            rc == CELL_OK ? "OK" : (rc == (int64_t)CELL_FS_ENOSPC ? "ENOSPC" : "FAIL"));
    ctx->gpr[3] = (uint64_t)rc;
}

/* NID 0xAA3B4BCD. FIOS overlay compares planned cache.dat (numBlocks) against
 * block_size * free_block_count on /dev_hdd1. The toolkit fallback used to
 * report 1 GiB when translate_path missed that mount; that is smaller than
 * ACIT's overlay and disabled the cache. Query the mapped syscache volume. */
static void cellFsGetFreeSize(ppu_context* ctx)
{
    char gpath[1024], hpath[1100];
    guest_strcpy(gpath, (uint32_t)ctx->gpr[3], sizeof gpath);
    uint32_t block_size_ptr = (uint32_t)ctx->gpr[4];
    uint32_t free_count_ptr = (uint32_t)ctx->gpr[5];
    host_path(hpath, sizeof hpath, gpath);

    const uint32_t block_size = 4096;
    uint64_t free_bytes = cellfs_host_free_bytes(hpath);
    if (free_bytes == 0)
        free_bytes = 8ull * 1024ull * 1024ull * 1024ull;
    uint64_t free_blocks = free_bytes / (uint64_t)block_size;

    if (block_size_ptr)
        vm_write32(block_size_ptr, block_size);
    if (free_count_ptr)
        vm_write64(free_count_ptr, free_blocks);

    fprintf(stderr, "[fs] GetFreeSize '%s' -> '%s' block=%u free_blocks=%llu (%llu MiB)\n",
            gpath, hpath, block_size,
            (unsigned long long)free_blocks,
            (unsigned long long)(free_bytes / (1024ull * 1024ull)));
    ctx->gpr[3] = CELL_OK;
}

/* ---- cellFs AIO -----------------------------------------------------------
 *
 * Scott Pilgrim loads everything through cellFsAioRead: it opens gamedata.fat
 * with cellFsOpen and then never calls cellFsRead again. With the AIO NIDs
 * unresolved, its loader thread submitted requests that nothing ever completed
 * and the whole boot parked -- one open, no reads, and the main thread spinning.
 *
 * CellFsAio, big-endian, 0x28 bytes:
 *   0x00 u32 fd    0x08 u64 offset    0x10 u32 buf    0x18 u64 size    0x20 u64 user_data
 *
 * The completion callback is (CellFsAio* aio, s32 error, s32 id, u64 size).
 *
 * ponytail: the read runs synchronously and the callback fires before the
 * submit call returns. A title that submits, THEN arms the thing the callback
 * signals, would miss it -- move completion to a worker thread if one shows up.
 */

static uint32_t guest_be32(uint32_t a)
{
    if (ppu_vm_size && (uint64_t)a + 4 > ppu_vm_size) return 0;
    return ((uint32_t)vm_base[a] << 24) | ((uint32_t)vm_base[a+1] << 16) |
           ((uint32_t)vm_base[a+2] << 8) | (uint32_t)vm_base[a+3];
}
static uint64_t guest_be64(uint32_t a)
{
    return ((uint64_t)guest_be32(a) << 32) | guest_be32(a + 4);
}

static void cellFsAioInit(ppu_context* ctx)   { ctx->gpr[3] = CELL_OK; }
static void cellFsAioFinish(ppu_context* ctx) { ctx->gpr[3] = CELL_OK; }
/* Nothing is ever outstanding, so a cancel has nothing to find. Real cellFs
 * answers CELL_FS_ENOENT for an unknown id and callers treat that as "already
 * done", which is exactly true here. */
static void cellFsAioCancel(ppu_context* ctx) { ctx->gpr[3] = CELL_OK; }

static void cellFsAioRead(ppu_context* ctx)
{
    uint32_t aio    = (uint32_t)ctx->gpr[3];
    uint32_t id_ptr = (uint32_t)ctx->gpr[4];
    uint32_t cb_opd = (uint32_t)ctx->gpr[5];

    int      fd     = (int)guest_be32(aio + 0x00);
    uint64_t offset = guest_be64(aio + 0x08);
    uint32_t buf    = guest_be32(aio + 0x10);
    uint64_t size   = guest_be64(aio + 0x18);

    static int32_t s_next_id = 1;
    int32_t id = s_next_id++;
    if (id_ptr) vm_write32(id_ptr, (uint32_t)id);

    size_t n = 0;
    int32_t err = CELL_FS_ENOENT;
    if (fd >= 0 && fd < FS_MAX && g_files[fd]) {
        if (ppu_vm_size && (uint64_t)buf + size > ppu_vm_size) size = ppu_vm_size - buf;
        fs_prefault(buf, size);
        /* AIO reads are absolute -- they do not disturb the fd's own file
         * position, and the guest interleaves them freely across threads. */
        int64_t saved = (int64_t)HOST_FTELL64(g_files[fd]);
        if (HOST_FSEEK64(g_files[fd], (int64_t)offset, SEEK_SET) == 0)
            n = fread(vm_base + buf, 1, (size_t)size, g_files[fd]);
        HOST_FSEEK64(g_files[fd], saved, SEEK_SET);
        err = CELL_OK;
    }
    if (getenv("PS3_FSLOG"))
        fprintf(stderr, "[fs] aio read id=%d fd=%d off=%llu size=%llu -> %zu\n",
                id, fd, (unsigned long long)offset, (unsigned long long)size, n);

    ctx->gpr[3] = CELL_OK;
    if (cb_opd) ps3_invoke_guest(cb_opd, aio, (uint64_t)(int64_t)err,
                                 (uint64_t)(int64_t)id, (uint64_t)n, 0, 0, 0, 0);
}

static void cellFsAioWrite(ppu_context* ctx)
{
    uint32_t aio    = (uint32_t)ctx->gpr[3];
    uint32_t id_ptr = (uint32_t)ctx->gpr[4];
    uint32_t cb_opd = (uint32_t)ctx->gpr[5];

    int      fd     = (int)guest_be32(aio + 0x00);
    uint64_t offset = guest_be64(aio + 0x08);
    uint32_t buf    = guest_be32(aio + 0x10);
    uint64_t size   = guest_be64(aio + 0x18);

    static int32_t s_next_wr = 1;
    int32_t id = s_next_wr++;
    if (id_ptr) vm_write32(id_ptr, (uint32_t)id);

    size_t n = 0;
    int32_t err = CELL_FS_ENOENT;
    if (fd >= 0 && fd < FS_MAX && g_files[fd]) {
        if (ppu_vm_size && (uint64_t)buf + size > ppu_vm_size) size = ppu_vm_size - buf;
        fs_prefault(buf, size);
        int64_t cur = (int64_t)HOST_FTELL64(g_files[fd]);
        if (HOST_FSEEK64(g_files[fd], (int64_t)offset, SEEK_SET) == 0)
            n = fwrite(vm_base + buf, 1, (size_t)size, g_files[fd]);
        fflush(g_files[fd]);
        HOST_FSEEK64(g_files[fd], cur, SEEK_SET);
        err = (n < (size_t)size) ? (int32_t)cell_fs_from_errno() : CELL_OK;
    }
    fprintf(stderr, "[fs] aio write id=%d fd=%d off=%llu size=%llu -> %zu\n",
            id, fd, (unsigned long long)offset, (unsigned long long)size, n);

    ctx->gpr[3] = CELL_OK;
    if (cb_opd) ps3_invoke_guest(cb_opd, aio, (uint64_t)(int64_t)err,
                                 (uint64_t)(int64_t)id, (uint64_t)n, 0, 0, 0, 0);
}

extern "C" void ppu_fs_register(void)
{
    ps3_hle_register_ctx(ps3_compute_nid("cellFsOpen"),     "cellFsOpen",     cellFsOpen);
    /* Register by the literal import NID: LBP imports 0xB1840B53 for
     * cellFsSdataOpen, which ps3_compute_nid("cellFsSdataOpen") does NOT match
     * (the real exported symbol name differs from the friendly name). */
    ps3_hle_register_ctx(0xB1840B53u, "cellFsSdataOpen", cellFsSdataOpen);
    ps3_hle_register_ctx(ps3_compute_nid("cellFsClose"),    "cellFsClose",    cellFsClose);
    ps3_hle_register_ctx(ps3_compute_nid("cellFsRead"),     "cellFsRead",     cellFsRead);
    ps3_hle_register_ctx(ps3_compute_nid("cellFsWrite"),    "cellFsWrite",    cellFsWrite);
    ps3_hle_register_ctx(ps3_compute_nid("cellFsLseek"),    "cellFsLseek",    cellFsLseek);
    ps3_hle_register_ctx(ps3_compute_nid("cellFsStat"),     "cellFsStat",     cellFsStat);
    ps3_hle_register_ctx(ps3_compute_nid("cellFsFstat"),    "cellFsFstat",    cellFsFstat);
    ps3_hle_register_ctx(ps3_compute_nid("cellFsOpendir"),  "cellFsOpendir",  cellFsOpendir);
    ps3_hle_register_ctx(ps3_compute_nid("cellFsReaddir"),  "cellFsReaddir",  cellFsReaddir);
    ps3_hle_register_ctx(ps3_compute_nid("cellFsClosedir"), "cellFsClosedir", cellFsClosedir);
    ps3_hle_register_ctx(ps3_compute_nid("cellFsMkdir"),    "cellFsMkdir",    cellFsMkdir);
    ps3_hle_register_ctx(ps3_compute_nid("cellFsRmdir"),    "cellFsRmdir",    cellFsRmdir);
    ps3_hle_register_ctx(ps3_compute_nid("cellFsUnlink"),   "cellFsUnlink",   cellFsUnlink);
    ps3_hle_register_ctx(ps3_compute_nid("cellFsRename"),   "cellFsRename",   cellFsRename);
    ps3_hle_register_ctx(ps3_compute_nid("cellFsFsync"),    "cellFsFsync",    cellFsFsync);
    /* Literal NIDs: SHA-1(name + firmware suffix). Path form is the FIOS overlay
     * prealloc; ByFd shares this fd table so a later grow is not EBADF. */
    ps3_hle_register_ctx(0x7A0329A1u, "cellFsAllocateFileAreaWithoutZeroFill",
                         cellFsAllocateFileAreaWithoutZeroFill);
    ps3_hle_register_ctx(0x2CF1296Bu, "cellFsAllocateFileAreaByFdWithoutZeroFill",
                         cellFsAllocateFileAreaByFdWithoutZeroFill);
    ps3_hle_register_ctx(0x103B8632u, "cellFsAllocateFileAreaWithInitialData",
                         cellFsAllocateFileAreaWithInitialData);
    ps3_hle_register_ctx(0x3394F037u, "cellFsAllocateFileAreaByFdWithInitialData",
                         cellFsAllocateFileAreaByFdWithInitialData);
    ps3_hle_register_ctx(0x606F9F42u, "cellFsChangeFileSizeWithoutAllocation",
                         cellFsChangeFileSizeWithoutAllocation);
    ps3_hle_register_ctx(0xE15939C3u, "cellFsChangeFileSizeByFdWithoutAllocation",
                         cellFsChangeFileSizeByFdWithoutAllocation);
    /* Same fd table as Open/Write. Toolkit C Truncate/Ftruncate used a second table. */
    ps3_hle_register_ctx(ps3_compute_nid("cellFsTruncate"),      "cellFsTruncate",      cellFsTruncate);
    ps3_hle_register_ctx(ps3_compute_nid("cellFsFtruncate"),     "cellFsFtruncate",     cellFsFtruncate);
    ps3_hle_register_ctx(ps3_compute_nid("cellFsGetBlockSize"),  "cellFsGetBlockSize",  cellFsGetBlockSize);
    ps3_hle_register_ctx(ps3_compute_nid("cellFsFGetBlockSize"), "cellFsFGetBlockSize", cellFsFGetBlockSize);
    /* Literal import NID: FIOS overlay cache.dat vs /dev_hdd1 free space. */
    ps3_hle_register_ctx(0xAA3B4BCDu, "cellFsGetFreeSize", cellFsGetFreeSize);
    /* AIO by literal import NID -- ps3_compute_nid() of the friendly name does
     * not match the exported symbols. */
    ps3_hle_register_ctx(0xDB869F20u, "cellFsAioInit",   cellFsAioInit);
    ps3_hle_register_ctx(0x9F951810u, "cellFsAioFinish", cellFsAioFinish);
    ps3_hle_register_ctx(0xC1C507E7u, "cellFsAioRead",   cellFsAioRead);
    ps3_hle_register_ctx(0x4CEF342Eu, "cellFsAioWrite",  cellFsAioWrite);
    ps3_hle_register_ctx(0x7F13FC8Cu, "cellFsAioCancel", cellFsAioCancel);
}
