/*
 * ps3recomp - Filesystem syscalls (implementation)
 */

#include "sys_fs.h"
#include "../../libs/filesystem/edat.h"
#include "../memory/vm.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>

#ifdef _WIN32
  #include <direct.h>
  #include <io.h>
  #include <sys/utime.h>
  #include <time.h>
  #define stat _stat64
  #define S_ISDIR(m) (((m) & _S_IFDIR) != 0)
  #define S_ISREG(m) (((m) & _S_IFREG) != 0)
#else
  #include <unistd.h>
  #include <dirent.h>
  #include <time.h>     /* nanosleep: the delay in the open-retry loop */
  #include <sys/statvfs.h>
  #include <utime.h>
#endif

/* ---------------------------------------------------------------------------
 * Globals
 * -----------------------------------------------------------------------*/
sys_fs_fd_info  g_sys_fs_fds[SYS_FS_FD_MAX];
sys_fs_dir_info g_sys_fs_dirs[SYS_FS_DIR_MAX];
char            g_sys_fs_root[512] = ".";

static void write_be32(uint32_t addr, uint32_t val)
{
    uint32_t* p = (uint32_t*)vm_to_host(addr);
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ || defined(_WIN32)
    val = ((val >> 24) & 0xFF) | ((val >> 8) & 0xFF00) |
          ((val <<  8) & 0xFF0000) | ((val << 24) & 0xFF000000u);
#endif
    *p = val;
}

static void write_be64(uint32_t addr, uint64_t val)
{
    uint64_t* p = (uint64_t*)vm_to_host(addr);
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ || defined(_WIN32)
    val = ((val >> 56) & 0xFFULL) |
          ((val >> 40) & 0xFF00ULL) |
          ((val >> 24) & 0xFF0000ULL) |
          ((val >>  8) & 0xFF000000ULL) |
          ((val <<  8) & 0xFF00000000ULL) |
          ((val << 24) & 0xFF0000000000ULL) |
          ((val << 40) & 0xFF000000000000ULL) |
          ((val << 56) & 0xFF00000000000000ULL);
#endif
    *p = val;
}

static uint32_t read_be32(uint32_t addr)
{
    uint32_t val = *(uint32_t*)vm_to_host(addr);
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ || defined(_WIN32)
    val = ((val >> 24) & 0xFF) | ((val >> 8) & 0xFF00) |
          ((val <<  8) & 0xFF0000) | ((val << 24) & 0xFF000000u);
#endif
    return val;
}

static uint64_t read_be64(uint32_t addr)
{
    uint64_t val = *(uint64_t*)vm_to_host(addr);
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ || defined(_WIN32)
    val = ((val >> 56) & 0xFFULL) |
          ((val >> 40) & 0xFF00ULL) |
          ((val >> 24) & 0xFF0000ULL) |
          ((val >>  8) & 0xFF000000ULL) |
          ((val <<  8) & 0xFF00000000ULL) |
          ((val << 24) & 0xFF0000000000ULL) |
          ((val << 40) & 0xFF000000000000ULL) |
          ((val << 56) & 0xFF00000000000000ULL);
#endif
    return val;
}

static void fill_cell_stat(uint32_t stat_addr, struct stat* st);
static uint64_t sys_fs_host_total_bytes(const char* host_path);

/* ---------------------------------------------------------------------------
 * Path translation
 *
 * Maps PS3 virtual paths to host filesystem paths:
 *   /dev_hdd0/...  -> <root>/dev_hdd0/...
 *   /dev_bdvd/...  -> <root>/dev_bdvd/...
 *   /dev_flash/... -> <root>/dev_flash/...
 *   /app_home/...  -> <root>/app_home/...
 *   /dev_usb000/.. -> <root>/dev_usb000/...
 *   Others         -> <root>/<path>
 * -----------------------------------------------------------------------*/
static void fs_normalize_sep(char* p) {
#ifdef _WIN32
    for (char* c = p; *c; c++) if (*c == '/') *c = '\\';
#else
    (void)p;
#endif
}

/* Extracted game trees come in two shapes: the disc layout (<root>/PS3_GAME/
 * USRDIR/...) and a flattened one (<root>/USRDIR/...). cellGame hands the title
 * disc-style paths either way -- cellGameContentPermit reports
 * /dev_bdvd/PS3_GAME/USRDIR -- so on a flattened tree every path that came from
 * cellGame misses. flOw opendir'd /dev_bdvd/PS3_GAME/USRDIR, got ENOENT, and
 * read that as "no disc".
 *
 * Additive and last-resort: if the translated path does not exist but dropping
 * a PS3_GAME component yields one that does, use that. When neither exists the
 * original is kept, so failure messages still name the path the guest asked for.
 *
 * All three path translators call this -- sys_fs, cellFs and ppu_fs each do
 * their own translation (see docs), and a guest path can arrive at any of them. */
static char s_disc_root[1024];

void ps3_vfs_set_disc_root(const char* root)
{
    if (!root || !*root) {
        s_disc_root[0] = 0;
        return;
    }
    snprintf(s_disc_root, sizeof s_disc_root, "%s", root);
    for (char* c = s_disc_root; *c; c++)
        if (*c == '\\') *c = '/';
    size_t n = strlen(s_disc_root);
    while (n > 1 && s_disc_root[n - 1] == '/')
        s_disc_root[--n] = 0;
}

static char* ps3_vfs_find_ps3game(char* path)
{
    char* p = strstr(path, "/PS3_GAME/");
    if (p) return p;
    p = strstr(path, "\\PS3_GAME\\");
    if (p) return p;
    p = strstr(path, "/PS3_GAME\\");
    if (p) return p;
    return strstr(path, "\\PS3_GAME/");
}

void ps3_vfs_ps3game_fallback(char* path, size_t cap)
{
    struct stat st;
    if (!path || !*path || stat(path, &st) == 0)
        return;
    char* p = ps3_vfs_find_ps3game(path);
    if (!p)
        return;
    char alt[1024];
    /* Flattened tree: drop the PS3_GAME component. */
    size_t head = (size_t)(p - path);
    if (head + 1 < sizeof alt) {
        memcpy(alt, path, head);
        snprintf(alt + head, sizeof alt - head, "/%s", p + 10);
        if (stat(alt, &st) == 0) {
            snprintf(path, cap, "%s", alt);
            return;
        }
    }
    /* Wrong $PS3_VFS_ROOT (no disc files): rewrite onto the ELF dump root. */
    if (s_disc_root[0]) {
        snprintf(alt, sizeof alt, "%s/%s", s_disc_root, p + 1);
        for (char* c = alt; *c; c++)
            if (*c == '\\') *c = '/';
        if (stat(alt, &st) == 0) {
            fprintf(stderr, "[fs] disc-root fallback '%s' -> '%s'\n", path, alt);
            snprintf(path, cap, "%s", alt);
        }
    }
}

/* Host directory that /dev_hdd1 maps into (cellSysCacheMount / FIOS overlay).
 * Same order as ppu_fs.cpp hdd1_host_root / cellFs.c cellfs_hdd1_host_root:
 * $PS3_HDD1_ROOT, else $PS3_HDD0_ROOT/syscache, else game/hdd0/syscache when
 * that tree exists, else <VFS>/hdd0/syscache. Stripping /dev_hdd1/ onto the
 * disc dump looks for <VFS>/cache/<id>/cache.idx and FIOS logs err=-129. */
static void sys_fs_hdd1_host_root(char* out, size_t cap)
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
            if (stat("game/hdd0", &st) == 0 && S_ISDIR(st.st_mode))
                snprintf(out, cap, "game/hdd0/syscache");
            else
                snprintf(out, cap, "%s/hdd0/syscache",
                         g_sys_fs_root[0] ? g_sys_fs_root : ".");
        }
    }
    for (char* p = out; *p; p++)
        if (*p == '\\') *p = '/';
}

static int sys_fs_mkdir_p(const char* path)
{
    char tmp[1100];
    snprintf(tmp, sizeof tmp, "%s", path);
    size_t len = strlen(tmp);
    while (len > 1 && (tmp[len - 1] == '/' || tmp[len - 1] == '\\'))
        tmp[--len] = 0;
    for (char* p = tmp + 1; *p; p++) {
        if (*p == '/' || *p == '\\') {
            char c = *p;
            *p = 0;
#ifdef _WIN32
            _mkdir(tmp);
#else
            mkdir(tmp, 0755);
#endif
            *p = c;
        }
    }
#ifdef _WIN32
    int r = _mkdir(tmp);
#else
    int r = mkdir(tmp, 0755);
#endif
    if (r != 0) {
        struct stat st;
        if (stat(tmp, &st) == 0 && S_ISDIR(st.st_mode))
            r = 0;
    }
    return r;
}

static void sys_fs_hdd1_prepare(const char* guest, const char* hpath)
{
    char parent[1100];
    snprintf(parent, sizeof parent, "%s", hpath);
    char* slash = strrchr(parent, '/');
#ifdef _WIN32
    char* bslash = strrchr(parent, '\\');
    if (!slash || (bslash && bslash > slash))
        slash = bslash;
#endif
    if (slash && slash != parent) {
        *slash = 0;
        sys_fs_mkdir_p(parent);
    }
    const char* base = strrchr(guest, '/');
    base = base ? base + 1 : guest;
    /* FIOS 1.3 overlay: cache.idx is the index, cache.dat the payload. Seed
     * empty files on a freshly formatted syscache so openCacheFile is not
     * CELL_ENOENT (-129) when the guest hits this lv2 path instead of ppu_fs. */
    if (strcmp(base, "cache.idx") != 0 && strcmp(base, "cache.dat") != 0)
        return;
    struct stat st;
    if (stat(hpath, &st) == 0)
        return;
    FILE* sf = fopen(hpath, "wb");
    if (sf) {
        fclose(sf);
        fprintf(stderr, "[sys_fs] seeded empty syscache '%s' -> '%s'\n", guest, hpath);
    }
}

void sys_fs_translate_path(const char* ps3_path, char* host_path, int host_path_size)
{
    /* Lazily adopt the same root as ppu_fs (ppu_vfs_root), which may have
     * rejected a $PS3_VFS_ROOT that has no PARAM.SFO/game.psarc. */
    if (g_sys_fs_root[0] == '.' && g_sys_fs_root[1] == '\0') {
        extern const char* ppu_vfs_root;
        if (ppu_vfs_root && ppu_vfs_root[0] &&
            !(ppu_vfs_root[0] == '.' && ppu_vfs_root[1] == '\0')) {
            strncpy(g_sys_fs_root, ppu_vfs_root, sizeof(g_sys_fs_root) - 1);
            g_sys_fs_root[sizeof(g_sys_fs_root) - 1] = 0;
        } else {
            const char* env = getenv("PS3_VFS_ROOT");
            if (env && *env) {
                strncpy(g_sys_fs_root, env, sizeof(g_sys_fs_root) - 1);
                g_sys_fs_root[sizeof(g_sys_fs_root) - 1] = 0;
            }
        }
    }

    /* /dev_hdd1 is the title syscache, not disc data. ppu_fs / cellFs already
     * special-case this; the lv2 translator used to strip the mount prefix
     * onto <VFS>/cache/... like /dev_bdvd. */
    if (strncmp(ps3_path, "/dev_hdd1/", 10) == 0 || strcmp(ps3_path, "/dev_hdd1") == 0) {
        char root[1024];
        sys_fs_hdd1_host_root(root, sizeof root);
        const char* rest = (ps3_path[9] == '/') ? ps3_path + 10 : "";
        if (*rest)
            snprintf(host_path, (size_t)host_path_size, "%s/%s", root, rest);
        else
            snprintf(host_path, (size_t)host_path_size, "%s", root);
        for (char* p = host_path; *p; p++)
            if (*p == '\\') *p = '/';
        static int logged = 0;
        if (!logged) {
            logged = 1;
            fprintf(stderr, "[sys_fs] /dev_hdd1 -> '%s'\n", root);
        }
        sys_fs_hdd1_prepare(ps3_path, host_path);
        fs_normalize_sep(host_path);
        return;
    }

    /* /dev_hdd0 overlays the installed game-update dir (patchN.farc live there
     * for a disc title patched to e.g. v1.30) -- mirror ppu_fs.cpp host_path.
     * PS3_HDD0_ROOT = host dir that /dev_hdd0 maps into (contains game/<title>/). */
    {
        static const char* hdd0_root = NULL; static int hdd0_init = 0;
        if (!hdd0_init) { hdd0_root = getenv("PS3_HDD0_ROOT"); hdd0_init = 1; }
        if (hdd0_root && strncmp(ps3_path, "/dev_hdd0/", 10) == 0) {
            snprintf(host_path, (size_t)host_path_size, "%s/%s", hdd0_root, ps3_path + 10);
            fs_normalize_sep(host_path);
            return;
        }
    }

    /* /dev_flash is FIRMWARE, not game data. ppu_fs.cpp serves it from a real
     * dev_flash tree ($PS3_DEV_FLASH); this layer was missing the same branch, so
     * a firmware path opened through the raw syscall resolved to <root>/... and
     * missed. ps1_netemu loads its PS1 BIOS as /dev_flash/ps1emu/ps1_rom.bin and
     * refuses to boot without it. Keep both halves of the split filesystem agreed. */
    {
        static const char* fw = NULL; static int fw_init = 0;
        if (!fw_init) { fw = getenv("PS3_DEV_FLASH"); fw_init = 1; }
        if (fw && *fw && strncmp(ps3_path, "/dev_flash/", 11) == 0) {
            snprintf(host_path, (size_t)host_path_size, "%s/%s", fw, ps3_path + 11);
            fs_normalize_sep(host_path);
            return;
        }
    }

    /* Strip a known mount prefix so this sys_fs layer resolves to the SAME host
     * tree as the cellFs layer (ppu_fs.cpp host_path). Previously /dev_bdvd/X
     * mapped to <root>/dev_bdvd/X -- a directory that doesn't exist -- so a title
     * that opens disc content through the raw sys_fs path (LBP's Bink videos,
     * e.g. gamedata/videos/localisation_test.bik) failed even though the file is
     * present, stalling the loader waiting on the resource. /app_home/ is the
     * game's install dir (== the USRDIR root); the title also opens it in
     * non-leading spellings ("app_home/...", "e:/app_home/..."). */
    static const char* const mounts[] = {
        "/dev_bdvd/", "/app_home/", "/dev_hdd0/", "/dev_hdd1/",
        "/dev_flash/", "/host_root/", "/dev_usb000/", "/dev_usb/"
    };
    const char* rel = ps3_path;
    /* flОw first: map anything from a "USRDIR/" component onward to <root>/USRDIR/...
     * directly, bypassing the full /dev_hdd0/game/<ID>/USRDIR install tree (which we
     * normally provide via a filesystem junction) -- concurrent opens THROUGH a
     * Windows junction intermittently fail (the nondeterministic Cg shader "could
     * not be read" abort). The real assets live under <root>/USRDIR. flОw's failing
     * paths carry "USRDIR/"; LBP's raw-sys_fs disc paths (/dev_bdvd Bink videos) do
     * not, so they fall through to sagemono's mount-prefix stripping below. */
    const char* usrp = strstr(ps3_path, "USRDIR/");
    if (usrp) {
        /* PS3_USRDIR_BASE: a title installed as a full /dev_hdd0/game/<ID> tree
         * keeps its USRDIR there, not at the vfs root, so the flattening above
         * sends it to a directory that does not exist. ps1_netemu writes its
         * settings with a bare "/USRDIR/CONFIG" and read them back the same way;
         * both missed, and it printed "save config file: /USRDIR/CONFIG" /
         * "failed" on every boot. Opt-in so flattened trees keep the old path. */
        const char* ub = getenv("PS3_USRDIR_BASE");
        if (ub && *ub) {
            snprintf(host_path, (size_t)host_path_size, "%s/%s", ub, usrp);
            fs_normalize_sep(host_path);
            return;
        }
        rel = usrp;                        /* "USRDIR/..." */
    } else {
        /* Otherwise strip a known mount prefix so this sys_fs layer resolves to the
         * SAME host tree as the cellFs layer (ppu_fs.cpp). /dev_bdvd/X previously
         * mapped to <root>/dev_bdvd/X -- a dir that doesn't exist -- stalling titles
         * that open disc content through raw sys_fs. */
        int matched = 0;
        for (size_t i = 0; i < sizeof(mounts) / sizeof(mounts[0]); i++) {
            size_t n = strlen(mounts[i]);
            if (strncmp(ps3_path, mounts[i], n) == 0) { rel = ps3_path + n; matched = 1; break; }
        }
        if (!matched) {
            const char* ah = strstr(ps3_path, "app_home/");
            if (ah) rel = ah + 9;          /* non-leading app_home spelling */
            else if (rel[0] == '/') rel++; /* otherwise just strip leading slash */
        }
    }

    snprintf(host_path, (size_t)host_path_size, "%s/%s", g_sys_fs_root, rel);
    fs_normalize_sep(host_path);
    /* NOTE: do NOT stat() the direct path and fall back to the dev_hdd0 junction
     * on failure. stat() intermittently returns ENOENT for an existing file here
     * (a Windows transient), and the junction fallback made it WORSE -- the flaky
     * stat sent a real open to a path that doesn't exist. The direct <root>/USRDIR
     * layout holds the assets; a genuinely-missing direct path is handled by the
     * caller's open retry + the extracted-dump fallback below. */

    /* Extracted-dump fallback: our test setups often hold a title's data at
     * <root>/extracted/USRDIR/... rather than the full /dev_hdd0/game/<ID>/USRDIR
     * install tree the guest opens. If the primary path doesn't exist but a
     * USRDIR-relative path under <root>/extracted does, use that. Additive: only
     * affects paths unresolved at the primary location. */
    struct stat st;
    if (stat(host_path, &st) != 0) {
        const char* usr = strstr(ps3_path, "/USRDIR/");
        if (usr) {
            char alt[1024];
            snprintf(alt, sizeof(alt), "%s/extracted/USRDIR/%s", g_sys_fs_root, usr + 8);
            fs_normalize_sep(alt);
            if (stat(alt, &st) == 0)
                snprintf(host_path, (size_t)host_path_size, "%s", alt);
        }
    }

    /* Working-directory fallback: a PS3 title runs with its USRDIR as the
     * working directory, so a RELATIVE open resolves there -- not at the vfs
     * root. flOw opens "Data/Resources/first.xml" that way (it prints
     * "Command Line: /dev_bdvd/PS3_GAME/USRDIR/" and "Working Path: /"), and
     * resolving it against the root looked for <root>/Data/... while the file
     * sits at <root>/USRDIR/Data/... . Missing it left the engine without its
     * render config, which is what produced
     *   PCoreGcmRenderInterface::setScreenRenderTargetInternal(): No config
     * and therefore no draw calls at all.
     *
     * Additive and last-resort: only when the primary path does not exist and
     * the guest path is relative, so nothing that already resolves changes. */
    /* Use the STRIPPED path and do not require a relative spelling: the title
     * reports Working Path "/" and opens "/Data/Resources/first.xml", which is
     * absolute but still means "relative to my USRDIR". */
    if (stat(host_path, &st) != 0) {
        char alt2[1024];
        snprintf(alt2, sizeof(alt2), "%s/USRDIR/%s", g_sys_fs_root, rel);
        fs_normalize_sep(alt2);
        if (stat(alt2, &st) == 0)
            snprintf(host_path, (size_t)host_path_size, "%s", alt2);
    }
    ps3_vfs_ps3game_fallback(host_path, (size_t)host_path_size);
}

/* ---------------------------------------------------------------------------
 * sys_fs_open
 *
 * r3 = path (guest string pointer)
 * r4 = flags
 * r5 = pointer to receive fd (s32*)
 * r6 = mode (permissions, ignored on Windows)
 * r7 = arg (unused)
 * -----------------------------------------------------------------------*/
int64_t sys_fs_open(ppu_context* ctx)
{
    uint32_t path_addr  = LV2_ARG_PTR(ctx, 0);
    int32_t  flags      = LV2_ARG_S32(ctx, 1);
    uint32_t fd_out     = LV2_ARG_PTR(ctx, 2);
    /* uint32_t mode    = LV2_ARG_U32(ctx, 3); */

    if (path_addr == 0 || fd_out == 0)
        return (int64_t)(int32_t)CELL_EINVAL;

    const char* ps3_path = (const char*)vm_to_host(path_addr);
    char host_path[1024];
    sys_fs_translate_path(ps3_path, host_path, sizeof(host_path));

    { extern char* getenv(const char*); if (getenv("FLOW_TITLEOPEN") && ps3_path && strstr(ps3_path, "Titles")) {
        size_t _l = strlen(ps3_path);
        fprintf(stderr, "[TITLEOPEN] path='%s' (len=%zu, ends_slash=%d) guest_lr=0x%08X\n",
                ps3_path, _l, (_l && ps3_path[_l-1]=='/')?1:0, (uint32_t)ctx->lr); fflush(stderr);
#ifdef _WIN32
        { char* mb=(char*)GetModuleHandleA(0); void* bt[30]; unsigned short fr=RtlCaptureStackBackTrace(0,30,bt,0);
          char ln[820]; int p=snprintf(ln,sizeof ln,"[TITLEOPEN-bt] rva:");
          for(unsigned short i=0;i<fr;i++) p+=snprintf(ln+p,sizeof(ln)-p," %llX",(unsigned long long)((char*)bt[i]-mb));
          fprintf(stderr,"%s\n",ln); fflush(stderr); }
#endif
    } }
    /* DIAGNOSTIC (FLOW_TITLEFIX): the game builds the title path with an EMPTY filename
     * (a stale-lift string-construction bug) -> opens the dir. Redirect a bare
     * ".../Data/Titles/" open to the language file so we can confirm the render path. */
    { extern char* getenv(const char*); if (getenv("FLOW_TITLEFIX")) {
        size_t hl = strlen(host_path);
        if (hl >= 8 && (strcmp(host_path + hl - 8, "Titles\\") == 0 || strcmp(host_path + hl - 8, "Titles/") == 0
                        || strcmp(host_path + hl - 7, "Titles\\") == 0 || strcmp(host_path + hl - 7, "Titles/") == 0)) {
            if (hl + 20 < sizeof(host_path)) { strcat(host_path, "Titles_English.xml");
                fprintf(stderr, "[TITLEFIX] redirected empty-filename title open -> %s\n", host_path); fflush(stderr); }
        }
    } }

    /* NPDRM: a file that begins "NPD\0" is an EDAT, and on hardware the guest never
     * sees its ciphertext -- sceNpDrmIsAvailable primes the kernel and cellFsOpen
     * returns plaintext. Decrypt once into a cache file and open that instead, so
     * every read/seek/stat path below stays unchanged. (libs/filesystem/edat.c) */
    { char dec_path[1200];
      const char* use = edat_resolve(host_path, dec_path, sizeof dec_path);
      if (use != host_path) snprintf(host_path, sizeof host_path, "%s", use); }

    /* Find free fd slot */
    int slot = -1;
    for (int i = 0; i < SYS_FS_FD_MAX; i++) {
        if (!g_sys_fs_fds[i].active) { slot = i; break; }
    }
    if (slot < 0)
        return (int64_t)(int32_t)CELL_ENOMEM;

    /* Determine fopen mode from PS3 flags */
    const char* mode;
    int access = flags & CELL_FS_O_ACCMODE;

    if (flags & CELL_FS_O_CREAT) {
        if (flags & CELL_FS_O_TRUNC) {
            if (access == CELL_FS_O_RDWR)
                mode = "w+b";
            else
                mode = "wb";
        } else if (flags & CELL_FS_O_APPEND) {
            if (access == CELL_FS_O_RDWR)
                mode = "a+b";
            else
                mode = "ab";
        } else {
            /* Create but don't truncate: open for read/write, create if not exist */
            if (access == CELL_FS_O_RDWR)
                mode = "r+b";
            else if (access == CELL_FS_O_WRONLY)
                mode = "r+b";
            else
                mode = "rb";
        }
    } else if (flags & CELL_FS_O_TRUNC) {
        if (access == CELL_FS_O_RDWR)
            mode = "w+b";
        else
            mode = "wb";
    } else if (flags & CELL_FS_O_APPEND) {
        if (access == CELL_FS_O_RDWR)
            mode = "a+b";
        else
            mode = "ab";
    } else {
        if (access == CELL_FS_O_RDWR)
            mode = "r+b";
        else if (access == CELL_FS_O_WRONLY)
            mode = "r+b";
        else
            mode = "rb";
    }

    FILE* fp = fopen(host_path, mode);

    /* Transient open failures: on Windows an existing, correctly-pathed file can
     * intermittently fail to open (AV/indexer holding a share lock, or momentary
     * handle pressure) -- which surfaced as a NONDETERMINISTIC "Cg shader file
     * could not be read" abort. Retry a few times with a brief backoff for a
     * read-only open of a file that exists; deterministic and cheap. */
    /* ...but ONLY when the failure could plausibly be transient. ENOENT is not:
     * the path does not exist and never will within this open. Retrying it cost
     * 3000 x Sleep(1) = tens of seconds of dead wall-clock PER probe, which is
     * what made the Rubber Ducky demo look wedged at "Loading cviewer scene" --
     * libxml2 probes /etc/xml/catalog there, and that file is absent by design.
     * A share-lock/handle-pressure failure (the case this retry exists for)
     * reports EACCES/EBUSY, not ENOENT, so the recovery still works. */
    if (!fp && errno != ENOENT &&
        !(flags & (CELL_FS_O_CREAT | CELL_FS_O_WRONLY | CELL_FS_O_TRUNC))) {
        for (int _r = 0; !fp && _r < 3000; _r++) {
#ifdef _WIN32
            Sleep(1);
#else
            /* The delay is the retry. 3000 iterations is meant to be about
             * three seconds of waiting for a transient share lock or handle
             * pressure to clear; without it the loop spends microseconds
             * calling fopen 3000 times on a condition that has not changed,
             * and the recovery this exists for never happens. */
            { struct timespec _d = { 0, 1000 * 1000 }; nanosleep(&_d, NULL); }
#endif
            fp = fopen(host_path, mode);
        }
        if (fp) fprintf(stderr, "[sys_fs] open recovered after retry: %s\n", host_path);
    }

    /* If CREAT flag set and file doesn't exist, try creating it */
    if (!fp && (flags & CELL_FS_O_CREAT)) {
        fp = fopen(host_path, "w+b");
    }

    /* A DIRECTORY open is not a failure. fopen() cannot open one on Windows,
     * but real cellFsOpen answers CELL_EISDIR and titles branch on it -- flOw
     * opens Data/Titles/ (its localisation XML dir) and a generic ENOENT tells
     * it the content is missing rather than "that is a directory".
     * libs/filesystem/cellFs.c and runtime/ppu/ppu_fs.cpp already answer this
     * correctly; sys_fs is the third filesystem path and never got it. */
    if (!fp) {
        struct stat _sd;
        if (stat(host_path, &_sd) == 0 && (_sd.st_mode & S_IFMT) == S_IFDIR) {
            fprintf(stderr, "[sys_fs] open -> EISDIR (directory): %s\n", host_path);
            return (int64_t)(int32_t)CELL_EISDIR;
        }
    }

    if (!fp) {
        { struct stat _s2; int _ex = (stat(host_path,&_s2)==0);
          fprintf(stderr, "[sys_fs] open FAILED: %s (errno=%d %s, exists=%d, size=%lld)\n",
                  host_path, errno, strerror(errno), _ex, _ex?(long long)_s2.st_size:-1LL); }
        return (int64_t)(int32_t)CELL_ENOENT;
    }

    fprintf(stderr, "[sys_fs] open OK: %s\n", host_path);

    sys_fs_fd_info* f = &g_sys_fs_fds[slot];
    f->active = 1;
    f->fp     = fp;
    f->flags  = flags;
    strncpy(f->path, host_path, sizeof(f->path) - 1);
    f->path[sizeof(f->path) - 1] = '\0';

    /* FD is slot + 3 (reserve 0=stdin, 1=stdout, 2=stderr) */
    int32_t fd = slot + 3;
    write_be32(fd_out, (uint32_t)fd);

    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_read
 *
 * r3 = fd
 * r4 = buffer (guest pointer)
 * r5 = size
 * r6 = pointer to receive bytes read (u64*)
 * -----------------------------------------------------------------------*/
int64_t sys_fs_read(ppu_context* ctx)
{
    int32_t  fd         = LV2_ARG_S32(ctx, 0);
    uint32_t buf_addr   = LV2_ARG_PTR(ctx, 1);
    uint64_t size       = LV2_ARG_U64(ctx, 2);
    uint32_t nread_addr = LV2_ARG_PTR(ctx, 3);

    int slot = fd - 3;
    if (slot < 0 || slot >= SYS_FS_FD_MAX)
        return (int64_t)(int32_t)CELL_EBADF;

    sys_fs_fd_info* f = &g_sys_fs_fds[slot];
    if (!f->active || !f->fp)
        return (int64_t)(int32_t)CELL_EBADF;

    void* buf = vm_to_host(buf_addr);
    long pos_before = ftell(f->fp);
    size_t nread = fread(buf, 1, (size_t)size, f->fp);

    /* PS3_FSTRACE=<n>: every nth read, the fd and the file offset it came from.
     *
     * The PS1 title under test streams its intro movie off the disc image and
     * never stops -- 560 s undriven and the blit counter is still climbing. If
     * the offsets here keep advancing, the stream is progressing and the movie
     * genuinely has not reached its end; if they repeat, the disc read is stuck
     * and the movie is looping over the same sectors forever. Those need
     * opposite fixes and nothing else distinguishes them. */
    { static int s_ft = -1;
      if (s_ft < 0) { const char* e = getenv("PS3_FSTRACE");
                      s_ft = e ? (atoi(e) > 0 ? atoi(e) : 200) : 0; }
      if (s_ft) { static unsigned long fn;
          if ((++fn % (unsigned long)s_ft) == 0)
              fprintf(stderr, "[fs] n=%lu fd=%d off=%ld size=%llu -> %llu\n",
                      fn, fd, pos_before, (unsigned long long)size,
                      (unsigned long long)nread); } }

    if (nread_addr != 0) {
        write_be64(nread_addr, (uint64_t)nread);
    }

    /* FLOW_FSDBG: the lv2 path is what PhyreEngine titles actually use (they do
     * not go through the cellFs HLE), so PS3_FSLOG in ppu_fs.cpp never fires. */
    { extern char* getenv(const char*);
      if (getenv("FLOW_FSDBG")) {
        const unsigned char* b = (const unsigned char*)buf;
        fprintf(stderr, "[FSDBG] read fd=%d '%s' pos=%ld want=%llu got=%zu"
                        " head=%02X%02X%02X%02X lr=0x%08X\n",
                fd, f->path, pos_before, (unsigned long long)size, nread,
                nread > 0 ? b[0] : 0, nread > 1 ? b[1] : 0,
                nread > 2 ? b[2] : 0, nread > 3 ? b[3] : 0,
                (uint32_t)ctx->lr);
      } }

    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_write
 *
 * r3 = fd
 * r4 = buffer (guest pointer)
 * r5 = size
 * r6 = pointer to receive bytes written (u64*)
 * -----------------------------------------------------------------------*/
int64_t sys_fs_write(ppu_context* ctx)
{
    int32_t  fd           = LV2_ARG_S32(ctx, 0);
    uint32_t buf_addr     = LV2_ARG_PTR(ctx, 1);
    uint64_t size         = LV2_ARG_U64(ctx, 2);
    uint32_t nwritten_addr = LV2_ARG_PTR(ctx, 3);

    int slot = fd - 3;
    if (slot < 0 || slot >= SYS_FS_FD_MAX) {
        /* Invalid fd — pretend write succeeded (CRT stdio uses corrupted fds) */
        if (nwritten_addr != 0)
            write_be64(nwritten_addr, size);
        return CELL_OK;
    }

    sys_fs_fd_info* f = &g_sys_fs_fds[slot];
    if (!f->active || !f->fp) {
        if (nwritten_addr != 0)
            write_be64(nwritten_addr, size);
        return CELL_OK;
    }

    const void* buf = vm_to_host(buf_addr);
    size_t nwritten = fwrite(buf, 1, (size_t)size, f->fp);
    fflush(f->fp);

    if (nwritten_addr != 0) {
        write_be64(nwritten_addr, (uint64_t)nwritten);
    }

    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_close
 *
 * r3 = fd
 * -----------------------------------------------------------------------*/
int64_t sys_fs_close(ppu_context* ctx)
{
    int32_t fd = LV2_ARG_S32(ctx, 0);

    int slot = fd - 3;
    if (slot < 0 || slot >= SYS_FS_FD_MAX)
        return (int64_t)(int32_t)CELL_EBADF;

    sys_fs_fd_info* f = &g_sys_fs_fds[slot];
    if (!f->active)
        return (int64_t)(int32_t)CELL_EBADF;

    if (f->fp) {
        fclose(f->fp);
        f->fp = NULL;
    }
    f->active = 0;

    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_lseek
 *
 * r3 = fd
 * r4 = offset (s64)
 * r5 = whence
 * r6 = pointer to receive new position (u64*)
 * -----------------------------------------------------------------------*/
int64_t sys_fs_lseek(ppu_context* ctx)
{
    int32_t  fd        = LV2_ARG_S32(ctx, 0);
    int64_t  offset    = LV2_ARG_S64(ctx, 1);
    int32_t  whence    = LV2_ARG_S32(ctx, 2);
    uint32_t pos_addr  = LV2_ARG_PTR(ctx, 3);

    int slot = fd - 3;
    if (slot < 0 || slot >= SYS_FS_FD_MAX) {
        /* Invalid fd — return position 0 instead of EBADF.
         * The CRT may call lseek on uninitialized FILE structures. */
        if (pos_addr)
            write_be64(pos_addr, 0);
        return 0;
    }

    sys_fs_fd_info* f = &g_sys_fs_fds[slot];
    if (!f->active || !f->fp) {
        if (pos_addr)
            write_be64(pos_addr, 0);
        return 0;
    }

    int origin;
    switch (whence) {
        case CELL_FS_SEEK_SET: origin = SEEK_SET; break;
        case CELL_FS_SEEK_CUR: origin = SEEK_CUR; break;
        case CELL_FS_SEEK_END: origin = SEEK_END; break;
        default: return (int64_t)(int32_t)CELL_EINVAL;
    }

#ifdef _WIN32
    _fseeki64(f->fp, offset, origin);
    int64_t pos = _ftelli64(f->fp);
#else
    fseeko(f->fp, (off_t)offset, origin);
    int64_t pos = (int64_t)ftello(f->fp);
#endif

    if (pos_addr != 0) {
        write_be64(pos_addr, (uint64_t)pos);
    }

    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * Helper: fill CellFsStat from host stat
 * -----------------------------------------------------------------------*/
static void fill_cell_stat(uint32_t stat_addr, struct stat* st)
{
    uint32_t mode = CELL_FS_S_IRUSR | CELL_FS_S_IRGRP | CELL_FS_S_IROTH;
    if (S_ISDIR(st->st_mode)) {
        mode |= CELL_FS_S_IFDIR | CELL_FS_S_IXUSR;
    } else {
        mode |= CELL_FS_S_IFREG;
    }
#ifndef _WIN32
    if (st->st_mode & S_IWUSR) mode |= CELL_FS_S_IWUSR;
    if (st->st_mode & S_IXUSR) mode |= CELL_FS_S_IXUSR;
#else
    mode |= CELL_FS_S_IWUSR; /* assume writable on Windows */
#endif

    /* CellFsStat is 0x34 (52) bytes, 4-byte aligned: the s64/u64 members are
     * be_t<...,4> so there is NO pad after gid (RPCS3: CHECK_SIZE_ALIGN(...,52,4)).
     * mode@0 uid@4 gid@8 atime@0x0C mtime@0x14 ctime@0x1C size@0x24 blksize@0x2C.
     * The old 8-byte-aligned 0x38 layout overran the struct by 4 bytes and, for
     * a stat embedded inside a larger object, clobbered the field after it. */
    write_be32(stat_addr + 0x00, mode);
    write_be32(stat_addr + 0x04, 0);  /* uid */
    write_be32(stat_addr + 0x08, 0);  /* gid */
    write_be64(stat_addr + 0x0C, (uint64_t)st->st_atime);
    write_be64(stat_addr + 0x14, (uint64_t)st->st_mtime);
    write_be64(stat_addr + 0x1C, (uint64_t)st->st_ctime);
    write_be64(stat_addr + 0x24, (uint64_t)st->st_size);
    write_be64(stat_addr + 0x2C, 4096ULL);  /* blksize */
}

/* ---------------------------------------------------------------------------
 * sys_fs_stat
 *
 * r3 = path (guest pointer)
 * r4 = pointer to CellFsStat
 * -----------------------------------------------------------------------*/
int64_t sys_fs_stat(ppu_context* ctx)
{
    uint32_t path_addr = LV2_ARG_PTR(ctx, 0);
    uint32_t stat_addr = LV2_ARG_PTR(ctx, 1);

    if (path_addr == 0 || stat_addr == 0)
        return (int64_t)(int32_t)CELL_EINVAL;

    const char* ps3_path = (const char*)vm_to_host(path_addr);
    char host_path[1024];
    sys_fs_translate_path(ps3_path, host_path, sizeof(host_path));

    struct stat st;
#ifdef _WIN32
    if (_stat64(host_path, (struct _stat64*)&st) != 0)
#else
    if (stat(host_path, &st) != 0)
#endif
    {
        return (int64_t)(int32_t)CELL_ENOENT;
    }

    fill_cell_stat(stat_addr, &st);
    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_fstat
 *
 * r3 = fd
 * r4 = pointer to CellFsStat
 * -----------------------------------------------------------------------*/
int64_t sys_fs_fstat(ppu_context* ctx)
{
    int32_t  fd        = LV2_ARG_S32(ctx, 0);
    uint32_t stat_addr = LV2_ARG_PTR(ctx, 1);

    int slot = fd - 3;
    if (slot < 0 || slot >= SYS_FS_FD_MAX)
        return (int64_t)(int32_t)CELL_EBADF;

    sys_fs_fd_info* f = &g_sys_fs_fds[slot];
    if (!f->active || !f->fp)
        return (int64_t)(int32_t)CELL_EBADF;

    struct stat st;
#ifdef _WIN32
    int fno = _fileno(f->fp);
    if (_fstat64(fno, (struct _stat64*)&st) != 0)
#else
    int fno = fileno(f->fp);
    if (fstat(fno, &st) != 0)
#endif
    {
        return (int64_t)(int32_t)CELL_EBADF;
    }

    fill_cell_stat(stat_addr, &st);
    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_opendir
 *
 * r3 = path (guest pointer)
 * r4 = pointer to receive dir fd (s32*)
 * -----------------------------------------------------------------------*/
int64_t sys_fs_opendir(ppu_context* ctx)
{
    uint32_t path_addr = LV2_ARG_PTR(ctx, 0);
    uint32_t fd_out    = LV2_ARG_PTR(ctx, 1);

    if (path_addr == 0 || fd_out == 0)
        return (int64_t)(int32_t)CELL_EINVAL;

    const char* ps3_path = (const char*)vm_to_host(path_addr);
    char host_path[1024];
    sys_fs_translate_path(ps3_path, host_path, sizeof(host_path));

    int slot = -1;
    for (int i = 0; i < SYS_FS_DIR_MAX; i++) {
        if (!g_sys_fs_dirs[i].active) { slot = i; break; }
    }
    if (slot < 0)
        return (int64_t)(int32_t)CELL_ENOMEM;

    sys_fs_dir_info* d = &g_sys_fs_dirs[slot];
    memset(d, 0, sizeof(*d));

#ifdef _WIN32
    char search_path[1024];
    snprintf(search_path, sizeof(search_path), "%s\\*", host_path);
    d->find_handle = FindFirstFileA(search_path, &d->find_data);
    if (d->find_handle == INVALID_HANDLE_VALUE)
        return (int64_t)(int32_t)CELL_ENOENT;
    d->first_read = 1;
#else
    d->dp = opendir(host_path);
    if (!d->dp)
        return (int64_t)(int32_t)CELL_ENOENT;
#endif

    d->active = 1;
    strncpy(d->path, host_path, sizeof(d->path) - 1);

    int32_t dir_fd = slot + 1;
    write_be32(fd_out, (uint32_t)dir_fd);

    return CELL_OK;
}

/* Next directory entry. Returns 1 and fills name/is_dir, or 0 at EOF. */
static int sys_fs_dir_next(sys_fs_dir_info* d, const char** name, int* is_dir)
{
#ifdef _WIN32
    if (d->first_read) {
        d->first_read = 0;
        *name = d->find_data.cFileName;
        *is_dir = (d->find_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) ? 1 : 0;
        return 1;
    }
    if (!FindNextFileA(d->find_handle, &d->find_data))
        return 0;
    *name = d->find_data.cFileName;
    *is_dir = (d->find_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) ? 1 : 0;
    return 1;
#else
    struct dirent* entry = readdir(d->dp);
    if (!entry)
        return 0;
    *name = entry->d_name;
    *is_dir = (entry->d_type == DT_DIR) ? 1 : 0;
    return 1;
#endif
}

/* ---------------------------------------------------------------------------
 * sys_fs_readdir
 *
 * r3 = dir_fd
 * r4 = pointer to CellFsDirent (guest memory)
 *      struct { u8 d_type; u8 d_namlen; char d_name[256]; }
 * r5 = pointer to receive bytes read (u64*), 0 = end
 * -----------------------------------------------------------------------*/
int64_t sys_fs_readdir(ppu_context* ctx)
{
    int32_t  dir_fd     = LV2_ARG_S32(ctx, 0);
    uint32_t dirent_addr = LV2_ARG_PTR(ctx, 1);
    uint32_t nread_addr  = LV2_ARG_PTR(ctx, 2);

    if (dir_fd <= 0 || dir_fd > SYS_FS_DIR_MAX)
        return (int64_t)(int32_t)CELL_EBADF;

    sys_fs_dir_info* d = &g_sys_fs_dirs[dir_fd - 1];
    if (!d->active)
        return (int64_t)(int32_t)CELL_EBADF;

    const char* name = NULL;
    int is_dir = 0;
    if (!sys_fs_dir_next(d, &name, &is_dir)) {
        if (nread_addr != 0) write_be64(nread_addr, 0);
        return CELL_OK;
    }

    if (dirent_addr != 0 && name != NULL) {
        uint8_t* out = (uint8_t*)vm_to_host(dirent_addr);
        uint8_t namlen = (uint8_t)strlen(name);
        if (namlen > 255) namlen = 255;

        out[0] = is_dir ? 1 : 2;  /* d_type: 1=dir, 2=regular */
        out[1] = namlen;
        memcpy(out + 2, name, namlen);
        out[2 + namlen] = '\0';
    }

    if (nread_addr != 0) {
        /* Non-zero means success */
        write_be64(nread_addr, (uint64_t)(name ? strlen(name) + 2 : 0));
    }

    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_closedir
 *
 * r3 = dir_fd
 * -----------------------------------------------------------------------*/
int64_t sys_fs_closedir(ppu_context* ctx)
{
    int32_t dir_fd = LV2_ARG_S32(ctx, 0);

    if (dir_fd <= 0 || dir_fd > SYS_FS_DIR_MAX)
        return (int64_t)(int32_t)CELL_EBADF;

    sys_fs_dir_info* d = &g_sys_fs_dirs[dir_fd - 1];
    if (!d->active)
        return (int64_t)(int32_t)CELL_EBADF;

#ifdef _WIN32
    if (d->find_handle != INVALID_HANDLE_VALUE)
        FindClose(d->find_handle);
#else
    if (d->dp) closedir(d->dp);
#endif

    d->active = 0;
    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_mkdir
 *
 * r3 = path (guest pointer)
 * r4 = mode
 * -----------------------------------------------------------------------*/
int64_t sys_fs_mkdir(ppu_context* ctx)
{
    uint32_t path_addr = LV2_ARG_PTR(ctx, 0);
    /* uint32_t mode   = LV2_ARG_U32(ctx, 1); */

    if (path_addr == 0)
        return (int64_t)(int32_t)CELL_EINVAL;

    const char* ps3_path = (const char*)vm_to_host(path_addr);
    char host_path[1024];
    sys_fs_translate_path(ps3_path, host_path, sizeof(host_path));

#ifdef _WIN32
    int rc = _mkdir(host_path);
#else
    int rc = mkdir(host_path, 0755);
#endif

    if (rc != 0) {
        /* Check if it already exists */
        struct stat st;
#ifdef _WIN32
        if (_stat64(host_path, (struct _stat64*)&st) == 0 && S_ISDIR(st.st_mode))
#else
        if (stat(host_path, &st) == 0 && S_ISDIR(st.st_mode))
#endif
            return (int64_t)(int32_t)CELL_EEXIST;
        return (int64_t)(int32_t)CELL_ENOENT;
    }

    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_rename
 *
 * r3 = old path (guest pointer)
 * r4 = new path (guest pointer)
 * -----------------------------------------------------------------------*/
int64_t sys_fs_rename(ppu_context* ctx)
{
    uint32_t old_addr = LV2_ARG_PTR(ctx, 0);
    uint32_t new_addr = LV2_ARG_PTR(ctx, 1);

    if (old_addr == 0 || new_addr == 0)
        return (int64_t)(int32_t)CELL_EINVAL;

    const char* old_ps3 = (const char*)vm_to_host(old_addr);
    const char* new_ps3 = (const char*)vm_to_host(new_addr);

    char old_host[1024], new_host[1024];
    sys_fs_translate_path(old_ps3, old_host, sizeof(old_host));
    sys_fs_translate_path(new_ps3, new_host, sizeof(new_host));

    if (rename(old_host, new_host) != 0)
        return (int64_t)(int32_t)CELL_ENOENT;

    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_unlink
 *
 * r3 = path (guest pointer)
 * -----------------------------------------------------------------------*/
int64_t sys_fs_unlink(ppu_context* ctx)
{
    uint32_t path_addr = LV2_ARG_PTR(ctx, 0);

    if (path_addr == 0)
        return (int64_t)(int32_t)CELL_EINVAL;

    const char* ps3_path = (const char*)vm_to_host(path_addr);
    char host_path[1024];
    sys_fs_translate_path(ps3_path, host_path, sizeof(host_path));

#ifdef _WIN32
    if (_unlink(host_path) != 0)
#else
    if (unlink(host_path) != 0)
#endif
        return (int64_t)(int32_t)CELL_ENOENT;

    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_rmdir
 *
 * r3 = path (guest pointer)
 * -----------------------------------------------------------------------*/
int64_t sys_fs_rmdir(ppu_context* ctx)
{
    uint32_t path_addr = LV2_ARG_PTR(ctx, 0);

    if (path_addr == 0)
        return (int64_t)(int32_t)CELL_EINVAL;

    const char* ps3_path = (const char*)vm_to_host(path_addr);
    char host_path[1024];
    sys_fs_translate_path(ps3_path, host_path, sizeof(host_path));

#ifdef _WIN32
    if (_rmdir(host_path) != 0)
#else
    if (rmdir(host_path) != 0)
#endif
        return (int64_t)(int32_t)CELL_ENOENT;

    return CELL_OK;
}

static int64_t sys_fs_truncate_host(const char* host_path, uint64_t size)
{
#ifdef _WIN32
    FILE* f = fopen(host_path, "rb+");
    if (!f)
        return (int64_t)(int32_t)CELL_ENOENT;
    int fno = _fileno(f);
    int rc = _chsize_s(fno, (long long)size);
    fclose(f);
    return rc == 0 ? CELL_OK : (int64_t)(int32_t)CELL_EINVAL;
#else
    if (truncate(host_path, (off_t)size) != 0)
        return (int64_t)(int32_t)(errno == ENOENT ? CELL_ENOENT : CELL_EINVAL);
    return CELL_OK;
#endif
}

static sys_fs_fd_info* sys_fs_file_from_fd(int32_t fd)
{
    int slot = fd - 3;
    if (slot < 0 || slot >= SYS_FS_FD_MAX)
        return NULL;
    sys_fs_fd_info* f = &g_sys_fs_fds[slot];
    if (!f->active || !f->fp)
        return NULL;
    return f;
}

static sys_fs_dir_info* sys_fs_dir_from_fd(int32_t dir_fd)
{
    if (dir_fd <= 0 || dir_fd > SYS_FS_DIR_MAX)
        return NULL;
    sys_fs_dir_info* d = &g_sys_fs_dirs[dir_fd - 1];
    if (!d->active)
        return NULL;
    return d;
}

/* CellFsDirectoryEntry: CellFsStat (52) + CellFsDirent (258) = 310. */
#define SYS_FS_DIRECTORY_ENTRY_SIZE  310u

static int64_t sys_fs_fcntl_getdirents(int32_t fd, uint32_t arg, uint32_t size)
{
    /* dir_info: _code@0 _size@4 ptr@8 max@12 (RPCS3 lv2_file_op_dir::dir_info). */
    if (size < 0x10 || arg == 0)
        return (int64_t)(int32_t)CELL_EINVAL;

    sys_fs_dir_info* d = sys_fs_dir_from_fd(fd);
    if (!d)
        return (int64_t)(int32_t)CELL_EBADF;

    uint32_t max = read_be32(arg + 12);
    uint32_t ptr = read_be32(arg + 8);
    uint32_t wrote = 0;

    if (max && ptr) {
        const char* name = NULL;
        int is_dir = 0;
        if (sys_fs_dir_next(d, &name, &is_dir) && name) {
            uint32_t ent = ptr;
            char child[1024];
            snprintf(child, sizeof child, "%s/%s", d->path, name);
            struct stat st;
            memset(&st, 0, sizeof st);
#ifdef _WIN32
            if (_stat64(child, (struct _stat64*)&st) != 0) {
                st.st_mode = is_dir ? _S_IFDIR : _S_IFREG;
            }
#else
            if (stat(child, &st) != 0) {
                st.st_mode = is_dir ? S_IFDIR : S_IFREG;
            }
#endif
            fill_cell_stat(ent, &st);
            uint8_t* de = (uint8_t*)vm_to_host(ent + 52);
            uint8_t namlen = (uint8_t)strlen(name);
            if (namlen > 255) namlen = 255;
            de[0] = is_dir ? 1 : 2;
            de[1] = namlen;
            memcpy(de + 2, name, namlen);
            de[2 + namlen] = '\0';
            wrote = 1;
        }
        if (max > wrote)
            memset(vm_to_host(ptr + wrote * SYS_FS_DIRECTORY_ENTRY_SIZE), 0,
                   (max - wrote) * SYS_FS_DIRECTORY_ENTRY_SIZE);
    }

    write_be32(arg + 0, CELL_OK);
    write_be32(arg + 4, wrote);
    return CELL_OK;
}

static int64_t sys_fs_fcntl_rw_offset(int32_t fd, uint32_t op, uint32_t arg, uint32_t size)
{
    /* lv2_file_op_rw 0x38: buf@0x14 offset@0x18 size@0x20 out_code@0x28 out_size@0x30 */
    if (size < 0x38 || arg == 0)
        return (int64_t)(int32_t)CELL_EINVAL;

    sys_fs_fd_info* f = sys_fs_file_from_fd(fd);
    if (!f)
        return (int64_t)(int32_t)CELL_EBADF;

    uint32_t buf_addr = read_be32(arg + 0x14);
    uint64_t offset   = read_be64(arg + 0x18);
    uint64_t nbytes   = read_be64(arg + 0x20);
    void* buf = buf_addr ? vm_to_host(buf_addr) : NULL;
    if (nbytes && !buf)
        return (int64_t)(int32_t)CELL_EFAULT;

#ifdef _WIN32
    int64_t old_pos = _ftelli64(f->fp);
    if (_fseeki64(f->fp, (int64_t)offset, SEEK_SET) != 0)
        return (int64_t)(int32_t)CELL_EINVAL;
#else
    int64_t old_pos = (int64_t)ftello(f->fp);
    if (fseeko(f->fp, (off_t)offset, SEEK_SET) != 0)
        return (int64_t)(int32_t)CELL_EINVAL;
#endif

    size_t done = 0;
    if (nbytes && buf) {
        if (op == 0x8000000au)
            done = fread(buf, 1, (size_t)nbytes, f->fp);
        else
            done = fwrite(buf, 1, (size_t)nbytes, f->fp);
    }

#ifdef _WIN32
    _fseeki64(f->fp, old_pos, SEEK_SET);
#else
    fseeko(f->fp, (off_t)old_pos, SEEK_SET);
#endif

    write_be32(arg + 0x28, CELL_OK);
    write_be64(arg + 0x30, (uint64_t)done);
    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_link  (810)
 * r3 = from, r4 = to
 * -----------------------------------------------------------------------*/
int64_t sys_fs_link(ppu_context* ctx)
{
    uint32_t from_addr = LV2_ARG_PTR(ctx, 0);
    uint32_t to_addr   = LV2_ARG_PTR(ctx, 1);
    if (!from_addr || !to_addr)
        return (int64_t)(int32_t)CELL_EFAULT;

    char from_host[1024], to_host[1024];
    sys_fs_translate_path((const char*)vm_to_host(from_addr), from_host, sizeof from_host);
    sys_fs_translate_path((const char*)vm_to_host(to_addr), to_host, sizeof to_host);

#ifdef _WIN32
    if (!CreateHardLinkA(to_host, from_host, NULL))
        return (int64_t)(int32_t)CELL_ENOENT;
#else
    if (link(from_host, to_host) != 0)
        return (int64_t)(int32_t)CELL_ENOENT;
#endif
    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_utime  (815)
 * r3 = path, r4 = CellFsUtimbuf* { s64 actime; s64 modtime; } packed 16 bytes
 * -----------------------------------------------------------------------*/
int64_t sys_fs_utime(ppu_context* ctx)
{
    uint32_t path_addr = LV2_ARG_PTR(ctx, 0);
    uint32_t tbuf_addr = LV2_ARG_PTR(ctx, 1);
    if (!path_addr)
        return (int64_t)(int32_t)CELL_EFAULT;

    char host_path[1024];
    sys_fs_translate_path((const char*)vm_to_host(path_addr), host_path, sizeof host_path);

#ifdef _WIN32
    struct __utimbuf64 ut;
    if (tbuf_addr) {
        ut.actime  = (time_t)read_be64(tbuf_addr);
        ut.modtime = (time_t)read_be64(tbuf_addr + 8);
    } else {
        time_t now = time(NULL);
        ut.actime = ut.modtime = now;
    }
    if (_utime64(host_path, tbuf_addr ? &ut : NULL) != 0)
        return (int64_t)(int32_t)CELL_ENOENT;
#else
    struct utimbuf ut;
    if (tbuf_addr) {
        ut.actime  = (time_t)read_be64(tbuf_addr);
        ut.modtime = (time_t)read_be64(tbuf_addr + 8);
    } else {
        time_t now = time(NULL);
        ut.actime = ut.modtime = now;
    }
    if (utime(host_path, tbuf_addr ? &ut : NULL) != 0)
        return (int64_t)(int32_t)CELL_ENOENT;
#endif
    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_access  (816)
 *
 * r3 = path, r4 = mode
 * ABI: CellFsErrno sys_fs_access(const char *path, s32 mode)
 *   mode is POSIX F_OK=0 / X_OK=1 / W_OK=2 / R_OK=4 (cellFsAccess), and
 *   also accepts CELL_FS_S_I{R,W,X}USR. No ByFd form — 816 is path-only.
 * -----------------------------------------------------------------------*/
int64_t sys_fs_access(ppu_context* ctx)
{
    uint32_t path_addr = LV2_ARG_PTR(ctx, 0);
    int32_t  mode      = LV2_ARG_S32(ctx, 1);
    if (!path_addr)
        return (int64_t)(int32_t)CELL_EFAULT;

    const char* ps3_path = (const char*)vm_to_host(path_addr);
    if (!ps3_path || !ps3_path[0])
        return (int64_t)(int32_t)CELL_EINVAL;

    char host_path[1024];
    sys_fs_translate_path(ps3_path, host_path, sizeof host_path);

    int host_mode = 0; /* F_OK */
    if (mode & (CELL_FS_R_OK | CELL_FS_S_IRUSR | CELL_FS_S_IRGRP))
        host_mode |= 4;
    if (mode & (CELL_FS_W_OK | CELL_FS_S_IWUSR))
        host_mode |= 2;

#ifdef _WIN32
    if (_access(host_path, host_mode) != 0)
        return (int64_t)(int32_t)(errno == ENOENT ? CELL_ENOENT : CELL_EACCES);
#else
    int posix_mode = F_OK;
    if (host_mode & 4)
        posix_mode |= R_OK;
    if (host_mode & 2)
        posix_mode |= W_OK;
    if (mode & (CELL_FS_X_OK | CELL_FS_S_IXUSR))
        posix_mode |= X_OK;
    if (access(host_path, posix_mode) != 0) {
        if (errno == ENOENT) return (int64_t)(int32_t)CELL_ENOENT;
        if (errno == EACCES) return (int64_t)(int32_t)CELL_EACCES;
        return (int64_t)(int32_t)CELL_EINVAL;
    }
#endif
    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_fcntl  (817)
 *
 * r3 = fd, r4 = op, r5 = arg, r6 = size
 * GetDirectoryEntries is 0xe0000012; Read/WriteWithOffset 0x8000000a/0b.
 * -----------------------------------------------------------------------*/
int64_t sys_fs_fcntl(ppu_context* ctx)
{
    int32_t  fd   = LV2_ARG_S32(ctx, 0);
    uint32_t op   = LV2_ARG_U32(ctx, 1);
    uint32_t arg  = LV2_ARG_PTR(ctx, 2);
    uint32_t size = LV2_ARG_U32(ctx, 3);

    switch (op) {
    case 0x8000000au: /* cellFsReadWithOffset */
    case 0x8000000bu: /* cellFsWriteWithOffset */
        return sys_fs_fcntl_rw_offset(fd, op, arg, size);

    case 0xe0000012u: /* cellFsGetDirectoryEntries */
        return sys_fs_fcntl_getdirents(fd, arg, size);

    case 0xe0000017u: { /* cellFsAllocateFileAreaWithoutZeroFill */
        /* size@0 _x4@4=0x10 _x8@8=0x20 path@0x10 filesize@0x18 out_code@0x20 */
        if (size < 0x28 || arg == 0)
            return (int64_t)(int32_t)CELL_EINVAL;
        if (read_be32(arg + 4) != 0x10u || read_be32(arg + 8) != 0x20u)
            return (int64_t)(int32_t)CELL_EINVAL;
        uint32_t path_addr = read_be32(arg + 0x10);
        uint64_t file_size = read_be64(arg + 0x18);
        if (!path_addr)
            return (int64_t)(int32_t)CELL_EFAULT;
        char host_path[1024];
        sys_fs_translate_path((const char*)vm_to_host(path_addr), host_path, sizeof host_path);
        int64_t rc = sys_fs_truncate_host(host_path, file_size);
        write_be32(arg + 0x20, (uint32_t)rc);
        return CELL_OK;
    }

    case 0xc0000002u: { /* cellFsGetFreeSize (hdd0 fcntl form) */
        if (size < 0x28 || arg == 0)
            return (int64_t)(int32_t)CELL_EINVAL;
        uint32_t path_addr = read_be32(arg + 0x0C);
        char host_path[1024];
        host_path[0] = 0;
        if (path_addr) {
            const char* p = (const char*)vm_to_host(path_addr);
            if (p && p[0])
                sys_fs_translate_path(p, host_path, sizeof host_path);
        }
        uint64_t freeb = sys_fs_host_total_bytes(host_path[0] ? host_path : NULL);
        /* Prefer actual free space when the volume query succeeded. */
#ifdef _WIN32
        {
            ULARGE_INTEGER avail, total;
            const char* q = host_path[0] ? host_path : (g_sys_fs_root[0] ? g_sys_fs_root : ".");
            if (GetDiskFreeSpaceExA(q, &avail, &total, NULL) && avail.QuadPart)
                freeb = (uint64_t)avail.QuadPart;
        }
#endif
        if (freeb < 8ull * 1024ull * 1024ull * 1024ull)
            freeb = 8ull * 1024ull * 1024ull * 1024ull;
        write_be32(arg + 0x18, CELL_OK);
        write_be32(arg + 0x1C, 4096u);
        write_be64(arg + 0x20, freeb / 4096ull);
        return CELL_OK;
    }

    case 0x80000004u: /* unknown: write 0 */
        if (size > 4)
            return (int64_t)(int32_t)CELL_EINVAL;
        if (arg && size >= 4)
            write_be32(arg, 0);
        return CELL_OK;

    case 0xc0000006u: /* mount probe: out_code@0x18 = ENOTSUP, out_id@0x1c = 0 */
        if (arg && size >= 0x20) {
            write_be32(arg + 0x18, (uint32_t)CELL_ENOSYS);
            write_be32(arg + 0x1C, 0);
        }
        return CELL_OK;

    case 0xc0000008u: /* SetIoBuffer / SetDefaultContainer */
        if (arg && size >= 0x28)
            write_be32(arg + 0x20, CELL_OK);
        return CELL_OK;

    default:
        fprintf(stderr, "[sys_fs] fcntl fd=%d op=0x%X size=%u (ok stub)\n",
                fd, op, size);
        return CELL_OK;
    }
}

/* ---------------------------------------------------------------------------
 * sys_fs_fsync  (819 fdatasync / 820 fsync)
 * r3 = fd
 * -----------------------------------------------------------------------*/
int64_t sys_fs_fsync(ppu_context* ctx)
{
    int32_t fd = LV2_ARG_S32(ctx, 0);
    sys_fs_fd_info* f = sys_fs_file_from_fd(fd);
    if (!f)
        return (int64_t)(int32_t)CELL_EBADF;

    fflush(f->fp);
#ifdef _WIN32
    int hfd = _fileno(f->fp);
    if (hfd >= 0)
        _commit(hfd);
#else
    int hfd = fileno(f->fp);
    if (hfd >= 0)
        fsync(hfd);
#endif
    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_truncate  (831)
 * r3 = path, r4 = size
 * -----------------------------------------------------------------------*/
int64_t sys_fs_truncate(ppu_context* ctx)
{
    uint32_t path_addr = LV2_ARG_PTR(ctx, 0);
    uint64_t size      = LV2_ARG_U64(ctx, 1);
    if (!path_addr)
        return (int64_t)(int32_t)CELL_EFAULT;

    const char* ps3_path = (const char*)vm_to_host(path_addr);
    if (!ps3_path || !ps3_path[0])
        return (int64_t)(int32_t)CELL_EINVAL;

    char host_path[1024];
    sys_fs_translate_path(ps3_path, host_path, sizeof host_path);
    return sys_fs_truncate_host(host_path, size);
}

/* ---------------------------------------------------------------------------
 * sys_fs_symlink  (833)
 *
 * r3 = target, r4 = linkpath
 * ABI: CellFsErrno sys_fs_symbolic_link(const char *target, const char *linkpath)
 *   POSIX order (same as symlink(2) / cellFsSymbolicLink). No ByFd form.
 * -----------------------------------------------------------------------*/
int64_t sys_fs_symlink(ppu_context* ctx)
{
    uint32_t target_addr = LV2_ARG_PTR(ctx, 0);
    uint32_t link_addr   = LV2_ARG_PTR(ctx, 1);
    if (!target_addr || !link_addr)
        return (int64_t)(int32_t)CELL_EFAULT;

    const char* target_ps3 = (const char*)vm_to_host(target_addr);
    const char* link_ps3   = (const char*)vm_to_host(link_addr);
    if (!target_ps3 || !target_ps3[0] || !link_ps3 || !link_ps3[0])
        return (int64_t)(int32_t)CELL_EINVAL;

    char target_host[1024], link_host[1024];
    sys_fs_translate_path(target_ps3, target_host, sizeof target_host);
    sys_fs_translate_path(link_ps3, link_host, sizeof link_host);

#ifdef _WIN32
#ifndef SYMBOLIC_LINK_FLAG_DIRECTORY
#define SYMBOLIC_LINK_FLAG_DIRECTORY 0x1
#endif
#ifndef SYMBOLIC_LINK_FLAG_ALLOW_UNPRIVILEGED_CREATE
#define SYMBOLIC_LINK_FLAG_ALLOW_UNPRIVILEGED_CREATE 0x2
#endif
    DWORD flags = SYMBOLIC_LINK_FLAG_ALLOW_UNPRIVILEGED_CREATE;
    DWORD attr = GetFileAttributesA(target_host);
    if (attr != INVALID_FILE_ATTRIBUTES && (attr & FILE_ATTRIBUTE_DIRECTORY))
        flags |= SYMBOLIC_LINK_FLAG_DIRECTORY;
    if (!CreateSymbolicLinkA(link_host, target_host, flags)) {
        DWORD err = GetLastError();
        if (err == ERROR_ALREADY_EXISTS)
            return (int64_t)(int32_t)CELL_EEXIST;
        if (err == ERROR_FILE_NOT_FOUND || err == ERROR_PATH_NOT_FOUND)
            return (int64_t)(int32_t)CELL_ENOENT;
        if (err == ERROR_PRIVILEGE_NOT_HELD || err == ERROR_ACCESS_DENIED)
            return (int64_t)(int32_t)CELL_EPERM;
        return (int64_t)(int32_t)CELL_EACCES;
    }
#else
    if (symlink(target_host, link_host) != 0) {
        if (errno == EEXIST) return (int64_t)(int32_t)CELL_EEXIST;
        if (errno == ENOENT) return (int64_t)(int32_t)CELL_ENOENT;
        if (errno == EACCES || errno == EPERM) return (int64_t)(int32_t)CELL_EPERM;
        return (int64_t)(int32_t)CELL_EACCES;
    }
#endif
    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_chmod  (834)
 * r3 = path, r4 = mode
 * -----------------------------------------------------------------------*/
int64_t sys_fs_chmod(ppu_context* ctx)
{
    uint32_t path_addr = LV2_ARG_PTR(ctx, 0);
    int32_t  mode      = LV2_ARG_S32(ctx, 1);
    if (!path_addr)
        return (int64_t)(int32_t)CELL_EFAULT;

    char host_path[1024];
    sys_fs_translate_path((const char*)vm_to_host(path_addr), host_path, sizeof host_path);

    struct stat st;
#ifdef _WIN32
    if (_stat64(host_path, (struct _stat64*)&st) != 0)
        return (int64_t)(int32_t)CELL_ENOENT;
    (void)mode;
    _chmod(host_path, _S_IREAD | _S_IWRITE);
#else
    if (stat(host_path, &st) != 0)
        return (int64_t)(int32_t)CELL_ENOENT;
    if (chmod(host_path, (mode_t)mode) != 0)
        return (int64_t)(int32_t)CELL_EACCES;
#endif
    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_chown  (835)
 *
 * r3 = path, r4 = uid, r5 = gid
 * ABI: CellFsErrno sys_fs_chown(const char *path, s32 uid, s32 gid)
 *   cellFsChown; no ByFd. Host NTFS has no POSIX uid/gid — exist-check only.
 * -----------------------------------------------------------------------*/
int64_t sys_fs_chown(ppu_context* ctx)
{
    uint32_t path_addr = LV2_ARG_PTR(ctx, 0);
    int32_t  uid       = LV2_ARG_S32(ctx, 1);
    int32_t  gid       = LV2_ARG_S32(ctx, 2);
    if (!path_addr)
        return (int64_t)(int32_t)CELL_EFAULT;

    const char* ps3_path = (const char*)vm_to_host(path_addr);
    if (!ps3_path || !ps3_path[0])
        return (int64_t)(int32_t)CELL_EINVAL;

    char host_path[1024];
    sys_fs_translate_path(ps3_path, host_path, sizeof host_path);

#ifdef _WIN32
    struct _stat64 st;
    if (_stat64(host_path, &st) != 0)
        return (int64_t)(int32_t)CELL_ENOENT;
    (void)uid;
    (void)gid;
#else
    if (chown(host_path, (uid_t)uid, (gid_t)gid) != 0) {
        if (errno == ENOENT) return (int64_t)(int32_t)CELL_ENOENT;
        if (errno == EACCES || errno == EPERM) return (int64_t)(int32_t)CELL_EPERM;
        return (int64_t)(int32_t)CELL_EACCES;
    }
#endif
    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_ftruncate
 *
 * r3 = fd
 * r4 = size
 * -----------------------------------------------------------------------*/
int64_t sys_fs_ftruncate(ppu_context* ctx)
{
    int32_t  fd   = LV2_ARG_S32(ctx, 0);
    uint64_t size = LV2_ARG_U64(ctx, 1);

    int slot = fd - 3;
    if (slot < 0 || slot >= SYS_FS_FD_MAX)
        return (int64_t)(int32_t)CELL_EBADF;

    sys_fs_fd_info* f = &g_sys_fs_fds[slot];
    if (!f->active || !f->fp)
        return (int64_t)(int32_t)CELL_EBADF;

    fflush(f->fp);

#ifdef _WIN32
    int fno = _fileno(f->fp);
    if (_chsize_s(fno, (long long)size) != 0)
        return (int64_t)(int32_t)CELL_EINVAL;
#else
    int fno = fileno(f->fp);
    if (ftruncate(fno, (off_t)size) != 0)
        return (int64_t)(int32_t)CELL_EINVAL;
#endif

    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * Block / sector size (lv2 840 / 841)
 *
 * RPCS3 / SDK layout (cellFsGetBlockSize, cellFsFGetBlockSize, *2 variants):
 *   sys_fs_fget_block_size(fd, u64* sector_size, u64* block_size,
 *                          u64* arg4, s32* out_flags)
 *   sys_fs_get_block_size(path, u64* sector_size, u64* block_size, u64* arg4)
 *
 * Hardware writes sector_size, block_size, and arg4 (RPCS3 copies sector_size
 * into arg4). Returning CELL_OK with zeros makes FIOS treat the volume as
 * empty / 512-byte I/O. st_blksize and GetFreeSize in this port are 4096.
 * -----------------------------------------------------------------------*/
#define SYS_FS_SECTOR_SIZE  512ull
#define SYS_FS_BLOCK_SIZE   4096ull

static uint64_t sys_fs_host_total_bytes(const char* host_path)
{
    char path[1024];
    if (!host_path || !host_path[0])
        host_path = g_sys_fs_root[0] ? g_sys_fs_root : ".";
    snprintf(path, sizeof path, "%s", host_path);
#ifdef _WIN32
    for (char* p = path; *p; p++)
        if (*p == '/') *p = '\\';
#endif
    for (;;) {
#ifdef _WIN32
        ULARGE_INTEGER avail, total;
        if (GetDiskFreeSpaceExA(path, &avail, &total, NULL) && total.QuadPart)
            return (uint64_t)total.QuadPart;
#else
        struct statvfs info;
        if (statvfs(path, &info) == 0) {
            uint64_t fr = info.f_frsize ? (uint64_t)info.f_frsize : (uint64_t)info.f_bsize;
            uint64_t n = fr * (uint64_t)info.f_blocks;
            if (n)
                return n;
        }
#endif
        size_t len = strlen(path);
        while (len > 1 && (path[len - 1] == '/' || path[len - 1] == '\\'))
            path[--len] = '\0';
        char* slash = strrchr(path, '/');
#ifdef _WIN32
        char* bslash = strrchr(path, '\\');
        if (bslash && (!slash || bslash > slash))
            slash = bslash;
        if (len == 3 && path[1] == ':' && (path[2] == '\\' || path[2] == '/'))
            break;
        if (len == 2 && path[1] == ':')
            break;
#endif
        if (!slash)
            break;
        if (slash == path) {
            slash[1] = '\0';
            continue;
        }
#ifdef _WIN32
        if (slash == path + 2 && path[1] == ':') {
            slash[1] = '\0';
            continue;
        }
#endif
        *slash = '\0';
    }
    return 8ull * 1024ull * 1024ull * 1024ull;
}

static void sys_fs_write_block_outs(uint32_t sector_addr, uint32_t block_addr,
                                    uint32_t arg4_addr, const char* host_path)
{
    const uint64_t sector_size = SYS_FS_SECTOR_SIZE;
    const uint64_t block_size  = SYS_FS_BLOCK_SIZE;
    uint64_t total = sys_fs_host_total_bytes(host_path);
    if (total < 8ull * 1024ull * 1024ull * 1024ull)
        total = 8ull * 1024ull * 1024ull * 1024ull;
    /* arg4 is unnamed in the SDK; RPCS3 writes sector_size. Also expose a
     * non-zero sector count so callers that treat the 4th word as a count
     * (not CELL_OK + zeros) see a volume that can hold a FIOS overlay. */
    uint64_t sector_count = total / sector_size;
    if (sector_count < (8ull * 1024ull * 1024ull * 1024ull) / sector_size)
        sector_count = (8ull * 1024ull * 1024ull * 1024ull) / sector_size;

    if (sector_addr)
        write_be64(sector_addr, sector_size);
    if (block_addr)
        write_be64(block_addr, block_size);
    if (arg4_addr)
        write_be64(arg4_addr, sector_count);
}

/* ---------------------------------------------------------------------------
 * sys_fs_fget_block_size  (840)
 *
 * r3 = fd
 * r4 = u64* sector_size
 * r5 = u64* block_size
 * r6 = u64* arg4          (sector count; RPCS3 copies sector_size)
 * r7 = s32* out_flags     (open flags)
 * -----------------------------------------------------------------------*/
int64_t sys_fs_fget_block_size(ppu_context* ctx)
{
    int32_t  fd          = LV2_ARG_S32(ctx, 0);
    uint32_t sector_addr = LV2_ARG_PTR(ctx, 1);
    uint32_t block_addr  = LV2_ARG_PTR(ctx, 2);
    uint32_t arg4_addr   = LV2_ARG_PTR(ctx, 3);
    uint32_t flags_addr  = LV2_ARG_PTR(ctx, 4);

    int slot = fd - 3;
    if (slot < 0 || slot >= SYS_FS_FD_MAX)
        return (int64_t)(int32_t)CELL_EBADF;

    sys_fs_fd_info* f = &g_sys_fs_fds[slot];
    if (!f->active || !f->fp)
        return (int64_t)(int32_t)CELL_EBADF;

    sys_fs_write_block_outs(sector_addr, block_addr, arg4_addr, f->path);
    if (flags_addr)
        write_be32(flags_addr, (uint32_t)f->flags);

    fprintf(stderr, "[sys_fs] fget_block_size fd=%d '%s' sector=%llu block=%llu flags=%d\n",
            fd, f->path[0] ? f->path : "?",
            (unsigned long long)SYS_FS_SECTOR_SIZE,
            (unsigned long long)SYS_FS_BLOCK_SIZE,
            f->flags);
    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_get_block_size  (841)
 *
 * r3 = path (guest pointer)
 * r4 = u64* sector_size
 * r5 = u64* block_size
 * r6 = u64* arg4
 * -----------------------------------------------------------------------*/
int64_t sys_fs_get_block_size(ppu_context* ctx)
{
    uint32_t path_addr   = LV2_ARG_PTR(ctx, 0);
    uint32_t sector_addr = LV2_ARG_PTR(ctx, 1);
    uint32_t block_addr  = LV2_ARG_PTR(ctx, 2);
    uint32_t arg4_addr   = LV2_ARG_PTR(ctx, 3);

    if (path_addr == 0)
        return (int64_t)(int32_t)CELL_EFAULT;

    const char* ps3_path = (const char*)vm_to_host(path_addr);
    if (!ps3_path || !ps3_path[0])
        return (int64_t)(int32_t)CELL_EINVAL;

    char host_path[1024];
    sys_fs_translate_path(ps3_path, host_path, sizeof(host_path));

    /* /dev_hdd1 is the syscache mount. Ensure the host dir exists so volume
     * stats walk a real tree; do not mkdir a file path (cache.idx). */
    if (strcmp(ps3_path, "/dev_hdd1") == 0 || strcmp(ps3_path, "/dev_hdd1/") == 0)
        sys_fs_mkdir_p(host_path);

    sys_fs_write_block_outs(sector_addr, block_addr, arg4_addr, host_path);

    fprintf(stderr, "[sys_fs] get_block_size '%s' -> '%s' sector=%llu block=%llu\n",
            ps3_path, host_path,
            (unsigned long long)SYS_FS_SECTOR_SIZE,
            (unsigned long long)SYS_FS_BLOCK_SIZE);
    return CELL_OK;
}

/* ---------------------------------------------------------------------------
 * sys_fs_mapped_allocate  (845) / sys_fs_mapped_free  (846)
 *
 * ABI (SDK / RPCS3 0x34D / 0x34E — not remapped; 840/841 stay GetBlockSize):
 *   CellFsErrno sys_fs_mapped_allocate(s32 fd, u64 size, void **out_ptr)
 *   CellFsErrno sys_fs_mapped_free(s32 fd, void *ptr)
 * Copies `size` bytes from the current file position into a guest window
 * (does not move the fd offset). ACIT libfs does not import these.
 * -----------------------------------------------------------------------*/
#define SYS_FS_MAP_MAX        16
#define SYS_FS_MAP_BASE       0x51000000u
#define SYS_FS_MAP_END        0x53000000u
#define SYS_FS_MAP_SIZE_MAX   (64u * 1024u * 1024u)

typedef struct sys_fs_map_info {
    int      active;
    int32_t  fd;
    uint32_t addr;
    uint32_t size;
} sys_fs_map_info;

static sys_fs_map_info g_sys_fs_maps[SYS_FS_MAP_MAX];
static uint32_t        g_sys_fs_map_bump = SYS_FS_MAP_BASE;

int64_t sys_fs_mapped_allocate(ppu_context* ctx)
{
    int32_t  fd      = LV2_ARG_S32(ctx, 0);
    uint64_t size64  = LV2_ARG_U64(ctx, 1);
    uint32_t out_ptr = LV2_ARG_PTR(ctx, 2); /* void ** */

    if (!out_ptr)
        return (int64_t)(int32_t)CELL_EFAULT;
    if (size64 == 0 || size64 > SYS_FS_MAP_SIZE_MAX)
        return (int64_t)(int32_t)CELL_EINVAL;

    sys_fs_fd_info* f = sys_fs_file_from_fd(fd);
    if (!f)
        return (int64_t)(int32_t)CELL_EBADF;

    uint32_t size = (uint32_t)size64;
    uint32_t commit = VM_ALIGN_UP(size, VM_PAGE_SIZE);

    int slot = -1;
    for (int i = 0; i < SYS_FS_MAP_MAX; i++) {
        if (!g_sys_fs_maps[i].active) { slot = i; break; }
    }
    if (slot < 0)
        return (int64_t)(int32_t)CELL_ENOMEM;

    if (g_sys_fs_map_bump + commit > SYS_FS_MAP_END)
        return (int64_t)(int32_t)CELL_ENOMEM;

    uint32_t addr = g_sys_fs_map_bump;
    if (vm_commit(addr, commit) != CELL_OK)
        return (int64_t)(int32_t)CELL_ENOMEM;

    void* host = vm_to_host(addr);
    if (!host)
        return (int64_t)(int32_t)CELL_EFAULT;
    memset(host, 0, commit);

#ifdef _WIN32
    __int64 saved = _ftelli64(f->fp);
#else
    off_t saved = ftello(f->fp);
#endif
    fflush(f->fp);
    size_t got = fread(host, 1, (size_t)size, f->fp);
    (void)got;
#ifdef _WIN32
    if (saved >= 0)
        _fseeki64(f->fp, saved, SEEK_SET);
#else
    if (saved >= 0)
        fseeko(f->fp, saved, SEEK_SET);
#endif

    g_sys_fs_map_bump += commit;
    g_sys_fs_maps[slot].active = 1;
    g_sys_fs_maps[slot].fd     = fd;
    g_sys_fs_maps[slot].addr   = addr;
    g_sys_fs_maps[slot].size   = size;
    write_be32(out_ptr, addr);
    return CELL_OK;
}

int64_t sys_fs_mapped_free(ppu_context* ctx)
{
    int32_t  fd  = LV2_ARG_S32(ctx, 0);
    uint32_t ptr = LV2_ARG_PTR(ctx, 1);
    if (!ptr)
        return (int64_t)(int32_t)CELL_EFAULT;

    for (int i = 0; i < SYS_FS_MAP_MAX; i++) {
        sys_fs_map_info* m = &g_sys_fs_maps[i];
        if (m->active && m->addr == ptr && m->fd == fd) {
            m->active = 0;
            return CELL_OK;
        }
    }
    return (int64_t)(int32_t)CELL_EINVAL;
}

/* ---------------------------------------------------------------------------
 * Registration
 * -----------------------------------------------------------------------*/
void sys_fs_init(lv2_syscall_table* tbl)
{
    memset(g_sys_fs_fds,  0, sizeof(g_sys_fs_fds));
    memset(g_sys_fs_dirs, 0, sizeof(g_sys_fs_dirs));
    memset(g_sys_fs_maps, 0, sizeof(g_sys_fs_maps));
    g_sys_fs_map_bump = SYS_FS_MAP_BASE;

    lv2_syscall_register(tbl, SYS_FS_OPEN,      sys_fs_open);
    lv2_syscall_register(tbl, SYS_FS_READ,       sys_fs_read);
    lv2_syscall_register(tbl, SYS_FS_WRITE,      sys_fs_write);
    lv2_syscall_register(tbl, SYS_FS_CLOSE,      sys_fs_close);
    lv2_syscall_register(tbl, SYS_FS_OPENDIR,    sys_fs_opendir);
    lv2_syscall_register(tbl, SYS_FS_READDIR,    sys_fs_readdir);
    lv2_syscall_register(tbl, SYS_FS_CLOSEDIR,   sys_fs_closedir);
    lv2_syscall_register(tbl, SYS_FS_STAT,       sys_fs_stat);
    lv2_syscall_register(tbl, SYS_FS_FSTAT,      sys_fs_fstat);
    lv2_syscall_register(tbl, SYS_FS_MKDIR,      sys_fs_mkdir);
    lv2_syscall_register(tbl, SYS_FS_RENAME,     sys_fs_rename);
    lv2_syscall_register(tbl, SYS_FS_RMDIR,      sys_fs_rmdir);
    lv2_syscall_register(tbl, SYS_FS_UNLINK,     sys_fs_unlink);
    lv2_syscall_register(tbl, SYS_FS_LINK,       sys_fs_link);
    lv2_syscall_register(tbl, SYS_FS_UTIME,      sys_fs_utime);
    lv2_syscall_register(tbl, SYS_FS_ACCESS,     sys_fs_access);
    lv2_syscall_register(tbl, SYS_FS_FCNTL,      sys_fs_fcntl);
    lv2_syscall_register(tbl, SYS_FS_LSEEK,      sys_fs_lseek);
    lv2_syscall_register(tbl, SYS_FS_FDATASYNC,  sys_fs_fsync);
    lv2_syscall_register(tbl, SYS_FS_FSYNC,      sys_fs_fsync);
    lv2_syscall_register(tbl, SYS_FS_TRUNCATE,   sys_fs_truncate);
    lv2_syscall_register(tbl, SYS_FS_FTRUNCATE,  sys_fs_ftruncate);
    lv2_syscall_register(tbl, SYS_FS_SYMLINK,    sys_fs_symlink);
    lv2_syscall_register(tbl, SYS_FS_CHMOD,      sys_fs_chmod);
    lv2_syscall_register(tbl, SYS_FS_CHOWN,      sys_fs_chown);
    lv2_syscall_register(tbl, SYS_FS_FGET_BLOCK_SIZE, sys_fs_fget_block_size);
    lv2_syscall_register(tbl, SYS_FS_GET_BLOCK_SIZE,  sys_fs_get_block_size);
    lv2_syscall_register(tbl, SYS_FS_MAPPED_ALLOCATE, sys_fs_mapped_allocate);
    lv2_syscall_register(tbl, SYS_FS_MAPPED_FREE,     sys_fs_mapped_free);
}
