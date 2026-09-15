/* clang -std=gnu17 -I include libs/network/tests/test_score_init.c
 *   -Wl,-dead_strip -o /tmp/test_score_init && /tmp/test_score_init
 */
#include "../sceNp.c"
#include <assert.h>
#include <stdlib.h>
uint8_t* vm_base;
uint32_t ppu_vm_size;
int g_resv_store_active;
uint32_t g_ww_lo, g_ww_hi;
void ppu_resv_break_store(uint64_t ea) { (void)ea; }
void ps3_ww_report_inline(uint32_t a, uint64_t v, int w) { (void)a; (void)v; (void)w; }
int spu_coh_is_reserved(uint32_t a) { (void)a; return 0; }
void spu_coh_notify_write(uint32_t a) { (void)a; }
void spu_lockline_lock(void) {}
void spu_lockline_unlock(void) {}
int main(void)
{
    vm_base = calloc(1, 65536);
    assert(vm_base);
    assert((u32)sceNpScoreInit() == SCE_NP_ERROR_NOT_INITIALIZED);
    assert((u32)sceNpScoreTerm() == SCE_NP_COMMUNITY_ERROR_NOT_INITIALIZED);
    assert(sceNpInit(131072, (void*)0x1000) == CELL_OK);
    assert(sceNpScoreInit() == CELL_OK);
    assert((u32)sceNpScoreInit() == SCE_NP_COMMUNITY_ERROR_ALREADY_INITIALIZED);
    /* Local score setup must not claim a PSN connection or change its status. */
    assert(sceNpManagerGetStatus((void*)0x100) == CELL_OK);
    assert((s32)vm_read32(0x100) == SCE_NP_MANAGER_STATUS_OFFLINE);
    assert(sceNpScoreTerm() == CELL_OK);
    assert((u32)sceNpScoreTerm() == SCE_NP_COMMUNITY_ERROR_NOT_INITIALIZED);
    assert(sceNpScoreInit() == CELL_OK);
    assert(sceNpScoreTerm() == CELL_OK);
    /* Uninitialized ticket getters must not touch out-params. */
    assert((u32)sceNpTerm() == CELL_OK);
    vm_write32(0x300, 0xDEADBEEFu);
    assert((u32)sceNpManagerGetTicket((void*)0x400, (void*)0x300)
           == SCE_NP_ERROR_NOT_INITIALIZED);
    assert(vm_read32(0x300) == 0xDEADBEEFu);
    assert(sceNpInit(131072, (void*)0x1000) == CELL_OK);
    vm_write32(0x300, 0xDEADBEEFu);
    assert(sceNpManagerGetTicket(NULL, NULL) == SCE_NP_ERROR_INVALID_ARGUMENT);
    assert(sceNpManagerGetTicket(NULL, (void*)0x300) == CELL_OK);
    assert(vm_read32(0x300) == 0);
    vm_write32(0x300, 0xDEADBEEFu);
    assert(sceNpManagerGetTicket((void*)0x400, (void*)0x300) == CELL_OK);
    assert(vm_read32(0x300) == 0);
    memset(vm_base + 0x500, 0xA5, SCE_NP_TICKET_PARAM_DATA_LEN);
    assert(sceNpManagerGetTicketParam(SCE_NP_TICKET_PARAM_SERIAL_ID, NULL)
           == SCE_NP_ERROR_INVALID_ARGUMENT);
    assert(sceNpManagerGetTicketParam(-1, (void*)0x500)
           == SCE_NP_ERROR_INVALID_ARGUMENT);
    assert(sceNpManagerGetTicketParam(SCE_NP_TICKET_PARAM_SUBJECT_DOB + 1,
                                      (void*)0x500)
           == SCE_NP_ERROR_INVALID_ARGUMENT);
    assert(sceNpManagerGetTicketParam(SCE_NP_TICKET_PARAM_SUBJECT_ONLINE_ID,
                                      (void*)0x500) == CELL_OK);
    for (int i = 0; i < SCE_NP_TICKET_PARAM_DATA_LEN; i++)
        assert(vm_base[0x500 + i] == 0);
    memset(vm_base + 0x600, 0xA5, SCE_NP_AVATAR_URL_MAX_LENGTH + 1);
    assert(sceNpManagerGetAvatarUrl(NULL) == SCE_NP_ERROR_INVALID_ARGUMENT);
    assert(sceNpManagerGetAvatarUrl((void*)0x600) == CELL_OK);
    for (int i = 0; i < SCE_NP_AVATAR_URL_MAX_LENGTH + 1; i++)
        assert(vm_base[0x600 + i] == 0);
    assert((u32)sceNpManagerRequestTicket(NULL, (void*)0x100, NULL, 0, NULL, 0)
           == SCE_NP_AUTH_EINVALID_ARGUMENT);
    assert((u32)sceNpManagerRequestTicket((void*)0x100, NULL, NULL, 0, NULL, 0)
           == SCE_NP_AUTH_EINVALID_ARGUMENT);
    assert((u32)sceNpManagerRequestTicket((void*)0x100, (void*)0x200, NULL,
                                          SCE_NP_COOKIE_MAX_SIZE + 1, NULL, 0)
           == SCE_NP_AUTH_EINVALID_ARGUMENT);
    assert(sceNpManagerRequestTicket((void*)0x100, (void*)0x200, NULL, 0,
                                     NULL, 0) == CELL_OK);
    assert((u32)sceNpBasicGetFriendPresenceByIndex(0, NULL, (void*)0x700, 0)
           == SCE_NP_BASIC_ERROR_INVALID_ARGUMENT);
    assert((u32)sceNpBasicGetFriendPresenceByIndex(0, (void*)0x600, NULL, 0)
           == SCE_NP_BASIC_ERROR_INVALID_ARGUMENT);
    assert((u32)sceNpBasicGetFriendPresenceByIndex(0, (void*)0x600, (void*)0x700, 0)
           == SCE_NP_BASIC_ERROR_NOT_CONNECTED);
    assert(sceNpTerm() == CELL_OK);
    assert((u32)sceNpManagerGetAvatarUrl((void*)0x600)
           == SCE_NP_ERROR_NOT_INITIALIZED);
    assert((u32)sceNpManagerRequestTicket((void*)0x100, (void*)0x200, NULL, 0,
                                          NULL, 0)
           == SCE_NP_ERROR_NOT_INITIALIZED);
    assert((u32)sceNpBasicGetFriendPresenceByIndex(0, (void*)0x600, (void*)0x700, 0)
           == SCE_NP_BASIC_ERROR_NOT_INITIALIZED);
    assert((u32)sceNpScoreInit() == SCE_NP_ERROR_NOT_INITIALIZED);
    free(vm_base);
    puts("Score lifecycle and offline status checks passed");
}
