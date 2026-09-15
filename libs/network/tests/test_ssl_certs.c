/* clang -std=gnu17 -I include libs/network/tests/test_ssl_certs.c
 *   -o /tmp/test_ssl_certs && /tmp/test_ssl_certs
 */
#include "../cellSsl.c"
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
    assert(cellSslInit((void*)0x1000, 4096) == CELL_OK);

    vm_write32(0x200, 0xDEADBEEFu);
    assert(cellSslCertGetSubjectName(1, NULL) == (s32)CELL_SSL_ERROR_INVALID_ARG);
    assert(cellSslCertGetSubjectName(1, (void*)0x200) == CELL_OK);
    assert(vm_read32(0x200) == 0);

    vm_write32(0x204, 0xDEADBEEFu);
    assert(cellSslCertGetIssuerName(1, (void*)0x204) == CELL_OK);
    assert(vm_read32(0x204) == 0);

    vm_write32(0x208, 0xDEADBEEFu);
    assert(cellSslCertGetNameEntryCount(0, (void*)0x208)
           == (s32)CELL_SSL_ERROR_INVALID_ARG);
    assert(vm_read32(0x208) == 0);
    vm_write32(0x208, 0xDEADBEEFu);
    assert(cellSslCertGetNameEntryCount(1, (void*)0x208) == CELL_OK);
    assert(vm_read32(0x208) == 0);

    vm_write32(0x20C, 0x11111111u);
    vm_write32(0x210, 0x22222222u);
    vm_write32(0x214, 0x33333333u);
    assert(cellSslCertGetNameEntryInfo(1, 0, (void*)0x20C, (void*)0x210,
                                       (void*)0x214, 0) == CELL_OK);
    assert(vm_read32(0x20C) == 0);
    assert(vm_read32(0x210) == 0);
    assert(vm_read32(0x214) == 0);

    vm_write32(0x218, 0xDEADBEEFu);
    vm_write32(0x21C, 0xCAFEBABEu);
    assert(cellSslCertGetRsaPublicKeyModulus(1, (void*)0x218, (void*)0x21C)
           == CELL_OK);
    assert(vm_read32(0x218) == 0);
    assert(vm_read32(0x21C) == 0);
    vm_write32(0x218, 0xDEADBEEFu);
    vm_write32(0x21C, 0xCAFEBABEu);
    assert(cellSslCertGetRsaPublicKeyExponent(1, (void*)0x218, (void*)0x21C)
           == CELL_OK);
    assert(vm_read32(0x218) == 0);
    assert(vm_read32(0x21C) == 0);

    assert(cellSslEnd() == CELL_OK);
    free(vm_base);
    puts("SSL cert name/RSA empty out-params passed");
}
