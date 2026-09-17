/* STM32H753ZI — 2 MB flash (2 banks), 512 KB AXI SRAM (D1). */
MEMORY
{
    FLASH  : ORIGIN = 0x08000000, LENGTH = 2048K /* BANK_1 + BANK_2 */
    RAM    : ORIGIN = 0x24000000, LENGTH = 512K  /* AXI SRAM (D1) */
    RAM_D3 : ORIGIN = 0x38000000, LENGTH = 64K   /* SRAM4 (D3) */
}

SECTIONS
{
    .ram_d3 (NOLOAD) :
    {
        *(.ram_d3)
    } > RAM_D3
}
