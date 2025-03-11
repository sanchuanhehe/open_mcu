/**
  * @copyright Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2024. All rights reserved.
  * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
  * following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
  * disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
  * following disclaimer in the documentation and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
  * products derived from this software without specific prior written permission.
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * @file    flash.lds
  * @author  MCU Application Driver Team
  * @brief   RISCV flash link script
  */

OUTPUT_ARCH( "riscv" )

SRAM_START   = 0x2000000;
SRAM_END     = 0x2000000 + 64K;

RAM_CODE_START = 0x2000000;
RAM_CODE_SIZE  = 0;

RAM_START = SRAM_START;
RAM_END   = SRAM_END;
RAM_SIZE  = 65536;

FLASH_START     = 0x3000000;
FLASH_SIZE      = 0x80000;

MEMORY
{
    /* ram for code */
    RAM_CODE(xr) : ORIGIN = RAM_CODE_START, LENGTH = RAM_CODE_SIZE
    /* ram for common bss and data */
    RAM_DATA(xrw) : ORIGIN = RAM_START + RAM_CODE_SIZE, LENGTH = RAM_SIZE
    /* ram for target */
    FLASH_CODE(rx) : ORIGIN = FLASH_START, LENGTH = FLASH_SIZE
}

SECTIONS
{
    .text : ALIGN(4)
    {
        __start_addr = .;
        *(.text*)
        *(.ram.text*)
        . = ALIGN(4);
        __rodata_start = .;
        *(.rodata*)
        . = ALIGN(4);
        __rodata_end = .;
        *(.got*)
        __text_end = .;
    } > FLASH_CODE

    /* data section */
    .data : ALIGN(4)
    {
        __data_load = LOADADDR(.data);
        __data_start = .;
        *(.data*)
        *(.sdata*)
        . = ALIGN(4);
        __data_end = .;
    } > RAM_DATA AT> FLASH_CODE

    /* bss section */
    .bss (NOLOAD) : ALIGN(4)
    {
        __bss_begin__ = .;
        *(.bss*)
        *(.sbss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    }  > RAM_DATA
}