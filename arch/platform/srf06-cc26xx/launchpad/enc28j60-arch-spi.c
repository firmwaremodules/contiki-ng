/*
* Copyright (c) 2018, Firmware Modules Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "arch/dev/enc28j60/enc28j60.h"
#include "ti-lib.h"
#include "board-spi.h"


/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
void
enc28j60_arch_spi_init(void)
{
    board_spi_open(8000000, BOARD_IOID_SPI_CLK, IOID_UNUSED);

    ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_SPI_FSS);
    enc28j60_arch_spi_deselect();
}
/*---------------------------------------------------------------------------*/
void
enc28j60_arch_spi_select(void)
{
    ti_lib_gpio_clear_dio(BOARD_IOID_SPI_FSS);
}
/*---------------------------------------------------------------------------*/
void
enc28j60_arch_spi_deselect(void)
{
    ti_lib_gpio_set_dio(BOARD_IOID_SPI_FSS);
}
/*---------------------------------------------------------------------------*/
uint8_t
enc28j60_arch_spi_write(uint8_t output)
{
    board_spi_write(&output, 1);
    return 0;
}
/*---------------------------------------------------------------------------*/
uint8_t
enc28j60_arch_spi_read(void)
{
    uint8_t data = 0;
    board_spi_read(&data, 1);
    return data;
}
/*---------------------------------------------------------------------------*/


