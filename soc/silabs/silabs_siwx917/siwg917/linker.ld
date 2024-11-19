/*
 * Copyright (c) 2023 Antmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/arch/arm/cortex_m/scripts/linker.ld>

MEMORY
{
  udma0   (rwx)  : ORIGIN = 0x2fc00, LENGTH = 0x400
  udma1   (rwx)  : ORIGIN = 0x24061c00, LENGTH = 0x400
}

SECTIONS
{
	.common_tcm_code :
	{
		*(.common_tcm_code*)
	} > FLASH

	.udma_addr0 :
	{
		*(.udma_addr0*)
	} > udma0 AT> FLASH

	.udma_addr1 :
	{
		*(.udma_addr1*)
	} > udma1 AT> FLASH
}
