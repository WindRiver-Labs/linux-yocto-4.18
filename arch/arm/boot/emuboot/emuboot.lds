/*
 * emuboot.lds
 *
 * Copyright (C) 2011 ARM Limited. All rights reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE.txt file.
 */

OUTPUT_FORMAT("elf32-littlearm")
OUTPUT_ARCH(arm)
TARGET(binary)

INPUT(./emuboot.o)
INPUT(./image.emu)

PHYS_OFFSET = 0x00000000;

SECTIONS
{
	. = PHYS_OFFSET;
	.text : { emuboot.o }
	. = PHYS_OFFSET + 0x8000 - 0x0;
	kernel = . + 0x0;
	.kernel : { ./image.emu }
	. = PHYS_OFFSET + 0x00800000;
	filesystem = .;
	.data : { *(.data) }
	.bss : { *(.bss) }
}
