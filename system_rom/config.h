/*
 * Copyright (c) 2025 Jason R. Thorpe.
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
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef config_h_included
#define	config_h_included

/*
 * Configuration for the Playground 68030 Mk I.
 */
#ifdef CONFIG_MACH_PG68030_MK_I
#define	DRAM_START	0x00000000	/* start of DRAM */
#define	DRAM_MAXSIZE	0x10000000	/* max DRAM size */
#define	SRAM_START	0xfe000000	/* start of fast SRAM */
#define	SRAM_SIZE	0x00400000	/* size of fast SRAM */

#define	ISA_START	0xffe00000

#define	UART0_ADDR	(ISA_START+0x3f8)
#define	UART1_ADDR	(ISA_START+0x2f8)
#define	TIMER_ADDR	(ISA_START+0x040)
#define	ATA_ADDR	(ISA_START+0x1f0)
#define	ATA_AUX_ADDR	(ISA_START+0x3f6)
#define	ETH_ADDR	(ISA_START+0x300)
#define	I2C_ADDR	0xffe10000
#define	PSUC_ADDR	0xffefffe0
#define	INTC_ADDR	0xffeffff0

#define	CONFIG_RIISP		(SRAM_START+SRAM_SIZE)
#define	CONFIG_MC68030
#define	CONFIG_DEVICETREE
#define	CONFIG_UART_16550
#define	CONFIG_UART_FREQ	1843200
#define	CONFIG_UART_SPEED	115200
#define	CONFIG_CONSOLE_UART	0
#define	CONFIG_ETH_RTL8019AS
#define	CONFIG_I2C_PCF8584
#endif /* CONFIG_MACH_PG68030_MK_I */

#ifndef CONFIG_RIISP
#error No machine configured!
#endif

#ifndef CONFIG_DEV_REGSHIFT
#define	CONFIG_DEV_REGSHIFT	0
#endif

#endif /* config_h_included */
