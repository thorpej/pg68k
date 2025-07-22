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

#define	CONFIG_ROM_VERSION_MAJOR	0
#define	CONFIG_ROM_VERSION_MINOR	1

/*
 * Configuration for the Playground 68030 Mk I.
 */
#ifdef CONFIG_MACH_PG68030_MK_I
#define	RAM0_START	0x00000000	/* start of DRAM */
#define	RAM0_MAXSIZE	0x10000000	/* max DRAM size */
#define	RAM0_DESC	"DRAM"
#define	RAM4_START	0xfe000000	/* start of fast SRAM */
#define	RAM4_SIZE	0x00400000	/* fixed size of fast SRAM */
#define	RAM4_DESC	"Fast SRAM"

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

#define	CONFIG_MACHINE_STRING	"Playground 68030 Mk I"
#define	CONFIG_RIISP		(RAM4_START+RAM4_SIZE)
#define	CONFIG_MC68030
#define	CONFIG_DEVICETREE
#define	CONFIG_UART_16550
#define	CONFIG_UART_FREQ	1843200
#define	CONFIG_UART_SPEED	115200
#define	CONFIG_CONSOLE_UART	0
#define	CONFIG_ATA_GENERIC
#define	CONFIG_ETH_RTL8019AS
#define	CONFIG_I2C_PCF8584
#define	CONFIG_FS_UFS
#define	CONFIG_FS_DOSFS
#endif /* CONFIG_MACH_PG68030_MK_I */

/*
 * Host-based ROM simulator.
 */
#ifdef CONFIG_MACH_HOST_SIM
#define	RAM0_START	0x00000000	/* start of DRAM */
#define	RAM0_MAXSIZE	0x10000000	/* max DRAM size */
#define	RAM0_DESC	"DRAM"

#define	ISA_START	0xffe00000

#define	UART0_ADDR	(ISA_START+0x3f8)
#define	ATA_ADDR	(ISA_START+0x1f0)
#define	ATA_AUX_ADDR	(ISA_START+0x3f6)

#define	CONFIG_MACHINE_STRING	"Playground 68k Hosted ROM simulator"
#define	CONFIG_RIISP		0	/* just a dummy */
#define	CONFIG_DEVICETREE
#define	CONFIG_UART_HOST_SIM
#define	CONFIG_UART_SPEED	115200	/* just a dummy */
#define	CONFIG_CONSOLE_UART	0
#define	CONFIG_ATA_HOST_SIM
#endif /* CONFIG_MACH_HOST_SIM */

#ifndef CONFIG_RIISP
#error No machine configured!
#endif

#ifndef CONFIG_DEV_REGSHIFT
#define	CONFIG_DEV_REGSHIFT	0
#endif

#if defined(CONFIG_ATA_GENERIC) || defined(CONFIG_ATA_HOST_SIM)
#define	CONFIG_DEV_ATA
#endif

#ifndef RAM0_SIZE
#define	RAM0_SIZE		0
#endif
#ifndef RAM1_SIZE
#define	RAM1_SIZE		0
#endif
#ifndef RAM2_SIZE
#define	RAM2_SIZE		0
#endif
#ifndef RAM3_SIZE
#define	RAM3_SIZE		0
#endif
#ifndef RAM4_SIZE
#define	RAM4_SIZE		0
#endif

#ifndef RAM0_MAXSIZE
#define	RAM0_MAXSIZE		RAM0_SIZE
#endif
#ifndef RAM1_MAXSIZE
#define	RAM1_MAXSIZE		RAM1_SIZE
#endif
#ifndef RAM2_MAXSIZE
#define	RAM2_MAXSIZE		RAM2_SIZE
#endif
#ifndef RAM3_MAXSIZE
#define	RAM3_MAXSIZE		RAM3_SIZE
#endif
#ifndef RAM4_MAXSIZE
#define	RAM4_MAXSIZE		RAM4_SIZE
#endif

#if (RAM0_SIZE != 0 || RAM0_MAXSIZE != 0) && !defined(RAM0_DESC)
#error RAM0_DESC not defined
#endif
#if (RAM1_SIZE != 0 || RAM1_MAXSIZE != 0) && !defined(RAM1_DESC)
#error RAM1_DESC not defined
#endif
#if (RAM2_SIZE != 0 || RAM2_MAXSIZE != 0) && !defined(RAM2_DESC)
#error RAM2_DESC not defined
#endif
#if (RAM3_SIZE != 0 || RAM3_MAXSIZE != 0) && !defined(RAM3_DESC)
#error RAM3_DESC not defined
#endif
#if (RAM4_SIZE != 0 || RAM4_MAXSIZE != 0) && !defined(RAM4_DESC)
#error RAM4_DESC not defined
#endif

#define	BOOT_ELF32		/* support loading Elf32 binaries */

#endif /* config_h_included */
