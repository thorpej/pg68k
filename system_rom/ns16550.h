/*	From NetBSD: ns16550reg.h,v 1.14 2022/10/06 19:59:55 riastradh Exp	*/
/*	From NetBSD: comreg.h,v 1.28 2022/10/06 19:59:55 riastradh Exp	*/

/*-
 * Copyright (c) 1991 The Regents of the University of California.
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
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)ns16550.h	7.1 (Berkeley) 5/9/91
 */

#ifndef ns16550_h_included
#define	ns16550_h_included

#ifndef COM_TOLERANCE
#define	COM_TOLERANCE	30	/* baud rate tolerance, in 0.1% units */
#endif

/*
 * NS16550 UART registers
 */

#define	com_data	0	/* data register (R/W) */
#define	com_dlbl	0	/* divisor latch low (W) */
#define	com_dlbh	1	/* divisor latch high (W) */
#define	com_ier		1	/* interrupt enable (W) */
#define	com_iir		2	/* interrupt identification (R) */
#define	com_fifo	2	/* FIFO control (W) */
#define	com_lctl	3	/* line control register (R/W) */
#define	com_cfcr	3	/* line control register (R/W) */
#define	com_lcr		com_cfcr
#define	com_mcr		4	/* modem control register (R/W) */
#define	com_lsr		5	/* line status register (R/W) */
#define	com_msr		6	/* modem status register (R/W) */
#define	com_scratch	7	/* scratch register (R/W) */

/* interrupt enable register */
#define	IER_ERXRDY	0x1	/* Enable receiver interrupt */
#define	IER_ETXRDY	0x2	/* Enable transmitter empty interrupt */
#define	IER_ERLS	0x4	/* Enable line status interrupt */
#define	IER_EMSC	0x8	/* Enable modem status interrupt */
#define	IER_ERTS	0x40	/* Enable RTS interrupt */
#define	IER_ECTS	0x80	/* Enable CTS interrupt */

/* interrupt identification register */
#define	IIR_IMASK	0xf
#define	IIR_RXTOUT	0xc
#define	IIR_RLS		0x6	/* Line status change */
#define	IIR_RXRDY	0x4	/* Receiver ready */
#define	IIR_TXRDY	0x2	/* Transmitter ready */
#define	IIR_MLSC	0x0	/* Modem status */
#define	IIR_NOPEND	0x1	/* No pending interrupts */
#define	IIR_64B_FIFO	0x20	/* 64byte FIFO Enabled (16750) */
#define	IIR_FIFO_MASK	0xc0	/* set if FIFOs are enabled */
#define IIR_BUSY	0x7	/* Busy indicator (16750/SUNXI) */

/* fifo control register */
#define	FIFO_ENABLE	0x01	/* Turn the FIFO on */
#define	FIFO_RCV_RST	0x02	/* Reset RX FIFO */
#define	FIFO_XMT_RST	0x04	/* Reset TX FIFO */
#define	FIFO_DMA_MODE	0x08
#define	FIFO_UART_ON	0x10	/* JZ47xx only */
#define	FIFO_64B_ENABLE	0x20	/* 64byte FIFO Enable (16750) */
#define	FIFO_TRIGGER_1	0x00	/* Trigger RXRDY intr on 1 character */
#define	FIFO_TRIGGER_4	0x40	/* ibid 4 */
#define	FIFO_TRIGGER_8	0x80	/* ibid 8 */
#define	FIFO_TRIGGER_14	0xc0	/* ibid 14 */

/* enhanced feature register */
#define	EFR_AUTOCTS	0x80	/* Automatic CTS flow control */
#define	EFR_AUTORTS	0x40	/* Automatic RTS flow control */
#define	EFR_SPECIAL	0x20	/* Special char detect */
#define	EFR_EFCR	0x10	/* Enhanced function control bit */
#define	EFR_TXFLOWBOTH	0x0c	/* Automatic transmit XON/XOFF 1 and 2 */
#define	EFR_TXFLOW1	0x08	/* Automatic transmit XON/XOFF 1 */
#define	EFR_TXFLOW2	0x04	/* Automatic transmit XON/XOFF 2 */
#define	EFR_TXFLOWNONE	0x00	/* No automatic XON/XOFF transmit */
#define	EFR_RXFLOWBOTH	0x03	/* Automatic receive XON/XOFF 1 and 2 */
#define	EFR_RXFLOW1	0x02	/* Automatic receive XON/XOFF 1 */
#define	EFR_RXFLOW2	0x01	/* Automatic receive XON/XOFF 2 */
#define	EFR_RXFLOWNONE	0x00	/* No automatic XON/XOFF receive */

/* line control register */
#define	LCR_EERS	0xBF	/* Enable access to Enhanced Register Set */
#define	LCR_DLAB	0x80	/* Divisor latch access enable */
#define	LCR_SBREAK	0x40	/* Break Control */
#define	LCR_PZERO	0x38	/* Space parity */
#define	LCR_PONE	0x28	/* Mark parity */
#define	LCR_PEVEN	0x18	/* Even parity */
#define	LCR_PODD	0x08	/* Odd parity */
#define	LCR_PNONE	0x00	/* No parity */
#define	LCR_PENAB	0x08	/* XXX - low order bit of all parity */
#define	LCR_STOPB	0x04	/* 2 stop bits per serial word */
#define	LCR_8BITS	0x03	/* 8 bits per serial word */
#define	LCR_7BITS	0x02	/* 7 bits */
#define	LCR_6BITS	0x01	/* 6 bits */
#define	LCR_5BITS	0x00	/* 5 bits */

/* modem control register */
#define MCR_PRESCALE	0x80	/* 16650/16950: Baud rate prescaler select */
#define MCR_MDCE	0x80	/* Ingenic: modem control enable */
#define MCR_TCR_TLR	0x40	/* OMAP: enables access to the TCR & TLR regs */
#define MCR_FCM		0x40	/* Ingenic: 1 - hardware flow control */
#define MCR_XONENABLE	0x20	/* OMAP XON_EN */
#define MCR_AFE		0x20	/* tl16c750: Flow Control Enable */
#define	MCR_LOOPBACK	0x10	/* Loop test: echos from TX to RX */
#define	MCR_IENABLE	0x08	/* Out2: enables UART interrupts */
#define	MCR_DRS		0x04	/* Out1: resets some internal modems */
#define	MCR_RTS		0x02	/* Request To Send */
#define	MCR_DTR		0x01	/* Data Terminal Ready */

/* line status register */
#define	LSR_RCV_FIFO	0x80
#define	LSR_TSRE	0x40	/* Transmitter empty: byte sent */
#define	LSR_TXRDY	0x20	/* Transmitter buffer empty */
#define	LSR_BI		0x10	/* Break detected */
#define	LSR_FE		0x08	/* Framing error: bad stop bit */
#define	LSR_PE		0x04	/* Parity error */
#define	LSR_OE		0x02	/* Overrun, lost incoming byte */
#define	LSR_RXRDY	0x01	/* Byte ready in Receive Buffer */
#define	LSR_RCV_MASK	0x1f	/* Mask for incoming data or error */

/* modem status register */
/* All deltas are from the last read of the MSR. */
#define	MSR_DCD		0x80	/* Current Data Carrier Detect */
#define	MSR_RI		0x40	/* Current Ring Indicator */
#define	MSR_DSR		0x20	/* Current Data Set Ready */
#define	MSR_CTS		0x10	/* Current Clear to Send */
#define	MSR_DDCD	0x08	/* DCD has changed state */
#define	MSR_TERI	0x04	/* RI has toggled low to high */
#define	MSR_DDSR	0x02	/* DSR has changed state */
#define	MSR_DCTS	0x01	/* CTS has changed state */

#endif	/* ns16550_h_included */
