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

#include "config.h"
#include "syslib.h"

#include "uart.h"

#ifdef UART0_ADDR
const uintptr_t uart_addrs[] = {
	UART0_ADDR,
#ifdef UART1_ADDR
	UART1_ADDR,
#endif
};
int	uart_count = arraycount(uart_addrs);
#endif /* UART0_ADDR */

void
uart_configure(bool do_init)
{
	int i;

	for (i = 0; i < uart_count; i++) {
		configure_printf("uart%d at 0x%08lx\n",
		    i, (u_long)uart_addrs[i]);

		if (do_init) {
			/* Console UART already configured. */
			if (i != CONFIG_CONSOLE_UART) {
				uart_init(i, CONFIG_UART_SPEED);
			}
		}
	}
}

#if defined(CONFIG_UART_16550)
#include "uart_ns16550.c"
#elif defined(CONFIG_UART_HOST_SIM)
#include "uart_hostsim.c"
#else
#error No UART configured.
#endif
