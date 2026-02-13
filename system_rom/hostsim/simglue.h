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

#ifndef simglue_h_included
#define	simglue_h_included

void	sim_uart_init(void);
int	sim_uart_pollc(void);
int	sim_uart_getc(void);
void	sim_uart_putc(int);

void	sim_ata_init(bool);
int	sim_ata_strategy(void *, int, uint64_t, size_t, void *, size_t *);
int	sim_ata_open(int, void **);
int	sim_ata_close(void *);
int	sim_ata_ioctl(void *, unsigned long, void *);

void	sim_clock_delay(int);
time_t	sim_clock_getsecs(void);

ssize_t	sim_loader_read(int, uintptr_t, size_t);
void	sim_loader_bcopy(const void *, uintptr_t, size_t);
void	sim_loader_bzero(uintptr_t, size_t);

const void *sim_rom_fdt(void);
void	sim_boot_fdt(const void *, size_t);
#endif /* simglue_h_included */
