/*	From NetBSD: ether.c,v 1.24 2019/03/31 20:08:45 christos Exp	*/

/*
 * Copyright (c) 1992 Regents of the University of California.
 * All rights reserved.
 *
 * This software was developed by the Computer Systems Engineering group
 * at Lawrence Berkeley Laboratory under DARPA contract BG 91-66 and
 * contributed to Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by the University of
 *	California, Lawrence Berkeley Laboratory and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
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
 * @(#) Header: net.c,v 1.9 93/08/06 19:32:15 leres Exp  (LBL)
 */

#include "systypes.h"
#include "syslib.h"
#include "sysfile.h"
#include "net.h"
#include "ethernet.h"

const uint8_t ether_bcaddr[ETHER_ADDR_LEN] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

int
ether_send(struct open_file *f, struct packet *pkt, const uint8_t *ea,
    uint16_t etype)
{
	struct ether_header eh;
	int error;

	memcpy(eh.ether_dhost, ea, ETHER_ADDR_LEN);
	memcpy(eh.ether_shost, f->f_net.f_enaddr, ETHER_ADDR_LEN);
	eh.ether_type = htons(etype);

	error = packet_prepend(pkt, &eh, sizeof(eh));
	if (error == 0) {
		error = dev_send(f, pkt);
	}
	return error;
}

int
ether_recv(struct open_file *f, struct packet *pkt, int tleft,
    uint16_t *etypep)
{
	struct ether_header *eh;
	size_t len;
	int error;

	error = dev_recv(f, pkt, tleft);
	if (error) {
		return error;
	}
	error = packet_get_layer(pkt, (void **)&eh, &len);
	if (error) {
		return error;
	}

	if (len < sizeof(*eh)) {
		return -1;
	}

	if (memcmp(f->f_net.f_enaddr, eh->ether_dhost, ETHER_ADDR_LEN) != 0 &&
	    memcmp(ether_bcaddr, eh->ether_dhost, ETHER_ADDR_LEN) != 0) {
		return -1;
	}

	*etypep = ntohs(eh->ether_type);

	return packet_advance(pkt, sizeof(*eh));
}
