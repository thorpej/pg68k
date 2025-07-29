/*	From NetBSD: net.c,v 1.36 2019/03/31 20:08:45 christos Exp	*/

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

#include "sysfile.h"
#include "net.h"
#include "clock.h"

#define	sendrecv_delay()	/* XXX nothing */

/*
 * Send a packet and wait for a reply, with exponential backoff.
 */
int
sendrecv(struct open_file *f,
    sendproc_t sproc, void *payload, size_t payloadlen,
    recvproc_t rproc, void *rbuf, size_t rbuflen, void **rp, size_t *rlenp)
{
	int error;
	time_t t, tlast;
	time_t tmo, tleft;
	struct packet txpkt, rxpkt;

	tmo = MINTMO;
	tlast = 0;
	tleft = 0;
	t = clock_getsecs();
	for (;;) {
		if (tleft <= 0) {
			if (tmo >= MAXTMO) {
				errno = ETIMEDOUT;
				return -1;
			}
			packet_init_tx(&txpkt, payload, payloadlen);
			error = (*sproc)(f, &txpkt);

			tleft = tmo;
			tmo <<= 1;
			if (tmo > MAXTMO)
				tmo = MAXTMO;

			/*
			 * -1 means "try again".  0 means "success",
			 * any other return value is an error.
			 */
			if (error == -1) {
				while ((clock_getsecs() - t) < tmo)
					sendrecv_delay();
				tleft = 0;
				continue;
			} else if (error) {
				return error;
			}
			tlast = t;
		}

		/*
		 * Try to get a packet and process it.  N.B. the
		 * bottom layer may adjust the Rx packet buffer
		 * because it understands the alignment constraints.
		 */
		packet_init_rx(&rxpkt, rbuf, rbuflen);
		error = (*rproc)(f, &rxpkt, tleft);

		/*
		 * -1 means "try again".  0 means "success",
		 * any other return value is an error.
		 */
		if (error == 0) {
			return packet_get_layer(&rxpkt, rp, rlenp);
		} else if (error != -1) {
			return error;
		}

		/* Timed out or didn't get the packet we're waiting for */
		t = clock_getsecs();
		tleft -= t - tlast;
		tlast = t;
	}
}
