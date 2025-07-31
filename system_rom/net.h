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

#ifndef net_h_included
#define	net_h_included

#include "systypes.h"
#include "syslib.h"
#include "endian.h"

/*
 * The packet structure is just a conventient way for us to encapsulate
 * a slightly-flattened OSI layer cake.  The "payload" buffer (in this
 * case, whatever protocol we want to run over TCP or UDP) is separateed
 * from the L4 (TCP/UDP), L3 (IP), and L2 (Ethernet) buffers.
 *
 * On receive, the caller provides a buffer, and the packet is sliced
 * and diced to point to the relevant layers.
 */
struct pktlayer {
	void	*ptr;
	size_t	len;
};

#define	PKT_LAYER_2		0
#define	PKT_LAYER_3		1
#define	PKT_LAYER_4		2
#define	PKG_LAYER_4_5		3	/* Oh, RPC, my sweet summer child.. */
#define	PKT_PAYLOAD		4
#define	PKT_NLAYERS		5

struct packet {
	struct pktlayer		pkt_layers[PKT_NLAYERS];
	int			pkt_curlayer;
	size_t			pkt_len;
};

static inline void
packet_init_layer(struct packet *pkt, void *ptr, size_t len, int layer)
{
	memset(pkt, 0, sizeof(*pkt));
	pkt->pkt_layers[layer].ptr = ptr;
	pkt->pkt_layers[layer].len = len;
	pkt->pkt_curlayer = layer;
}

/* Initialize a packet for transmit. */
static inline void
packet_init_tx(struct packet *pkt, void *ptr, size_t len)
{
	packet_init_layer(pkt, ptr, len, PKT_PAYLOAD);
}

static inline int
packet_prepend(struct packet *pkt, void *ptr, size_t len)
{
	int layer = pkt->pkt_curlayer;

	if (layer == PKT_LAYER_2) {
		return EINVAL;
	}
	layer--;
	pkt->pkt_layers[layer].ptr = ptr;
	pkt->pkt_layers[layer].len = len;
	pkt->pkt_curlayer = layer;
	pkt->pkt_len += len;
	return 0;
}

/* Initialize a packet for receive. */
static inline void
packet_init_rx(struct packet *pkt, void *ptr, size_t len)
{
	packet_init_layer(pkt, ptr, len, PKT_LAYER_2);
	pkt->pkt_len = len;
}

static inline int
packet_adjust(struct packet *pkt, size_t adj)
{
	int layer = pkt->pkt_curlayer;
	void *optr = pkt->pkt_layers[layer].ptr;
	size_t olen = pkt->pkt_layers[layer].len;

	if (layer != PKT_LAYER_2 || olen <= adj ||
	    pkt->pkt_len <= adj) {
		return EINVAL;
	}
	pkt->pkt_layers[layer].ptr = (void *)((uintptr_t)optr + adj);
	pkt->pkt_layers[layer].len = olen - adj;
	pkt->pkt_len -= adj;
	return 0;
}

static inline int
packet_advance(struct packet *pkt, size_t adv)
{
	int layer = pkt->pkt_curlayer;
	void *optr = pkt->pkt_layers[layer].ptr;
	size_t olen = pkt->pkt_layers[layer].len;

	if (layer == PKT_PAYLOAD) {
		return EINVAL;
	}
	pkt->pkt_layers[layer].len = adv;
	layer++;
	pkt->pkt_layers[layer].ptr = (void *)((uintptr_t)optr + adv);
	pkt->pkt_layers[layer].len = olen - adv;
	return 0;
}

static inline int
packet_get_layer(const struct packet *pkt, void **bufp, size_t *lenp)
{
	int layer = pkt->pkt_curlayer;
	if (pkt->pkt_layers[layer].ptr == NULL ||
	    pkt->pkt_layers[layer].len == 0) {
		return EIO;
	}
	*bufp = pkt->pkt_layers[layer].ptr;
	*lenp = pkt->pkt_layers[layer].len;
	return 0;
}

struct open_file;

/* Ethernet layer (L2) */
extern const uint8_t ether_bcaddr[];
int	ether_send(struct open_file *, struct packet *, const uint8_t *,
	    uint16_t);
int	ether_recv(struct open_file *, struct packet *, int, uint16_t *);

/* (ARP is in-between) */
const uint8_t *arp_whohas(struct open_file *, in_addr_t);
void	arp_reply(struct open_file *, struct packet *);

/* IP layer (L3) */
int	ip_send(struct open_file *, struct packet *, uint8_t);
int	ip_recv(struct open_file *, struct packet *, int, uint8_t);
int	ip_cksum(const void *, size_t);
char *	inet_ntoa(in_addr_t, char *, size_t);

#define	INET_ADDR_STRLEN	sizeof("255.255.255.255")

#define	SAMENET(a1, a2, m)	(((a1) & (m)) == ((a2) & (m)))

/* (TCP)/UDP layer (L4) */
int	udp_send(struct open_file *, struct packet *);
int	udp_recv(struct open_file *, struct packet *, int);

/* Application layer interface. */
#define	MAXTMO	20	/* seconds */
#define	MINTMO	2	/* seconds */
typedef	int (*sendproc_t)(struct open_file *, struct packet *);
typedef	int (*recvproc_t)(struct open_file *, struct packet *, int);
int	sendrecv(struct open_file *,
	    sendproc_t, struct packet *,
	    recvproc_t, struct packet *);

#endif /* ! net_h_included */
