/*	From NetBSD: arp.c,v 1.35 2019/03/31 20:08:45 christos Exp	*/

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
 * @(#) Header: arp.c,v 1.5 93/07/15 05:52:26 leres Exp  (LBL)
 */

#include "sysfile.h"
#include "net.h"

/*
 * Address Resolution Protocol.
 *
 * See RFC 826 for protocol description.  ARP packets are variable
 * in size; the arphdr structure defines the fixed-length portion.
 * Protocol type values are the same as those for 10 Mb/s Ethernet.
 * It is followed by the variable-sized fields ar_sha, arp_spa,
 * arp_tha and arp_tpa in that order, according to the lengths
 * specified.  Field names used correspond to RFC 826.
 */
struct arphdr {
	uint16_t ar_hrd;	  /* format of hardware address */
#define	ARPHRD_ETHER		1 /* ethernet hardware format */
	uint16_t ar_pro;	  /* format of protocol address */
	uint8_t  ar_hln;	  /* length of hardware address */
	uint8_t  ar_pln;	  /* length of protocol address */
	uint16_t ar_op;		  /* one of: */
#define	ARPOP_REQUEST		1 /* request to resolve address */
#define	ARPOP_REPLY		2 /* response to previous request */
#define	ARPOP_REVREQUEST	3 /* request protocol address given hardware */
#define	ARPOP_REVREPLY		4 /* response giving protocol address */
#define	ARPOP_INVREQUEST	8 /* request to identify peer */
#define	ARPOP_INVREPLY		9 /* response identifying peer */
/*
 * The remaining fields are variable in size,
 * according to the sizes above.
 */
#if 0
	uint8_t  ar_sha[];	  /* sender hardware address */
	uint8_t  ar_spa[];	  /* sender protocol address */
	uint8_t  ar_tha[];	  /* target hardware address */
	uint8_t  ar_tpa[];	  /* target protocol address */
#endif
};

/*
 * Ethernet Address Resolution Protocol.
 *
 * See RFC 826 for protocol description.  Structure below is adapted
 * to resolving internet addresses.  Field names used correspond to
 * RFC 826.
 */
struct ether_arp {
	struct	arphdr ea_hdr;			/* fixed-size header */
	uint8_t arp_sha[ETHER_ADDR_LEN];	/* sender hardware address */
	uint8_t arp_spa[4];			/* sender protocol address */
	uint8_t arp_tha[ETHER_ADDR_LEN];	/* target hardware address */
	uint8_t arp_tpa[4];			/* target protocol address */
};
#define	arp_hrd	ea_hdr.ar_hrd
#define	arp_pro	ea_hdr.ar_pro
#define	arp_hln	ea_hdr.ar_hln
#define	arp_pln	ea_hdr.ar_pln
#define	arp_op	ea_hdr.ar_op

/* Cache stuff */
#define ARP_NUM 8			/* need at most 3 arp entries */

struct arp_list {
	in_addr_t       addr;
	uint8_t		ea[ETHER_ADDR_LEN];
} arp_list[ARP_NUM] = {
	/* XXX - net order `INADDR_BROADCAST' must be a constant */
	{ 0xffffffffU, { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff } }
};
int arp_num = 1;

static int
arp_send(struct open_file *f, struct packet *pkt)
{
	return ether_send(f, pkt, ether_bcaddr, ETHERTYPE_ARP);
}

static int
arp_recv(struct open_file *f, struct packet *pkt, int tleft)
{
	struct ether_arp *ah;
	size_t len;
	int error;
	uint16_t etype;		/* host order */

	error = ether_recv(f, pkt, tleft, &etype);
	if (error) {
		return error;
	}
	error = packet_get_layer(pkt, (void **)&ah, &len);
	if (error) {
		return error;
	}

	if (len < sizeof(*ah)) {
		return -1;
	}

	if (etype != ETHERTYPE_ARP) {
		return -1;
	}

	if (ah->arp_hrd != htons(ARPHRD_ETHER) ||
	    ah->arp_pro != htons(ETHERTYPE_IP) ||
	    ah->arp_hln != sizeof(ah->arp_sha) ||
	    ah->arp_pln != sizeof(ah->arp_spa) )
	{
		return -1;
	}

	if (ah->arp_op == htons(ARPOP_REQUEST)) {
		arp_reply(f, pkt);
		return -1;
	}

	if (ah->arp_op != htons(ARPOP_REPLY)) {
		return -1;
	}

	/* Is the reply from the source we want? */
	if (memcmp(&arp_list[arp_num].addr,
			 ah->arp_spa, sizeof(ah->arp_spa)))
	{
		return -1;
	}
	/* We don't care who the reply was sent to. */

	/* We have our answer. */
	return 0;
}

/*
 * Convert an ARP request into a reply and send it.
 * Notes:  Re-uses buffer.  Pad to length = 46.
 */
void
arp_reply(struct open_file *f, struct packet *pkt)
{
	struct ether_arp *arp;
	size_t len;
	int error;

	error = packet_get_layer(pkt, (void **)&arp, &len);
	if (error) {
		return;
	}
	if (len < sizeof(*arp)) {
		return;
	}

	if (arp->arp_hrd != htons(ARPHRD_ETHER) ||
	    arp->arp_pro != htons(ETHERTYPE_IP) ||
	    arp->arp_hln != sizeof(arp->arp_sha) ||
	    arp->arp_pln != sizeof(arp->arp_spa) )
	{
		return;
	}

	if (arp->arp_op != htons(ARPOP_REQUEST)) {
		return;
	}

	/* If we are not the target, ignore the request. */
	if (memcmp(arp->arp_tpa, &f->f_net.f_enaddr, sizeof(arp->arp_tpa)))
		return;

	arp->arp_op = htons(ARPOP_REPLY);
	/* source becomes target */
	(void)memcpy(arp->arp_tha, arp->arp_sha, sizeof(arp->arp_tha));
	(void)memcpy(arp->arp_tpa, arp->arp_spa, sizeof(arp->arp_tpa));
	/* here becomes source */
	(void)memcpy(arp->arp_sha, f->f_net.f_enaddr, sizeof(arp->arp_sha));
	(void)memcpy(arp->arp_spa, &f->f_net.f_laddr, sizeof(arp->arp_spa));

	/*
	 * No need to get fancy here.  If the send fails, the
	 * requestor will just ask again.
	 */
	ether_send(f, pkt, arp->arp_tha, ETHERTYPE_ARP);
}

/* Broadcast an ARP packet, asking who has addr on interface d */
const uint8_t *
arp_whohas(struct open_file *f, in_addr_t addr)
{
	int error, i;
	struct ether_arp arpreq;
	struct ether_arp *ah = &arpreq;
	struct arp_list *al;
	size_t len;
	void *rbuf;
	size_t rbuflen;
	char addrstr[INET_ADDR_STRLEN];
	const uint8_t *ret = NULL;
	struct packet txpkt, rxpkt;

	/* Try for cached answer first */
	for (i = 0, al = arp_list; i < arp_num; ++i, ++al) {
		if (addr == al->addr) {
			return al->ea;
		}
	}

	rbuf = dev_alloc_pktbuf(f, &rbuflen);
	if (rbuf == NULL) {
		printf("%s: out of memory\n", __func__);
		return NULL;
	}

	/* Don't overflow cache */
	if (arp_num > ARP_NUM - 1) {
		arp_num = 1;	/* recycle */
		printf("%s: overflowed arp_list!\n", __func__);
	}

	(void)memset(ah, 0, sizeof(*ah));
	ah->arp_hrd = htons(ARPHRD_ETHER);
	ah->arp_pro = htons(ETHERTYPE_IP);
	ah->arp_hln = sizeof(ah->arp_sha); /* hardware address length */
	ah->arp_pln = sizeof(ah->arp_spa); /* protocol address length */
	ah->arp_op = htons(ARPOP_REQUEST);
	memcpy(ah->arp_sha, f->f_net.f_enaddr, sizeof(ah->arp_sha));
	memcpy(ah->arp_spa, &f->f_net.f_laddr, sizeof(ah->arp_spa));
	/* Leave zeros in arp_tha */
	memcpy(ah->arp_tpa, &addr, sizeof(ah->arp_tpa));

	/* Store ip address in cache (incomplete entry). */
	al->addr = addr;

	packet_init_tx(&txpkt, ah, sizeof(*ah));
	packet_init_rx(&rxpkt, rbuf, sizeof(rbuf));
	error = sendrecv(f,
	    arp_send, &txpkt,
	    arp_recv, &rxpkt);
	if (error == 0) {
		error = packet_get_layer(&rxpkt, (void **)&ah, &len);
	}
	if (error) {
		printf("%s: no response for %s\n", __func__,
		    inet_ntoa(addr, addrstr, sizeof(addrstr)));
		goto out;
	}

	/* Store ethernet address in cache */
	if (len < sizeof(*ah)) {
		printf("%s: malformed response for %s\n", __func__,
		    inet_ntoa(addr, addrstr, sizeof(addrstr)));
		goto out;
	}
	memcpy(al->ea, ah->arp_sha, sizeof(al->ea));
	++arp_num;

 	ret = al->ea;
 out:
	free(rbuf);
	return ret;
}
