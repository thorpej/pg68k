/*	From NetBSD: rpc.c,v 1.31 2019/04/05 20:09:29 christos Exp	*/

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
 * @(#) Header: rpc.c,v 1.12 93/09/28 08:31:56 leres Exp  (LBL)
 */

/*
 * RPC functions used by NFS and bootparams.
 * Note that bootparams requires the ability to find out the
 * address of the server from which its response has come.
 * This is supported by keeping the IP/UDP headers in the
 * buffer space provided by the caller.  (See rpc_fromaddr)
 */

#include "syslib.h"
#include "sysfile.h"
#include "net.h"
#include "rpc.h"
#include "rpcv2.h"

struct auth_info {
	int32_t 	authtype;	/* auth type */
	uint32_t	authlen;	/* auth length */
};

struct auth_unix {
	int32_t   ua_time;
	int32_t   ua_hostname;	/* null */
	int32_t   ua_uid;
	int32_t   ua_gid;
	int32_t   ua_gidlist;	/* null */
};

struct rpc_call {
	uint32_t	rp_xid;		/* request transaction id */
	int32_t 	rp_direction;	/* call direction (0) */
	uint32_t	rp_rpcvers;	/* rpc version (2) */
	uint32_t	rp_prog;	/* program */
	uint32_t	rp_vers;	/* version */
	uint32_t	rp_proc;	/* procedure */
};

struct rpc_reply {
	uint32_t	rp_xid;		/* request transaction id */
	int32_t 	rp_direction;	/* call direction (1) */
	int32_t 	rp_astatus;	/* accept status (0: accepted) */
	union {
		uint32_t	rpu_errno;
		struct {
			struct auth_info rok_auth;
			uint32_t	rok_status;
		} rpu_rok;
	} rp_u;
};

/* Local forwards */
static int	rpc_getport(struct open_file *, n_long, n_long, uint16_t *);
static int	rpc_recv(struct open_file *, struct packet *, int);

int rpc_xid;
int rpc_port = 0x400;	/* predecrement */

int
rpc_call(struct open_file *f, n_long prog, n_long vers, n_long proc,
	struct packet *txpkt, struct packet *rxpkt)
{
	struct {
		struct rpc_call call;
		struct auth_info creds_info;
		struct auth_unix creds;
		struct auth_info verifier_info;
	} rpc_header;
	struct rpc_call *call;
	struct auth_info *auth;
	struct rpc_reply *reply;
	size_t replen;
	n_long x;
	int error;
	uint16_t port;	/* host order */

	if (!DEV_IS_NETDEV(f->f_dev)) {
		return EIO;
	}

#ifdef RPC_DEBUG
	if (debug)
		printf("%s: prog=0x%x vers=%d proc=%d\n", __func__,
		    prog, vers, proc);
#endif

	error = rpc_getport(f, prog, vers, &port);
	if (error) {
		return error;
	}
	f->f_net.f_rport = htons(port);

	memset(&rpc_header, 0, sizeof(rpc_header));

	/* Auth verifier is always auth_null */
	auth = &rpc_header.verifier_info;
	auth->authtype = htonl(RPCAUTH_NULL);
	auth->authlen = 0;

	/* Auth credentials: always auth unix (as root) */
	auth = &rpc_header.creds_info;
	auth->authtype = htonl(RPCAUTH_UNIX);
	auth->authlen = htonl(sizeof(rpc_header.creds));

	/* RPC call structure. */
	call = &rpc_header.call;
	rpc_xid++;
	call->rp_xid       = htonl(rpc_xid);
	call->rp_direction = htonl(RPC_CALL);
	call->rp_rpcvers   = htonl(RPC_VER2);
	call->rp_prog = htonl(prog);
	call->rp_vers = htonl(vers);
	call->rp_proc = htonl(proc);

	error = sendrecv(f,
	    udp_send, txpkt,
	    rpc_recv, rxpkt);

	if (error == 0) {
		error = packet_get_layer(rxpkt, (void **)&reply, &replen);
	}
	if (error) {
		return error;
	}

	if (replen <= sizeof(*reply)) {
		return EBADRPC;
	}

	/*
	 * Check the RPC reply status.
	 * The xid, dir, astatus were already checked.
	 */
	auth = &reply->rp_u.rpu_rok.rok_auth;
	x = ntohl(auth->authlen);
	if (x != 0) {
#ifdef RPC_DEBUG
		if (debug)
			printf("%s: reply auth != NULL\n", __func__);
#endif
		return EBADRPC;
	}
	x = ntohl(reply->rp_u.rpu_rok.rok_status);
	if (x != 0) {
		printf("%s: error = %d\n", __func__, x);
		return EBADRPC;
	}

	return packet_advance(rxpkt, sizeof(*reply));
}

/*
 * Returns true if packet is the one we're waiting for.
 * This just checks the XID, direction, acceptance.
 * Remaining checks are done by callrpc
 */
static int
rpc_recv(struct open_file *f, struct packet *pkt, int tleft)
{
	struct rpc_reply *reply;
	size_t	replen;
	int	x;
	int	error;

	error = udp_recv(f, pkt, tleft);
	if (error) {
		return error;
	}
	error = packet_get_layer(pkt, (void **)&reply, &replen);
	if (replen <= (4 * 4)) {
		return -1;
	}

	x = ntohl(reply->rp_xid);
	if (x != rpc_xid) {
#ifdef RPC_DEBUG
		if (debug)
			printf("%s: rp_xid %d != xid %d\n",
			    __func__, x, rpc_xid);
#endif
		return -1;
	}

	x = ntohl(reply->rp_direction);
	if (x != RPC_REPLY) {
#ifdef RPC_DEBUG
		if (debug)
			printf("%s: rp_direction %d != REPLY\n", __func__, x);
#endif
		return -1;
	}

	x = ntohl(reply->rp_astatus);
	if (x != RPC_MSGACCEPTED) {
		error = ntohl(reply->rp_u.rpu_errno);
		printf("%s: reject, astat=%d, errno=%d\n", __func__, x, error);
		return -1;
	}

	/* rpc_call() will advance the packet. */
	return 0;
}

/*
 * RPC Portmapper cache
 */
#define PMAP_NUM 8			/* need at most 5 pmap entries */

static int rpc_pmap_num;
static struct pmap_list {
	in_addr_t addr;		/* server, net order */
	u_int	prog;		/* host order */
	u_int	vers;		/* host order */
	int 	port;		/* host order */
} rpc_pmap_list[PMAP_NUM];

/*
 * return port number in host order, or -1.
 * arguments are:
 *  addr .. server, net order.
 *  prog .. host order.
 *  vers .. host order.
 */
static int
rpc_pmap_getcache(in_addr_t addr, u_int prog, u_int vers)
{
	struct pmap_list *pl;

	for (pl = rpc_pmap_list; pl < &rpc_pmap_list[rpc_pmap_num]; pl++) {
		if (pl->addr == addr && pl->prog == prog && pl->vers == vers) {
			return pl->port;
		}
	}
	return -1;
}

/*
 * arguments are:
 *  addr .. server, net order.
 *  prog .. host order.
 *  vers .. host order.
 *  port .. host order.
 */
static void
rpc_pmap_putcache(in_addr_t addr, u_int prog, u_int vers, int port)
{
	struct pmap_list *pl;

	/* Don't overflow cache... */
	if (rpc_pmap_num >= PMAP_NUM) {
		/* ... just re-use the last entry. */
		rpc_pmap_num = PMAP_NUM - 1;
	}

	pl = &rpc_pmap_list[rpc_pmap_num];
	rpc_pmap_num++;

	/* Cache answer */
	pl->addr = addr;
	pl->prog = prog;
	pl->vers = vers;
	pl->port = port;
}

/*
 * Request a port number from the port mapper.
 * Returns the port in host order.
 * prog and vers are host order.
 */
static int
rpc_getport(struct open_file *f, n_long prog, n_long vers, uint16_t *portp)
{
	struct args {
		n_long	prog;		/* call program */
		n_long	vers;		/* call version */
		n_long	proto;		/* call protocol */
		n_long	port;		/* call port (unused) */
	} args;
	struct res {
		n_long port;
	} *res;
	struct packet txpkt, rxpkt;
	void *rbuf;
	size_t rbuflen, reslen;
	int port, error;

#ifdef RPC_DEBUG
	if (debug)
		printf("%s: prog=0x%x vers=%d\n", __func__, prog, vers);
#endif

	if (!DEV_IS_NETDEV(f->f_dev)) {
		return EIO;
	}

	/* This one is fixed forever. */
	if (prog == PMAPPROG) {
		*portp = PMAPPORT;
		return 0;
	}

	/* Try for cached answer first */
	port = rpc_pmap_getcache(f->f_net.f_raddr, prog, vers);
	if (port != -1) {
		*portp = (uint16_t)port;
		return 0;
	}

	rbuf = dev_alloc_pktbuf(f, &rbuflen);
	if (rbuf == NULL) {
		printf("%s: out of memory\n", __func__);
		return ENOMEM;
	}

	args.prog = htonl(prog);
	args.vers = htonl(vers);
	args.proto = htonl(IPPROTO_UDP);
	args.port = 0;

	packet_init_tx(&txpkt, &args, sizeof(args));
	packet_init_rx(&rxpkt, rbuf, rbuflen);
	error = rpc_call(f, PMAPPROG, PMAPVERS, PMAPPROC_GETPORT,
	    &txpkt, &txpkt);
	if (error == 0) {
		error = packet_get_layer(&rxpkt, (void **)&res, &reslen);
	}
	if (error) {
		printf("%s: %s", __func__, strerror(error));
		error = EBADRPC;
		goto out;
	}
	if (reslen < sizeof(*res)) {
		printf("%s: malformed response\n", __func__);
		error = EBADRPC;
		goto out;
	}

	port = (int)ntohl(res->port);
	*portp = (uint16_t)port;

	rpc_pmap_putcache(f->f_net.f_raddr, prog, vers, port);
 out:
	free(rbuf);
	return error;

}
