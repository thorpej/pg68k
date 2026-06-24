/*
 * Copyright (c) 2026 Jason R. Thorpe.
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

#include "syslib.h"
#include "console.h"
#include "uart.h"

#include "srec.h"

typedef enum {
	STATE_TYPE_0		= 0,
	STATE_TYPE_1,
	STATE_BCOUNT_0,
	STATE_BCOUNT_1,
	STATE_ADDR,
	STATE_DATA,
	STATE_CKSUM_0,
	STATE_CKSUM_1,
	STATE_EOR,
	STATE_SKIP,
	STATE_DONE,		/* terminal states start here */
	STATE_CANCELLED,
	STATE_ABORTED,
} srec_state_t;

struct srec_context {
	int		unit;
	u_int		record_count;
	u_int		data_record_count;
	u_int		error_count;
	srec_state_t	state;
	uintptr_t	addr;
	uintptr_t	first_addr;
	uintptr_t	last_addr;
	uint16_t	addr_nybbles;
	uint16_t	data_nybbles;
	uint8_t		curbyte;
	uint8_t		cksum;
	bool		half_byte;
	bool		last_record;	/* no data in last record */
	char		error_msg[80];
};

static void __printflike(2,3)
srec_error(struct srec_context *ctx, const char *fmt, ...)
{
	va_list ap;

	ctx->error_count++;

	/*
	 * Save and print the first error.  Any subsequent errors are
	 * likely to be due to loss of synchronization with the stream.
	 */
	if (ctx->error_msg[0] == '\0') {
		va_start(ap, fmt);
		vsnprintf(ctx->error_msg, sizeof(ctx->error_msg), fmt, ap);
		va_end(ap);
	}
}

static int
srec_getc(struct srec_context *ctx)
{
	int ch;

	for (ch = -1; ch == -1;) {
		/*
		 * If we're using a secondary UART for the transfer,
		 * check for cancellation on the console.
		 */
		if (ctx->unit != CONFIG_CONSOLE_UART) {
			ch = cons_pollc();
			if (ch != 3) {	/* ETX - ^C */
				ch = -1;
			}
		}
		if (ch == -1) {
			ch = uart_pollc(ctx->unit);
		}
	}

	return ch;
}

static int
srec_get_nybble(struct srec_context *ctx, int ch)
{
	uint8_t val;

	if (ch >= '0' && ch <= '9') {
		val = ch - '0';
	} else if (ch >= 'A' && ch <= 'F') {
		val = 10 + (ch - 'A');
	} else {
		return -1;
	}
	ctx->curbyte = (ctx->curbyte << 4) | val;
	ctx->half_byte ^= true;
	if (ctx->state >= STATE_BCOUNT_0 &&
	    ctx->state < STATE_CKSUM_0 &&
	    !ctx->half_byte) {
		ctx->cksum += ctx->curbyte;
	}
	return ctx->curbyte;
}

static srec_state_t
srec_get_type(struct srec_context *ctx, int ch)
{
	ctx->addr = 0;
	ctx->addr_nybbles = 0;
	ctx->data_nybbles = 0;
	ctx->curbyte = 0;
	ctx->cksum = 0;

	if (ctx->state == STATE_TYPE_0) {
		if (ch == 'S') {
			return STATE_TYPE_1;
		}
		goto bad_record;
	}

	ctx->record_count++;

	switch (ch) {
	case '0':	/* header record; just skip it. */
		return STATE_SKIP;
	
	case '1':	/* data with 16-bit address */
		ctx->data_record_count++;
		ctx->addr_nybbles = (16 / 8) * 2;
		return STATE_BCOUNT_0;

	case '2':	/* data with 24-bit address */
		ctx->data_record_count++;
		ctx->addr_nybbles = (24 / 8) * 2;
		return STATE_BCOUNT_0;

	case '3':	/* data with 32-bit address */
		ctx->data_record_count++;
		ctx->addr_nybbles = (32 / 8) * 2;
		return STATE_BCOUNT_0;

	case '4':	/* undefined */
		return STATE_SKIP;

	case '5':	/* unofficial 16-bit S1/2/3 record count */
		return STATE_SKIP;

	case '6':	/* unofficial 24-bit S1/2/3 record count */
		return STATE_SKIP;

	case '7':	/* 32-bit start address */
		ctx->addr_nybbles = (32 / 8) * 2;
		ctx->last_record = true;
		return STATE_BCOUNT_0;

	case '8':	/* 24-bit start address */
		ctx->addr_nybbles = (24 / 8) * 2;
		ctx->last_record = true;
		return STATE_BCOUNT_0;

	case '9':	/* 16-bit start address */
		ctx->addr_nybbles = (16 / 8) * 2;
		ctx->last_record = true;
		return STATE_BCOUNT_0;

	default:
		break;
	}

 bad_record:
	srec_error(ctx, "SREC #%u: malformed record type", ctx->record_count);
	return STATE_ABORTED;
}

static srec_state_t
srec_get_bcount(struct srec_context *ctx, int ch)
{
	int val = srec_get_nybble(ctx, ch);
	int addr_len = ctx->addr_nybbles / 2;

	if (val == -1) {
		goto bad_record;
	}

	if (ctx->state == STATE_BCOUNT_0) {
		return STATE_BCOUNT_1;
	}

	/*
	 * The byte count must be large enough for the address
	 * field and the checksum.
	 */
	if (val < addr_len + 1) {
		goto bad_length;
	}

	ctx->data_nybbles = (val - (addr_len + 1)) * 2;
	return STATE_ADDR;

 bad_record:
	srec_error(ctx, "SREC #%u: malformed length", ctx->record_count);
	return STATE_ABORTED;

 bad_length:
	srec_error(ctx, "SREC #%u: bad record length %d",
	    ctx->record_count, val);
	return STATE_ABORTED;
}

static srec_state_t
srec_get_addr(struct srec_context *ctx, int ch)
{
	int val = srec_get_nybble(ctx, ch);

	if (val == -1) {
		goto bad_record;
	}

	ctx->addr_nybbles--;
	if ((ctx->addr_nybbles & 1) == 0) {
		ctx->addr = (ctx->addr << 8) | (uint8_t)val;
	}
	if (ctx->addr_nybbles == 0) {
		if (ctx->data_nybbles) {
			if (ctx->last_record) {
				goto unexpected_data;
			}
			return STATE_DATA;
		}
		return STATE_CKSUM_0;
	}

	return ctx->state;

 bad_record:
	srec_error(ctx, "SREC #%u: malformed address", ctx->record_count);
	return STATE_ABORTED;

 unexpected_data:
	srec_error(ctx, "SREC #%u: unexpected data", ctx->record_count);
	return STATE_ABORTED;
}

static srec_state_t
srec_get_data(struct srec_context *ctx, int ch)
{
	int val = srec_get_nybble(ctx, ch);

	if (val == -1) {
		goto bad_record;
	}

	ctx->data_nybbles--;
	if ((ctx->data_nybbles & 1) == 0) {
		*(uint8_t *)ctx->addr = (uint8_t)val;
		if (ctx->addr < ctx->first_addr) {
			ctx->first_addr = ctx->addr;
		}
		if (ctx->addr > ctx->last_addr) {
			ctx->last_addr = ctx->addr;
		}
		ctx->addr++;
	}
	if (ctx->data_nybbles == 0) {
		return STATE_CKSUM_0;
	}

	return ctx->state;

 bad_record:
	srec_error(ctx, "SREC #%u: malformed data", ctx->record_count);
	return STATE_ABORTED;
}

static srec_state_t
srec_get_cksum(struct srec_context *ctx, int ch)
{
	int val = srec_get_nybble(ctx, ch);
	uint8_t cksum;

	if (val == -1) {
		goto bad_record;
	}

	if (ctx->state == STATE_CKSUM_0) {
		return STATE_CKSUM_1;
	}

	cksum = 0xff - ctx->cksum;
	if (cksum != val) {
		goto bad_checksum;
	}

	return STATE_EOR;

 bad_record:
	srec_error(ctx, "SREC #%u: malformed checksum", ctx->record_count);
	return STATE_ABORTED;

 bad_checksum:
	srec_error(ctx, "SREC #%u: bad checksum (0x%02x != 0x%02x)",
	    ctx->record_count, cksum, val);
	return STATE_ABORTED;
}

static srec_state_t
srec_get_eor(struct srec_context *ctx, int ch)
{
	if (ch == '\r' || ch == '\n') {
		return ctx->last_record ? STATE_DONE : STATE_TYPE_0;
	}

	srec_error(ctx, "SREC #%u: malformed end-of-record",
	    ctx->record_count);
	return STATE_ABORTED;
}

static srec_state_t
srec_skip_record(struct srec_context *ctx, int ch)
{
	if (ch == '\r' || ch == '\n') {
		return STATE_TYPE_0;
	}
	return STATE_SKIP;
}

static srec_state_t
srec_process_char(struct srec_context *ctx, int ch)
{
	if (ch == 3) {	/* ETX - ^C */
		return STATE_CANCELLED;
	}

	switch (ctx->state) {
	case STATE_TYPE_0:
		/* Skip newline-adjacent characters. */
		if (ch == '\r' || ch == '\n') {
			return ctx->state;
		}
		/* FALLTHROUGH */
	case STATE_TYPE_1:
		return srec_get_type(ctx, ch);
	
	case STATE_BCOUNT_0:
	case STATE_BCOUNT_1:
		return srec_get_bcount(ctx, ch);

	case STATE_ADDR:
		return srec_get_addr(ctx, ch);

	case STATE_DATA:
		return srec_get_data(ctx, ch);

	case STATE_CKSUM_0:
	case STATE_CKSUM_1:
		return srec_get_cksum(ctx, ch);

	case STATE_EOR:
		return srec_get_eor(ctx, ch);

	case STATE_SKIP:
		return srec_skip_record(ctx, ch);

	default:
		/* Invalid state. */
		srec_error(ctx, "SREC #%u: invalid state", ctx->record_count);
		return STATE_ABORTED;
	}
}

static void
srec_process(struct srec_context *ctx)
{
	srec_state_t next_state;
	int ch;

	for (;;) {
		ch = srec_getc(ctx);
		next_state = srec_process_char(ctx, ch);
		ctx->state = next_state;
		if (ctx->state >= STATE_DONE) {
			return;
		}
	}
}

bool
srec_load(int unit, uintptr_t *first_addrp, uintptr_t *last_addrp,
    uintptr_t *entryp)
{
	struct srec_context ctx = {
		.unit = unit,
		.first_addr = 0xffffffff,
	};

	srec_process(&ctx);

	if (ctx.state == STATE_DONE) {
		printf("Loaded %u data record%s (%u record%s total).\n",
		    ctx.data_record_count, plural(ctx.data_record_count),
		    ctx.record_count, plural(ctx.record_count));
		printf("Loaded image: 0x%08x - 0x%08x\n",
		    ctx.first_addr, ctx.last_addr);
		printf("Entry point: 0x%08x\n", ctx.addr);
		*first_addrp = ctx.first_addr;
		*last_addrp = ctx.last_addr;
		*entryp = ctx.addr;
		return true;
	}

	if (ctx.state == STATE_CANCELLED) {
		printf("S-Record load cancelled.\n");
		return false;
	}

	if (ctx.error_msg[0] != '\0') {
		printf("%s\n", ctx.error_msg);
	}
	printf("S-Record load aborted.\n");
	return false;
}
