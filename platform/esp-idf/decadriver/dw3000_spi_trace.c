#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "dw3000_spi.h"
#include "log.h"

#if DW3000_SPI_TRACE

#define DW3000_SPI_TRACE_RAW	  0
#define DW3000_SPI_TRACE_REALTIME 0 // Real-time output (normally too slow)

#define DBGS_CNT  256
#define DBGS_BODY 12

struct spi_dbg {
	bool rw;
	uint8_t hdr[2];
	uint8_t hdr_len;
	uint8_t bdy[DBGS_BODY];
	uint8_t bdy_len;
};

static const char* TAG = "DW3000_SPI";
static struct spi_dbg dbgs[DBGS_CNT];
static int dbgs_cnt;

static char* spi_dbg_out_reg(bool rw, const uint8_t* headerBuffer,
							 uint16_t headerLength)
{
	static char buf[64];
	uint8_t reg = 0;
	uint8_t mode = 0;
	uint8_t sub = 0;

	reg = (headerBuffer[0] & 0x3e) >> 1;
	mode = headerBuffer[0] & 0xc1;

	if (headerLength > 1) {
		sub = (headerBuffer[1] & 0xFC) >> 2;
		sub |= (headerBuffer[0] & 0x01) << 6;
	}

	char* modestr = "-unk-";
	if (mode == 0x81) {
		modestr = "fast";
	} else if (mode == 0x80) {
		modestr = "short write";
	} else if (mode == 0x00) {
		modestr = "short read";
	} else if ((mode & 0xc0) == 0x40) {
		modestr = "full read";
	} else if ((mode & 0xc0) == 0xc0) {
		uint8_t mask = headerBuffer[1] & 0x03;
		if (mask == 0x00) {
			modestr = "full write";
		} else if (mask == 0x01) {
			modestr = "masked 8bit";
		} else if (mask == 0x02) {
			modestr = "masked 16bit";
		} else if (mask == 0x03) {
			modestr = "masked 32bit";
		}
	}

	snprintf(buf, sizeof(buf), "SPI %s %02X:%02X (%s)", rw ? "READ" : "WRITE",
			 reg, sub, modestr);
	return buf;
}

void dw3000_spi_trace_in(bool rw, const uint8_t* headerBuffer,
						 uint16_t headerLength, const uint8_t* bodyBuffer,
						 uint16_t bodyLength)
{
#if DW3000_SPI_TRACE_RAW
	LOG_INF("---SPI #%d %s hdrlen %d bodylen %d", dbgs_cnt,
			rw ? "READ" : "WRITE", headerLength, bodyLength);
	LOG_HEXDUMP("   SPI HEADER: ", headerBuffer, headerLength);
	LOG_HEXDUMP("   SPI BODY: ", bodyBuffer, bodyLength);
#endif

	if (dbgs_cnt >= DBGS_CNT) {
		LOG_ERR("ERR not enough debug space");
		return;
	}

	dbgs[dbgs_cnt].rw = rw;

	if (headerLength <= 2) {
		memcpy(dbgs[dbgs_cnt].hdr, headerBuffer, headerLength);
	} else {
		LOG_ERR("ERR hdr len %d", headerLength);
	}

	dbgs[dbgs_cnt].hdr_len = headerLength;
	if (bodyLength > DBGS_BODY) {
		// LOG_ERR("ERR bdy len %d at #%d", bodyLength, dbgs_cnt);
		bodyLength = DBGS_BODY;
	}
	memcpy(dbgs[dbgs_cnt].bdy, bodyBuffer, bodyLength);
	dbgs[dbgs_cnt].bdy_len = bodyLength;

#if DW3000_SPI_TRACE_REALTIME
	char* s = spi_dbg_out_reg(dbgs[dbgs_cnt].rw, dbgs[dbgs_cnt].hdr,
							  dbgs[dbgs_cnt].hdr_len);
	if (dbgs[dbgs_cnt].bdy_len) {
		LOG_HEXDUMP(s, dbgs[dbgs_cnt].bdy, dbgs[dbgs_cnt].bdy_len);
	} else {
		LOG_INF("%s", s);
	}
#endif

	dbgs_cnt++;
}

#endif

void dw3000_spi_trace_output(void)
{
#if DW3000_SPI_TRACE
	LOG_INF("--- SPI DBG START");
	for (int i = 0; i < DBGS_CNT && i < dbgs_cnt; i++) {
		struct spi_dbg* d = &dbgs[i];
		// hexdump("   SPI HEADER: ", d->hdr, d->hdr_len);
		char* s = spi_dbg_out_reg(d->rw, d->hdr, d->hdr_len);
		if (d->bdy_len) {
			LOG_HEXDUMP(s, d->bdy, d->bdy_len);
		} else {
			LOG_INF("%s", s);
		}
	}
	dbgs_cnt = 0;
	LOG_INF("--- SPI DBG END");
#endif
}
