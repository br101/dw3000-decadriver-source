#include <driver/spi_master.h>
#include <string.h>

#include "deca_device_api.h"
#include "dw3000_spi.h"
#include "log.h"

/* This file implements the SPI functions required by decadriver */

#define DECA_SPI_HOST SPI2_HOST

static const char* LOG_TAG = "DW3000";

static spi_device_handle_t dw_spi;
static spi_device_interface_config_t dw_cfg = {
	.clock_speed_hz = 2000000, // Slow: 2MHz
	.spics_io_num = CONFIG_DW3000_SPI_CS,
	.mode = 0,
	.queue_size = 1,
};

#if DW3000_SPI_TRACE
void dw3000_spi_trace_in(bool rw, const uint8_t* headerBuffer,
						 uint16_t headerLength, const uint8_t* bodyBuffer,
						 uint16_t bodyLength);
#endif

int dw3000_spi_init(void)
{
	esp_err_t ret;

	LOG_INF("SPI Init (MOSI:%d MISO:%d CLK:%d CS:%d)", CONFIG_DW3000_SPI_MOSI,
			CONFIG_DW3000_SPI_MISO, CONFIG_DW3000_SPI_CLK,
			CONFIG_DW3000_SPI_CS);

	spi_bus_config_t buscfg = {
		.mosi_io_num = CONFIG_DW3000_SPI_MOSI,
		.miso_io_num = CONFIG_DW3000_SPI_MISO,
		.sclk_io_num = CONFIG_DW3000_SPI_CLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.flags = SPICOMMON_BUSFLAG_MASTER,
		.intr_flags = ESP_INTR_FLAG_LEVEL2,
	};

	// Initialize the SPI bus
	ret = spi_bus_initialize(DECA_SPI_HOST, &buscfg, SPI_DMA_DISABLED);
	assert(ret == ESP_OK);

	// Add a device to the bus
	ret = spi_bus_add_device(DECA_SPI_HOST, &dw_cfg, &dw_spi);
	assert(ret == ESP_OK);

	return ret;
}

static int dw3000_spi_speed_set(int hz)
{
	spi_bus_remove_device(dw_spi);

	dw_cfg.clock_speed_hz = hz;

	int rc = spi_bus_add_device(DECA_SPI_HOST, &dw_cfg, &dw_spi);
	if (rc) {
		LOG_ERR("Set SPI speed error %x", rc);
	}
	return rc;
}

void dw3000_spi_speed_slow(void)
{
	dw3000_spi_speed_set(2000000); // 2 MHz
}

void dw3000_spi_speed_fast(void)
{
	/* DW3000 apparently supports up to 38 MHz.
	 * ESP documentation says: full-duplex transfers routed over the GPIO matrix
	 * only support speeds up to 26MHz.
	 * In my tests on the Makerfabs UWB DW3000 board at 24MHz we start to get
	 * wrong results reading devid */
	dw3000_spi_speed_set(22000000); // 22 MHz
}

void dw3000_spi_fini(void)
{
	spi_bus_remove_device(dw_spi);
	spi_bus_free(DECA_SPI_HOST);
}

int dw3000_spi_write_crc(uint16_t headerLength, const uint8_t* headerBuffer,
						 uint16_t bodyLength, const uint8_t* bodyBuffer,
						 uint8_t crc8)
{
	LOG_ERR("WRITE WITH CRC NOT IMPLEMENTED!");
	return DWT_ERROR;
}

int dw3000_spi_write(uint16_t headerLength, const uint8_t* headerBuffer,
					 uint16_t bodyLength, const uint8_t* bodyBuffer)
{
#if 1
	static uint8_t txbuf[100];

	if (headerLength + bodyLength > 100) {
		LOG_ERR("TX buffer too small");
		return ESP_ERR_INVALID_SIZE;
	}

	decaIrqStatus_t stat = decamutexon();

	memcpy(txbuf, headerBuffer, headerLength);
	memcpy(txbuf + headerLength, bodyBuffer, bodyLength);

	spi_transaction_t th = {
		.length = headerLength * 8 + bodyLength * 8,
		.tx_buffer = txbuf,
	};

#if DW3000_SPI_TRACE
	dw3000_spi_trace_in(false, headerBuffer, headerLength, bodyBuffer,
						bodyLength);
#endif

	esp_err_t ret = spi_device_polling_transmit(dw_spi, &th);
	if (ret != ESP_OK) {
		LOG_ERR("SPI ERR");
	}

	decamutexoff(stat);
	return ret;

#else

	// TODO, this would be nice because we don't have to copy the data, but it
	// results in the wrong data transmitted

	spi_transaction_t th = {
		.flags = SPI_TRANS_CS_KEEP_ACTIVE,
		.length = headerLength * 8,
		.tx_buffer = headerBuffer,
	};

	decaIrqStatus_t stat = decamutexon();

#if DW3000_SPI_TRACE
	dw3000_spi_trace_in(false, headerBuffer, headerLength, bodyBuffer,
						bodyLength);
#endif

	spi_device_acquire_bus(dw_spi, portMAX_DELAY);

	esp_err_t ret = spi_device_polling_transmit(dw_spi, &th);
	if (ret != ESP_OK) {
		goto exit;
	}

	if (bodyLength > 0) {
		spi_transaction_t tb = {
			.length = bodyLength * 8,
			.tx_buffer = bodyBuffer,
		};

		ret = spi_device_polling_transmit(dw_spi, &tb);
	}

exit:
	spi_device_release_bus(dw_spi);
	decamutexoff(stat);
	return ret;
#endif
}

int dw3000_spi_read(uint16_t headerLength, uint8_t* headerBuffer,
					uint16_t readLength, uint8_t* readBuffer)
{
	spi_transaction_t th = {
		.flags = SPI_TRANS_CS_KEEP_ACTIVE,
		.length = headerLength * 8,
		.tx_buffer = headerBuffer,
	};

	decaIrqStatus_t stat = decamutexon();
	spi_device_acquire_bus(dw_spi, portMAX_DELAY);

	esp_err_t ret = spi_device_polling_transmit(dw_spi, &th);
	if (ret != ESP_OK) {
		goto exit;
	}

	spi_transaction_t tr = {
		.length = readLength * 8,
		.tx_buffer = NULL,
		.rxlength = readLength * 8,
		.rx_buffer = readBuffer,
	};

	ret = spi_device_polling_transmit(dw_spi, &tr);

#if DW3000_SPI_TRACE
	dw3000_spi_trace_in(true, headerBuffer, headerLength, readBuffer,
						readLength);
#endif

exit:
	spi_device_release_bus(dw_spi);
	decamutexoff(stat);
	return ret;
}
