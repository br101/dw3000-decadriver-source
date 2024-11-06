#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_error.h"
#include "nrf_gpio.h"
#include "nrfx_spim.h"

#include "config.h"
#include "deca_device_api.h"
#include "dw3000_spi.h"
#include "log.h"

/* This file implements the SPI functions used by deca_port.c*/

#define DW3000_SPI_INSTANCE 0

static const char* LOG_TAG = "DW3000";
static const nrfx_spim_t dw_spi = NRFX_SPIM_INSTANCE(DW3000_SPI_INSTANCE);

#if CONFIG_DW3000_SPI_TRACE
void dw3000_spi_trace_in(bool rw, const uint8_t* headerBuffer,
						 uint16_t headerLength, const uint8_t* bodyBuffer,
						 uint16_t bodyLength);
#endif

int dw3000_spi_init(void)
{
	nrfx_err_t ret;

	LOG_INF("SPI Init (MOSI:%d MISO:%d CLK:%d CS:%d)", CONFIG_DW3000_SPI_MOSI,
			CONFIG_DW3000_SPI_MISO, CONFIG_DW3000_SPI_CLK,
			CONFIG_DW3000_SPI_CS);

	nrfx_spim_config_t spi_config = {
		.sck_pin = CONFIG_DW3000_SPI_CLK,
		.mosi_pin = CONFIG_DW3000_SPI_MOSI,
		.miso_pin = CONFIG_DW3000_SPI_MISO,
		.ss_pin = NRFX_SPIM_PIN_NOT_USED,
		.irq_priority = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY,
		.orc = 0xFF,
		.frequency = NRF_SPIM_FREQ_2M,
		.mode = NRF_SPIM_MODE_0,
		.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST,
	};

	ret = nrfx_spim_init(&dw_spi, &spi_config, NULL, NULL);
	if (ret != NRFX_SUCCESS) {
		LOG_ERR("SPI init failed (error 0x%lx)", ret);
		return ret;
	}

	/*
	 * Slave select must be set as high before setting it as output,
	 * otherwise it can cause glitches (nRF source code)
	 */
	nrf_gpio_pin_set(CONFIG_DW3000_SPI_CS);
	nrf_gpio_cfg_output(CONFIG_DW3000_SPI_CS);
	nrf_gpio_pin_set(CONFIG_DW3000_SPI_CS);

	return NRF_SUCCESS;
}

void dw3000_spi_speed_slow(void)
{
	nrf_spim_frequency_set(NRF_SPIM0, NRF_SPIM_FREQ_2M);
}

void dw3000_spi_speed_fast(void)
{
	if (CONFIG_DW3000_SPI_MAX_MHZ == 8) {
		nrf_spim_frequency_set(NRF_SPIM0, NRF_SPIM_FREQ_8M);
#if defined(SPIM_FREQUENCY_FREQUENCY_M16)
	} else if (CONFIG_DW3000_SPI_MAX_MHZ == 16) {
		nrf_spim_frequency_set(NRF_SPIM0, NRF_SPIM_FREQ_16M);
#endif
#if defined(SPIM_FREQUENCY_FREQUENCY_M16)
	} else if (CONFIG_DW3000_SPI_MAX_MHZ == 32) {
		nrf_spim_frequency_set(NRF_SPIM0, NRF_SPIM_FREQ_32M);
#endif
	} else {
		LOG_ERR("Unknown SPI speed %d MHz", CONFIG_DW3000_SPI_MAX_MHZ);
		return;
	}
}

void dw3000_spi_fini(void)
{
	nrfx_spim_uninit(&dw_spi);

	// TODO?
	// nrf_gpio_cfg_default(CONFIG_DW3000_SPI_CS);
}

int32_t dw3000_spi_write_crc(uint16_t headerLength, const uint8_t* headerBuffer,
							 uint16_t bodyLength, const uint8_t* bodyBuffer,
							 uint8_t crc8)
{
	LOG_ERR("WRITE WITH CRC NOT IMPLEMENTED!");
	return DWT_ERROR;
}

int32_t dw3000_spi_write(uint16_t headerLength, const uint8_t* headerBuffer,
						 uint16_t bodyLength, const uint8_t* bodyBuffer)
{
	decaIrqStatus_t stat = decamutexon();

#if CONFIG_DW3000_SPI_TRACE
	dw3000_spi_trace_in(false, headerBuffer, headerLength, bodyBuffer,
						bodyLength);
#endif

	nrf_gpio_pin_clear(CONFIG_DW3000_SPI_CS);

	nrfx_spim_xfer_desc_t hdr = {
		.p_tx_buffer = headerBuffer,
		.tx_length = headerLength,
		.p_rx_buffer = NULL,
		.rx_length = 0,
	};

	nrfx_err_t ret = nrfx_spim_xfer(&dw_spi, &hdr, 0);
	if (ret != NRFX_SUCCESS) {
		LOG_ERR("SPI error");
		goto exit;
	}

	nrfx_spim_xfer_desc_t bdy = {
		.p_tx_buffer = bodyBuffer,
		.tx_length = bodyLength,
		.p_rx_buffer = NULL,
		.rx_length = 0,
	};

	ret = nrfx_spim_xfer(&dw_spi, &bdy, 0);
	if (ret != NRFX_SUCCESS) {
		LOG_ERR("SPI error");
	}

exit:
	nrf_gpio_pin_set(CONFIG_DW3000_SPI_CS);
	decamutexoff(stat);
	return ret == NRFX_SUCCESS ? DWT_SUCCESS : DWT_ERROR;
}

int32_t dw3000_spi_read(uint16_t headerLength, uint8_t* headerBuffer,
						uint16_t readLength, uint8_t* readBuffer)
{
	decaIrqStatus_t stat = decamutexon();
	nrf_gpio_pin_clear(CONFIG_DW3000_SPI_CS);

	nrfx_spim_xfer_desc_t hdr = {
		.p_tx_buffer = headerBuffer,
		.tx_length = headerLength,
		.p_rx_buffer = NULL,
		.rx_length = 0,
	};

	nrfx_err_t ret = nrfx_spim_xfer(&dw_spi, &hdr, 0);
	if (ret != NRFX_SUCCESS) {
		LOG_ERR("SPI error");
		goto exit;
	}

	nrfx_spim_xfer_desc_t bdy = {
		.p_tx_buffer = NULL,
		.tx_length = 0,
		.p_rx_buffer = readBuffer,
		.rx_length = readLength,
	};

	ret = nrfx_spim_xfer(&dw_spi, &bdy, 0);
	if (ret != NRFX_SUCCESS) {
		LOG_ERR("SPI error");
		goto exit;
	}

#if CONFIG_DW3000_SPI_TRACE
	dw3000_spi_trace_in(true, headerBuffer, headerLength, readBuffer,
						readLength);
#endif

exit:
	nrf_gpio_pin_set(CONFIG_DW3000_SPI_CS);
	decamutexoff(stat);
	return ret == NRFX_SUCCESS ? DWT_SUCCESS : DWT_ERROR;
}
