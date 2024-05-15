#include <stdbool.h>
#include <stdint.h>

#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "nrfx_spim.h"

#include "deca_device_api.h"
#include "dw3000_spi.h"
#include "log.h"

/* This file implements the SPI functions used by deca_port.c*/

#define DW3000_SPI_INSTANCE 0

static const char* LOG_TAG = "DW3000";
static const nrfx_spim_t dw_spi = NRFX_SPIM_INSTANCE(DW3000_SPI_INSTANCE);
static struct dw3000_hw_cfg* dw_hw_cfg;

#if DW3000_SPI_TRACE
void dw3000_spi_trace_in(bool rw, const uint8_t* headerBuffer,
						 uint16_t headerLength, const uint8_t* bodyBuffer,
						 uint16_t bodyLength);
#endif

int dw3000_spi_init(struct dw3000_hw_cfg* cfg)
{
	nrfx_err_t ret;
	dw_hw_cfg = cfg;

	LOG_INF("SPI Init (MOSI:%d MISO:%d CLK:%d CS:%d)", cfg->spi_mosi_pin,
			cfg->spi_miso_pin, cfg->spi_clk_pin, cfg->spi_cs_pin);

	nrfx_spim_config_t spi_config = {
		.sck_pin = cfg->spi_clk_pin,
		.mosi_pin = cfg->spi_mosi_pin,
		.miso_pin = cfg->spi_miso_pin,
		.ss_pin = NRFX_SPIM_PIN_NOT_USED,
		.irq_priority = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY,
		.orc = 0xFF,
		.frequency = NRF_SPIM_FREQ_2M,
		.mode = NRF_SPIM_MODE_0,
		.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST,
	};

	ret = nrfx_spim_init(&dw_spi, &spi_config, NULL, NULL);
	if (ret != NRFX_SUCCESS) {
		LOG_ERR("SPI init failed");
		return false;
	}

	/*
	 * Slave select must be set as high before setting it as output,
	 * otherwise it can cause glitches (nRF source code)
	 */
	nrf_gpio_pin_set(cfg->spi_cs_pin);
	nrf_gpio_cfg_output(cfg->spi_cs_pin);
	nrf_gpio_pin_set(cfg->spi_cs_pin);
	return true;
}

void dw3000_spi_speed_slow(void)
{
	nrf_spim_frequency_set(NRF_SPIM0, NRF_SPIM_FREQ_2M);
}

void dw3000_spi_speed_fast(void)
{
	if (dw_hw_cfg->spi_max_mhz == 8) {
		nrf_spim_frequency_set(NRF_SPIM0, NRF_SPIM_FREQ_8M);
#if defined(SPIM_FREQUENCY_FREQUENCY_M16)
	} else if (dw_hw_cfg->spi_max_mhz == 16) {
		nrf_spim_frequency_set(NRF_SPIM0, NRF_SPIM_FREQ_16M);
#endif
#if defined(SPIM_FREQUENCY_FREQUENCY_M16)
	} else if (dw_hw_cfg->spi_max_mhz == 32) {
		nrf_spim_frequency_set(NRF_SPIM0, NRF_SPIM_FREQ_32M);
#endif
	} else {
		LOG_ERR("Unknown SPI speed %ld MHz", dw_hw_cfg->spi_max_mhz);
		return;
	}
}

void dw3000_spi_fini(void)
{
	nrfx_spim_uninit(&dw_spi);

	// TODO?
	// nrf_gpio_cfg_default(dw_hw_cfg->spi_clk_pin);
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
	decaIrqStatus_t stat = decamutexon();

#if DW3000_SPI_TRACE
	dw3000_spi_trace_in(false, headerBuffer, headerLength, bodyBuffer,
						bodyLength);
#endif

	nrf_gpio_pin_clear(dw_hw_cfg->spi_cs_pin);

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
	nrf_gpio_pin_set(dw_hw_cfg->spi_cs_pin);
	decamutexoff(stat);
	return ret == NRFX_SUCCESS ? DWT_SUCCESS : DWT_ERROR;
}

int dw3000_spi_read(uint16_t headerLength, uint8_t* headerBuffer,
					uint16_t readLength, uint8_t* readBuffer)
{
	decaIrqStatus_t stat = decamutexon();
	nrf_gpio_pin_clear(dw_hw_cfg->spi_cs_pin);

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

#if DW3000_SPI_TRACE
	dw3000_spi_trace_in(true, headerBuffer, headerLength, readBuffer,
						readLength);
#endif

exit:
	nrf_gpio_pin_set(dw_hw_cfg->spi_cs_pin);
	decamutexoff(stat);
	return ret == NRFX_SUCCESS ? DWT_SUCCESS : DWT_ERROR;
}
