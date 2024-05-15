#include <nrf_delay.h>
#include <nrf_gpio.h>
#include <nrf_gpiote.h>
#include <nrfx_gpiote.h>

#include "deca_device_api.h"
#include "dw3000_hw.h"
#include "dw3000_spi.h"
#include "log.h"

static const char* LOG_TAG = "DW3000";
static struct dw3000_hw_cfg* dw_hw_cfg;

int dw3000_hw_init(struct dw3000_hw_cfg* cfg)
{
	LOG_INF("RESET:%d WAKEUP:%d IRQ:%d", cfg->reset_pin, cfg->wakeup_pin,
			cfg->irq_pin);

	dw_hw_cfg = cfg;

	/*
	 * RESET: output low, open drain, no pull-up
	 * normally used as input to see when DW1000 is ready
	 * Ref:
	 * https://devzone.nordicsemi.com/question/22112/nrf51822-gpio-as-open-collector-driver/
	 */
	if (cfg->reset_pin != -1) {
		nrf_gpio_cfg_input(cfg->reset_pin, NRF_GPIO_PIN_NOPULL);

		/* check reset state */
		int timeout = 1000;
		while (!nrf_gpio_pin_read(cfg->reset_pin) && --timeout > 0) {
			nrf_delay_ms(1);
		}
		if (timeout <= 0) {
			LOG_ERR("did not come out of reset");
			return false;
		}
	}

	/*
	 * WAKEUP: output high
	 */
	if (cfg->wakeup_pin != -1) {
		nrf_gpio_cfg_output(cfg->wakeup_pin);
	}

	return dw3000_spi_init(cfg);
}

static void dw3000_isr(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	while (nrf_gpio_pin_read(dw_hw_cfg->irq_pin)) {
		dwt_isr();
	}
}

int dw3000_hw_init_interrupt()
{
	if (dw_hw_cfg->irq_pin == -1) {
		LOG_ERR("IRQ pin is not defined");
		return NRFX_ERROR_INVALID_PARAM;
	}

	/*
	 * IRQ: active high, internal pull down
	 */
	nrfx_err_t ret = nrfx_gpiote_init();
	if (ret != NRFX_SUCCESS && ret != NRFX_ERROR_INVALID_STATE) {
		LOG_ERR("ERR: GPIOTE init failed");
		return false;
	}

	nrfx_gpiote_in_config_t config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	nrfx_gpiote_in_init(dw_hw_cfg->irq_pin, &config, dw3000_isr);
	if (ret != NRFX_SUCCESS && ret != NRFX_ERROR_INVALID_STATE) {
		LOG_ERR("ERR: IRQ init failed");
		return false;
	}
	nrfx_gpiote_in_event_enable(dw_hw_cfg->irq_pin, true);
	return true;
}

void dw3000_hw_interrupt_enable(void)
{
	if (dw_hw_cfg->irq_pin != -1) {
		nrf_gpiote_int_enable(dw_hw_cfg->irq_pin);
	}
}

void dw3000_hw_interrupt_disable(void)
{
	if (dw_hw_cfg->irq_pin != -1) {
		nrf_gpiote_int_disable(dw_hw_cfg->irq_pin);
	}
}

bool dw3000_hw_interrupt_is_enabled(void)
{
	if (dw_hw_cfg->irq_pin != -1) {
		return nrf_gpiote_int_is_enabled(dw_hw_cfg->irq_pin);
	} else {
		return false;
	}
}

void dw3000_hw_fini(void)
{
	LOG_INF("HW fini");

	if (dw_hw_cfg->irq_pin != -1) {
		nrfx_gpiote_in_event_disable(dw_hw_cfg->irq_pin);
		nrfx_gpiote_in_uninit(dw_hw_cfg->irq_pin);
		nrfx_gpiote_uninit();
	}

	if (dw_hw_cfg->reset_pin != -1) {
		nrf_gpio_cfg_default(dw_hw_cfg->reset_pin);
	}

	if (dw_hw_cfg->wakeup_pin != -1) {
		nrf_gpio_cfg_default(dw_hw_cfg->wakeup_pin);
	}

	dw3000_spi_fini();
}

void dw3000_hw_reset(void)
{
	if (dw_hw_cfg->reset_pin == -1) {
		LOG_ERR("Reset pin is not defined");
		return;
	}

	LOG_INF("HW reset");
	nrf_gpio_cfg_output(dw_hw_cfg->reset_pin);
	nrf_gpio_pin_clear(dw_hw_cfg->reset_pin);
	nrf_delay_ms(1); // API guide says 10ns
	nrf_gpio_pin_set(dw_hw_cfg->reset_pin);
	nrf_delay_ms(2); // From sample platform code
	nrf_gpio_cfg_input(dw_hw_cfg->reset_pin, NRF_GPIO_PIN_NOPULL);
}

/** wakeup either using the WAKEUP pin or SPI CS */
void dw3000_hw_wakeup(void)
{
	if (dw_hw_cfg->wakeup_pin != -1) {
		/* Use WAKEUP pin if available */
		LOG_INF("WAKEUP PIN");
		nrf_gpio_pin_set(dw_hw_cfg->wakeup_pin);
		nrf_delay_ms(1);
		nrf_gpio_pin_clear(dw_hw_cfg->wakeup_pin);
	} else {
		/* Use SPI CS pin */
		LOG_INF("WAKEUP CS");
		nrf_gpio_pin_clear(dw_hw_cfg->spi_cs_pin);
		nrf_delay_us(500);
		nrf_gpio_pin_set(dw_hw_cfg->spi_cs_pin);
		nrf_delay_ms(1);
	}
}

/** set WAKEUP pin low if available */
void dw3000_hw_wakeup_pin_low(void)
{
	if (dw_hw_cfg->wakeup_pin != -1) {
		nrf_gpio_pin_clear(dw_hw_cfg->wakeup_pin);
	}
}
