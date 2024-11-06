#include <nrf_delay.h>
#include <nrf_error.h>
#include <nrf_gpio.h>
#include <nrf_gpiote.h>
#include <nrfx_gpiote.h>

#include "config.h"
#include "deca_device_api.h"
#include "dw3000_hw.h"
#include "dw3000_spi.h"
#include "log.h"

static const char* LOG_TAG = "DW3000";

int dw3000_hw_init(void)
{
	LOG_INF("HW Init (RESET:%d WAKEUP:%d IRQ:%d)", CONFIG_DW3000_GPIO_RESET,
			CONFIG_DW3000_GPIO_WAKEUP, CONFIG_DW3000_GPIO_IRQ);

	/*
	 * RESET: output low, open drain, no pull-up
	 * normally used as input to see when DW3000 is ready
	 */
#if CONFIG_DW3000_GPIO_RESET != -1
	nrf_gpio_cfg_input(CONFIG_DW3000_GPIO_RESET, NRF_GPIO_PIN_NOPULL);

	/* check reset state */
	int timeout = 1000;
	while (!nrf_gpio_pin_read(CONFIG_DW3000_GPIO_RESET) && --timeout > 0) {
		nrf_delay_ms(1);
	}
	if (timeout <= 0) {
		LOG_ERR("did not come out of reset");
		return NRF_ERROR_TIMEOUT;
	}
#endif

	/*
	 * WAKEUP: output high
	 */
#if CONFIG_DW3000_GPIO_WAKEUP != -1
	nrf_gpio_cfg_output(CONFIG_DW3000_GPIO_WAKEUP);
#endif

	return dw3000_spi_init();
}

static void dw3000_isr(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	while (nrf_gpio_pin_read(CONFIG_DW3000_GPIO_IRQ)) {
		dwt_isr();
	}
}

int dw3000_hw_init_interrupt()
{
#if CONFIG_DW3000_GPIO_IRQ == -1
	LOG_ERR("IRQ pin is not defined");
	return NRFX_ERROR_INVALID_PARAM;
#else
	/*
	 * IRQ: active high, internal pull down
	 */
	nrfx_err_t ret = nrfx_gpiote_init();
	if (ret != NRFX_SUCCESS && ret != NRFX_ERROR_INVALID_STATE) {
		LOG_ERR("ERR: GPIOTE init failed");
		return ret;
	}

	nrfx_gpiote_in_config_t config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	nrfx_gpiote_in_init(CONFIG_DW3000_GPIO_IRQ, &config, dw3000_isr);
	if (ret != NRFX_SUCCESS && ret != NRFX_ERROR_INVALID_STATE) {
		LOG_ERR("ERR: IRQ init failed");
		return ret;
	}
	nrfx_gpiote_in_event_enable(CONFIG_DW3000_GPIO_IRQ, true);
	return NRF_SUCCESS;
#endif
}

void dw3000_hw_interrupt_enable(void)
{
#if CONFIG_DW3000_GPIO_IRQ != -1
	nrf_gpiote_int_enable(CONFIG_DW3000_GPIO_IRQ);
#endif
}

void dw3000_hw_interrupt_disable(void)
{
#if CONFIG_DW3000_GPIO_IRQ != -1
	nrf_gpiote_int_disable(CONFIG_DW3000_GPIO_IRQ);
#endif
}

bool dw3000_hw_interrupt_is_enabled(void)
{
#if CONFIG_DW3000_GPIO_IRQ != -1
	return nrf_gpiote_int_is_enabled(CONFIG_DW3000_GPIO_IRQ);
#else
	return false;
#endif
}

void dw3000_hw_fini(void)
{
	LOG_INF("HW fini");

#if CONFIG_DW3000_GPIO_IRQ != -1
	nrfx_gpiote_in_event_disable(CONFIG_DW3000_GPIO_IRQ);
	nrfx_gpiote_in_uninit(CONFIG_DW3000_GPIO_IRQ);
	nrfx_gpiote_uninit();
#endif

#if CONFIG_DW3000_GPIO_RESET != -1
	nrf_gpio_cfg_default(CONFIG_DW3000_GPIO_RESET);
#endif

#if CONFIG_DW3000_GPIO_WAKEUP != -1
	nrf_gpio_cfg_default(CONFIG_DW3000_GPIO_WAKEUP);
#endif

	dw3000_spi_fini();
}

void dw3000_hw_reset(void)
{
#if CONFIG_DW3000_GPIO_RESET == -1
	LOG_ERR("Reset pin is not defined");
#else
	LOG_INF("HW reset");
	nrf_gpio_cfg_output(CONFIG_DW3000_GPIO_RESET);
	nrf_gpio_pin_clear(CONFIG_DW3000_GPIO_RESET);
	nrf_delay_ms(1); // API guide says 10ns
	nrf_gpio_pin_set(CONFIG_DW3000_GPIO_RESET);
	nrf_delay_ms(2); // From sample platform code
	nrf_gpio_cfg_input(CONFIG_DW3000_GPIO_RESET, NRF_GPIO_PIN_NOPULL);
#endif
}

/** wakeup either using the WAKEUP pin or SPI CS */
void dw3000_hw_wakeup(void)
{
#if CONFIG_DW3000_GPIO_WAKEUP != -1
	/* Use WAKEUP pin if available */
	LOG_INF("WAKEUP PIN");
	nrf_gpio_pin_set(CONFIG_DW3000_GPIO_WAKEUP);
	nrf_delay_us(500);
	nrf_gpio_pin_clear(CONFIG_DW3000_GPIO_WAKEUP);
#else
	/* Use SPI CS pin */
	LOG_INF("WAKEUP CS");
	nrf_gpio_pin_clear(CONFIG_DW3000_SPI_CS);
	nrf_delay_us(500);
	nrf_gpio_pin_set(CONFIG_DW3000_SPI_CS);
#endif
	nrf_delay_ms(1);
}

/** set WAKEUP pin low if available */
void dw3000_hw_wakeup_pin_low(void)
{
#if CONFIG_DW3000_GPIO_WAKEUP != -1
	nrf_gpio_pin_clear(CONFIG_DW3000_GPIO_WAKEUP);
#endif
}
