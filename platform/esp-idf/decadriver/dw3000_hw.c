
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "deca_device_api.h"
#include "dw3000_hw.h"
#include "dw3000_spi.h"
#include "log.h"

static const char* LOG_TAG = "DW3000";
static const struct dw3000_hw_cfg* dw_hw_cfg;
static bool dw3000_interrupt_enabled;

int dw3000_hw_init(const struct dw3000_hw_cfg* cfg)
{
	LOG_INF("RESET:%d WAKEUP:%d IRQ:%d", cfg->reset_pin, cfg->wakeup_pin,
			cfg->irq_pin);

	dw_hw_cfg = cfg;

	/*
	 * RESET: output low, open drain, no pull-up
	 * normally used as input to see when DW3000 is ready
	 */
	if (cfg->reset_pin != -1) {
		gpio_config_t io_conf_reset = {
			.mode = GPIO_MODE_INPUT,
			.pin_bit_mask = (uint64_t)1 << cfg->reset_pin,
		};
		gpio_config(&io_conf_reset);

		/* check reset state */
		int timeout = 1000;
		while (!gpio_get_level(cfg->reset_pin) && --timeout > 0) {
			vTaskDelay(1);
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
		gpio_config_t io_conf_wakeup = {
			.mode = GPIO_MODE_OUTPUT,
			.pin_bit_mask = (uint64_t)1 << cfg->wakeup_pin,
		};
		gpio_config(&io_conf_wakeup);
		gpio_set_level(dw_hw_cfg->wakeup_pin, 0);
	}

	return dw3000_spi_init(cfg);
}

static void dw3000_isr(void* args)
{
	while (gpio_get_level(dw_hw_cfg->irq_pin)) {
		dwt_isr();
	}
}

int dw3000_hw_init_interrupt(void)
{
	if (dw_hw_cfg->irq_pin == -1) {
		LOG_ERR("IRQ pin is not defined");
		return ESP_ERR_NOT_SUPPORTED;
	}

	gpio_config_t io_conf = {
		.intr_type = GPIO_INTR_POSEDGE,
		.pin_bit_mask = (uint64_t)1 << dw_hw_cfg->irq_pin,
		.mode = GPIO_MODE_INPUT,
	};
	gpio_config(&io_conf);

	gpio_install_isr_service(0);
	return gpio_isr_handler_add(dw_hw_cfg->irq_pin, dw3000_isr, NULL);
}

void dw3000_hw_interrupt_enable(void)
{
	if (dw_hw_cfg->irq_pin != -1) {
		gpio_intr_enable(dw_hw_cfg->irq_pin);
		dw3000_interrupt_enabled = true;
	}
}

void dw3000_hw_interrupt_disable(void)
{
	if (dw_hw_cfg->irq_pin != -1) {
		gpio_intr_disable(dw_hw_cfg->irq_pin);
		dw3000_interrupt_enabled = false;
	}
}

bool dw3000_hw_interrupt_is_enabled(void)
{
	return dw3000_interrupt_enabled;
}

void dw3000_hw_fini(void)
{
	LOG_INF("HW fini");

	if (dw_hw_cfg->irq_pin != -1) {
		if (dw3000_interrupt_enabled) {
			gpio_intr_disable(dw_hw_cfg->irq_pin);
	}
		gpio_isr_handler_remove(dw_hw_cfg->irq_pin);
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
	gpio_set_direction(dw_hw_cfg->reset_pin, GPIO_MODE_OUTPUT);
	gpio_set_level(dw_hw_cfg->reset_pin, 0);
	vTaskDelay(1); // 10 us?
	gpio_set_level(dw_hw_cfg->reset_pin, 1);
	vTaskDelay(2); // From sample platform code
	gpio_set_direction(dw_hw_cfg->reset_pin, GPIO_MODE_INPUT);
}

/** wakeup either using the WAKEUP pin or SPI CS */
void dw3000_hw_wakeup(void)
{
	if (dw_hw_cfg->wakeup_pin != -1) {
		/* Use WAKEUP pin if available */
		LOG_INF("WAKEUP PIN");
		gpio_set_level(dw_hw_cfg->wakeup_pin, 1);
		vTaskDelay(1); // 500 usec
		gpio_set_level(dw_hw_cfg->wakeup_pin, 0);
	} else {
		/* Use SPI CS pin */
		LOG_INF("WAKEUP CS");
		// TODO: set from SPI to GPIO output
		gpio_set_level(dw_hw_cfg->spi_cs_pin, 0);
		vTaskDelay(1); // 500 usec
		gpio_set_level(dw_hw_cfg->spi_cs_pin, 1);
	}
	vTaskDelay(1);
}

/** set WAKEUP pin low if available */
void dw3000_hw_wakeup_pin_low(void)
{
	if (dw_hw_cfg->wakeup_pin != -1) {
		gpio_set_level(dw_hw_cfg->wakeup_pin, 0);
	}
}
