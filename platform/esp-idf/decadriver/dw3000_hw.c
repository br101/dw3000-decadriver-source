
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "deca_device_api.h"
#include "dw3000_hw.h"
#include "dw3000_spi.h"
#include "log.h"

static const char* LOG_TAG = "DW3000";
static bool dw3000_interrupt_enabled;

int dw3000_hw_init(void)
{
	LOG_INF("RESET:%d WAKEUP:%d IRQ:%d", CONFIG_DW3000_GPIO_RESET,
			CONFIG_DW3000_GPIO_WAKEUP, CONFIG_DW3000_GPIO_IRQ);

	/* Reset (normally used as input to see when DW3000 is ready) */
#if CONFIG_DW3000_GPIO_RESET != -1
	gpio_config_t io_conf_reset = {
		.mode = GPIO_MODE_INPUT,
		.pin_bit_mask = (uint64_t)1 << CONFIG_DW3000_GPIO_RESET,
	};
	gpio_config(&io_conf_reset);

	/* check reset state */
	int timeout = 1000;
	while (!gpio_get_level(CONFIG_DW3000_GPIO_RESET) && --timeout > 0) {
		vTaskDelay(1);
	}
	if (timeout <= 0) {
		LOG_ERR("did not come out of reset");
		return false;
	}
#endif

#if CONFIG_DW3000_GPIO_WAKEUP != -1
	gpio_config_t io_conf_wakeup = {
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = (uint64_t)1 << CONFIG_DW3000_GPIO_WAKEUP,
	};
	gpio_config(&io_conf_wakeup);
#endif

	return dw3000_spi_init();
}

#if CONFIG_DW3000_GPIO_IRQ != -1
static void dw3000_isr(void* args)
{
	while (gpio_get_level(CONFIG_DW3000_GPIO_IRQ)) {
		dwt_isr();
	}
}
#endif

int dw3000_hw_init_interrupt(void)
{
#if CONFIG_DW3000_GPIO_IRQ != -1
	gpio_config_t io_conf = {
		.intr_type = GPIO_INTR_POSEDGE,
		.pin_bit_mask = (uint64_t)1 << CONFIG_DW3000_GPIO_IRQ,
		.mode = GPIO_MODE_INPUT,
	};
	gpio_config(&io_conf);

	gpio_install_isr_service(0);
	return gpio_isr_handler_add(CONFIG_DW3000_GPIO_IRQ, dw3000_isr, NULL);
#else
	return ESP_ERR_NOT_SUPPORTED;
#endif
}

void dw3000_hw_interrupt_enable(void)
{
#if CONFIG_DW3000_GPIO_IRQ != -1
	gpio_intr_enable(CONFIG_DW3000_GPIO_IRQ);
	dw3000_interrupt_enabled = true;
#endif
}

void dw3000_hw_interrupt_disable(void)
{
#if CONFIG_DW3000_GPIO_IRQ != -1
	gpio_intr_disable(CONFIG_DW3000_GPIO_IRQ);
	dw3000_interrupt_enabled = false;
#endif
}

bool dw3000_hw_interrupt_is_enabled(void)
{
	return dw3000_interrupt_enabled;
}

void dw3000_hw_fini(void)
{
#if CONFIG_DW3000_GPIO_IRQ != -1
	if (dw3000_interrupt_enabled) {
		gpio_intr_disable(CONFIG_DW3000_GPIO_IRQ);
	}
	gpio_isr_handler_remove(CONFIG_DW3000_GPIO_IRQ);
#endif

	dw3000_spi_fini();
}

void dw3000_hw_reset()
{
#if CONFIG_DW3000_GPIO_RESET != -1
	gpio_set_direction(CONFIG_DW3000_GPIO_RESET, GPIO_MODE_OUTPUT);
	gpio_set_level(CONFIG_DW3000_GPIO_RESET, 0);
	vTaskDelay(1); // 10 us?
	gpio_set_direction(CONFIG_DW3000_GPIO_RESET, GPIO_MODE_INPUT);
	vTaskDelay(2);
#endif
}

/** wakeup either using the WAKEUP pin or SPI CS */
void dw3000_hw_wakeup(void)
{
#if CONFIG_DW3000_GPIO_WAKEUP != -1
	/* Use WAKEUP pin if available */
	LOG_INF("WAKEUP PIN");
	gpio_set_level(CONFIG_DW3000_GPIO_WAKEUP, 1);
	vTaskDelay(1);
	gpio_set_level(CONFIG_DW3000_GPIO_WAKEUP, 0);
#else
	/* Use SPI CS pin */
	LOG_INF("WAKEUP CS");
	// TODO: set from SPI to GPIO output
	gpio_set_level(CONFIG_DW3000_SPI_CS, 1);
	vTaskDelay(1); // 500 usec
	gpio_set_level(CONFIG_DW3000_SPI_CS, 0);
#endif
}

/** set WAKEUP pin low if available */
void dw3000_hw_wakeup_pin_low(void)
{
#if CONFIG_DW3000_GPIO_WAKEUP != -1
	gpio_set_level(CONFIG_DW3000_GPIO_WAKEUP, 0);
#endif
}
