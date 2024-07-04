#ifndef DW3000_HW_H
#define DW3000_HW_H

#include <stdbool.h>
#include <stdint.h>

struct dw3000_hw_cfg {
	int reset_pin;
	int wakeup_pin;
	int irq_pin;
	int spi_cs_pin;
	int spi_clk_pin;
	int spi_miso_pin;
	int spi_mosi_pin;
	uint32_t spi_max_mhz;
};

int dw3000_hw_init(const struct dw3000_hw_cfg* cfg);
int dw3000_hw_init_interrupt(void);
void dw3000_hw_fini(void);
void dw3000_hw_reset(void);
void dw3000_hw_wakeup(void);
void dw3000_hw_wakeup_pin_low(void);
void dw3000_hw_interrupt_enable(void);
void dw3000_hw_interrupt_disable(void);
bool dw3000_hw_interrupt_is_enabled(void);

#endif
