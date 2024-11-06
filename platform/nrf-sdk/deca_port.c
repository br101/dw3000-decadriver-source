#include "nrf_delay.h"

#include "deca_version.h"
#ifdef DW3000_DRIVER_VERSION // == 0x040000
#include "deca_device_api.h"
#else
#include "deca_interface.h"
#endif

#include "dw3000_hw.h"
#include "dw3000_spi.h"

/* This file implements the functions required by decadriver */

void wakeup_device_with_io(void)
{
	/* this would only be used if you use the dwt_wakeup_ic() API, but it's
	 * easier to directly call our own dw3000_hw_wakeup() function */
	dw3000_hw_wakeup();
}

decaIrqStatus_t decamutexon(void)
{
	bool s = dw3000_hw_interrupt_is_enabled();
	if (s) {
		dw3000_hw_interrupt_disable();
	}
	return s;
}

void decamutexoff(decaIrqStatus_t s)
{
	if (s) {
		dw3000_hw_interrupt_enable();
	}
}

void deca_sleep(unsigned int time_ms)
{
	nrf_delay_ms(time_ms);
}

void deca_usleep(unsigned long time_us)
{
	nrf_delay_us(time_us);
}

#ifdef DW3000_DRIVER_VERSION // == 0x040000

int readfromspi(uint16_t headerLength, uint8_t* headerBuffer,
				uint16_t readLength, uint8_t* readBuffer)
{
	return dw3000_spi_read(headerLength, headerBuffer, readLength, readBuffer);
}

int writetospi(uint16_t headerLength, const uint8_t* headerBuffer,
			   uint16_t bodyLength, const uint8_t* bodyBuffer)
{
	return dw3000_spi_write(headerLength, headerBuffer, bodyLength, bodyBuffer);
}

int writetospiwithcrc(uint16_t headerLength, const uint8_t* headerBuffer,
					  uint16_t bodyLength, const uint8_t* bodyBuffer,
					  uint8_t crc8)
{
	return dw3000_spi_write_crc(headerLength, headerBuffer, bodyLength,
								bodyBuffer, crc8);
}

#elif DRIVER_VERSION_HEX >= 0x080202

static const struct dwt_spi_s dw3000_spi_fct = {
	.readfromspi = dw3000_spi_read,
	.writetospi = dw3000_spi_write,
	.writetospiwithcrc = dw3000_spi_write_crc,
	.setslowrate = dw3000_spi_speed_slow,
	.setfastrate = dw3000_spi_speed_fast,
};

#if CONFIG_DW3000_CHIP_DW3000
extern const struct dwt_driver_s dw3000_driver;
#elif CONFIG_DW3000_CHIP_DW3720
extern const struct dwt_driver_s dw3720_driver;
#endif

const struct dwt_driver_s* tmp_ptr[] = {
#if CONFIG_DW3000_CHIP_DW3000
	&dw3000_driver,
#elif CONFIG_DW3000_CHIP_DW3720
	&dw3720_driver
#endif
};

const struct dwt_probe_s dw3000_probe_interf = {
	.dw = NULL,
	.spi = (void*)&dw3000_spi_fct,
	.wakeup_device_with_io = dw3000_hw_wakeup,
	.driver_list = (struct dwt_driver_s**)tmp_ptr,
	.dw_driver_num = 1,
};

#elif DRIVER_VERSION_HEX >= 0x060007

static const struct dwt_spi_s dw3000_spi_fct = {
	.readfromspi = dw3000_spi_read,
	.writetospi = dw3000_spi_write,
	.writetospiwithcrc = dw3000_spi_write_crc,
	.setslowrate = dw3000_spi_speed_slow,
	.setfastrate = dw3000_spi_speed_fast,
};

const struct dwt_probe_s dw3000_probe_interf = {
	.dw = NULL,
	.spi = (void*)&dw3000_spi_fct,
	.wakeup_device_with_io = dw3000_hw_wakeup,
};

#endif
