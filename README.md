# Multiplatform driver for Qorvo/Decawave DW3000

This is a module with a driver for Qorvo/Decawave DW3000 for different platforms. It contains the last available source release of "decadriver" from Qorvo (from DWS3000_Release_v1.1 / DW3000_API_C0_rev4p0), and the minimal platform code to make it work.

At the moment the following platforms are supported:
 * ESP-IDF (version 5.1.1)
 * NRF SDK (version 17.1.0)

## ESP-IDF

For ESP-IDF it can be used by adding it as a component by setting:
```
set(EXTRA_COMPONENT_DIRS components/dw3000-decadriver-source/platform/esp-idf)
```
and by defining the necessary CONFIG_DW3000 for setting the SPI and GPIO pins for your board (see `platform/esp-idf/decadriver/Kconfig`)

## NRF SDK v 17

For NRF SDK you just add the necessary files to the build system.

For example in a Makefile:
```
INCLUDES	+= -Ilib/decadriver/decadriver
INCLUDES	+= -Ilib/decadriver/platform/nrf-sdk

SRC			+= lib/decadriver/decadriver/deca_device.c
SRC			+= lib/decadriver/platform/nrf-sdk/deca_port.c
SRC			+= lib/decadriver/platform/nrf-sdk/dw3000_hw.c
SRC			+= lib/decadriver/platform/nrf-sdk/dw3000_spi.c
SRC			+= lib/decadriver/platform/nrf-sdk/dw3000_spi_trace.c
```

## Usage and first steps

This is a minimal code fragment to check the basic functionality (reading the device ID).

```
dw3000_hw_init();
dw3000_hw_reset();
uint32_t dev_id = dwt_readdevid();
LOG_INF("DEVID %x", devid);
```
