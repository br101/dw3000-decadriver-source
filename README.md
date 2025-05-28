# Multiplatform driver for Qorvo/Decawave DW3000

This is a module with a driver for Qorvo/Decawave DW3000 for different platforms. It contains the source release of the "dwt_uwb_driver" from Qorvo (version 08.02.02 from DW3_QM33_SDK_1.0.2.zip), and the minimal platform code to make it work. The driver files released from Qorvo have been minimally modified to support only one DW3000 chip per board and to remove the big IOCTL function which is a huge waste of space on embedded platforms.

At the moment the following platforms are supported:
 * Zephyr (version 3.6, NRF Connect SDK v2.7.0)
 * ESP-IDF (version 5.1.4)
 * NRF SDK (version 17.1.0)

It has been tested with the following Qorvo chips:
 * DW3110 (DEVID: 0xDECA0302) on DWM3000
 * DW3120 (DEVID: 0xDECA0312)

There are also branches for the older source release and the binary-only library releases from Qorvo for reference.


## Zephyr

You can add this as a module to Zephyr using west or by adding the following to the main CMakeLists.txt:
```
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/lib/decadriver/platform)
```

Then configure it in prj.conf:
```
CONFIG_DW3000=y
CONFIG_DW3000_CHIP_DW3000=y
CONFIG_SPI=y
CONFIG_GPIO=y
```


## ESP-IDF

For ESP-IDF it can be used by adding it as a component by setting:
```
set(EXTRA_COMPONENT_DIRS components/dw3000-decadriver-source/platform/esp-idf)
```
and by defining the necessary CONFIG_DW3000 for setting the SPI and GPIO pins for your board (see `platform/esp-idf/decadriver/Kconfig`)


## NRF SDK

For NRF SDK you just add the necessary files to the build system.

For example in a Makefile:
```
INCLUDES	+= -Ilib/decadriver/decadriver
INCLUDES	+= -Ilib/decadriver/platform/nrf-sdk

SRC		+= lib/decadriver/decadriver/deca_device.c
SRC		+= lib/decadriver/platform/nrf-sdk/deca_port.c
SRC		+= lib/decadriver/platform/nrf-sdk/dw3000_hw.c
SRC		+= lib/decadriver/platform/nrf-sdk/dw3000_spi.c
SRC		+= lib/decadriver/platform/nrf-sdk/dw3000_spi_trace.c
```

And you also need to make sure the LOG_ macros are defined in a file called `log.h`


## Usage and first steps

This is a minimal code fragment to check the basic functionality (reading the device ID).

```
dw3000_hw_init();
dw3000_hw_reset();
uint32_t dev_id = dwt_readdevid();
LOG_INF("DEVID %x", devid);
```

## Next steps

You can use this library to write your own code directly using the API provided by `decadriver` or you also include my higher level library [libdeca](https://github.com/br101/libdeca) which adds some convenient functions, proper IRQ handling, and a simple implementation of two way ranging (TWR).

## Licenses

The driver code from Qorvo is licensed according to their own license: https://github.com/br101/dw3000-decadriver-source/blob/master/dwt_uwb_driver/LICENSES/LicenseRef-QORVO-2.txt.

My own code is under the ISC license: https://github.com/br101/dw3000-decadriver-source/blob/master/LICENSE.txt

