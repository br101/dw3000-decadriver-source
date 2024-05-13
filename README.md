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
and by defining the necessary CONFIG for setting the SPI and GPIO pins for your board.

## NRF SDK v 17

For NRF SDK you just add the necessary files to the build system.


## Usage and first steps

This is a minimal code fragment to check the basic functionality (reading the device ID).

```
dw3000_hw_init();
dw3000_hw_reset();
dw3000_hw_init_interrupt();
uint32_t dev_id = dwt_readdevid();
LOG_INF("DEVID %x", devid);
```
