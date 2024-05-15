idf_component_register(SRCS ../../../decadriver/deca_device.c deca_port.c
                            dw3000_hw.c dw3000_spi.c ../../dw3000_spi_trace.c
                       INCLUDE_DIRS ../../../decadriver .
                       PRIV_INCLUDE_DIRS priv/
                       REQUIRES driver)
