idf_component_register(SRCS ../../../decadriver/deca_device.c deca_port.c
                            dw3000_hw.c dw3000_spi.c ../../dw3000_spi_trace.c
                       PRIV_INCLUDE_DIRS priv
                       INCLUDE_DIRS ../../../decadriver .
                       REQUIRES driver)
