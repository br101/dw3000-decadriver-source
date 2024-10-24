
set(srcs
     ../../../dwt_uwb_driver/deca_interface.c
     ../../../dwt_uwb_driver/deca_rsl.c
     ../../../dwt_uwb_driver/lib/qmath/src/qmath.c
     ../../deca_compat.c
     deca_port.c dw3000_hw.c dw3000_spi.c ../../dw3000_spi_trace.c)

set(incl
     . ../.. ../../../dwt_uwb_driver
     ../../../dwt_uwb_driver/lib/qmath/include)

if (CONFIG_DW3000_CHIP_DW3000)
    list(APPEND srcs ../../../dwt_uwb_driver/dw3000/dw3000_device.c)
    list(APPEND incl ../../../dwt_uwb_driver/dw3000)
elseif(CONFIG_DW3000_CHIP_DW3720)
    list(APPEND srcs ../../../dwt_uwb_driver/dw3720/dw3720_device.c)
    list(APPEND incl ../../../dwt_uwb_driver/dw3720)
endif()

idf_component_register(SRCS ${srcs}
                       PRIV_INCLUDE_DIRS priv
                       INCLUDE_DIRS ${incl}
                       REQUIRES driver)
