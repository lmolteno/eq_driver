cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(eq3_rp2040)

pico_sdk_init()

add_executable(eq3_rp2040
    src/main.c
    src/tmc2209.c
	src/motor_control.c
	src/motor_control.h
		src/telescope_protocol.c
		src/telescope_protocol.h
)

pico_generate_pio_header(eq3_rp2040 ${CMAKE_CURRENT_LIST_DIR}/src/step_generator.pio)

target_link_libraries(eq3_rp2040
    pico_stdlib
    hardware_uart
    hardware_gpio
    hardware_clocks
		hardware_pio
)

target_compile_definitions(eq3_rp2040 PRIVATE
	XOSC_MHZ=10
        # System PLL - 133MHz 
        PLL_SYS_REFDIV=1
        PLL_SYS_VCO_FREQ_HZ=1330000000
        PLL_SYS_POSTDIV1=5
        PLL_SYS_POSTDIV2=2
        SYS_CLK_HZ=133000000
        # USB PLL - 48MHz (required for USB)
        PLL_USB_REFDIV=1
        PLL_USB_VCO_FREQ_HZ=1200000000
        PLL_USB_POSTDIV1=5
        PLL_USB_POSTDIV2=5
)

pico_enable_stdio_usb(eq3_rp2040 1)
pico_enable_stdio_uart(eq3_rp2040 0)

pico_add_extra_outputs(eq3_rp2040)

