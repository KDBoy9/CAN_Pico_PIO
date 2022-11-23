cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(can_rx_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init() 

add_executable(can_rx_pico)

pico_generate_pio_header(can_rx_pico ${CMAKE_CURRENT_LIST_DIR}/can_rx.pio)

target_sources(can_rx_pico PRIVATE can_rx.c)

target_link_libraries(can_rx_pico PRIVATE
	pico_stdlib
	hardware_pio
	hardware_clocks
	hardware_dma
)

pico_enable_stdio_usb(can_rx_pico 1)
pico_enable_stdio_uart(can_rx_pico 0)

pico_add_extra_outputs(can_rx_pico)
