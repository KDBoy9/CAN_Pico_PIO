cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(can_tx_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init() 

add_executable(can_tx_pico)

pico_generate_pio_header(can_tx_pico /home/kedar/Programs/pico/CAN_TX/can_tx.pio)

target_sources(can_tx_pico PRIVATE can_tx.c)

target_link_libraries(can_tx_pico PRIVATE
	pico_stdlib
	hardware_pio
	hardware_clocks
	hardware_dma
)

pico_add_extra_outputs(can_tx_pico)
