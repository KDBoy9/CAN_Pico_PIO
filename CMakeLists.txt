cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(can_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init() 

add_executable(can_pico)

pico_generate_pio_header(can_pico /home/kedar/Programs/pico/mypico/can.pio)

target_sources(can_pico PRIVATE can.c)

target_link_libraries(can_pico PRIVATE
	pico_stdlib
	hardware_pio
	hardware_clocks
	hardware_dma
)

pico_add_extra_outputs(can_pico)
