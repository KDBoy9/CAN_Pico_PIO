# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/kedar/Programs/pico/pico-sdk/tools/pioasm"
  "/home/kedar/Programs/pico/CAN_RX/build/pioasm"
  "/home/kedar/Programs/pico/CAN_RX/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm"
  "/home/kedar/Programs/pico/CAN_RX/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm/tmp"
  "/home/kedar/Programs/pico/CAN_RX/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm/src/PioasmBuild-stamp"
  "/home/kedar/Programs/pico/CAN_RX/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm/src"
  "/home/kedar/Programs/pico/CAN_RX/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm/src/PioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/kedar/Programs/pico/CAN_RX/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm/src/PioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/kedar/Programs/pico/CAN_RX/build/pico-sdk/src/rp2_common/cyw43_driver/pioasm/src/PioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
