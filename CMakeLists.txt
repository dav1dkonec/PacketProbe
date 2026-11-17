cmake_minimum_required(VERSION 3.13...3.31)

# must come before project(): sets up the cross toolchain (PICO_SDK_PATH)
set(PICO_BOARD pico CACHE STRING "Board type")  # Pico H: RP2040, no wireless
include(pico_sdk_import.cmake)

project(probe C CXX ASM)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

add_compile_options(-Wall -Wextra)

# probe_core: timers and matching, everything except main()
add_library(probe_core STATIC src/edge_timer.cpp)
target_include_directories(probe_core PUBLIC src)
pico_generate_pio_header(probe_core ${CMAKE_CURRENT_LIST_DIR}/src/edge_timer.pio)
pico_generate_pio_header(probe_core ${CMAKE_CURRENT_LIST_DIR}/src/sim.pio)
target_link_libraries(probe_core PUBLIC pico_stdlib hardware_pio hardware_dma)

add_executable(probe src/main.cpp)
target_link_libraries(probe PRIVATE probe_core pico_multicore)

# console over USB CDC (/dev/cu.usbmodem*), not over UART pins
pico_enable_stdio_usb(probe 1)
pico_enable_stdio_uart(probe 0)

pico_add_extra_outputs(probe)  # .uf2, .bin, .hex, .dis
