cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(meshtastic_fatal_tests)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${CMAKE_BINARY_DIR}/modules/meshtastic-zephyr/src
  ${CMAKE_BINARY_DIR}/modules/meshtastic-zephyr/src/proto
)

target_sources(app PRIVATE src/main.c)

# The handler ends in arch_system_halt() with RESET_ON_FATAL_ERROR off; the
# POSIX arch defines it (it exits the process), so the test takes it over at
# link time and jumps back into the case instead.
zephyr_link_libraries(-Wl,--wrap=arch_system_halt)
