cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(meshtastic_battery_cutoff_tests)

# The unit under test is a header with no Zephyr dependencies, so it can be
# included here directly without dragging in ADC, devicetree, or
# HAS_POWEROFF -- which native_sim does not have, and which is exactly why
# this logic needed to be extractable in the first place (see the header's
# own comment).
target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src
)

target_sources(app PRIVATE src/main.c)
