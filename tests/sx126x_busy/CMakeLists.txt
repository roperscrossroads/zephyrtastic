cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(meshtastic_sx126x_busy_tests)

# The unit under test is a header in the SX126x driver, deliberately free of Zephyr
# dependencies so it can be included here without dragging in SPI, GPIO or a devicetree.
target_include_directories(app PRIVATE
  $ENV{ZEPHYR_BASE}/drivers/lora/native/sx126x
)

target_sources(app PRIVATE src/main.c)
