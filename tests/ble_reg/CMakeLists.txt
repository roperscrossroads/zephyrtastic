cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(meshtastic_ble_registry_tests)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src
)

# The registry is deliberately BT-free (see meshtastic_ble_registry.h), so it
# compiles straight into the ztest with no host, controller, or CONFIG_MESHTASTIC
# — that isolation is what makes the PM-ledger property testable on native_sim.
target_sources(app PRIVATE
  src/main.c
  ../../src/meshtastic_ble_registry.c
)
