cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(meshtastic_ble_peer_codec_tests)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src
)

# The codec is deliberately BT-free (see meshtastic_ble_peer_codec.h), so it
# compiles straight into the ztest with no host or controller — every byte of
# the wire formats and the loss arithmetic verified on native_sim.
target_sources(app PRIVATE
  src/main.c
  ../../src/meshtastic_ble_peer_codec.c
)
