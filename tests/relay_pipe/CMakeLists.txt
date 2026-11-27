# The cross-preset relay as two images (the relay's bench pair, on native_sim):
# the receiving half (this image) and the rx-only worker (the sysbuild image "worker_rx",
# sysbuild.cmake), joined through the link pipe's WIRE and ENV kinds.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(meshtastic_relay_pipe)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common/link_pipe/link_pipe.cmake)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src
  ${CMAKE_BINARY_DIR}/modules/meshtastic-zephyr/src
  ${CMAKE_BINARY_DIR}/modules/meshtastic-zephyr/src/proto
  ${LINK_PIPE_INCLUDE}
)
target_sources(app PRIVATE src/main.c ${LINK_PIPE_SOURCES})
