# The cross-preset relay as two images (the relay's bench pair, on native_sim):
# the receiving half (this image) and the ear (the sysbuild image "ear",
# sysbuild.cmake), joined through the attachment pipe's WIRE and ENV kinds.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(meshtastic_relay_pipe)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common/attach_pipe/attach_pipe.cmake)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src
  ${CMAKE_BINARY_DIR}/modules/meshtastic-zephyr/src
  ${CMAKE_BINARY_DIR}/modules/meshtastic-zephyr/src/proto
  ${ATTACH_PIPE_INCLUDE}
)
target_sources(app PRIVATE src/main.c ${ATTACH_PIPE_SOURCES})
