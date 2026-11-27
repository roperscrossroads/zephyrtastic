# The two-image link harness (ATTACHMENT-REVIEW part 2, layer 3): the
# same application built as a hub (this image) and as a keyless worker (the
# sysbuild image "worker", sysbuild.cmake), joined through the link pipe.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(meshtastic_link_pipe)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common/link_pipe/link_pipe.cmake)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src
  ${CMAKE_BINARY_DIR}/modules/meshtastic-zephyr/src
  ${CMAKE_BINARY_DIR}/modules/meshtastic-zephyr/src/proto
  ${LINK_PIPE_INCLUDE}
)
target_sources(app PRIVATE src/main.c ${LINK_PIPE_SOURCES})

# The worker image carries the negative capability gate, as on hardware.
if(CONFIG_MESHTASTIC_WORKER)
  add_custom_target(keyless_assert ALL DEPENDS ${logical_target_for_zephyr_elf}
    COMMAND ${CMAKE_COMMAND} -DNM=${CMAKE_NM} -DELF=${PROJECT_BINARY_DIR}/zephyr/${KERNEL_ELF_NAME}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/../../samples/meshtastic/cmake/keyless-assert.cmake)
endif()
