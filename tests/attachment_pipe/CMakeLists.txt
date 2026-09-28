# The two-image attachment harness (ATTACHMENT-REVIEW part 2, layer 3): the
# same application built as a brain (this image) and as a keyless head (the
# sysbuild image "head", sysbuild.cmake), joined through the attachment pipe.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(meshtastic_attachment_pipe)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common/attach_pipe/attach_pipe.cmake)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src
  ${CMAKE_BINARY_DIR}/modules/meshtastic-zephyr/src
  ${CMAKE_BINARY_DIR}/modules/meshtastic-zephyr/src/proto
  ${ATTACH_PIPE_INCLUDE}
)
target_sources(app PRIVATE src/main.c ${ATTACH_PIPE_SOURCES})

# The head image carries the negative capability gate, as on hardware.
if(CONFIG_MESHTASTIC_ATTACHMENT_HEAD)
  add_custom_target(keyless_assert ALL DEPENDS ${logical_target_for_zephyr_elf}
    COMMAND ${CMAKE_COMMAND} -DNM=${CMAKE_NM} -DELF=${PROJECT_BINARY_DIR}/zephyr/${KERNEL_ELF_NAME}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/../../samples/meshtastic/cmake/keyless-assert.cmake)
endif()
