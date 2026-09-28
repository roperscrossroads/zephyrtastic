cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(meshtastic_attachment_tests)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${CMAKE_BINARY_DIR}/modules/meshtastic-zephyr/src
  ${CMAKE_BINARY_DIR}/modules/meshtastic-zephyr/src/proto
)

target_sources_ifdef(CONFIG_MESHTASTIC_ATTACHMENT_BRAIN app PRIVATE src/main.c)
target_sources_ifdef(CONFIG_MESHTASTIC_ATTACHMENT_HEAD app PRIVATE src/head.c)

# The head scenario is also the negative capability gate's first run.
if(CONFIG_MESHTASTIC_ATTACHMENT_HEAD)
  # Zephyr's post_build_commands is not extensible from an application, and a
  # POST_BUILD on the ELF target must be declared in zephyr/'s own directory;
  # an ALL target that depends on the ELF is the portable hook.
  add_custom_target(keyless_assert ALL DEPENDS ${logical_target_for_zephyr_elf}
    COMMAND ${CMAKE_COMMAND} -DNM=${CMAKE_NM} -DELF=${PROJECT_BINARY_DIR}/zephyr/${KERNEL_ELF_NAME}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/../../samples/meshtastic/cmake/keyless-assert.cmake)
endif()
