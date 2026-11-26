cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(meshtastic_shell_tests)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src
  ${CMAKE_BINARY_DIR}/modules/meshtastic-zephyr/src
  ${CMAKE_BINARY_DIR}/modules/meshtastic-zephyr/src/proto
  # shell_utils.h, for z_shell_cmd_get() (test_every_command_is_wrapped_or_listed_as_staying)
  ${ZEPHYR_BASE}/subsys/shell
)

target_sources(app PRIVATE src/main.c)
