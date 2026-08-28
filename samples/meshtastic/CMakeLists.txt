# SPDX-License-Identifier: GPL-3.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(meshtastic_sample)

target_sources(app PRIVATE src/main.c)

# The fleet class is baked twice — CONFIG_MESHTASTIC_FLEET_CLASS in the firmware
# (beat, `meshtastic version`) and protected TLV 0x00A0 in the signed image (what a
# depot reads). Kconfig cannot derive one string from the other, so each variant's
# overlay states both; this refuses a build where they drift apart.
if(CONFIG_MESHTASTIC_FLEET_CLASS AND NOT CONFIG_MESHTASTIC_FLEET_CLASS EQUAL 0
   AND CONFIG_BOOTLOADER_MCUBOOT)
  math(EXPR _fleet_class_hex "${CONFIG_MESHTASTIC_FLEET_CLASS}" OUTPUT_FORMAT HEXADECIMAL)
  string(REGEX REPLACE "^0x" "" _fleet_class_hex "${_fleet_class_hex}")
  string(LENGTH "${_fleet_class_hex}" _len)
  if(_len EQUAL 1)
    set(_fleet_class_hex "0${_fleet_class_hex}")
  endif()
  set(_want "--custom-tlv 0x00A0 0x${_fleet_class_hex}")
  string(FIND "${CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS}" "${_want}" _pos)
  if(_pos EQUAL -1)
    message(FATAL_ERROR
      "CONFIG_MESHTASTIC_FLEET_CLASS=${CONFIG_MESHTASTIC_FLEET_CLASS} but the image would not be "
      "signed with it: CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS is '${CONFIG_MCUBOOT_EXTRA_IMGTOOL_ARGS}', "
      "expected to contain '${_want}' (set it in the variant's overlay next to the class).")
  endif()
endif()
