# SPDX-License-Identifier: Apache-2.0

# Reuse the plain-V4 FEM detection/init (R8 fits the KCT8103L front-end). The application only:
# MCUboot has no radio (see the V4 board's CMakeLists).
if(CONFIG_GPIO_ESP32 AND NOT CONFIG_MCUBOOT)
  zephyr_library()
  zephyr_library_sources(
    ${CMAKE_CURRENT_LIST_DIR}/../heltec_wifi_lora32_v4/heltec_wifi_lora32_v4_fem.c
  )
endif()
