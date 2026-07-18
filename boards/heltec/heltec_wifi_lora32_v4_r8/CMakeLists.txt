# SPDX-License-Identifier: Apache-2.0

# Reuse the plain-V4 FEM detection/init (R8 fits the KCT8103L front-end).
if(CONFIG_GPIO_ESP32)
  zephyr_library()
  zephyr_library_sources(
    ${CMAKE_CURRENT_LIST_DIR}/../heltec_wifi_lora32_v4/heltec_wifi_lora32_v4_fem.c
  )
endif()
