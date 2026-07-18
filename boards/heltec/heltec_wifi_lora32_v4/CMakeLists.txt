# SPDX-License-Identifier: Apache-2.0

if(CONFIG_GPIO_ESP32)
  zephyr_library()
  zephyr_library_sources(heltec_wifi_lora32_v4_fem.c)
endif()
