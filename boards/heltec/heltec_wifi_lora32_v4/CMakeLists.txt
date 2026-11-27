# SPDX-License-Identifier: Apache-2.0

# The application only: MCUboot has no radio, and the file's SYS_INITs would power the front end
# and pulse the GNSS reset in the bootloader too (the GNSS was reset twice per boot).
if(CONFIG_GPIO_ESP32 AND NOT CONFIG_MCUBOOT)
  zephyr_library()
  zephyr_library_sources(heltec_wifi_lora32_v4_fem.c)
endif()
