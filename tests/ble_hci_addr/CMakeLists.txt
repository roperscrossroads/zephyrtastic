cmake_minimum_required(VERSION 3.20.0)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common/fake_hci/fake_hci.cmake)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(meshtastic_ble_hci_addr_tests)

# The real Zephyr BT host on native_sim, against the fake controller — no
# firmware module code, just the host and our exact advertise/scan params.
target_include_directories(app PRIVATE ${FAKE_HCI_INCLUDE})
target_sources(app PRIVATE src/main.c ${FAKE_HCI_SOURCES})
