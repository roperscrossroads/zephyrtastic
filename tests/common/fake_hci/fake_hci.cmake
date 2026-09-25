# Include BEFORE find_package(Zephyr): points devicetree at the fake controller.
#   include(${CMAKE_CURRENT_SOURCE_DIR}/../common/fake_hci/fake_hci.cmake)
# then, after find_package:  target_sources(app PRIVATE ${FAKE_HCI_SOURCES})
set(FAKE_HCI_DIR ${CMAKE_CURRENT_LIST_DIR})
list(APPEND DTS_ROOT ${FAKE_HCI_DIR})
list(APPEND EXTRA_DTC_OVERLAY_FILE ${FAKE_HCI_DIR}/fake_hci.overlay)
set(FAKE_HCI_SOURCES ${FAKE_HCI_DIR}/fake_hci.c)
set(FAKE_HCI_INCLUDE ${FAKE_HCI_DIR})
