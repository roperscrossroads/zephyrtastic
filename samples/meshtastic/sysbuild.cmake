# SPDX-License-Identifier: GPL-3.0
#
# Sysbuild hook for the meshtastic sample. Included by sysbuild after the
# images exist and before any of them is configured, so this is where a
# board-specific image configuration is attached without a command-line knob.
#
# XIAO nRF52840: MCUboot is chainloaded from the resident Seeed/Adafruit UF2
# bootloader and the second slot lives on the QSPI flash (mcuboot-xiao/). The
# contract is deliberately simple —
#
#   west build --sysbuild -b xiao_ble/nrf52840 ...   -> MCUboot A/B layout
#   west build           -b xiao_ble/nrf52840 ...   -> legacy single image @0x27000
#
# — because the two layouts are not interchangeable on the wire: a legacy
# image DFU'd onto a board that has MCUboot simply replaces MCUboot (fine), but
# a slot0-linked app written at 0x27000 without its bootloader would never
# boot. Keying the layout on --sysbuild, which is also what builds the
# bootloader, means the two cannot be mixed by a stale variable.
#
# Mechanism. Vanilla sysbuild has no add_overlay_config(); the image-scoped
# <image>_EXTRA_CONF_FILE / <image>_EXTRA_DTC_OVERLAY_FILE cache variables are
# how an image's fragment list is extended from here. One trap, verified in
# zephyr_get() (cmake/modules/extensions.cmake): for the MAIN app the
# unprefixed EXTRA_CONF_FILE the user passed on the command line is consulted
# ONLY when no prefixed variable exists — defining meshtastic_EXTRA_CONF_FILE
# silently drops the user's overlay-ota-shell.conf. So the prefixed list is
# SEEDED from the unprefixed value first (relative paths resolved against the
# app dir, as the image itself would), and this file's fragments are appended
# after it so the layout is what wins on any conflict.

if(BOARD MATCHES "^xiao_ble")
  set(_xm ${CMAKE_CURRENT_LIST_DIR}/mcuboot-xiao)

  # Seed <main>_<VAR> from the unprefixed VAR (and OVERLAY_CONFIG, the
  # deprecated alias of EXTRA_CONF_FILE), if the prefixed one is not yet set.
  # Second trap (2026-08-28): the prefixed list is a CACHE variable, so once
  # seeded it survived every later configure of the same build dir — a changed
  # -DEXTRA_CONF_FILE on the command line was silently ignored (a secure
  # overlay added to build-xiao-f7 never reached the image; only a pristine
  # build did). Remember what the list was seeded FROM and re-seed whenever
  # the unprefixed value differs from that record.
  foreach(seed "EXTRA_CONF_FILE;EXTRA_CONF_FILE;OVERLAY_CONFIG"
               "EXTRA_DTC_OVERLAY_FILE;EXTRA_DTC_OVERLAY_FILE")
    list(POP_FRONT seed var)
    set(unprefixed)
    foreach(alias ${seed})
      list(APPEND unprefixed ${${alias}})
    endforeach()
    if(NOT DEFINED ${DEFAULT_IMAGE}_${var}
       OR NOT "${unprefixed}" STREQUAL "${${DEFAULT_IMAGE}_${var}_SEEDED_FROM}")
      set(seeded)
      foreach(alias ${seed})
        foreach(f ${${alias}})
          if(NOT IS_ABSOLUTE ${f} AND EXISTS ${APP_DIR}/${f})
            set(f ${APP_DIR}/${f})
          endif()
          list(APPEND seeded ${f})
        endforeach()
      endforeach()
      set(${DEFAULT_IMAGE}_${var}_SEEDED_FROM "${unprefixed}" CACHE INTERNAL
          "meshtastic sysbuild.cmake: the unprefixed ${var} this image's list was seeded from")
      if(seeded)
        set(${DEFAULT_IMAGE}_${var} ${seeded} CACHE INTERNAL
            "meshtastic sysbuild.cmake: seeded from the unprefixed ${var}")
      else()
        # Re-seeded from nothing: drop the stale list too, or it would live on.
        unset(${DEFAULT_IMAGE}_${var} CACHE)
      endif()
    endif()
  endforeach()

  foreach(pair
          "${DEFAULT_IMAGE};EXTRA_CONF_FILE;${_xm}/app.conf"
          "${DEFAULT_IMAGE};EXTRA_DTC_OVERLAY_FILE;${_xm}/app.overlay"
          "mcuboot;EXTRA_CONF_FILE;${_xm}/mcuboot.conf"
          "mcuboot;EXTRA_DTC_OVERLAY_FILE;${_xm}/mcuboot.overlay")
    list(GET pair 0 img)
    list(GET pair 1 var)
    list(GET pair 2 file)
    set(current ${${img}_${var}})
    if(NOT file IN_LIST current)
      list(APPEND current ${file})
    endif()
    set(${img}_${var} ${current} CACHE INTERNAL "meshtastic sysbuild.cmake: XIAO MCUboot layout")
  endforeach()

  message(STATUS "meshtastic: XIAO nRF52840 MCUboot layout attached (mcuboot-xiao/)")
  message(STATUS "meshtastic:   ${DEFAULT_IMAGE} EXTRA_CONF_FILE = ${${DEFAULT_IMAGE}_EXTRA_CONF_FILE}")
  message(STATUS "meshtastic:   ${DEFAULT_IMAGE} EXTRA_DTC_OVERLAY_FILE = ${${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE}")
endif()
