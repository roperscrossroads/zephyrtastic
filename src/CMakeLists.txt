# SPDX-License-Identifier: GPL-3.0

if(CONFIG_MESHTASTIC)

  zephyr_library_named(meshtastic)

  list(APPEND CMAKE_MODULE_PATH ${ZEPHYR_BASE}/modules/nanopb)
  include(nanopb)
  set(NANOPB_IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/proto)

  # Generate C sources from the Meshtastic protobuf schemas.
  set(MESHTASTIC_NANOPB_PROTOS
    proto/meshtastic/atak.proto
    proto/meshtastic/channel.proto
    proto/meshtastic/config.proto
    proto/meshtastic/deviceonly.proto
    proto/meshtastic/device_ui.proto
    proto/meshtastic/module_config.proto
    proto/meshtastic/portnums.proto
    proto/meshtastic/telemetry.proto
    proto/meshtastic/xmodem.proto
    proto/meshtastic/mesh.proto
  )
  if(CONFIG_MESHTASTIC_MQTT)
    list(APPEND MESHTASTIC_NANOPB_PROTOS proto/meshtastic/mqtt.proto)
  endif()
  if(CONFIG_MESHTASTIC_ADMIN)
    list(APPEND MESHTASTIC_NANOPB_PROTOS
      proto/meshtastic/connection_status.proto
      proto/meshtastic/admin.proto
    )
  endif()
  if(CONFIG_MESHTASTIC_CLUSTER)
    # zephyrtastic's OWN schema (cluster sync, docs/CLUSTER-SYNC-M4.md) —
    # private protocol on PRIVATE_APP, forks nothing upstream.
    list(APPEND MESHTASTIC_NANOPB_PROTOS proto/zephyrtastic/cluster.proto)
  endif()
  zephyr_nanopb_sources(meshtastic ${MESHTASTIC_NANOPB_PROTOS})

  # Build identity (git describe + build time) regenerated on every build into a
  # header, isolated in meshtastic_build.c. See cmake/gen_build_id.cmake.
  set(MT_BUILD_ID_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
  set(MT_BUILD_ID_H ${MT_BUILD_ID_DIR}/meshtastic_build_id.h)
  add_custom_target(meshtastic_build_id_gen
    BYPRODUCTS ${MT_BUILD_ID_H}
    COMMAND ${CMAKE_COMMAND}
      -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}
      -DOUT=${MT_BUILD_ID_H}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_build_id.cmake
    COMMENT "meshtastic: capturing build id"
    VERBATIM)
  add_dependencies(meshtastic meshtastic_build_id_gen)

  target_include_directories(meshtastic PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/proto
    ${MT_BUILD_ID_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

  zephyr_linker_sources(SECTIONS iterables.ld)

  zephyr_library_sources(
    meshtastic.c
    meshtastic_build.c
    meshtastic_channels.c
    meshtastic_clock.c
    meshtastic_config_store.c
    meshtastic_fatal.c
    meshtastic_hlc.c
    meshtastic_modules.c
    meshtastic_packet.c
    meshtastic_preset.c
    meshtastic_radio.c
    meshtastic_outbound.c
    meshtastic_contention.c
    meshtastic_tx_power.c
    meshtastic_sched.c
    meshtastic_router.c
    meshtastic_routing.c
    meshtastic_settings_apply.c
    # Was PHONEAPI-only when it just built the want_config preset map. The radio
    # path now resolves modem presets through it, so it must build even in a
    # phone-less image. The map builder itself is dropped by --gc-sections when
    # PHONEAPI is off.
    meshtastic_region_presets.c
  )
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_SETTINGS meshtastic_settings.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_PHONEAPI
    meshtastic_phoneapi.c
    meshtastic_phoneapi_config.c
  )
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_ADMIN
    meshtastic_admin.c
    meshtastic_admin_session.c
  )
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_ADMIN_CLIENT meshtastic_admin_client.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_CLUSTER
    meshtastic_cluster.c
    meshtastic_cluster_doc.c
  )
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_FLEET meshtastic_fleet.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_OTA_AUTOCONFIRM meshtastic_ota_confirm.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_BLE
    meshtastic_ble.c
    meshtastic_ble_name.c
    meshtastic_ble_registry.c
  )
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_BLE_PEER
    meshtastic_ble_peer.c
    meshtastic_ble_peer_codec.c
  )
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_SERIAL meshtastic_serial.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_TCP meshtastic_tcp.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_BOOTLOG meshtastic_bootlog.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_FAULTMARK meshtastic_faultmark.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_SCANNER meshtastic_scanner.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_RF_PATH_REPORT meshtastic_rf_path.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_RF_HIST meshtastic_rf_measure.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_SHELL meshtastic_shell.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_NODEDB meshtastic_nodedb.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_POSITION meshtastic_position.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_GNSS meshtastic_gnss.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_GNSS_PPS meshtastic_gnss_pps.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_CLOCK_PERSIST meshtastic_clock_persist.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_AIRTIME meshtastic_airtime.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_DUTY_CYCLE meshtastic_duty.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_POWERMON meshtastic_powermon.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_WATCHDOG meshtastic_watchdog.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_DFU_TRIGGER meshtastic_dfu_trigger.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_SMP_CENTRAL meshtastic_smp_central.c)
  # img_mgmt_client.h drags in bootutil/image.h, whose include path is an INTERFACE of
  # the MCUBOOT_BOOTUTIL target (not global) — link it the way Zephyr's own client lib does.
  zephyr_library_link_libraries_ifdef(CONFIG_MESHTASTIC_SMP_CENTRAL MCUBOOT_BOOTUTIL)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_HEAP_TRACE meshtastic_heap_trace.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_NETLOG_FILTER meshtastic_netlog_filter.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_SHELL_UART0 meshtastic_shell_uart0.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_THREAD_SUMMARY meshtastic_thread_summary.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_SUPERVISOR meshtastic_supervisor.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_WIFI_DIAG meshtastic_wifi_diag.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_LOGRING meshtastic_logring.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_PHONELOG meshtastic_phonelog.c)
  zephyr_library_sources_ifdef(CONFIG_PM meshtastic_power.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_DEVICE_METRICS meshtastic_metrics.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_ENVIRONMENT_METRICS meshtastic_environment.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_TELEMETRY_WANT_RESPONSE meshtastic_telemetry_common.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_NODEINFO meshtastic_nodeinfo.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_MESSAGE meshtastic_message.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_TRACEROUTE meshtastic_traceroute.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_RELIABLE_DELIVERY meshtastic_reliable.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_MQTT meshtastic_mqtt.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_WIFI_AUTOCONNECT meshtastic_wifi_auto.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_SNTP meshtastic_sntp.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_PKI meshtastic_pki.c)
  zephyr_library_sources_ifdef(CONFIG_MESHTASTIC_DISPLAY meshtastic_display.c cfb_font_0609.c)

endif()
