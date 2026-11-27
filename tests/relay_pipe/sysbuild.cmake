# The second image: this same application, configured as the relay's rx-only worker.
# Sysbuild reads sysbuild.cmake for every image from this directory; add once.
get_property(_worker_rx_added GLOBAL PROPERTY RELAY_PIPE_WORKER_RX_ADDED)
if(NOT _worker_rx_added)
  set_property(GLOBAL PROPERTY RELAY_PIPE_WORKER_RX_ADDED TRUE)
  ExternalZephyrProject_Add(
    APPLICATION worker_rx
    SOURCE_DIR ${APP_DIR}
  )
  set(worker_rx_EXTRA_CONF_FILE ${APP_DIR}/worker_rx.conf CACHE INTERNAL "the rx-only worker's role")
endif()
