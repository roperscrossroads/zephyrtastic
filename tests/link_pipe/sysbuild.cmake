# The second image: this same application, configured as a keyless worker.
# Sysbuild reads an image's sysbuild.cmake for every image built from its
# source directory -- the worker's included -- so add the worker once.
get_property(_worker_added GLOBAL PROPERTY LINK_PIPE_WORKER_ADDED)
if(NOT _worker_added)
  set_property(GLOBAL PROPERTY LINK_PIPE_WORKER_ADDED TRUE)
  ExternalZephyrProject_Add(
    APPLICATION worker
    SOURCE_DIR ${APP_DIR}
  )
  set(worker_EXTRA_CONF_FILE ${APP_DIR}/worker.conf CACHE INTERNAL "the worker's role")
endif()
