# The second image: this same application, configured as the relay's ear.
# Sysbuild reads sysbuild.cmake for every image from this directory; add once.
get_property(_ear_added GLOBAL PROPERTY RELAY_PIPE_EAR_ADDED)
if(NOT _ear_added)
  set_property(GLOBAL PROPERTY RELAY_PIPE_EAR_ADDED TRUE)
  ExternalZephyrProject_Add(
    APPLICATION ear
    SOURCE_DIR ${APP_DIR}
  )
  set(ear_EXTRA_CONF_FILE ${APP_DIR}/ear.conf CACHE INTERNAL "the ear's role")
endif()
