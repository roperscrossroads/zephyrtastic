# The second image: this same application, configured as a keyless head.
# Sysbuild reads an image's sysbuild.cmake for every image built from its
# source directory -- the head's included -- so add the head once.
get_property(_head_added GLOBAL PROPERTY ATTACH_PIPE_HEAD_ADDED)
if(NOT _head_added)
  set_property(GLOBAL PROPERTY ATTACH_PIPE_HEAD_ADDED TRUE)
  ExternalZephyrProject_Add(
    APPLICATION head
    SOURCE_DIR ${APP_DIR}
  )
  set(head_EXTRA_CONF_FILE ${APP_DIR}/head.conf CACHE INTERNAL "the head's role")
endif()
