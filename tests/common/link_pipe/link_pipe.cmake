# The link pipe: a test bearer that carries envelopes between two
# native_sim processes over an AF_UNIX socket (ATTACHMENT-SCOPE §4). After
# find_package(Zephyr):
#   include(${CMAKE_CURRENT_SOURCE_DIR}/../common/link_pipe/link_pipe.cmake)
#   target_sources(app PRIVATE ${LINK_PIPE_SOURCES})
#   target_include_directories(app PRIVATE ${LINK_PIPE_INCLUDE})
set(LINK_PIPE_DIR ${CMAKE_CURRENT_LIST_DIR})
set(LINK_PIPE_SOURCES ${LINK_PIPE_DIR}/link_pipe.c ${LINK_PIPE_DIR}/link_pipe_radio.c)
set(LINK_PIPE_INCLUDE ${LINK_PIPE_DIR})
# The socket half runs against the HOST's libc, outside the Zephyr image.
target_sources(native_simulator INTERFACE ${LINK_PIPE_DIR}/link_pipe_bottom.c)
