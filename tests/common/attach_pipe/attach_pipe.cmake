# The attachment pipe: a test bearer that carries envelopes between two
# native_sim processes over an AF_UNIX socket (ATTACHMENT-SCOPE §4). After
# find_package(Zephyr):
#   include(${CMAKE_CURRENT_SOURCE_DIR}/../common/attach_pipe/attach_pipe.cmake)
#   target_sources(app PRIVATE ${ATTACH_PIPE_SOURCES})
#   target_include_directories(app PRIVATE ${ATTACH_PIPE_INCLUDE})
set(ATTACH_PIPE_DIR ${CMAKE_CURRENT_LIST_DIR})
set(ATTACH_PIPE_SOURCES ${ATTACH_PIPE_DIR}/attach_pipe.c ${ATTACH_PIPE_DIR}/attach_pipe_radio.c)
set(ATTACH_PIPE_INCLUDE ${ATTACH_PIPE_DIR})
# The socket half runs against the HOST's libc, outside the Zephyr image.
target_sources(native_simulator INTERFACE ${ATTACH_PIPE_DIR}/attach_pipe_bottom.c)
