cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(meshtastic_cluster_doc_tests)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src
)

# The document logic is deliberately module-free (see meshtastic_cluster_doc.h):
# it compiles straight into the ztest with no radio, channels, or settings, so
# every merge/hash/effective decision — including the anti-shadowing semantics
# the whole design exists for — is verified on native_sim. meshtastic_hlc.c
# rides along for the stamp comparisons; its clock wrappers are satisfied by a
# stub in main.c.
target_sources(app PRIVATE
  src/main.c
  ../../src/meshtastic_cluster_doc.c
  ../../src/meshtastic_hlc.c
)
