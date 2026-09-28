# SPDX-License-Identifier: GPL-3.0
#
# The keyless head's negative capability gate (ATTACHMENT-DESIGN §4, D1/D5):
# an image built as a radio head must contain none of the code that decodes,
# encrypts, signs or remembers on the identity's behalf. --gc-sections does the
# removing; this proves it, by name, on the final ELF. Run as a post-build step
# with -DNM=<nm> -DELF=<zephyr.elf>.
#
# Every pattern is a symbol PREFIX of a subsystem the head must not carry. A
# match names the symbol and the build fails: find who references it, the
# reference is the bug.
if(NOT NM OR NOT ELF)
  message(FATAL_ERROR "keyless-assert: NM and ELF are required")
endif()
execute_process(COMMAND ${NM} ${ELF} OUTPUT_VARIABLE _syms RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "keyless-assert: ${NM} ${ELF} failed (${_rc})")
endif()
set(_forbidden
  meshtastic_try_decode_wire_packet   # channel decrypt entry (the router's)
  meshtastic_build_wire_from_mesh     # channel encrypt entry (origination)
  meshtastic_router_process_rx        # the router itself
  meshtastic_pki_                     # X25519 / AES-CCM, the private key
  meshtastic_xeddsa_                  # signing with that key
  meshtastic_nodedb_                  # the identity's memory of the mesh
  meshtastic_cluster_                 # the replicated document
  meshtastic_position_                # own position, the clock from a peer
  meshtastic_admin_                   # config from the phone
)
# The NODEDB=n arm's stubs (meshtastic.c): the API the PhoneAPI, the router and
# the reliable module call whether or not a NodeDB exists. They hold nothing.
set(_allowed
  meshtastic_nodedb_count
  meshtastic_nodedb_get_next_after
  meshtastic_nodedb_get_next_hop
  meshtastic_nodedb_note_route_failure
)
set(_hits "")
string(REGEX MATCHALL "[^\n]+" _lines "${_syms}")
foreach(_line IN LISTS _lines)
  foreach(_pat IN LISTS _forbidden)
    if(_line MATCHES " [TtDdBbRr] ${_pat}")
      set(_ok FALSE)
      foreach(_a IN LISTS _allowed)
        if(_line MATCHES " [TtDdBbRr] ${_a}$")
          set(_ok TRUE)
        endif()
      endforeach()
      if(NOT _ok)
        list(APPEND _hits "${_line}")
      endif()
    endif()
  endforeach()
endforeach()
if(_hits)
  list(REMOVE_DUPLICATES _hits)
  string(REPLACE ";" "\n  " _hits_txt "${_hits}")
  message(FATAL_ERROR "keyless-assert: a radio head image links identity code:\n  ${_hits_txt}")
endif()
message(STATUS "keyless-assert: ${ELF} carries no decode, crypto, NodeDB, cluster, position or admin symbols")
