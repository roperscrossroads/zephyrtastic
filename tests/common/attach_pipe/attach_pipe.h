/* SPDX-License-Identifier: GPL-3.0 */
/*
 * The attachment pipe: a test bearer (ATTACHMENT-SCOPE §4) that carries
 * envelopes between two native_sim images -- a real brain and a real head --
 * through a hub process (the test driver) over an AF_UNIX stream socket.
 *
 * Every frame on the socket is
 *
 *   [len u16 LE][crc16 u16 LE][kind u8][peer u32 LE][payload]
 *
 * with len counting kind..payload and the CRC (CRC-16/CCITT-FALSE, the one a
 * UART bearer would use) over the same bytes. Kind ENV is the bearer itself:
 * an attachment envelope to or from @p peer, untouched. The UART bearer is the
 * same framing with only ENV and no peer (a wire has one). The other kinds are
 * the harness talking to the hub:
 *
 *   HELLO  image -> hub   peer = this image's node id; payload [role]
 *   LINK   hub -> image   the link to peer changed: [up][auth][takes_env][rtt_ms u16]
 *   RF     hub -> image   a frame on the air: [preset][rssi i16][snr i8]
 *                         [freq_hz u32][sf u8][bw_khz u16][wire]
 *   CMD    hub -> image   a text command for the application
 *   EVENT  image -> hub   a text line the test reads
 *   WIRE   either way     a bare wire frame over the peer link (the relay's
 *                         ear -> receiving half), not an attachment envelope
 *
 * The hub owns latency, loss, reordering and trust (it says what auth a link
 * has): the knobs live in one place, in the driver, per test.
 */
#ifndef ATTACH_PIPE_H_
#define ATTACH_PIPE_H_

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#include "meshtastic_attach_bearer.h"

enum attach_pipe_kind {
	ATTACH_PIPE_ENV = 0,
	ATTACH_PIPE_HELLO = 1,
	ATTACH_PIPE_LINK = 2,
	ATTACH_PIPE_RF = 3,
	ATTACH_PIPE_CMD = 4,
	ATTACH_PIPE_EVENT = 5,
	ATTACH_PIPE_WIRE = 6,
};

enum attach_pipe_role {
	ATTACH_PIPE_ROLE_BRAIN = 1,
	ATTACH_PIPE_ROLE_HEAD = 2,
	ATTACH_PIPE_ROLE_RELAY = 3, /* the relay's receiving half */
	ATTACH_PIPE_ROLE_EAR = 4,
};

/* A frame on the air (hub RF frame): sent on @p preset at this tuning, heard
 * with this signal if the radio is tuned there. The hub works the tuning out
 * itself, so a radio on the wrong frequency does not hear it. */
struct attach_pipe_rf {
	uint8_t preset;
	int16_t rssi;
	int8_t snr;
	uint32_t freq_hz;
	uint8_t sf;
	uint16_t bw_khz;
};
typedef void (*attach_pipe_rf_cb)(const struct attach_pipe_rf *rf, const uint8_t *wire, size_t len);
/* A bare wire frame from @p peer (kind WIRE): what the BLE peer link's frame
 * channel carries when it is not an envelope. */
typedef void (*attach_pipe_wire_cb)(uint32_t peer, const uint8_t *wire, size_t len);
/* A text command from the hub, NUL-terminated. */
typedef void (*attach_pipe_cmd_cb)(const char *cmd);

/* Connect to the hub at $ATTACH_PIPE_SOCK, say HELLO, register the bearer and
 * start the RX thread. Returns 0, or a negative errno. */
int attach_pipe_start(enum attach_pipe_role role, uint32_t node, attach_pipe_rf_cb rf,
		      attach_pipe_cmd_cb cmd);

/* Where WIRE frames go (set before or after start; NULL drops them). */
void attach_pipe_set_wire_cb(attach_pipe_wire_cb cb);

/* Send a bare wire frame to @p peer over the pipe. -EHOSTUNREACH when the hub
 * has not said the link to @p peer is up. */
int attach_pipe_send_wire(uint32_t peer, const uint8_t *wire, size_t len);

/* One text line to the hub (kind EVENT). */
void attach_pipe_event(const char *fmt, ...);

/* A value from the host environment, or @p dflt. */
const char *attach_pipe_getenv(const char *name, const char *dflt);

#if defined(CONFIG_MESHTASTIC_ATTACH_BEARER)
extern const struct meshtastic_attach_bearer attach_pipe_bearer;
#endif

#endif /* ATTACH_PIPE_H_ */
