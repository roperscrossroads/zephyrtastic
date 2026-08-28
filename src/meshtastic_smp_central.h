/* SPDX-License-Identifier: GPL-3.0 */

#ifndef MESHTASTIC_SMP_CENTRAL_H_
#define MESHTASTIC_SMP_CENTRAL_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/addr.h>

/*
 * The SMP (mcumgr) client over BLE — this node manages a peer node's images.
 * Mechanism and shell (`smpc`) in meshtastic_smp_central.c. This header is the
 * programmatic surface: what a caller that is not the shell (a fleet
 * reconciler acting on the cluster document) needs in order to drive the same
 * ladder the shell does. Single link, single job, by design.
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Connect to (or ADOPT — Zephyr keeps one bt_conn per address, so if the peer
 * already holds a link to us, this rides it) the peer's SMP GATT service and
 * subscribe. Blocks up to `timeout` for the link to become SMP-ready.
 * -EBUSY if a link is already held; -ETIMEDOUT / the connect error otherwise.
 */
int meshtastic_smpc_connect(const bt_addr_le_t *addr, k_timeout_t timeout);

/* Drop the link — or, for an adopted link, only our subscription on it. */
int meshtastic_smpc_disconnect(void);

/* True when SMP frames can flow (subscribed). */
bool meshtastic_smpc_ready(void);

/*
 * What a local image is, read from its MCUboot header: the byte count to send
 * (header + body + protected TLVs) and the semantic version. `path` names a
 * file under the depot (CONFIG_MESHTASTIC_DEPOT), or NULL/"" for this node's
 * own slot1.
 */
struct meshtastic_smpc_image {
	uint32_t size;
	uint8_t major, minor;
	uint16_t revision;
	uint32_t build;
	uint8_t class_id;	/* protected TLV 0x00A0 (CONFIG_MESHTASTIC_FLEET_CLASS at sign
				 * time); 0 = the image does not say what it is for */
};

int meshtastic_smpc_local_image(const char *path, struct meshtastic_smpc_image *out);

/*
 * A job: a whole image transfer. PUSH uploads into the target's slot1 and
 * stops; UPDATE runs the full ladder (mark test, reset, reconnect, verify,
 * confirm); UPDATE_NOCONFIRM stops after verify, leaving the target to confirm
 * itself (R8). Values match the internal enum (asserted in the .c).
 */
enum meshtastic_smpc_job_kind {
	MESHTASTIC_SMPC_JOB_NONE = 0,
	MESHTASTIC_SMPC_JOB_PUSH,
	MESHTASTIC_SMPC_JOB_UPDATE,
	MESHTASTIC_SMPC_JOB_UPDATE_NOCONFIRM,
};

struct meshtastic_smpc_job {
	enum meshtastic_smpc_job_kind kind;
	bool running;
	int rc;            /* mgmt error of the last/current job; 0 = ok */
	const char *stage; /* "queued"/"upload"/…/"done"/"failed" */
	const char *fail_stage; /* the stage a failed job was in ("" if none/ok) */
	uint32_t off, total;
};

/* Start a job on an already-ready link. -ENOTCONN if not ready, -EBUSY if one
 * runs. Returns at once; watch it with job_get. */
int meshtastic_smpc_job_start(enum meshtastic_smpc_job_kind kind, const char *path);

/* A snapshot of the current/last job. */
void meshtastic_smpc_job_get(struct meshtastic_smpc_job *out);

/*
 * The depot index (F4, DECLARATIVE-FLEET.md §10.2): what /depot holds, read from
 * each file's MCUboot header and protected TLVs — the image is the manifest, the
 * filename is for humans. Built lazily on first use, rebuilt by rescan() (the
 * shell's `fleet rescan`, after loading cargo) and once on a lookup miss, so an
 * image uploaded mid-run is found without a reboot.
 */
#define MESHTASTIC_SMPC_DEPOT_ROWS 16

struct meshtastic_smpc_depot_row {
	char name[40];		/* the file's name under /depot */
	uint32_t version;	/* packed (major << 24 | minor << 16 | revision) */
	uint32_t size;
	uint8_t class_id;	/* 0 = unclassed: never offered to anyone */
};

/* Rebuild the index. Returns the row count, or -errno (-ENOTSUP without a depot). */
int meshtastic_smpc_depot_rescan(void);

/* Copy up to @p max rows out; returns the count. @p not_indexed (may be NULL)
 * receives how many files in /depot are NOT in the index — images beyond the
 * row cap, plus files that are not signed images — so a listing can say so
 * instead of silently looking complete (bench 2026-08-27: a ninth file was the
 * newest cargo, and the courier reported "no such image"). */
uint16_t meshtastic_smpc_depot_rows(struct meshtastic_smpc_depot_row *out, uint16_t max,
				    uint16_t *not_indexed);

/*
 * The depot file of class @p class_id whose version packs to @p packed_version
 * -> @p path_out. -EINVAL for class 0 (a node that does not say what it is gets
 * nothing; an image that does not say what it is for is offered to nobody),
 * -ENOENT if there is no such image, -ENOTSUP without a depot.
 */
int meshtastic_smpc_depot_find(uint8_t class_id, uint32_t packed_version, char *path_out,
			       size_t path_len);


#ifdef __cplusplus
}
#endif

#endif /* MESHTASTIC_SMP_CENTRAL_H_ */
