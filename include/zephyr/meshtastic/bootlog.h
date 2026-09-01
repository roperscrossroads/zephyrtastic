/* SPDX-License-Identifier: GPL-3.0
 *
 * Why did this node reboot? A per-boot record that outlives the boot.
 */
#ifndef ZEPHYR_MESHTASTIC_BOOTLOG_H_
#define ZEPHYR_MESHTASTIC_BOOTLOG_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

/** One boot. Twelve bytes, so a useful history costs ~100 B of retained RAM. */
struct meshtastic_boot_record {
	/** Monotonic across warm resets; restarts at 1 when retained RAM is lost. */
	uint32_t boot_num;
	/** Zephyr hwinfo RESET_* bits as read at boot, 0 if unavailable there. */
	uint32_t cause;
	/** MESHTASTIC_BOOT_F_* */
	uint16_t flags;
	/** Seconds this node had been up when it last wrote a heartbeat, so a
	 *  reboot can be placed relative to the run that preceded it. 0 = unknown. */
	uint16_t prev_uptime_s;
};

/** Retained RAM survived, so the previous boot ended in a WARM reset — the CPU
 *  restarted but the RAM rail never dropped. Its absence is the interesting
 *  case: it means power was lost, or something cleared RAM. */
#define MESHTASTIC_BOOT_F_WARM     BIT(0)
/** hwinfo returned a non-zero cause. On a board whose bootloader consumes the
 *  reset-reason register before the app runs, this stays clear and `cause` is
 *  meaningless rather than merely zero — the two must not look alike. */
#define MESHTASTIC_BOOT_F_CAUSE_OK BIT(1)

/**
 * One boot, in FLASH. Sixteen bytes.
 *
 * The retained-RAM ring above reports a power event by ABSENCE — the magic
 * fails, the counter restarts at 1, and everything before it is gone. That is
 * enough to say "power was lost" and nothing else: not when, not how often, not
 * how long the node had been up first. On a fleet whose only bearers are LoRa
 * and BLE the gap is worse than untidy, because the only way to recover a node
 * that has stopped answering both is to carry it to a USB host, and that removes
 * power — so the recovery destroys the evidence, every time. A node that dies in
 * the field is therefore un-diagnosable by construction (see the 2026-08-31
 * courier fault, where `crashinfo` read "none pending" purely because the rescue
 * had cleared it).
 *
 * This ring is the durable half. It survives the power cycle that clears the
 * other one, so the RATE of resets — usually most of the signal — outlives it.
 */
struct meshtastic_boot_durable {
	/** Boot counter at the time of writing. Restarts with the RAM history. */
	uint32_t boot_num;
	/** hwinfo cause, as for the RAM record. Meaningful only with F_CAUSE_OK. */
	uint32_t cause;
	/** Epoch seconds when this record was written, or 0 if the clock was not
	 *  valid yet. NOT the moment of the reset: it is early in the NEXT boot,
	 *  which is the closest honest timestamp available without deferring the
	 *  write (and a deferred write is one a dying node never makes). */
	uint32_t wall_s;
	/** MESHTASTIC_BOOT_F_* — the warm/cold bit is the load-bearing one. */
	uint16_t flags;
	/** How long the run BEFORE this boot lasted, seconds. 0 = unknown. */
	uint16_t prev_uptime_s;
};

/** @brief This boot's record. */
void meshtastic_bootlog_this_boot(struct meshtastic_boot_record *out);

/**
 * @brief Copy out the retained history, oldest first.
 *
 * @return number written (0 when retained RAM was lost, i.e. this is boot 1 of
 *         a fresh history — which is itself the answer to "was it a cold start").
 */
size_t meshtastic_bootlog_history(struct meshtastic_boot_record *out, size_t max);

/** @brief Decode @p cause into a human string ("WATCHDOG PIN", "none", …). */
const char *meshtastic_bootlog_cause_str(uint32_t cause, char *buf, size_t buflen);

/** @brief Log this boot's record plus the retained history. */
void meshtastic_bootlog_report(void);

/**
 * @brief Note that the node is alive and has been up @p uptime_s seconds.
 *
 * Cheap enough to call often: it writes one halfword to retained RAM and does
 * nothing else. The point is that the NEXT boot can say how long the previous
 * run lasted, which is the difference between "it reboots" and "it reboots
 * about five minutes in".
 */
void meshtastic_bootlog_heartbeat(uint32_t uptime_s);

/**
 * @brief Copy out the FLASH-backed history, oldest first.
 *
 * Unlike meshtastic_bootlog_history(), a non-zero return here spans power
 * cycles. Returns 0 when the durable ring is disabled or has never been written.
 */
size_t meshtastic_bootlog_durable_history(struct meshtastic_boot_durable *out, size_t max);

/** @brief Log the durable history (the `resets --all` body). */
void meshtastic_bootlog_durable_report(void);

/* Test hooks. The ring arithmetic — append, wrap, oldest-first ordering — is
 * where the bugs live and is pure, so it is tested directly on native_sim
 * rather than through a flash backend. The settings glue around it is the same
 * shape already proven by the courier-arm and cluster-scope records. */
void meshtastic_bootlog_test_durable_reset(void);
void meshtastic_bootlog_test_durable_append(const struct meshtastic_boot_durable *rec);
/** Persist the cached ring now, as the boot-time record does. Lets a test prove
 *  the round trip THROUGH flash — append, wipe RAM, settings_load(), read back —
 *  which is the only property this whole feature exists for. */
int meshtastic_bootlog_test_durable_save(void);

#endif /* ZEPHYR_MESHTASTIC_BOOTLOG_H_ */
