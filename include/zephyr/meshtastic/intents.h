/*
 * Role intents: what a module remembers across reboots about the ROLE the node was given,
 * as opposed to what the node is (identity), how its radio is set, which channels it has
 * and who owns it.
 *
 * A BLE peer target the courier dials, the brain a head reports to and the heads a brain
 * wants on which presets, the ear's receiving half, the courier's arm, the relay's
 * direction, a listener's pinned presets, a cluster's document: each is set by a command,
 * persisted by its module, and restored at boot. None of them keys off a declared setting
 * the way the cluster module keys off a channel named "cluster", so a node moved from a
 * rig layout to a plain one kept dialling its old peers (the bench, 2026-10-06).
 *
 * This is the one place they are all listed, so that a tool can ask "what roles does this
 * node still remember?" and say "none" and have it be so. Each module registers its intent
 * with MESHTASTIC_INTENT_DEFINE(); the shell's `meshtastic intents` lists them and
 * `meshtastic intents clear` forgets every one, then reboots, so the modules come up with
 * nothing to restore. Identity, radio, channels, owner and device settings are untouched.
 */
#ifndef ZEPHYR_INCLUDE_MESHTASTIC_INTENTS_H_
#define ZEPHYR_INCLUDE_MESHTASTIC_INTENTS_H_

#include <stdbool.h>
#include <stddef.h>
#include <zephyr/sys/iterable_sections.h>

#ifdef __cplusplus
extern "C" {
#endif

struct meshtastic_intent {
	/** A short name: "peer", "head", "brain", "ear", "courier", "relay", "scan", "cluster". */
	const char *name;
	/** The settings keys it lives under, for the reader. */
	const char *keys;
	/**
	 * Is the intent set now: stored, or active in RAM this boot. Writes a few words about
	 * it into @p detail (may be empty) when it is.
	 */
	bool (*is_set)(char *detail, size_t len);
	/**
	 * Forget it: delete what is stored and, where the module can, stand down in RAM. The
	 * caller reboots afterwards, so standing down need not be complete. Returns 0 or -errno.
	 */
	int (*clear)(void);
};

/**
 * @brief Register a module's role intent.
 */
#define MESHTASTIC_INTENT_DEFINE(_id, _name, _keys, _is_set, _clear)                             \
	static const STRUCT_SECTION_ITERABLE(meshtastic_intent, mt_intent_##_id) = {             \
		.name = _name, .keys = _keys, .is_set = _is_set, .clear = _clear}

/** @brief How many intents are set now. */
size_t meshtastic_intents_set(void);

/**
 * @brief Forget every intent that is set. Returns how many were cleared, or -errno on the
 *        first failure (the ones before it are cleared).
 */
int meshtastic_intents_clear(void);

/** @brief Walk the registry. */
void meshtastic_intents_each(void (*fn)(const struct meshtastic_intent *it, bool set,
					 const char *detail, void *arg),
			     void *arg);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_MESHTASTIC_INTENTS_H_ */
