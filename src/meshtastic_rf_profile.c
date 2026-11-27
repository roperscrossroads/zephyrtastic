/* SPDX-License-Identifier: GPL-3.0 */
/* Radio profiles: see meshtastic_rf_profile.h and tooling RF-PROFILES.md. */
#include <errno.h>
#include <string.h>

#include "meshtastic_rf_profile.h"

struct rf_profile {
	const char *name;
	bool boosted;  /* sx126x_rx_boosted_gain */
	bool lna;      /* front-end LNA in the receive path (where it can be switched) */
	bool sets_power;
	const char *tradeoff;
};

static const struct rf_profile profiles[] = {
	{"standard", true, true, false,
	 "most nodes: best range; a little extra current for the receive amplifiers"},
	{"battery", false, true, false,
	 "solar or battery nodes: less receive current, a little less range "
	 "(set a lower power yourself; the front end powering down in sleep is not built yet)"},
	{"crowded", true, false, false,
	 "near strong transmitters: LNA off so they do not overload the receiver; "
	 "weak, distant nodes are heard less well"},
	{"bench", false, false, true,
	 "boards inches apart: 2 dBm, LNA and boosted gain off, so close signals do not "
	 "saturate the receiver; no use for range"},
};

const char *const meshtastic_rf_profile_names[] = {"standard", "battery", "crowded", "bench",
						    NULL};

static const struct rf_profile *find(const char *name)
{
	for (size_t i = 0; i < sizeof(profiles) / sizeof(profiles[0]); i++) {
		if (strcmp(profiles[i].name, name) == 0) {
			return &profiles[i];
		}
	}
	return NULL;
}

const char *meshtastic_rf_profile_of(const meshtastic_Config_LoRaConfig *l, bool lna_controllable)
{
	const bool boosted = l->sx126x_rx_boosted_gain;
	const bool lna = l->fem_lna_mode != meshtastic_Config_LoRaConfig_FEM_LNA_Mode_DISABLED;
	const bool bench_power = (l->tx_power > 0) && (l->tx_power <= MESHTASTIC_RF_PROFILE_BENCH_DBM);

	if (!lna_controllable) {
		/* No switchable LNA: Standard and Crowded read the same, as do Battery and Bench
		 * apart from the power. */
		if (boosted) {
			return "standard";
		}
		return bench_power ? "bench" : "battery";
	}
	if (boosted) {
		return lna ? "standard" : "crowded";
	}
	if (lna) {
		return "battery";
	}
	return bench_power ? "bench" : "custom";
}

int meshtastic_rf_profile_set(meshtastic_Config_LoRaConfig *l, const char *name)
{
	const struct rf_profile *p = find(name);

	if (p == NULL) {
		return -EINVAL;
	}
	l->sx126x_rx_boosted_gain = p->boosted;
	/* ENABLED on a board without LNA control is normalised to NOT_PRESENT by the
	 * config store, as for any other write. */
	l->fem_lna_mode = p->lna ? meshtastic_Config_LoRaConfig_FEM_LNA_Mode_ENABLED
				 : meshtastic_Config_LoRaConfig_FEM_LNA_Mode_DISABLED;
	if (p->sets_power) {
		l->tx_power = MESHTASTIC_RF_PROFILE_BENCH_DBM;
	}
	return 0;
}

const char *meshtastic_rf_profile_tradeoff(const char *name)
{
	const struct rf_profile *p = find(name);

	if (p != NULL) {
		return p->tradeoff;
	}
	if (strcmp(name, "adaptive") == 0) {
		return "not built: will be implemented later, after careful consideration and testing";
	}
	return NULL;
}
