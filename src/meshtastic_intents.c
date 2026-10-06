/*
 * Role intents: the registry walk. The intents themselves live with their modules
 * (MESHTASTIC_INTENT_DEFINE in each); see include/zephyr/meshtastic/intents.h for why.
 */
#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/meshtastic/intents.h>

LOG_MODULE_REGISTER(mt_intents, CONFIG_MESHTASTIC_LOG_LEVEL);

size_t meshtastic_intents_set(void)
{
	size_t n = 0U;
	char detail[48];

	STRUCT_SECTION_FOREACH(meshtastic_intent, it) {
		detail[0] = '\0';
		if (it->is_set(detail, sizeof(detail))) {
			n++;
		}
	}
	return n;
}

int meshtastic_intents_clear(void)
{
	int cleared = 0;
	char detail[48];

	STRUCT_SECTION_FOREACH(meshtastic_intent, it) {
		detail[0] = '\0';
		if (!it->is_set(detail, sizeof(detail))) {
			continue;
		}
		int ret = it->clear();

		if (ret != 0) {
			LOG_ERR("intents: %s: clear failed (%d)", it->name, ret);
			return ret;
		}
		LOG_INF("intents: %s forgotten (%s)", it->name, detail);
		cleared++;
	}
	return cleared;
}

void meshtastic_intents_each(void (*fn)(const struct meshtastic_intent *it, bool set,
					 const char *detail, void *arg),
			     void *arg)
{
	char detail[48];

	STRUCT_SECTION_FOREACH(meshtastic_intent, it) {
		detail[0] = '\0';
		bool set = it->is_set(detail, sizeof(detail));

		fn(it, set, detail, arg);
	}
}
