/* SPDX-FileCopyrightText: Benjamin Cabé <kartben@gmail.com>
 * SPDX-License-Identifier: GPL-3.0
 */

#ifndef ZEPHYR_SUBSYS_MESHTASTIC_PHONEAPI_H_
#define ZEPHYR_SUBSYS_MESHTASTIC_PHONEAPI_H_

#include "meshtastic/channel.pb.h"
#include "meshtastic_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum protobuf message bytes used by the Meshtastic phone API. */
#define MESHTASTIC_API_FRAME_MAX 512U

struct meshtastic_phoneapi;

struct meshtastic_phoneapi_frame {
	uint8_t data[MESHTASTIC_API_FRAME_MAX];
	uint16_t len;
	bool protected; /* survives selective eviction (see meshtastic sched phone.evict) */
};

typedef void (*meshtastic_phoneapi_data_ready_cb_t)(struct meshtastic_phoneapi *api);
typedef void (*meshtastic_phoneapi_disconnect_cb_t)(struct meshtastic_phoneapi *api);
typedef void (*meshtastic_phoneapi_invalidate_cb_t)(struct meshtastic_phoneapi *api);

enum meshtastic_phoneapi_config_state {
	MESHTASTIC_PHONEAPI_CONFIG_IDLE,
	MESHTASTIC_PHONEAPI_CONFIG_MY_INFO,
	MESHTASTIC_PHONEAPI_CONFIG_DEVICE_UI,
	MESHTASTIC_PHONEAPI_CONFIG_NODE_INFO,
	MESHTASTIC_PHONEAPI_CONFIG_METADATA,
	MESHTASTIC_PHONEAPI_CONFIG_REGION_PRESETS,
	MESHTASTIC_PHONEAPI_CONFIG_CHANNELS,
	MESHTASTIC_PHONEAPI_CONFIG_CONFIGS,
	MESHTASTIC_PHONEAPI_CONFIG_MODULES,
	MESHTASTIC_PHONEAPI_CONFIG_OTHER_NODEINFOS,
	MESHTASTIC_PHONEAPI_CONFIG_FILEMANIFEST,
	MESHTASTIC_PHONEAPI_CONFIG_QUEUE_STATUS,
	MESHTASTIC_PHONEAPI_CONFIG_COMPLETE,
};

struct meshtastic_phoneapi {
	const char *name;
	struct k_mutex lock;
	struct meshtastic_phoneapi_frame *queue;
	struct meshtastic_phoneapi_frame current;
	uint8_t queue_size;
	uint8_t head;
	uint8_t tail;
	uint8_t count;
	bool current_valid;
	uint32_t from_num;
	enum meshtastic_phoneapi_config_state config_state;
	uint8_t config_index;
	uint32_t config_request_id;
	meshtastic_phoneapi_data_ready_cb_t data_ready;
	meshtastic_phoneapi_disconnect_cb_t disconnect;
	meshtastic_phoneapi_invalidate_cb_t invalidate_delivery;
	void *user_data;
	/*
	 * Caller-owned decode/encode scratch (508 B / 768 B — see
	 * meshtastic_phoneapi.c's own comment on meshtastic_phoneapi_handle_toradio()
	 * for why these must not be stack locals). Each transport supplies its own
	 * storage at meshtastic_phoneapi_init() time, exactly like `queue` above —
	 * every transport has exactly one serving thread that ever touches its own
	 * `api` instance, so a per-instance scratch needs no locking. Transports
	 * are expected to place this in PSRAM via MESHTASTIC_EXT_RAM_BSS_ATTR where
	 * available (see meshtastic_ext_ram.h) since it is CPU-only and never
	 * touched by DMA, an ISR, or a flash write.
	 */
	meshtastic_ToRadio *to_scratch;
	meshtastic_FromRadio *from_scratch;
};

#if defined(CONFIG_MESHTASTIC_PHONEAPI)
void meshtastic_phoneapi_init(struct meshtastic_phoneapi *api, const char *name,
			      struct meshtastic_phoneapi_frame *queue, uint8_t queue_size,
			      meshtastic_phoneapi_data_ready_cb_t data_ready,
			      meshtastic_phoneapi_disconnect_cb_t disconnect,
			      meshtastic_phoneapi_invalidate_cb_t invalidate_delivery, void *user_data,
			      meshtastic_ToRadio *to_scratch, meshtastic_FromRadio *from_scratch);
void meshtastic_phoneapi_release_current_frame(struct meshtastic_phoneapi *api);
void meshtastic_phoneapi_register(struct meshtastic_phoneapi *api);
void meshtastic_phoneapi_reset(struct meshtastic_phoneapi *api);
void meshtastic_phoneapi_notify_data_ready(struct meshtastic_phoneapi *api);
uint32_t meshtastic_phoneapi_from_num(struct meshtastic_phoneapi *api);
uint32_t meshtastic_phoneapi_pending_count(struct meshtastic_phoneapi *api);
bool meshtastic_phoneapi_pop_frame(struct meshtastic_phoneapi *api,
				   struct meshtastic_phoneapi_frame *frame);
void meshtastic_phoneapi_push_frame_front(struct meshtastic_phoneapi *api,
					  const struct meshtastic_phoneapi_frame *frame);
bool meshtastic_phoneapi_current_frame(struct meshtastic_phoneapi *api,
				       struct meshtastic_phoneapi_frame *frame);
void meshtastic_phoneapi_current_frame_complete(struct meshtastic_phoneapi *api);
void meshtastic_phoneapi_current_frame_reset(struct meshtastic_phoneapi *api);
int meshtastic_phoneapi_encode_fromradio_frame(const meshtastic_FromRadio *from,
					       struct meshtastic_phoneapi_frame *frame);
int meshtastic_phoneapi_next_config_frame(struct meshtastic_phoneapi *api,
					  struct meshtastic_phoneapi_frame *frame);
int meshtastic_phoneapi_enqueue_fromradio(struct meshtastic_phoneapi *api,
					  const meshtastic_FromRadio *from);
void meshtastic_phoneapi_enqueue_my_info(struct meshtastic_phoneapi *api, uint32_t request_id);
void meshtastic_phoneapi_enqueue_rebooted(struct meshtastic_phoneapi *api);
void meshtastic_phoneapi_enqueue_phone_config(struct meshtastic_phoneapi *api, uint32_t request_id);
int meshtastic_phoneapi_set_channel(uint8_t index, const meshtastic_Channel *channel);
void meshtastic_phoneapi_enqueue_queue_status(struct meshtastic_phoneapi *api, int res,
					      uint32_t mesh_packet_id);
void meshtastic_phoneapi_handle_toradio(struct meshtastic_phoneapi *api, const uint8_t *buf,
					size_t len);
#if defined(CONFIG_MESHTASTIC_PHONELOG)
/**
 * @brief Fan a LogRecord out to every attached transport that has room.
 *
 * Deliberately does not reuse meshtastic_phoneapi_enqueue_fromradio(): that
 * function logs on every path it takes, and logging from inside a log backend is
 * a feedback loop. This one is silent, and drops on a full queue rather than
 * evicting -- diagnostics must not push out the traffic they describe. @p
 * dropped, when non-NULL, is incremented once per transport skipped.
 *
 * @return number of transports the record was queued on (0 if none attached).
 */
int meshtastic_phoneapi_enqueue_log_record(const meshtastic_LogRecord *record, uint32_t *dropped);
#endif
/*
 * @param decoded_mesh Optional fully decoded MeshPacket for this frame. When non-NULL it
 *        is delivered to the phone verbatim (carrying fields the flat struct cannot model,
 *        e.g. Data.emoji / MeshPacket.rx_time); when NULL the packet is rebuilt via
 *        meshtastic_packet_to_mesh_pb (C3 Phase 2).
 */
void meshtastic_phoneapi_on_packet(const struct meshtastic_packet *packet,
				   const meshtastic_MeshPacket *decoded_mesh);
#else
static inline void meshtastic_phoneapi_on_packet(const struct meshtastic_packet *packet,
						 const meshtastic_MeshPacket *decoded_mesh)
{
	ARG_UNUSED(packet);
	ARG_UNUSED(decoded_mesh);
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_MESHTASTIC_PHONEAPI_H_ */
