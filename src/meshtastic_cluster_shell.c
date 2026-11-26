/* SPDX-License-Identifier: GPL-3.0 */
/*
 * `meshtastic cluster`: fleet config convergence. Built with the shell and
 * CONFIG_MESHTASTIC_CLUSTER (src/CMakeLists.txt); moved from meshtastic_shell.c
 * (agents-ls39.4.4.2).
 */
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/meshtastic/meshtastic.h>

#include "meshtastic_cluster.h"
#include "meshtastic_cluster_doc.h"
#include "meshtastic_core.h"
#include "meshtastic_shell_internal.h"

/* Config-section names for `cluster promote`/`pin`/`unpin` — the shareable set
 * plus lora (which promote and pin both refuse with the §7.9 explanation, a
 * better teacher than "unknown section"). */
static int cluster_section_parse(const char *name, uint16_t *tag)
{
	static const struct {
		const char *name;
		uint16_t tag;
	} sections[] = {
		{"device", meshtastic_Config_device_tag},
		{"position", meshtastic_Config_position_tag},
		{"power", meshtastic_Config_power_tag},
		{"display", meshtastic_Config_display_tag},
		{"lora", meshtastic_Config_lora_tag},
		{"bluetooth", meshtastic_Config_bluetooth_tag},
	};

	for (size_t i = 0; i < ARRAY_SIZE(sections); i++) {
		if (strcmp(name, sections[i].name) == 0) {
			*tag = sections[i].tag;
			return 0;
		}
	}
	return -EINVAL;
}

static int cmd_cluster_status(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_cluster_stats st;
	struct meshtastic_cluster_entry e;
	const char *sync;
	uint32_t sync_peer = 0U;
	uint8_t ch_index;
	bool have_ch;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_cluster_stats_get(&st);
	have_ch = meshtastic_cluster_channel_resolved(&ch_index);
	sync = meshtastic_cluster_sync_state(&sync_peer);

	if (have_ch) {
		shell_print(sh, "channel : \"%s\" = index %u",
			    CONFIG_MESHTASTIC_CLUSTER_CHANNEL_NAME, ch_index);
	} else {
		shell_print(sh, "channel : \"%s\" NOT PROVISIONED — module idle",
			    CONFIG_MESHTASTIC_CLUSTER_CHANNEL_NAME);
	}
	shell_print(sh, "doc     : %u entr%s, hash %08x",
		    (unsigned int)meshtastic_cluster_entry_count(),
		    meshtastic_cluster_entry_count() == 1U ? "y" : "ies",
		    (unsigned int)meshtastic_cluster_doc_hash_now());
	/*
	 * WHAT THIS NODE CLAIMS TO TRACK — printed next to the document because
	 * the two only mean something together: the hash above is taken over
	 * the claim, not over the table.
	 *
	 * CORE says so loudly. It is not a fault and it costs this node nothing
	 * in what it RUNS (CORE is exactly the closure of effective(me, ·)),
	 * but it does mean the fleet has one fewer place to recover a node's
	 * pins from, and nothing else on the node would ever say so.
	 */
	{
		bool pinned;
		const char *scope = meshtastic_cluster_scope_name(&pinned);
		bool core = (scope[0] == 'C');

		shell_print(sh, "scope   : %s (%s) — %u of %u entries%s", scope,
			    st.scope_demotions ? "narrowed under table pressure"
					       : (pinned ? "pinned" : "auto"),
			    (unsigned int)meshtastic_cluster_entry_count(),
			    (unsigned int)CONFIG_MESHTASTIC_CLUSTER_MAX_ENTRIES,
			    core ? "" : ", tracking the whole fleet");
		if (core) {
			shell_print(sh, "        : base + MY OWN sections only — this node is "
					"NOT a restore source for other nodes (§7.7)");
		}
		if (st.scope_demotions) {
			shell_print(sh, "        : the fleet outgrew this table — raise "
					"MESHTASTIC_CLUSTER_MAX_ENTRIES and reflash to claim "
					"everything again");
		}
		if (st.entry_rx_out_of_scope || st.entries_evicted || st.want_no_space) {
			shell_print(sh, "        : out_of_scope=%u evicted=%u no_space=%u",
				    st.entry_rx_out_of_scope, st.entries_evicted,
				    st.want_no_space);
		}
		if (core) {
			/* The one mechanism by which a narrowed node ever hears
			 * about its OWN entries — worth being able to see. */
			if (CONFIG_MESHTASTIC_CLUSTER_BACKSTOP_PERIODS == 0) {
				shell_print(sh, "backstop: OFF — this node cannot recover "
						"its own pins from the fleet");
			} else {
				shell_print(sh, "backstop: every %u digest periods, walks=%u%s",
					    (unsigned int)
						    CONFIG_MESHTASTIC_CLUSTER_BACKSTOP_PERIODS,
					    st.backstop_walks,
					    st.backstop_walks
						    ? ""
						    : "  (none yet — no peer claiming the "
						      "whole document has been heard)");
			}
		}
	}
	/*
	 * THE DRIFT HORIZON'S PRECONDITION, said out loud (agents-xhli.14).
	 *
	 * The horizon (D12) is measured against the wall clock, so a node whose
	 * clock is unset has NO horizon and accepts a stamp from any year — one
	 * entry dated 2100 would then win every comparison for good. That is the
	 * deliberate choice (a node that cannot say when anything happened has no
	 * basis to refuse, and refusing everything would strand a fresh node
	 * permanently — the same unrecoverable trade the lora ingest gate turned
	 * down). What was NOT deliberate was that nothing said so: the horizon
	 * simply never fired and the status was silent about it.
	 *
	 * So it is a line in status now, for the same reason `sections_held` is:
	 * a deliberate non-enforcement that nobody can see is indistinguishable
	 * from a broken one. Fix it with `admin remote <us> set-time` from a node
	 * that has a clock.
	 */
	{
		uint32_t epoch = meshtastic_clock_now_epoch();

		if (epoch == 0U) {
			shell_print(sh, "horizon : ⚠ NONE — clock unset, so EVERY stamp is "
					"accepted (see CLUSTER-SYNC-M4.md D12)");
		} else {
			shell_print(sh, "horizon : ±%u min around epoch %u (clock source %s)",
				    (unsigned int)(MESHTASTIC_HLC_MAX_DRIFT_MS / 60000U), epoch,
				    mtsh_clock_quality_name(meshtastic_clock_get_quality()));
		}
	}
	/* "queued", not "sent": the counter increments where the digest enters
	 * the send path, and a node with config.lora.tx_enabled = false has its
	 * frames dropped further down, at the radio choke point. A digest is a
	 * broadcast, so no peer bearer carries it either — on such a node this
	 * number counts digests that never left. */
	shell_print(sh, "digests : queued=%u%s rx match=%u MISMATCH=%u incomparable=%u%s",
		    st.digest_tx, mt.tx_enabled ? "" : " (radio TX OFF — none of these aired)",
		    st.digest_rx_match, st.digest_rx_mismatch, st.digest_rx_incomparable,
		    st.digest_rx_incomparable ? "  (a peer declares no document scope — an "
						"older build; it cannot be compared, only "
						"waited out)"
					      : "");
	if (sync_peer != 0U) {
		shell_print(sh, "sync    : %s with 0x%08x", sync, sync_peer);
	} else {
		shell_print(sh, "sync    : %s", sync);
	}
	shell_print(sh, "walk    : pulls=%u timed_out=%u vector tx=%u rx=%u entry tx=%u",
		    st.pull_started, st.pull_timed_out, st.vector_tx, st.vector_rx,
		    st.entry_tx);
	/* fruitless climbing with applied flat is the signature of a mismatch
	 * that cannot be resolved from here — usually a peer holding an entry
	 * authored by a master this node does not trust. held>0 means the
	 * backoff has taken over, which is the bound doing its job, not a
	 * fault. */
	shell_print(sh, "        : fruitless=%u (empty=%u) held=%u%s", st.pull_fruitless,
		    st.pull_empty, st.pull_held,
		    st.pull_held ? "  (backing off — an unresolvable mismatch; check "
				   "`admin trust` on both ends)"
				 : "");
	shell_print(sh, "merge   : applied=%u stale=%u REFUSED=%u unsolicited=%u busy=%u%s",
		    st.entry_rx_applied, st.entry_rx_stale, st.entry_rx_refused,
		    st.rx_unsolicited, st.tx_busy,
		    st.entry_rx_no_space ? "  ** DOCUMENT TABLE FULL — pushed entries are "
					   "being dropped (a walk would narrow the claim "
					   "instead; an unsolicited push deliberately does "
					   "not) **"
					 : "");
	/* suppressed>0 is the rate bound working, not a fault: a push is a
	 * latency optimisation over the digest, so dropping one costs a digest
	 * period and never correctness. */
	shell_print(sh, "push    : sent=%u suppressed=%u%s", st.push_tx, st.push_suppressed,
		    st.push_suppressed ? "  (rate-bounded — the digest still carries every "
					 "change)"
				       : "");
	shell_print(sh, "config  : sections applied=%u kept_local=%u%s", st.sections_applied,
		    st.sections_kept_local,
		    st.sections_held ? "  ** a fleet LoRa section is REPLICATED BUT HELD "
				       "(applying it would re-key this radio; see §7.9) **"
				     : "");
	shell_print(sh, "rx      : wrong_channel=%u undecodable=%u not_implemented=%u%s",
		    st.rx_wrong_channel, st.rx_undecodable, st.rx_not_implemented,
		    st.entry_rx_future ? "  ** entries refused for being stamped beyond the "
					 "clock drift horizon — someone's clock is wrong **"
				       : "");
	/* Entry fragmentation (agents-ooma.36). Printed even when idle, and that
	 * is the point: an all-zero row says "nothing here has ever needed more
	 * than one frame", which is a different and much more useful statement
	 * than silence. The verify-path counters had to be added later for
	 * exactly this reason — a feature whose only evidence is a DBG line the
	 * bench images compile out cannot be observed working at all. */
	shell_print(sh, "frag    : held=%u assembled=%u out_of_order=%u displaced=%u "
			"timed_out=%u%s",
		    st.frag_rx, st.frag_assembled, st.frag_out_of_order, st.frag_displaced,
		    st.frag_timed_out,
		    st.frag_out_of_order ? "  (a fragment arrived at an offset the slot was "
					   "not expecting — reordering, or a peer probing)"
					 : "");
	/* Author signatures (agents-ooma.31). "bad" is the one that should never
	 * move: an honest node does not send a signature that fails. */
	shell_print(sh, "sig     : signed=%u sign_failed=%u verified=%u unsigned=%u "
			"REFUSED bad=%u no_key=%u malformed=%u unsigned=%u%s",
		    st.sig_signed, st.sig_sign_failed, st.sig_verified, st.sig_unsigned,
		    st.sig_refused_bad, st.sig_refused_no_key, st.sig_refused_malformed,
		    st.sig_refused_unsigned,
		    st.sig_refused_bad ? "  ** an entry arrived with a signature that does not "
					 "verify — a forgery, or a bug **"
				       : "");

	/* The per-node rows for THIS node are the ones an operator is reasoning
	 * about during a pin/unpin, and they are visually identical to every
	 * other node's. Say which are ours: a live one is what effective() is
	 * currently returning for that section, a tombstoned one is why it is
	 * NOT (§2.2). */
	for (uint16_t i = 0U; meshtastic_cluster_entry_get(i, &e); i++) {
		bool mine = (e.key.layer == MESHTASTIC_CLUSTER_LAYER_NODE) &&
			    (e.key.node_id == meshtastic_get_node_id());

		/* "signed" means the entry carries its author's signature, which
		 * this node verified on the way in (or made, if it is the author). */
		shell_print(sh, "  [%c/%08x/%u] %s%u B, %s, stamp %lld.%u by 0x%08x%s",
			    e.key.layer == MESHTASTIC_CLUSTER_LAYER_BASE ? 'b' : 'n',
			    e.key.node_id, (unsigned int)e.key.section,
			    e.tombstone ? "TOMBSTONE, " : "", (unsigned int)e.payload_len,
			    e.has_sig ? "signed" : "UNSIGNED",
			    (long long)e.stamp.physical_ms, e.stamp.counter, e.stamp.node_id,
			    mine ? (e.tombstone ? "  <- MY UNPIN (base applies)"
						: "  <- MY PIN (wins over base)")
				 : "");
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_cluster_status)

static int cmd_cluster_promote(const struct shell *sh, size_t argc, char **argv)
{
	uint16_t tag;
	int ret;

	if (mtsh_store_locked(sh)) {
		return -EACCES;
	}
	if (argc != 2U || cluster_section_parse(argv[1], &tag) != 0) {
		shell_error(sh, "usage: meshtastic cluster promote "
			    "<device|position|power|display|bluetooth>");
		return -EINVAL;
	}

	ret = meshtastic_cluster_promote(tag);
	if (ret == -EPERM) {
		shell_error(sh, "refused: section not shareable (secret boundary), or lora "
			    "(a missed fleet preset change orphans nodes — see "
			    "CONFIG-CONVERGENCE.md §7.9)");
		return ret;
	}
	if (ret == -EALREADY) {
		shell_error(sh, "%s already IS the fleet base — this node is running the "
			    "document's own copy, so there is nothing new to promote "
			    "(edit the section first)", argv[1]);
		return ret;
	}
	if (ret < 0) {
		shell_error(sh, "promote failed (%d)", ret);
		return ret;
	}
	shell_print(sh, "%s promoted to fleet base (broadcast once; the digest backs it up)",
		    argv[1]);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_cluster_promote)

/*
 * meshtastic cluster pin <sec> — this node's own override for one section.
 *
 * The pin does not replace or hide the fleet base: base/<sec> keeps replicating
 * underneath with its own stamp and the pin wins only when effective(me) is
 * computed. That separation is the whole reason the document has two layers
 * (CONFIG-CONVERGENCE.md §7.7) — it is what makes `unpin` land on the base as
 * it stands AT THAT MOMENT rather than on the one frozen when the pin was made.
 */
static int cmd_cluster_pin(const struct shell *sh, size_t argc, char **argv)
{
	uint16_t tag;
	int ret;

	if (argc != 2U || cluster_section_parse(argv[1], &tag) != 0) {
		shell_error(sh, "usage: meshtastic cluster pin "
			    "<device|position|power|display|bluetooth>");
		return -EINVAL;
	}

	ret = meshtastic_cluster_pin(tag);
	if (ret == -EPERM) {
		shell_error(sh, "refused: section not shareable (secret boundary), lora (a "
			    "doc-borne lora section is replicated but never applied, so the pin "
			    "could not take effect), or this node is managed");
		return ret;
	}
	if (ret == -EALREADY) {
		shell_error(sh, "%s is already this node's pin — the stored value came FROM "
			    "that pin, so re-pinning would only mint a second stamp for the "
			    "same bytes (edit the section first)", argv[1]);
		return ret;
	}
	if (ret == -ENOENT) {
		shell_error(sh, "no %s section in the config store", argv[1]);
		return ret;
	}
	if (ret < 0) {
		shell_error(sh, "pin failed (%d)", ret);
		return ret;
	}
	shell_print(sh, "%s pinned for this node — the fleet base keeps replicating "
		    "underneath and `unpin` returns to whatever it holds then", argv[1]);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_cluster_pin)

/* meshtastic cluster unpin <sec> — drop the override. A TOMBSTONE, not a
 * deletion: removal has to replicate, or the next anti-entropy pass pulls the
 * pin back from a peer that still holds it (D7). */
static int cmd_cluster_unpin(const struct shell *sh, size_t argc, char **argv)
{
	uint16_t tag;
	int ret;

	if (argc != 2U || cluster_section_parse(argv[1], &tag) != 0) {
		shell_error(sh, "usage: meshtastic cluster unpin "
			    "<device|position|power|display|bluetooth>");
		return -EINVAL;
	}

	ret = meshtastic_cluster_unpin(tag);
	if (ret == -ENOENT) {
		shell_error(sh, "this node has no pin on %s", argv[1]);
		return ret;
	}
	if (ret == -EALREADY) {
		shell_error(sh, "%s is already unpinned (the tombstone is replicating)",
			    argv[1]);
		return ret;
	}
	if (ret == -EPERM) {
		shell_error(sh, "refused: this node is managed (SecurityConfig.is_managed)");
		return ret;
	}
	if (ret < 0) {
		shell_error(sh, "unpin failed (%d)", ret);
		return ret;
	}
	shell_print(sh, "%s unpinned — this node now follows the fleet base as it stands "
		    "now (with no fleet base for the section, it keeps what it has)", argv[1]);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_cluster_unpin)

static int cmd_cluster_digest(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	meshtastic_cluster_digest_now();
	shell_print(sh, "digest queued");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_cluster_digest)

/* meshtastic cluster pull <node> — run the anti-entropy walk against one peer
 * now instead of waiting for its next digest. The convergence proof, on demand:
 * the walk is identical either way, only the trigger differs. */
static int cmd_cluster_pull(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t node_num;
	int ret;

	if (argc != 2U) {
		shell_error(sh, "usage: meshtastic cluster pull <node>");
		return -EINVAL;
	}
	ret = mtsh_parse_u32(sh, argv[1], &node_num);
	if (ret < 0) {
		return ret;
	}

	ret = meshtastic_cluster_pull(node_num);
	if (ret == -EBUSY) {
		shell_error(sh, "an exchange is already in flight (one at a time)");
		return ret;
	}
	if (ret == -ENOTCONN) {
		shell_error(sh, "no channel named \"%s\" — nothing to pull over",
			    CONFIG_MESHTASTIC_CLUSTER_CHANNEL_NAME);
		return ret;
	}
	if (ret < 0) {
		shell_error(sh, "pull failed (%d)", ret);
		return ret;
	}
	shell_print(sh, "pulling from 0x%08x — `cluster status` shows the walk", node_num);
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_cluster_pull)

static int cmd_cluster_scope(const struct shell *sh, size_t argc, char **argv)
{
	int choice;
	int ret;

	if (argc != 2) {
		shell_error(sh, "usage: meshtastic cluster scope <full|core|auto>");
		return -EINVAL;
	}
	if (strcmp(argv[1], "auto") == 0) {
		choice = MESHTASTIC_CLUSTER_SCOPE_CHOICE_AUTO;
	} else if (strcmp(argv[1], "full") == 0) {
		choice = MESHTASTIC_CLUSTER_SCOPE_CHOICE_FULL;
	} else if (strcmp(argv[1], "core") == 0) {
		choice = MESHTASTIC_CLUSTER_SCOPE_CHOICE_CORE;
	} else {
		shell_error(sh, "unknown scope \"%s\" — full, core or auto", argv[1]);
		return -EINVAL;
	}

	ret = meshtastic_cluster_scope_select(choice);
	if (ret == -EALREADY) {
		shell_print(sh, "already claiming that");
		return 0;
	}
	if (ret != 0) {
		shell_error(sh, "scope change failed (%d)", ret);
		return ret;
	}

	if (choice == MESHTASTIC_CLUSTER_SCOPE_CHOICE_CORE) {
		shell_print(sh, "now claiming CORE — base + this node's own sections. "
				"Everything else has been dropped, from flash too. This node "
				"is no longer a restore source for other nodes; what it RUNS "
				"is unchanged.");
	} else {
		shell_print(sh, "now claiming FULL%s. The fleet's rows come back on the "
				"next walk.",
			    choice == MESHTASTIC_CLUSTER_SCOPE_CHOICE_AUTO
				    ? " (auto — free to narrow again under table pressure)"
				    : " (pinned — it will NOT narrow, so a full table stops "
				      "tracking part of the fleet)");
	}
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_cluster_scope)

static int cmd_cluster_reset(const struct shell *sh, size_t argc, char **argv)
{
	int dropped;

	/* Confirmation required. Every other destructive verb here writes a
	 * versioned entry that the fleet can argue with; this one throws away
	 * the whole local copy, and a fat-fingered `cluster reset` next to
	 * `cluster status` should not be able to do that silently. */
	if (mtsh_store_locked(sh)) {
		return -EACCES;
	}
	if (argc != 2 || strcmp(argv[1], "--confirm") != 0) {
		shell_error(sh, "usage: meshtastic cluster reset --confirm");
		shell_print(sh, "drops the ENTIRE document — every entry, the persisted "
				"records, and any recorded scope.");
		return -EINVAL;
	}

	dropped = meshtastic_cluster_reset();
	if (dropped < 0) {
		shell_error(sh, "reset failed (%d)", dropped);
		return dropped;
	}
	shell_print(sh, "document cleared — %d entr%s dropped.", dropped,
		    dropped == 1 ? "y" : "ies");
	shell_print(sh, "This node told NOBODY to forget anything. The next digest from a "
			"peer that still holds the document brings it all back.");
	shell_print(sh, "To clear a FLEET, isolate every member on BOTH bearers first — "
			"`lora tx off` AND `blepeer scan off` + `blepeer disconnect` on the "
			"central side of each peer link. A cluster pull is a unicast, so it "
			"diverts to a live BLE link and a LoRa-mute node still converges: "
			"that is the peer-transport property working, not a fault.");
	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_cluster_reset)

/* `cluster fleet` — who agrees with me, and can I safely make a fleet-wide move.
 *
 * The pre-flight half of the straggler sweep (CONFIG-CONVERGENCE.md §7.9). A
 * fleet preset change orphans any member that misses it — new frequency, new
 * channel hashes, mutually deaf — so the question worth answering BEFORE the
 * move is "does every member already hold what I am about to make current".
 *
 * The verdict deliberately reports what it cannot see as loudly as what it can.
 * A node that is already orphaned has no digest on record and so cannot appear
 * as a disagreement; it is simply absent, and an absence must not read as
 * assent. */
static int cmd_cluster_fleet(const struct shell *sh, size_t argc, char **argv)
{
	struct meshtastic_cluster_fleet f;
	uint16_t n = meshtastic_cluster_peer_count();
	uint32_t now = (uint32_t)k_uptime_seconds();
	bool converged;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	converged = meshtastic_cluster_fleet_converged(&f);

	shell_print(sh, "my doc  : %08x / %u entries",
		    (unsigned int)meshtastic_cluster_doc_hash_now(),
		    meshtastic_cluster_entry_count());
	shell_print(sh, "window  : %us (3x the digest period)", f.window_sec);

	if (n == 0U) {
		shell_print(sh, "peers   : none heard");
	}
	for (uint16_t i = 0; i < n; i++) {
		struct meshtastic_cluster_peer p;
		uint32_t age;
		const char *verdict;

		if (!meshtastic_cluster_peer_get(i, &p)) {
			continue;
		}
		age = (now > p.heard_uptime_sec) ? (now - p.heard_uptime_sec) : 0U;

		if (age > f.window_sec) {
			verdict = "STALE";
		} else if (!p.comparable) {
			verdict = "not comparable";
		} else if (p.agreed) {
			verdict = "agrees";
		} else {
			verdict = "DIVERGED";
		}

		shell_print(sh, "  0x%08x %-14s doc=%08x/%-3u %us ago", p.node_id, verdict,
			    (unsigned int)p.doc_hash, p.entry_count, age);
	}

	shell_print(sh, "totals  : %u known — %u agree, %u diverged, %u stale, %u incomparable",
		    f.known, f.agreed, f.diverged, f.stale, f.incomparable);

	if (converged) {
		shell_print(sh, "verdict : every peer I KNOW OF agrees");
	} else if (f.known == 0U) {
		shell_print(sh, "verdict : NO — nothing heard, which is not the same as "
				"agreement");
	} else {
		shell_print(sh, "verdict : NO — do not make a fleet-wide preset change");
	}
	shell_warn(sh, "a node already orphaned, asleep or out of range has no digest here "
		       "and cannot fail this check — it is absent, not agreeing");

	return 0;
}
MTSH_DEFINE_ON_WORK(cmd_cluster_fleet)

SHELL_STATIC_SUBCMD_SET_CREATE(meshtastic_cluster_cmds,
			       SHELL_CMD(fleet, NULL,
					 SHELL_HELP("Who agrees with my document — the "
						    "pre-flight for a fleet-wide change.",
						    NULL),
					 cmd_cluster_fleet_on_work),
			       SHELL_CMD(status, NULL,
					 SHELL_HELP("Cluster doc, digest stats, channel binding.",
						    NULL),
					 cmd_cluster_status_on_work),
			       SHELL_CMD(promote, NULL,
					 SHELL_HELP("Promote my current section to fleet base.",
						    "<device|position|power|display|bluetooth>"),
					 cmd_cluster_promote_on_work),
			       SHELL_CMD(pin, NULL,
					 SHELL_HELP("Override one section for THIS node "
						    "(base keeps replicating underneath).",
						    "<device|position|power|display|bluetooth>"),
					 cmd_cluster_pin_on_work),
			       SHELL_CMD(unpin, NULL,
					 SHELL_HELP("Drop this node's override (a replicating "
						    "tombstone); follow the current base.",
						    "<device|position|power|display|bluetooth>"),
					 cmd_cluster_unpin_on_work),
			       SHELL_CMD(reset, NULL,
					 SHELL_HELP("Forget the whole document (local only — a "
						    "peer that still holds it will re-seed).",
						    "--confirm"),
					 cmd_cluster_reset_on_work),
			       SHELL_CMD(scope, NULL,
					 SHELL_HELP("What this node claims to track. core = "
						    "base + my own sections (constrained tier).",
						    "<full|core|auto>"),
					 cmd_cluster_scope_on_work),
			       SHELL_CMD(digest, NULL,
					 SHELL_HELP("Broadcast one digest now.", NULL),
					 cmd_cluster_digest_on_work),
			       SHELL_CMD(pull, NULL,
					 SHELL_HELP("Run the anti-entropy walk against a peer "
						    "now.", "<node>"),
					 cmd_cluster_pull_on_work),
			       SHELL_SUBCMD_SET_END);

SHELL_SUBCMD_ADD((meshtastic), cluster, &meshtastic_cluster_cmds,
		 SHELL_HELP("Fleet config convergence.", NULL),
		 cmd_cluster_status_on_work, 0, 0);
