Zephyr-native Meshtastic stack
##############################

This repository packages a Zephyr native Meshtastic® stack that should work out-of-the-box on any
Zephyr-supported board + LoRa transceiver.

**DISCLAIMER**: this project is not affiliated with or endorsed by Meshtastic LLC. Meshtastic® is a
registered trademark of Meshtastic LLC.

Why Zephyr?
***********

The `original Meshtastic firmware <https://github.com/meshtastic/firmware>`_ is a PlatformIO/Arduino
application with a (very!) large amount of device-specific integration code (including drivers for
sensors, displays, etc.) and lots of places in the code where conditional compilation is used to
adapt to different hardware platforms.

Zephyr RTOS _is_ designed to abstract away the hardware details and already provides ready-to-use
subsystems for many of the features that Meshtastic needs (LoRa, Bluetooth, GNSS, ...).

Feature support
***************

The initial goal is to provide a minimal set of API for setting up and configuring the stack, as
wel as sending and receiving messages.

A typical "original" Meshtastic firmware would typically expose a UI but this initial part doesn't
have any at this point and instead focus has been put on providing shell commands that can be
executed directly from the device's console (e.g. ``meshtastic status``, ``meshtastic nodedb list``,
...), and on providing a PhoneAPI interface over Bluetooth LE and serial so that a host application
(see https://meshtastic.org/downloads/) can provide a nice frontend to the device.

The original Meshtastic firmware is inherently very re-configurable (e.g. LoRa radio settings, node
identification, ...) without having to recompile it (i.e. change your LoRa presets, reboot device,
done!). This project, however, is currently mainly proposing *build-time* configuration, (using
Kconfig) so trying to reconfigure it using your favorite app's config UI will simply not work (and
likely fail miserably). Support for settings (persisted in flash) has started to be introduced
though, and configuring channels (tested through shell commands only at the time of writing) works
as expected and channels survive a reboot.

As this project is mostly a rewrite from scratch, and as there aren't really (to the best of my
knowledge) any interoperability test suites available, the support matrix below reflects what has
been implemented and tested against the reference
`Meshtastic firmware <https://github.com/meshtastic/firmware>`_ and the official
`module documentation <https://meshtastic.org/docs/configuration/module/>`_.

Legend: ✅ Full · 🟡 Partial · ❌ Not yet

.. list-table::
   :header-rows: 1
   :widths: 30 12 58

   * - Feature
     - Status
     - Notes
   * - **Core mesh**
     -
     -
   * - Packet routing / flood relay
     - ✅
     - Wire-compatible encode/decode, hop-limit rebroadcast
   * - Channel encryption
     - ✅
     - AES-CTR, per-channel PSK (PSA crypto backend)
   * - Duplicate suppression
     - ✅
     - Configurable packet dedup cache
   * - **Phone interface**
     -
     -
   * - phoneAPI (ToRadio / FromRadio)
     - ✅
     - ToRadio / FromRadio protobuf protocol
   * - Bluetooth LE transport
     - ✅
     - GATT service, compatible with the Meshtastic apps (tested on Android, macOS, and web client)
   * - Serial / UART transport
     - ✅
     - StreamAPI framing over a chosen UART (test on Android, macOS, and web client)
   * - Settings edit over phoneAPI
     - 🟡
     - Local admin write implemented (config / module / channel / owner, edit
       transactions, reboot, favorite / ignore / remove-node, fixed position) —
       pending hardware test; no session-passkey validation or remote admin yet
   * - **Messaging & telemetry**
     -
     -
   * - Text messages
     - ✅
     - Send / receive UTF-8 plaintext
   * - Reliable delivery
     - ✅
     - Retransmits unacked want_ack DMs (explicit / implicit ACK, NAK, or
       give-up with a delivery-failure report to the app); tunable via
       ``sched reliable.*``
   * - Position / GNSS
     - ✅
     - Integration with Zephyr GNSS subsystem, periodic position broadcast; the
       Position module also runs GNSS-less to advertise an admin-set fixed
       position, which persists across reboot
   * - NodeInfo
     - ✅
     - Periodic identity broadcast + node discovery
   * - NodeDB
     - 🟡
     - In-RAM node cache; peer public keys and the curated set (favorite /
       ignored nodes, with their identity) persist to flash. Other node data is
       re-learned from the mesh after a reboot
   * - Device telemetry
     - ✅
     - Uptime, optional battery / fuel-gauge metrics, airtime/channel utilization metrics
   * - Environment telemetry
     - ✅
     - Temperature, humidity, pressure, gas, light sensors.
       Automatically picks up default sensors (e.g. ``ambient-temp0`` from Devicetree).
   * - MQTT
     - 🟡
     - Gateway uplink/downlink bridge ; no per-channel configuration for now
   * - TraceRoute
     - 🟡
     - Responds to path-mapping requests (RouteDiscovery); no relay-path
       accumulation or local trace initiator
   * - **Configuration**
     -
     -
   * - Local config persistence
     - ✅
     - Zephyr settings subsystem backed by NVS flash — device / module config,
       owner, and channels all persist across reboot

Shell commands
**************

When ``CONFIG_MESHTASTIC_SHELL`` is enabled, the firmware registers a ``meshtastic`` command tree on
the Zephyr . Commands marked *(optional)* are only present when their corresponding
``CONFIG_MESHTASTIC_*`` Kconfig option is set.

.. list-table::
   :header-rows: 1
   :widths: 42 58

   * - Command
     - Description
   * - ``meshtastic status``
     - Show node ID, init/BLE state, TX/RX stats, channel, role
   * - ``meshtastic channel list``
     - List the 8 channel slots
   * - ``meshtastic channel show <index>``
     - Show one channel slot
   * - ``meshtastic channel set <index> [name|role|psk|uplink|downlink]...``
     - Update fields of a channel slot *(config write)*
   * - ``meshtastic channel disable <index>``
     - Disable a channel slot *(config write)*
   * - ``meshtastic device role [name]``
     - Get the device role; setting one is a *(config write)*
   * - ``meshtastic device rebroadcast [mode]``
     - Get the rebroadcast mode; setting one is a *(config write)*
   * - ``meshtastic text send [-c <index>] [dest|broadcast] <message>``
     - Send a text message *(optional)*
   * - ``meshtastic nodedb list``
     - List known nodes *(optional)*
   * - ``meshtastic nodedb show <node|0xnode>``
     - Show one NodeDB entry *(optional)*
   * - ``meshtastic gnss send [dest|broadcast]``
     - Send the current GNSS position *(optional)*
   * - ``meshtastic metrics send [dest|broadcast]``
     - Send device telemetry *(optional)*
   * - ``meshtastic environment send [dest|broadcast]``
     - Send environment telemetry *(optional)*
   * - ``meshtastic nodeinfo send [dest|broadcast]``
     - Send node information *(optional)*
   * - ``meshtastic sched [show]``
     - Show the scheduler / QoS policy and knobs
   * - ``meshtastic sched policy <name>``
     - Apply a policy preset (``default``, ``no-backoff``)
   * - ``meshtastic sched set <key> <value>``
     - Tune one knob live: ``tx.order``, ``tx.overflow``, ``tx.depth``, ``phone.evict``, ``airtime.max`` (% channel util that gates background self-broadcasts, 0=off), ``dedup.ttl`` (seconds a duplicate is remembered, 0=never), ``reliable.retries`` (retransmits of an unacked DM, 0=off), ``reliable.timeout`` (ms between retransmits), ``route.ttl`` (seconds a learned next-hop stays trusted, 0=never), ``cw.min``/``cw.max`` (contention-window exponents, ``cw.max 0`` disables), ``cw.offset`` (client relay offset in ``cw.max`` slots), ``cw.slot`` (slot-time override in ms, 0=derive from preset)
   * - ``meshtastic sched stats [reset]``
     - Show live TX/RX counters, per-tier egress, phone-queue drops, airtime-gated broadcasts, dedup TTL expiries and reliable-delivery acked/failed; ``reset`` to zero them

Commands marked *(config write)* persist changes to the device configuration.
The shell is **not** an authenticated transport — anyone with console access
(UART, USB or RTT) reaches it with no pairing, passkey or key check of any
kind — so these are gated two ways:

* ``CONFIG_MESHTASTIC_SHELL_CONFIG_WRITE`` (default ``y``) compiles them in.
  Set it to ``n`` for a production build and only the read-only forms remain,
  so ``meshtastic device role`` still reports the current role.
* At runtime they honour ``SecurityConfig.is_managed``. A managed node refuses
  them exactly as it refuses local admin over the PhoneAPI.

``meshtastic channel show`` prints a PSK summary (kind and length) but never
the key itself. Raw hex requires ``CONFIG_MESHTASTIC_SHELL_PSK_HEX`` (default
``n``), which discloses live key material to anyone who can read the console.

Defaults: behave like a stock node
==================================

**Every runtime knob defaults to the value that makes this node behave like a
stock meshtastic/firmware node.** The mesh is shared infrastructure — other
people's nodes relay our traffic and we relay theirs — so the default posture is
to be an ordinary, well-behaved participant, not to be locally optimal at the
shared channel's expense. Anything that trades airtime fairness for our own
latency or delivery rate is opt-in, never the default.

The contention window is the clearest example. ``cw.min``/``cw.max``/``cw.offset``
default to the reference firmware's CWmin 3, CWmax 8 and 2×CWmax client offset,
and ``cw.slot`` derives the slot time from the active modem preset exactly as
upstream does. A node nobody has touched waits before relaying, the same way its
neighbours do.

``cw.max 0`` switches that off and transmits as soon as the channel is clear.
The ``no-backoff`` policy preset does exactly this, along with disabling the
airtime gate and dedup TTL — it removes every voluntary wait the node performs,
restoring how this port behaved before those mechanisms existed. It is retained
because it is the control arm for measuring them (see ``meshtastic sched stats``,
which counts relays a peer also relayed), **not because it is a reasonable way to
run a node on a shared mesh.** A node running ``no-backoff`` transmits ahead of
every neighbour that is backing off politely, and takes more than its share of a
channel it does not own. Use it to measure, then put it back.

It is named for what it does rather than when it was written: "legacy" read like
"the safe old default" to anyone who had not read this section, which is exactly
backwards.

The scheduler policy is a runtime-tunable QoS surface (RAM-only — a reboot
restores compiled defaults). Changing a knob or applying a preset resets the
live stats, so each policy is measured from a clean slate. See
``docs/ARCH-REVIEW.md``.

Setup
*****

This repository can be used to directly provision a full-blown workspace from scratch, or it can be
pulled into an existing workspace as a module.

As a standalone workspace
-------------------------

Before getting started, make sure you have followed the initial steps of the Zephyr Getting Started
Guide up to install/update host dependencies that Zephyr requires such as Python, CMake, ...

At the "Get the Zephyr source code" step, replace the default ``west init <workspace_location>`` step
with the following command so that you use this repository as the main manifest for your workspace:

.. code-block:: console

   $ west init -m https://github.com/kartben/meshtastic-zephyr --mr main <workspace_location>


As a module in an existing workspace
------------------------------------

Add the module to your west manifest or a Zephyr submanifest:

.. code-block:: yaml

   manifest:
     projects:
       - name: meshtastic-zephyr
         url: https://github.com/kartben/meshtastic-zephyr.git
         revision: main
         path: modules/lib/meshtastic-zephyr

Then update the workspace:

.. code-block:: console

   $ west update

Note: The meshtastic-zephyr module requires Zephyr's ``nanopb`` module to be present in the west
workspace (which should be the case by default).

Sample
******

Build the Meshtastic sample with:

.. code-block:: console

   $ west build -b <board> samples/meshtastic

Refer to the [sample's README](samples/meshtastic/README.rst) for more information on how to build
and run the sample and enable additional features.

License
*******

This project is licensed under GPL-3.0, similar to the original Meshtastic firmware.
