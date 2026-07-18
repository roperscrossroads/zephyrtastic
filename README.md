# Zephyrtastic

A Zephyr RTOS port of the [Meshtastic](https://meshtastic.org) mesh networking stack, with a focus on getting Heltec v3 and Heltec v4 boards working.

> **Disclaimer:** This project is not affiliated with or endorsed by
> Meshtastic LLC. Meshtastic® is a registered trademark of Meshtastic LLC.

## Goals

- Bring up Heltec WiFi LoRa 32 boards (ESP32-S3 + Semtech SX1262) as first-class targets.
- Experiment with different ways to configure individual nodes and groups of nodes.
- Experiment with different ways to perform firmware updates.

Initial targets are the **Heltec V3** and **Heltec V4 rev 4.2**. Other Heltec V4  variants share the same architecture and *should* work, but are currently untested. See [`docs/heltec-boards.md`](docs/heltec-boards.md) for per-board specs (MCU, PSRAM, flash layout, FEM, and pin mapping).

This is early-stage, actively-developed work. Expect gaps and breaking changes.

It is also, candidly, a learning project — a way to get hands-on with Zephyr RTOS internals and the Meshtastic protocol stack. It is not affiliated with the Meshtastic project and comes with no guarantees of correctness, completeness, or regulatory compliance. Caveat emptor: understand your local RF regulations and verify behavior before relying on it.

## Building

Provision a standalone workspace using this repository as the west manifest:

```console
west init -m https://github.com/roperscrossroads/zephyrtastic --mr main <workspace>
cd <workspace>
west update
```

Build the sample for a supported board:

```console
west build -b <board> samples/meshtastic
```

See [`samples/meshtastic/README.rst`](samples/meshtastic/README.rst) for
build options and optional features (BLE, GNSS, shell, MQTT, telemetry).

For the detailed feature matrix and shell command reference, see
[`README.rst`](README.rst).

## Credit

Zephyrtastic builds directly on the excellent
[meshtastic-zephyr](https://github.com/kartben/meshtastic-zephyr) project by
**Benjamin Cabé** ([@kartben](https://github.com/kartben)), which established
a clean, Zephyr-native Meshtastic stack from scratch. That work is the
foundation for everything here, and I'm grateful that he shared it.

I am learning about Zephyr OS and the inner workings of Meshtastic, and I expect this to heavily diverge. Zephyrtastic starts from upstream commit
[`1365e7c`](https://github.com/kartben/meshtastic-zephyr/commit/1365e7c033a67e4545b62285645ff87b97905cec).
All original attribution and the GPL-3.0 license are preserved.

## License

GPL-3.0, matching both the upstream Meshtastic firmware and the original
meshtastic-zephyr project. See [LICENSE](LICENSE).
