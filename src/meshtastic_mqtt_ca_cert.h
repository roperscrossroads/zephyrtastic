/* MQTT broker CA trust anchor (public certificate — no secret material).
 *
 * PLACEHOLDER — intentionally empty. This port defaults to plaintext MQTT, so
 * no CA is bundled by default. To use TLS MQTT (CONFIG_MESHTASTIC_MQTT_TLS
 * without CONFIG_MESHTASTIC_MQTT_TLS_NO_VERIFY), paste your broker's CA
 * certificate chain here in PEM form — one quoted, '\n'-terminated line per
 * row — so the broker's leaf certificate validates against it.
 */
#ifndef MESHTASTIC_MQTT_CA_CERT_H
#define MESHTASTIC_MQTT_CA_CERT_H

static const unsigned char mqtt_ca_cert[] = "";

#endif /* MESHTASTIC_MQTT_CA_CERT_H */
