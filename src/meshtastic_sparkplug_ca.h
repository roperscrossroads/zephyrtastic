/* SPDX-License-Identifier: GPL-3.0
 *
 * Trust anchor for the Sparkplug gateway's TLS connection (MESHTASTIC_SPARKPLUG_TLS): Let's
 * Encrypt's ECDSA root, ISRG Root X2. A public certificate, no secret material. One PEM buffer,
 * NUL-terminated, registered once as a CA credential (more roots may be appended; mbedTLS parses
 * every certificate in it, and refuses the whole buffer if one does not parse).
 *
 * Why X2 alone: the bench broker's certificate is an ECDSA one from Let's Encrypt (chain leaf <-
 * YE1 <- ISRG Root YE <- ISRG Root X2, 2026-11), and every Let's Encrypt ECDSA chain (the YE and
 * E-series intermediates) anchors at X2, so a change of intermediate needs no new image. ISRG Root
 * X1 (RSA 4096) would cover RSA certificates too, but this build's mbedTLS does not parse it
 * (the buffer is then refused and no connection verifies), and it would pull RSA in: add it when
 * a broker with an RSA certificate is wanted, with the RSA PSA options.
 *
 * Regenerate from a host's CA store (Debian/Ubuntu ca-certificates): /etc/ssl/certs/
 * ISRG_Root_X2.pem, one quoted line per PEM line.
 */
#ifndef MESHTASTIC_SPARKPLUG_CA_H_
#define MESHTASTIC_SPARKPLUG_CA_H_

static const unsigned char spb_ca_pem[] =
	/* ISRG Root X2: ECDSA P-384, valid to 2040-09-17; SHA-256 69:72:9B:8E:15:A8:6E:FC:17:7A:57:AF:B7:17:1D:FC:64:AD:D2:8C:2F:CA:8C:F1:50:7E:34:45:3C:CB:14:70 */
	"-----BEGIN CERTIFICATE-----\n"
	"MIICGzCCAaGgAwIBAgIQQdKd0XLq7qeAwSxs6S+HUjAKBggqhkjOPQQDAzBPMQsw\n"
	"CQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJuZXQgU2VjdXJpdHkgUmVzZWFyY2gg\n"
	"R3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBYMjAeFw0yMDA5MDQwMDAwMDBaFw00\n"
	"MDA5MTcxNjAwMDBaME8xCzAJBgNVBAYTAlVTMSkwJwYDVQQKEyBJbnRlcm5ldCBT\n"
	"ZWN1cml0eSBSZXNlYXJjaCBHcm91cDEVMBMGA1UEAxMMSVNSRyBSb290IFgyMHYw\n"
	"EAYHKoZIzj0CAQYFK4EEACIDYgAEzZvVn4CDCuwJSvMWSj5cz3es3mcFDR0HttwW\n"
	"+1qLFNvicWDEukWVEYmO6gbf9yoWHKS5xcUy4APgHoIYOIvXRdgKam7mAHf7AlF9\n"
	"ItgKbppbd9/w+kHsOdx1ymgHDB/qo0IwQDAOBgNVHQ8BAf8EBAMCAQYwDwYDVR0T\n"
	"AQH/BAUwAwEB/zAdBgNVHQ4EFgQUfEKWrt5LSDv6kviejM9ti6lyN5UwCgYIKoZI\n"
	"zj0EAwMDaAAwZQIwe3lORlCEwkSHRhtFcP9Ymd70/aTSVaYgLXTWNLxBo1BfASdW\n"
	"tL4ndQavEi51mI38AjEAi/V3bNTIZargCyzuFJ0nN6T5U6VR5CmD1/iQMVtCnwr1\n"
	"/q4AaOeMSQ+2b1tbFfLn\n"
	"-----END CERTIFICATE-----\n";

#endif /* MESHTASTIC_SPARKPLUG_CA_H_ */
