/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 * SPDX-License-Identifier: GPL-3.0
 *
 * TF-PSA-Crypto user configuration for CONFIG_MESHTASTIC_PKI_X25519_EVEREST: included after
 * Zephyr's config-tf-psa-crypto.h (TF_PSA_CRYPTO_USER_CONFIG_FILE). With this defined,
 * mbedtls_psa_key_agreement_ecdh() sends X25519 to Project Everest's HACL* Curve25519
 * (drivers/everest) instead of the generic ECP code; nothing in our PSA calls changes.
 */
#define MBEDTLS_ECDH_VARIANT_EVEREST_ENABLED
