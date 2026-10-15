/*
 * Copyright (c) 2026 The Zephyr Project Contributors
 * SPDX-License-Identifier: GPL-3.0
 */

#ifndef MESHTASTIC_X25519_REF10_H_
#define MESHTASTIC_X25519_REF10_H_

#include <stdint.h>

/**
 * X25519(scalar, u) as RFC 7748 §5 defines it: the scalar is clamped here, the top bit of
 * u is ignored, and a non-canonical u is reduced. No checks: the caller rejects an
 * all-zero result (a small-order peer key).
 */
void meshtastic_x25519_ref10(uint8_t out[32], const uint8_t scalar[32], const uint8_t u[32]);

#endif /* MESHTASTIC_X25519_REF10_H_ */
