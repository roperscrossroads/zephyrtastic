/* SPDX-License-Identifier: GPL-3.0 */
/*
 * The host half of the attachment pipe: a Unix socket and the environment,
 * which Zephyr code on native_sim cannot reach itself. Plain C types only: this
 * header is included from both sides of the native_sim boundary.
 */
#ifndef ATTACH_PIPE_BOTTOM_H_
#define ATTACH_PIPE_BOTTOM_H_

/* Connect to the hub's socket at @p path. Returns a descriptor, or -1. */
int attach_pipe_bottom_connect(const char *path);
/* Read what is waiting, without blocking. Returns bytes read, 0 when nothing
 * is waiting, -1 when the hub closed the socket. */
int attach_pipe_bottom_read(int fd, unsigned char *buf, int cap);
/* Write all of @p buf. Returns 0, or -1. */
int attach_pipe_bottom_write(int fd, const unsigned char *buf, int len);
/* The host environment's value for @p name, or NULL. */
const char *attach_pipe_bottom_getenv(const char *name);

#endif /* ATTACH_PIPE_BOTTOM_H_ */
