/* SPDX-License-Identifier: GPL-3.0 */
/*
 * The host half of the link pipe: a Unix socket and the environment,
 * which Zephyr code on native_sim cannot reach itself. Plain C types only: this
 * header is included from both sides of the native_sim boundary.
 */
#ifndef LINK_PIPE_BOTTOM_H_
#define LINK_PIPE_BOTTOM_H_

/* Connect to the switchboard's socket at @p path. Returns a descriptor, or -1. */
int link_pipe_bottom_connect(const char *path);
/* Read what is waiting, without blocking. Returns bytes read, 0 when nothing
 * is waiting, -1 when the switchboard closed the socket. */
int link_pipe_bottom_read(int fd, unsigned char *buf, int cap);
/* Write all of @p buf. Returns 0, or -1. */
int link_pipe_bottom_write(int fd, const unsigned char *buf, int len);
/* The host environment's value for @p name, or NULL. */
const char *link_pipe_bottom_getenv(const char *name);

#endif /* LINK_PIPE_BOTTOM_H_ */
