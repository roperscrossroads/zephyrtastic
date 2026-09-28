/* SPDX-License-Identifier: GPL-3.0 */
/* The host half of the attachment pipe (attach_pipe_bottom.h). */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "attach_pipe_bottom.h"

int attach_pipe_bottom_connect(const char *path)
{
	struct sockaddr_un addr;
	int fd;

	if (path == NULL || strlen(path) >= sizeof(addr.sun_path)) {
		return -1;
	}
	fd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (fd < 0) {
		return -1;
	}
	memset(&addr, 0, sizeof(addr));
	addr.sun_family = AF_UNIX;
	strcpy(addr.sun_path, path);
	if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
		close(fd);
		return -1;
	}
	/* Reads poll; the Zephyr side sleeps between them, which is what lets
	 * simulated time advance. A blocking read would stall the whole image. */
	(void)fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
	return fd;
}

int attach_pipe_bottom_read(int fd, unsigned char *buf, int cap)
{
	ssize_t n = read(fd, buf, (size_t)cap);

	if (n > 0) {
		return (int)n;
	}
	if (n == 0) {
		return -1; /* EOF: the hub is gone */
	}
	return (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) ? 0 : -1;
}

int attach_pipe_bottom_write(int fd, const unsigned char *buf, int len)
{
	int off = 0;

	while (off < len) {
		ssize_t n = write(fd, buf + off, (size_t)(len - off));

		if (n > 0) {
			off += (int)n;
		} else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
			struct pollfd pfd = { .fd = fd, .events = POLLOUT };

			(void)poll(&pfd, 1, 1);
		} else {
			return -1;
		}
	}
	return 0;
}

const char *attach_pipe_bottom_getenv(const char *name)
{
	return getenv(name);
}
