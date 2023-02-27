/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause) */
/*
 * canloopback.c - Echo CAN frames received
 *
 * Copyright (c) 2002-2007 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Volkswagen nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * The provided data structures and external interfaces from this code
 * are not restricted to be used by modules with a GPL compatible license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * Send feedback to <linux-can@vger.kernel.org>
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <nuttx/can.h>
#include <netpacket/can.h>

#include "lib.h"

static void print_usage_send(char *prg)
{
	fprintf(stderr, "%s - echo CAN-frames received (only tested on CAN2.0 basic frames).\n", prg);
	fprintf(stderr, "\nUsage: %s <device>.\n", prg);
}



int main(int argc, char **argv)
{
	int s; /* can raw socket */
	struct sockaddr_can addr;
	struct ifreq ifr;

	/* check command line options */
	if (argc != 2) {
		print_usage_send(argv[0]);
		return 1;
	}

	/* open socket */
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		return 1;
	}

	strncpy(ifr.ifr_name, argv[1], IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
	if (!ifr.ifr_ifindex) {
		perror("if_nametoindex");
		return 1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	/* disable default receive filter on this RAW socket */
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	while(1) {
		// File desriptor for CAN.
		struct pollfd fds;
		int result;

		memset(&fds, 0, sizeof(struct pollfd));
		fds.fd = s;
		fds.events = POLLIN;

		// Any received CAN messages will cause the poll statement to unblock and run
		// This way CAN read runs with minimal latency.
		// Note that multiple messages may be received in a short time, so this will try to read any availible in a loop
		result = poll(&fds, 1, -1);
		//if (result < 0) {
			fprintf(stderr, "poll returned %d.  ", result);
		//}

		// Only execute this part if can0 is changed.
		if (fds.revents & POLLIN) {
			// Try to read.
			struct can_frame receive_msg;
			const ssize_t nbytes = read(fds.fd, &receive_msg, sizeof(receive_msg));
			fprintf(stderr, "read: 0x%lx, ", receive_msg.can_id);

			if (nbytes < 0 || (size_t)nbytes > sizeof(receive_msg)) {
				// error - todo - could this happen if we read in the middle of a packet receive?
				fprintf(stderr, "error on receive. Bytes read: %d\n", nbytes);
			} else {
				// send what was received
				fprintf(stderr, "send 0x%lx\n", receive_msg.can_id);
				if (write(fds.fd, &receive_msg, sizeof(receive_msg)) != sizeof(receive_msg)) {
					perror("write");
					return 1;
				}
			}
		}
	}

	close(s);

	return 0;
}
