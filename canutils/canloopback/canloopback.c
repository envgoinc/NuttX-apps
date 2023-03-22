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
#include <errno.h>
#include <time.h>
#include <poll.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>


#include <nuttx/can.h>
#include <netpacket/can.h>

#include "lib.h"

//static const int canfd_on = 1;


static void print_usage_send(char *prg)
{
	fprintf(stderr, "%s - echo CAN-frames received (only tested on CAN2.0 basic frames).\n", prg);
	fprintf(stderr, "\nUsage: %s <device>.\n", prg);
}



int main(int argc, char **argv)
{
	int s; /* can raw socket */
	fd_set rdfs;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct iovec iov;
	struct msghdr msg;
	struct can_frame receive_msg;
	char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3*sizeof(struct timespec) + sizeof(__u32))];
	int ret;

	/* check command line options */
	if (argc != 2) {
		print_usage_send(argv[0]);
		return 1;
	}

	/* open socket */
	s = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
	if (s < 0) {
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
	printf("interface: %s\n", ifr.ifr_name);

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	/* disable default receive filter on this RAW socket */
	// This makes recvmsg block indefinitely!!!
	//setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	/* try to switch the socket into CAN FD mode */
	//setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	iov.iov_base = &receive_msg;
	msg.msg_name = &addr;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = &ctrlmsg;


	while(1) {
#if 0
		FD_ZERO(&rdfs);
		FD_SET(s, &rdfs);

		printf("calling select\n");

		if ((ret = select(s+1, &rdfs, NULL, NULL, NULL)) <= 0) {
			perror("select");
			continue;
		}
		printf("select returned!\n");

		if (FD_ISSET(s, &rdfs)) {
			iov.iov_len = sizeof(receive_msg);
			msg.msg_namelen = sizeof(addr);
			msg.msg_controllen = sizeof(ctrlmsg);
			msg.msg_flags = 0;
			printf("calling recvmsg\n");

			const ssize_t nbytes = recvmsg(s, &msg, 0);

			if (nbytes < 0 || (size_t)nbytes > sizeof(receive_msg)) {
				// error - todo - could this happen if we read in the middle of a packet receive?
				printf("error on receive. Bytes read: %d, error %d\n", nbytes, errno);
			} else {
				printf("trying to send socket back\n");
				// send what was received
				//fprintf(stderr, "send 0x%lx\n", receive_msg.can_id);
				if (send(s, &receive_msg, sizeof(receive_msg), 0) != sizeof(receive_msg)) {
					perror("send");
					return 1;
				}
			}
		}
#endif

		//printf("trying to read\n");

		// Try to read.
		struct can_frame receive_msg;
		const ssize_t nbytes = recv(s, &receive_msg, sizeof(receive_msg), 0);
		//printf("read: 0x%lx, \n", receive_msg.can_id);

		if(nbytes < 0) {
			if(errno != EAGAIN) {
				printf("returned %d, errno: 0x%x\n", nbytes, errno);
			}
		}
		else if((size_t)nbytes > sizeof(receive_msg)) {
			// error - todo - could this happen if we read in the middle of a packet receive?
			printf("error on receive. Bytes read: %d\n", nbytes);
		}
		else {
			// send what was received
			// printf("received 0x%lx\n", receive_msg.can_id);
			if (write(s, &receive_msg, sizeof(receive_msg)) != sizeof(receive_msg)) {
				perror("write");
				return 1;
			}
		}
		usleep(3000);

#if 0
		// File desriptor for CAN.
		struct pollfd fds;
		int result;

		memset(&fds, 0, sizeof(struct pollfd));
		fds.fd = s;
		fds.events = POLLIN;

		printf("calling poll\n");

		// Any received CAN messages will cause the poll statement to unblock and run
		// This way CAN read runs with minimal latency.
		// Note that multiple messages may be received in a short time, so this will try to read any availible in a loop
		result = poll(&fds, 1, -1);
		//if (result < 0) {
			printf("poll returned %d\n", result);
		//}

		// Only execute this part if can0 is changed.
		if (fds.revents & POLLIN) {
			// Try to read.
			struct can_frame receive_msg;
			const ssize_t nbytes = read(s, &receive_msg, sizeof(receive_msg));
			printf("read: 0x%lx, ", receive_msg.can_id);

			if (nbytes < 0 || (size_t)nbytes > sizeof(receive_msg)) {
				// error - todo - could this happen if we read in the middle of a packet receive?
				printf("error on receive. Bytes read: %d\n", nbytes);
			} else {
				// send what was received
				printf("send 0x%lx\n", receive_msg.can_id);
				if (write(s, &receive_msg, sizeof(receive_msg)) != sizeof(receive_msg)) {
					perror("write");
					return 1;
				}
			}
		}
#endif
	}

	close(s);

	return 0;
}
