/*
 * udp_server.h
 *
 *  Created on: 13 ���. 2019 �.
 *      Author: User
 */

#ifndef UDP_SERVER_H_
#define UDP_SERVER_H_

#include <stdio.h>

void udp_server_init(void);
void send_boot_ack(uint8_t id);
void send_erase_ack(uint8_t num);

#endif /* UDP_SERVER_H_ */
