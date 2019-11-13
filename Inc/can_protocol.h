/*
 * can_protocol.h
 *
 *  Created on: 21 окт. 2019 г.
 *      Author: User
 */

#ifndef CAN_PROTOCOL_H_
#define CAN_PROTOCOL_H_

#include <stdint.h>

#define LOW_PACKET_PRIORITY		1
#define HIGH_PACKET_PRIORITY	2

#define		UNKNOWN_TYPE	0
#define		PC_TO_ALL		1
#define		PC_TO_GROUP		2
#define		PC_TO_POINT		3
#define		POINT_TO_ALL	4
#define		POINT_TO_PC		5

#define OPUS_PACKET_MAX_LENGTH	64

#define MODE_PC_TO_ALL		0
#define MODE_PC_TO_POINT	1
#define MODE_PC_TO_GROUP	2

typedef struct
{
 uint32_t param: 8;
 uint32_t cmd: 4;
 uint32_t group_addr: 7;
 uint32_t point_addr: 7;
 uint32_t type: 3;
 uint32_t unused_bits: 3;
} id_field;

void divide_to_packets_and_send_to_can(uint8_t dest_group, uint8_t dest_point, uint8_t len, uint8_t *ptr);
void check_can_rx(uint8_t can_num);
void can_write_from_stack();
void divide_to_packets_and_send_to_can(uint8_t dest_group, uint8_t dest_point, uint8_t len, uint8_t *ptr);
void send_alarm_packet(uint8_t len, uint8_t *ptr);

#endif /* CAN_PROTOCOL_H_ */
