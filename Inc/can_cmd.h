/*
 * can_cmd.h
 *
 *  Created on: 23 рту. 2019 у.
 *      Author: User
 */

#ifndef CAN_CMD_H_
#define CAN_CMD_H_

#include <stdint.h>

#define		AUDIO_PACKET		1
#define		FIND_NEXT_POINT		2
#define		SCAN_GROUP			3
#define		POINT_STATE			4
#define		SET_OUTS			5
#define		LAST_POINT			6
#define		SET_ALL_OUTS		7
#define		GET_POINTS_STATE	8
#define		GATE_STATE			9
#define		BOOT				10
#define		POINT_CONFIG		11

// TYPES

#define		FIND_REQUEST	1
#define		FIND_ANSWER		2
#define		UNUSED_TYPE		6

// AUDIO

#define		UNKNOWN_TYPE		0
#define		PC_TO_ALL			1
#define		PC_TO_GROUP			2
#define		PC_TO_POINT			3
#define		POINT_TO_ALL		4
#define		POINT_TO_PC			5
#define		AUDIO_INFO			6
#define		POINT_CALL			7

// BOOT

#define		BOOT_WRITE_HEADER		0
#define		BOOT_WRITE_ACK			1
#define		BOOT_WRITE_DATA			2
#define		BOOT_ERASE_PAGE_REQ		3
#define		BOOT_ERASE_PAGE_ACK		4
#define		BOOT_SWITCH				5
#define		BOOT_PROG_FINISHED		6

// LAST POINT TYPES

#define BREAK_FINISH		0
#define	NORMAL_FINISH		1


void send_set_volume(uint8_t group, uint8_t point, uint8_t volume);
void send_set_point_inputs(uint8_t group, uint8_t point, uint8_t filter, uint8_t en);
void send_write_boot_data(uint8_t addr1, uint8_t addr2, uint8_t addr3, uint8_t addr4, uint8_t group, uint8_t point, uint8_t id,uint8_t length, uint8_t *ptr);
void send_erase_page(uint8_t num, uint8_t group, uint8_t point);
void send_switch_to_boot(uint8_t group, uint8_t point);
void send_reset_bootloder(uint8_t group, uint8_t point);
void send_get_state();
void send_scan_cmd_from_pc();
void send_scan_cmd_from_gate();
void manage_relay(uint8_t point_num, uint8_t relay_num, uint8_t state);
void manage_all_relays(uint8_t relay_num, uint8_t state);
void get_points_state();

#endif /* CAN_CMD_H_ */
