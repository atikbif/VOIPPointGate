/*
 * can_cmd.c
 *
 *  Created on: 23 авг. 2019 г.
 *      Author: User
 */

#include "can_cmd.h"
#include "can_tx_stack.h"
#include "can_protocol.h"

extern uint8_t current_group;
extern tx_stack can1_tx_stack;
extern tx_stack can2_tx_stack;
extern uint8_t p_cnt;
extern uint16_t group_bits;

void send_set_volume(uint8_t group, uint8_t point, uint8_t volume) {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = UNUSED_TYPE;
	p_id->point_addr = point;
	p_id->group_addr = group;
	p_id->cmd = POINT_CONFIG;
	p_id->param = 0;
	packet.length = 2;
	packet.data[0] = 0x01;
	packet.data[1] = volume;
	add_tx_can_packet(&can1_tx_stack,&packet);
	add_tx_can_packet(&can2_tx_stack,&packet);
}

void send_set_point_inputs(uint8_t group, uint8_t point, uint8_t filter, uint8_t en) {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = UNUSED_TYPE;
	p_id->point_addr = point;
	p_id->group_addr = group;
	p_id->cmd = POINT_CONFIG;
	p_id->param = 0;
	packet.length = 3;
	packet.data[0] = 0x02;
	packet.data[1] = filter;
	packet.data[2] = en;
	add_tx_can_packet(&can1_tx_stack,&packet);
	add_tx_can_packet(&can2_tx_stack,&packet);
}

void send_write_boot_data(uint8_t addr1, uint8_t addr2, uint8_t addr3, uint8_t addr4, uint8_t group, uint8_t point, uint8_t id,uint8_t length, uint8_t *ptr) {

	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	uint8_t i=1, j=0;
	if(length>7*8) length=7*8;

	p_id->type = BOOT_WRITE_HEADER;
	p_id->point_addr = point;
	p_id->group_addr = group;
	p_id->cmd = BOOT;
	p_id->param = 0;
	packet.length = 6;
	packet.data[0] = length;
	packet.data[1] = id;
	packet.data[2] = addr1;
	packet.data[3] = addr2;
	packet.data[4] = addr3;
	packet.data[5] = addr4;
	add_tx_can_packet(&can1_tx_stack,&packet);
	add_tx_can_packet(&can2_tx_stack,&packet);
	while(length) {
		p_id->type = BOOT_WRITE_DATA;
		p_id->point_addr = point;
		p_id->group_addr = group;
		p_id->cmd = BOOT;
		p_id->param = i++;
		if(length>=7) {packet.length = 8;length-=7;}
		else {packet.length = 1+length;length=0;}
		packet.data[0] = id;
		for(j=0;j<packet.length-1;j++) {packet.data[1+j]=*ptr;ptr++;}
		add_tx_can_packet(&can1_tx_stack,&packet);
		add_tx_can_packet(&can2_tx_stack,&packet);
	}
}

void send_erase_page(uint8_t num, uint8_t group, uint8_t point) {

	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = BOOT_ERASE_PAGE_REQ;
	p_id->point_addr = point;
	p_id->group_addr = group;
	p_id->cmd = BOOT;
	p_id->param = num;
	packet.length = 0;
	add_tx_can_packet(&can1_tx_stack,&packet);
	add_tx_can_packet(&can2_tx_stack,&packet);
}

void send_switch_to_boot(uint8_t group, uint8_t point) {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = BOOT_SWITCH;
	p_id->point_addr = point;
	p_id->group_addr = group;
	p_id->cmd = BOOT;
	p_id->param = 0;
	packet.length = 0;
	add_tx_can_packet(&can1_tx_stack,&packet);
	add_tx_can_packet(&can2_tx_stack,&packet);
}

void send_reset_bootloder(uint8_t group, uint8_t point) {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = BOOT_PROG_FINISHED;
	p_id->point_addr = point;
	p_id->group_addr = group;
	p_id->cmd = BOOT;
	p_id->param = 0;
	packet.length = 0;
	add_tx_can_packet(&can1_tx_stack,&packet);
	add_tx_can_packet(&can2_tx_stack,&packet);
}

void send_get_state() {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = UNUSED_TYPE;
	p_id->point_addr = 0x00;
	p_id->group_addr = current_group;
	p_id->cmd = GATE_STATE;
	p_id->param = 0;
	packet.priority = LOW_PACKET_PRIORITY;
	packet.length = 3;
	packet.data[0] = p_cnt;
	packet.data[1] = group_bits>>8;
	packet.data[2] = group_bits & 0xFF;
	add_tx_can_packet(&can2_tx_stack,&packet);
	add_tx_can_packet(&can1_tx_stack,&packet);
}

void send_scan_cmd_from_pc() {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = UNUSED_TYPE;
	p_id->point_addr = 0x00;
	p_id->group_addr = current_group;
	p_id->cmd = SCAN_GROUP;
	p_id->param = 1;
	packet.priority = LOW_PACKET_PRIORITY;
	packet.length = 0;
	add_tx_can_packet(&can1_tx_stack,&packet);
	add_tx_can_packet(&can2_tx_stack,&packet);
}

void send_scan_cmd_from_gate() {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = UNUSED_TYPE;
	p_id->point_addr = 0x00;
	p_id->group_addr = current_group;
	p_id->cmd = SCAN_GROUP;
	p_id->param = 0;
	packet.priority = LOW_PACKET_PRIORITY;
	packet.length = 0;
	add_tx_can_packet(&can1_tx_stack,&packet);
}

void manage_relay(uint8_t point_num, uint8_t relay_num, uint8_t state) {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = UNUSED_TYPE;
	p_id->point_addr = 0;
	p_id->group_addr = current_group;
	p_id->cmd = SET_OUTS;
	p_id->param = point_num;
	packet.priority = LOW_PACKET_PRIORITY;
	packet.length = 2;
	packet.data[0] = relay_num;
	packet.data[1] = state;
	// добавить состояние входов выходов
	add_tx_can_packet(&can1_tx_stack,&packet);
}

void manage_all_relays(uint8_t relay_num, uint8_t state) {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = UNUSED_TYPE;
	p_id->point_addr = 0;
	p_id->group_addr = current_group;
	p_id->cmd = SET_ALL_OUTS;
	p_id->param = 0;
	packet.priority = LOW_PACKET_PRIORITY;
	packet.length = 2;
	packet.data[0] = relay_num;
	packet.data[1] = state;
	// добавить состояние входов выходов
	add_tx_can_packet(&can1_tx_stack,&packet);
}

void get_points_state() {
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);
	p_id->type = UNUSED_TYPE;
	p_id->point_addr = 0x00;
	p_id->group_addr = current_group;
	p_id->cmd = GET_POINTS_STATE;
	p_id->param = 0;
	packet.priority = LOW_PACKET_PRIORITY;
	packet.length = 0;
	add_tx_can_packet(&can1_tx_stack,&packet);
}
