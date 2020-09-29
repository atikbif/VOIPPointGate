/*
 * point.c
 *
 *  Created on: 29 сент. 2020 г.
 *      Author: User
 */

#include "point.h"

enum input_state get_input1(struct point_data* p) {
	uint8_t h_byte = p->bits >> 8;
	uint8_t l_byte = p->bits & 0xFF;
	if(h_byte&0x08) return INP_UNUSED;
	if(l_byte&0x02) return INP_BR;
	if(l_byte&0x04) return INP_SHORT;
	if(l_byte&0x01) return INP_ON;
	return INP_OFF;
}

enum input_state get_input2(struct point_data* p) {
	uint8_t h_byte = p->bits >> 8;
	uint8_t l_byte = p->bits & 0xFF;
	if(h_byte&0x10) return INP_UNUSED;
	if(l_byte&0x10) return INP_BR;
	if(l_byte&0x20) return INP_SHORT;
	if(l_byte&0x08) return INP_ON;
	return INP_OFF;
}

unsigned char get_limit_switch(struct point_data* p) {
	if(p->bits & 0x0400) return 1;
	return 0;
}

enum audio_state get_audio_state(struct point_data* p) {
	uint8_t h_byte = p->bits >> 8;
	if(h_byte&0x02) {
		if(h_byte & 0x01) return AUD_OK;
		else return AUD_PROBLEM;
	}
	return AUD_UNCHECKED;
}

enum inp2_type get_inp2_type(struct point_data* p) {
	uint8_t h_byte = p->bits >> 8;
	h_byte = (h_byte>>5) & 0x03;
	if(h_byte==0) return KSL;
	if(h_byte==1) return CROSSING;
	if(h_byte==2) return FENCE;
	return JAMMING;
}

unsigned char get_out1(struct point_data* p) {
	uint8_t l_byte = p->bits & 0xFF;
	if(l_byte & 0x40) return 1;
	return 0;
}

unsigned char get_out2(struct point_data* p) {
	uint8_t l_byte = p->bits & 0xFF;
	if(l_byte & 0x80) return 1;
	return 0;
}

