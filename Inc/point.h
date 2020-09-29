/*
 * point.h
 *
 *  Created on: 29 ñåíò. 2020 ã.
 *      Author: User
 */

#ifndef POINT_H_
#define POINT_H_

#include <stdint.h>

enum input_state{INP_ON, INP_OFF, INP_BR, INP_SHORT, INP_UNUSED};
enum audio_state{AUD_UNCHECKED, AUD_OK, AUD_PROBLEM};
enum inp2_type {KSL,CROSSING,FENCE,JAMMING}; // ÊÑË ÏÅĞÅÅÇÄ ÎÃĞÀÆÄÅÍÈÅ ÇÀØÒÛÁÎÂÊÀ

struct point_data{
	uint8_t gr_num;
	uint8_t point_num;
	uint8_t power;
	uint8_t battery;
	uint16_t bits;
	uint8_t version;
	uint8_t gain;
	uint8_t inp_filters;
	struct point_data *next;
};

enum input_state get_input1(struct point_data* p);
enum input_state get_input2(struct point_data* p);
unsigned char get_limit_switch(struct point_data* p);
enum audio_state get_audio_state(struct point_data* p);
enum inp2_type get_inp2_type(struct point_data* p);
unsigned char get_out1(struct point_data* p);
unsigned char get_out2(struct point_data* p);

#endif /* POINT_H_ */
