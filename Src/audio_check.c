/*
 * audio_check.c
 *
 *  Created on: 23 окт. 2020 г.
 *      Author: User
 */

#include "point.h"
#include "audio_check.h"

#define AUDIO_CHECK_WAIT_TIME	(10*3)
#define AUDIO_CHECK_PAUSE_TIME	(10*2)

struct prestart{
   uint8_t enable_cmd;
   uint8_t disable_cmd;
   uint16_t tmr;
   uint8_t state;
   enum audio_state check_result;
};

static struct prestart audio_test;

uint16_t get_prestart_period() {
	return AUDIO_CHECK_WAIT_TIME + AUDIO_CHECK_PAUSE_TIME;
}

void init_prestart() {
	audio_test.check_result = AUD_UNCHECKED;
	audio_test.enable_cmd = 0;
	audio_test.disable_cmd = 0;
	audio_test.state = 0;
	audio_test.tmr = 0;
}

void enable_prestart() {
	audio_test.enable_cmd = 1;
}

void disable_prestart() {
	audio_test.disable_cmd = 1;
}

void prestart_cycle(struct group_data *group) {	// should be called every 100 ms
	switch(audio_test.state) {
		case 0:
			if(audio_test.enable_cmd) {
				audio_test.enable_cmd = 0;
				audio_test.state = 1;
				audio_test.check_result = AUD_UNCHECKED;
			}
			break;
		case 1:
			send_scan_cmd_from_gate();
			audio_test.state++;
			audio_test.tmr = 0;
			break;
		case 2:
			audio_test.tmr++;
			if(audio_test.tmr>=AUDIO_CHECK_WAIT_TIME) audio_test.state = 3;
			break;
		case 3:
			for(uint8_t i=0;i<group->point_cnt;i++) {
				struct point_data* p = is_point_created(group->num-1,i);
				if(p) {
					if(get_audio_state(p)==AUD_PROBLEM) {
						audio_test.check_result = AUD_PROBLEM;
						break;
					}
				}
			}
			if(audio_test.check_result != AUD_PROBLEM) audio_test.check_result = AUD_OK;
			audio_test.state++;
			audio_test.tmr = 0;
			break;
		case 4:
			if(audio_test.disable_cmd) {
				audio_test.disable_cmd = 0;
				audio_test.state = 0;
				break;
			}
			audio_test.tmr++;
			if(audio_test.tmr>=AUDIO_CHECK_PAUSE_TIME) {
				audio_test.state = 1;
			}
			break;
	}
}

enum audio_state get_audio_check_result() {
	return audio_test.check_result;
}
