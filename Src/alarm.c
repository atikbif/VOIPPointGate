/*
 * alarm.c
 *
 *  Created on: 13 но€б. 2019 г.
 *      Author: User
 */

#include "alarm.h"
#include "button_led.h"


#define MAX_ALARM_CNT	8

struct alarm {
	uint16_t num;
	uint8_t speak_flag;
};

static uint8_t cur_pos = 0;
static uint8_t alarm_flag = 0;
static volatile uint8_t disappeared_flag = 0;


static struct alarm alarms[MAX_ALARM_CNT];

uint8_t alarms_disappeared() {
	if(disappeared_flag) {
		disappeared_flag = 0;
		return 1;
	}
	return 0;
}

static uint8_t find_alarm(uint16_t value) {
	uint8_t i=0;
	for(i=0;i<MAX_ALARM_CNT;++i) {
		if(i>=cur_pos) break;
		if(alarms[i].num==value) {
			return i+1;
		}
	}
	return 0;
}

void delete_alarm(uint16_t value) {
	uint8_t i=0;
	uint8_t num = find_alarm(value);
	if(num && cur_pos) {
		uint8_t speak_flag = alarms[num-1].speak_flag;
		for(i=num-1;i<cur_pos-1;i++) {
			alarms[i].num = alarms[i+1].num;
			alarms[i].speak_flag = alarms[i+1].speak_flag;
		}
		cur_pos--;
		alarms[cur_pos].num = 0;
		alarms[cur_pos].speak_flag = 0;
		if(speak_flag) {
			for(i=0;i<MAX_ALARM_CNT;i++) {
				if(i>cur_pos) break;
				alarms[i].speak_flag = 0;
			}
		}
		if(cur_pos==0) disappeared_flag = 1;
	}
}

void delete_alarm_group(uint16_t value) {
	uint8_t i=0;
	uint8_t j=0;
	for(i=0;i<MAX_ALARM_CNT;++i) {
		if(i>cur_pos) break;
		if(alarms[i].num>>8==value>>8) {
			if(cur_pos) {
				uint8_t speak_flag = alarms[i].speak_flag;
				for(j=i;j<cur_pos-1;j++) {
					alarms[j].num = alarms[j+1].num;
					alarms[j].speak_flag = alarms[j+1].speak_flag;
				}
				cur_pos--;
				alarms[cur_pos].num = 0;
				alarms[cur_pos].speak_flag = 0;
				if(speak_flag) {
					for(j=0;j<MAX_ALARM_CNT;j++) {
						if(j>cur_pos) break;
						alarms[j].speak_flag = 0;
					}
				}
				if(cur_pos==0) disappeared_flag = 1;
				if(i) i--;
			}
		}
	}
}

uint16_t get_alarm() {
	uint8_t i=0;
	for(i=0;i<MAX_ALARM_CNT;++i) {
		if(i>=cur_pos) break;
		if(alarms[i].speak_flag==0) {
			alarms[i].speak_flag = 1;
			return alarms[i].num;
		}
	}
	return 0;
}

void clear_alarms() {
	uint8_t i=0;
	for(i=0;i<MAX_ALARM_CNT;++i) {
		alarms[i].num = 0;
		alarms[i].speak_flag=0;
	}
	cur_pos=0;
}

void add_alarm(uint16_t value) {
	if(cur_pos<MAX_ALARM_CNT) {
		disappeared_flag = 0;
		alarm_flag = 1;
		if(find_alarm(value)==0) {
			alarms[cur_pos].num = value;
			alarms[cur_pos].speak_flag = 0;
			cur_pos++;
		}
	}
}
