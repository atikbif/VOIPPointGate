/*
 * alarm.h
 *
 *  Created on: 13 но€б. 2019 г.
 *      Author: User
 */

#ifndef ALARM_H_
#define ALARM_H_

#include <stdint.h>

void clear_alarms();
void clear_alarms_excluding_type(uint8_t alarm_type);
void add_alarm(uint16_t value);
void delete_alarm(uint16_t value);
void delete_alarm_group(uint16_t value);
uint16_t get_alarm();
uint8_t alarms_disappeared();

#endif /* ALARM_H_ */
