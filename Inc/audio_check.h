/*
 * audio_check.h
 *
 *  Created on: 23 окт. 2020 г.
 *      Author: User
 */

#ifndef AUDIO_CHECK_H_
#define AUDIO_CHECK_H_

#include "dyn_data.h"

void init_prestart();
void enable_prestart();
void disable_prestart();
void prestart_cycle(struct group_data *group);
enum audio_state get_audio_check_result();
uint16_t get_prestart_period();

#endif /* AUDIO_CHECK_H_ */
