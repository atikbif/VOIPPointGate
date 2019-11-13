/*
 * audio_out.h
 *
 *  Created on: 1 но€б. 2019 г.
 *      Author: User
 */

#ifndef AUDIO_OUT_H_
#define AUDIO_OUT_H_

#include <stdint.h>

void init_audio_dictionary();
void create_empty_sentence();
void add_word_to_sentence(uint8_t dict_num);
void set_sentence_ready_to_speak();
uint8_t is_sentence_ready_to_speak();
uint8_t get_opus_packet(uint8_t *ptr);
void add_pause();
void add_number(uint8_t num);


#endif /* AUDIO_OUT_H_ */
