/*
 * dyn_data.h
 *
 *  Created on: 23 сент. 2019 г.
 *      Author: User
 */

#ifndef DYN_DATA_H_
#define DYN_DATA_H_

#include <stdint.h>
#include "point.h"


struct group_data{
	uint8_t num;
	uint8_t point_cnt;
	uint16_t bits;
	uint8_t version;
};

void init_points();
void init_groups();

struct point_data* is_point_created(uint8_t group_num, uint8_t point_num);
void add_group_data(uint8_t group_num, struct group_data *ptr);
void add_point_data(struct point_data *ptr);
uint16_t write_group_data_to_buf(uint8_t *ptr);
uint16_t write_point_data_to_buf(uint8_t part_num, uint8_t *ptr);



#define 	POINT_CNT	500
#define		GROUP_CNT	32

#endif /* DYN_DATA_H_ */
