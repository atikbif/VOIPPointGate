/*
 * dyn_data.c
 *
 *  Created on: 23 сент. 2019 г.
 *      Author: User
 */

#include "dyn_data.h"
#include "button_led.h"

struct group_data groups[GROUP_CNT];	// данные шлюзов
struct point_data points[POINT_CNT];	// данные точек
struct point_data* group_point[GROUP_CNT];	// указатель на первую захваченную для группы точку (дальше через next можно получить полный список точек группы)
uint16_t used_point_cnt = 0;	// общее число обнаруженных точек
uint16_t group_tmr[GROUP_CNT];	// таймер для детектирования наличия связи с группой

void init_points() {
	int16_t i=0;
	for(i=0;i<POINT_CNT;i++) {
		points[i].battery=0;
		points[i].bits=0;
		points[i].gr_num=0;
		points[i].next=0;
		points[i].point_num=0;
		points[i].power=0;
		points[i].version=0;
		points[i].inp_filters=0;
	}
}

void init_groups() {
	uint8_t i=0;
	for(i=0;i<GROUP_CNT;i++) {
		groups[i].num=0;
		groups[i].bits=0;
		groups[i].point_cnt=0;
		groups[i].version=0;
		group_point[i]=0;
		group_tmr[i]=0;
	}
}

struct point_data* is_point_created(uint8_t group_num, uint8_t point_num) {
	struct point_data* ptr = group_point[group_num];
	uint16_t i = 0;
	while(ptr) {
		if(ptr->point_num==point_num) return ptr;
		ptr=ptr->next;
		if((++i)>=POINT_CNT) return 0;
	}
	return 0;
}

static void copy_point_data(struct point_data *inp, struct point_data *out) {
	out->battery = inp->battery;
	out->bits = inp->bits;
	out->gr_num = inp->gr_num;
	out->point_num = inp->point_num;
	out->power = inp->power;
	out->version = inp->version;
	out->gain = inp->gain;
	out->inp_filters = inp->inp_filters;
}

static void add_new_point(struct point_data *ptr) {
	struct point_data* point = group_point[ptr->gr_num];
	uint16_t i = 0;
	if(used_point_cnt<POINT_CNT) {
		if(point==0) {
			point = &points[used_point_cnt];
			group_point[ptr->gr_num] = point;
			copy_point_data(ptr,point);
			point->next = 0;
			used_point_cnt++;
		}else {
			while(point->next) {
				point = point->next;
				if((++i)>=POINT_CNT) return;
			}
			point->next = &points[used_point_cnt];
			point = point->next;
			copy_point_data(ptr,point);
			point->next = 0;
			used_point_cnt++;
		}
	}
}

void add_group_data(uint8_t group_num, struct group_data *ptr) {
	if(group_num<GROUP_CNT) {
		groups[group_num].num = ptr->num;
		groups[group_num].bits = ptr->bits;
		groups[group_num].point_cnt = ptr->point_cnt;
		groups[group_num].version = ptr->version;
	}
}

void add_point_data(struct point_data *ptr) {
	if((ptr->gr_num<GROUP_CNT) && (ptr->point_num<POINT_CNT)) {
		struct point_data* point = is_point_created(ptr->gr_num,ptr->point_num);
		if(point) copy_point_data(ptr,point);
		else add_new_point(ptr);
	}
}

uint16_t write_group_data_to_buf(uint8_t *ptr) {
	uint16_t i=0;
	for(i=0;i<GROUP_CNT;i++) {
		ptr[i*5] = groups[i].num;
		ptr[i*5+1] = groups[i].point_cnt;
		ptr[i*5+2] = groups[i].version;
		ptr[i*5+3] = groups[i].bits >> 8;
		ptr[i*5+4] = groups[i].bits & 0xFF;
	}
	return GROUP_CNT*5;
}

uint16_t write_point_data_to_buf(uint8_t part_num, uint8_t *ptr) {
	uint16_t start_point=0;
	const uint16_t length = (POINT_CNT/5);
	uint16_t i = 0;
	while(part_num>=5) part_num-=5;
	start_point=(POINT_CNT/5)*part_num;
	ptr[0] = used_point_cnt >> 8;
	ptr[1] = used_point_cnt & 0xFF;
	for(i=0;i<length;i++) {
		if(start_point+i<used_point_cnt) {
			ptr[i*9+2] = points[start_point+i].gr_num+1;
			ptr[i*9+3] = points[start_point+i].point_num+1;
			ptr[i*9+4] = points[start_point+i].battery;
			ptr[i*9+5] = points[start_point+i].power;
			ptr[i*9+6] = points[start_point+i].version;
			ptr[i*9+7] = points[start_point+i].bits>>8;
			ptr[i*9+8] = points[start_point+i].bits & 0xFF;
			ptr[i*9+9] = points[start_point+i].gain;
			ptr[i*9+10] = points[start_point+i].inp_filters;
		}else {
			ptr[i*9+2] = 0;
			ptr[i*9+3] = 0;
			ptr[i*9+4] = 0;
			ptr[i*9+5] = 0;
			ptr[i*9+6] = 0;
			ptr[i*9+7] = 0;
			ptr[i*9+8] = 0;
			ptr[i*9+9] = 0;
			ptr[i*9+10] = 0;
		}
	}
	return 2 + length*9;
}
