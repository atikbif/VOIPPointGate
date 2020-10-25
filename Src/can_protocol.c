/*
 * can_protocol.c
 *
 *  Created on: 21 окт. 2019 г.
 *      Author: User
 */

#include "can_protocol.h"
#include "button_led.h"
#include "can_cmd.h"
#include "modbus.h"
#include "dyn_data.h"
#include "udp_server.h"
#include "frame_stack.h"
#include "main.h"
#include "can_tx_stack.h"
#include "eeprom.h"
#include "audio_out.h"

extern struct {
	uint8_t finished;
	uint8_t activated;
	uint16_t tmr;
	uint8_t doing;
}micr_check;

extern uint16_t VirtAddVarTab[NB_OF_VAR];

extern unsigned short holdReg[HoldingRegistersLimit];
extern unsigned char discrInp[DiscreteInputsLimit];
extern unsigned short inpReg[InputRegistersLimit];
extern uint16_t group_tmr[GROUP_CNT];

extern uint8_t cur_mode;	// режим общения по ethernet (пк-точка, пк-группа, пк-все)
extern uint8_t destination_group;	// активный номер группы для передачи аудио от ПК к точке (получен в запросе по ethernet)
extern uint8_t destination_point;	// активный номер точки для передачи аудио от ПК к точке (получен в запросе по ethernet)
uint8_t rx_group = 0;	// активный номер группы для передачи аудио от точки к ПК (получен в запросе по can)
uint8_t rx_point = 0;	// активный номер точки для передачи аудио от точки к ПК (получен в запросе по can)

extern uint16_t	supposed_point_cnt;

static CAN_TxHeaderTypeDef   TxHeader;
static uint32_t              TxMailbox1=0;
static uint32_t              TxMailbox2=0;
static uint8_t               TxData[8];
static CAN_RxHeaderTypeDef   RxHeader;
static uint8_t               RxData[8];

tx_stack can1_tx_stack;
tx_stack can2_tx_stack;

tx_stack can_rx1_stack;
tx_stack can_rx2_stack;


uint32_t	can_caught_id = UNKNOWN_TYPE; // extended id последнего захваченного аудио пакета
uint8_t current_group = 1;	// сетевой адрес шлюза

static uint8_t	can_priority_frame[OPUS_PACKET_MAX_LENGTH];
static uint8_t	can_frame[OPUS_PACKET_MAX_LENGTH];
uint8_t can_pckt_length = 0;
uint16_t packet_tmr = 0;

uint8_t p_cnt=0;	// число подключенных к шлюзу точек
uint8_t prev_p_cnt=0;

uint8_t wr_stack_flag = 0;	// флаг незавершённой операции записи в стэк
uint16_t can_tmr = 0;	// таймер для проверки что к шлюзу не подключена ни одна точка

uint8_t call_flag = 0;
uint16_t call_tmr = 0;

extern uint8_t alarm_led;


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// функция выдаёт разрешение для трансляции команды вверх по шлюзу (из выходного во входной канал)
static uint8_t inline is_cmd_through_up(uint8_t cmd) {
	if(cmd == AUDIO_PACKET || cmd == POINT_STATE || cmd==GATE_STATE || cmd==BOOT) return 1;
	return 0;
}

// разрешение для трансляции данных вниз по шлюзу (из входного в выходной)
static uint8_t inline is_cmd_through_down(uint8_t cmd, uint8_t param) {
	if(cmd == AUDIO_PACKET  || ((cmd==SCAN_GROUP) && param) || cmd==BOOT || cmd==POINT_CONFIG) return 1;
	return 0;
}

void send_alarm_packet(uint8_t len, uint8_t *ptr) {
	uint8_t i=len;
	uint8_t cur_pckt = 1;
	uint8_t pckt_cnt = 0;
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);

	wr_stack_flag = 1;

	while(i) {
		pckt_cnt++;
		if(i<=8) {i=0;break;}
		i-=8;
	}
	i=1;
	while(cur_pckt<=pckt_cnt) {
		p_id->point_addr = 0x7F;
		p_id->group_addr = current_group;
		p_id->type = AUDIO_INFO;

		p_id->cmd = AUDIO_PACKET;
		p_id->param = (cur_pckt&0x0F)|((pckt_cnt&0x0F)<<4);
		packet.priority = LOW_PACKET_PRIORITY;
		if(cur_pckt==pckt_cnt) { // last packet
			packet.length = len;
			for(i=0;i<len;i++) packet.data[i] = ptr[(cur_pckt-1)*8+i];
			add_tx_can_packet(&can1_tx_stack,&packet);
			add_tx_can_packet(&can2_tx_stack,&packet);
			//toggle_first_led(GREEN);
			cur_pckt++;
			len=0;
		}else {
			packet.length = 8;
			for(i=0;i<8;i++) packet.data[i] = ptr[(cur_pckt-1)*8+i];
			add_tx_can_packet(&can1_tx_stack,&packet);
			add_tx_can_packet(&can2_tx_stack,&packet);
			cur_pckt++;
			len-=8;
		}
	}
	wr_stack_flag = 0;
}

// разбиение пакета на can посылки
void divide_to_packets_and_send_to_can(uint8_t dest_group, uint8_t dest_point, uint8_t len, uint8_t *ptr) {
	uint8_t i=len;
	uint8_t cur_pckt = 1;
	uint8_t pckt_cnt = 0;
	tx_stack_data packet;
	id_field *p_id = (id_field*)(&packet.id);

	wr_stack_flag = 1;

	while(i) {
		pckt_cnt++;
		if(i<=8) {i=0;break;}
		i-=8;
	}
	i=1;
	while(cur_pckt<=pckt_cnt) {
		p_id->point_addr = dest_point&0x7F;
		p_id->group_addr = dest_group&0x7F;
		if(dest_point==0xFF) p_id->type = PC_TO_ALL;
		else if(dest_point==0x00) {p_id->type = PC_TO_GROUP;p_id->point_addr = 0;}
		else p_id->type = PC_TO_POINT;

		p_id->cmd = AUDIO_PACKET;
		p_id->param = (cur_pckt&0x0F)|((pckt_cnt&0x0F)<<4);
		packet.priority = LOW_PACKET_PRIORITY;
		if(cur_pckt==pckt_cnt) { // last packet
			packet.length = len;
			for(i=0;i<len;i++) packet.data[i] = ptr[(cur_pckt-1)*8+i];
			add_tx_can_packet(&can1_tx_stack,&packet);
			add_tx_can_packet(&can2_tx_stack,&packet);
			//toggle_first_led(GREEN);
			cur_pckt++;
			len=0;
		}else {
			packet.length = 8;
			for(i=0;i<8;i++) packet.data[i] = ptr[(cur_pckt-1)*8+i];
			add_tx_can_packet(&can1_tx_stack,&packet);
			add_tx_can_packet(&can2_tx_stack,&packet);
			cur_pckt++;
			len-=8;
		}
	}
	wr_stack_flag = 0;
}

void can_write_from_stack() {
	tx_stack_data packet;
	uint8_t i = 0;
	if(wr_stack_flag) return;
		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)!=0) {
			if(get_tx_can_packet(&can1_tx_stack,&packet)) {
				TxHeader.StdId = 0;
				TxHeader.ExtId = packet.id;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.IDE = CAN_ID_EXT;
				TxHeader.TransmitGlobalTime = DISABLE;
				TxHeader.DLC = packet.length;
				for(i=0;i<packet.length;++i) TxData[i] = packet.data[i];
				HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox1);
				//toggle_first_led(GREEN);
			}else break;
		}

	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)!=0) {
		if(get_tx_can_packet(&can2_tx_stack,&packet)) {
			TxHeader.IDE = CAN_ID_EXT;
			TxHeader.StdId = 0;
			TxHeader.ExtId = packet.id;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.IDE = CAN_ID_EXT;
			TxHeader.TransmitGlobalTime = DISABLE;
			TxHeader.DLC = packet.length;
			for(i=0;i<packet.length;++i) TxData[i] = packet.data[i];
			HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox2);
		}else break;
	}
}

static uint8_t compare_id(id_field *input_id, id_field *cur_id) {
	if(input_id->type==AUDIO_INFO) {
		if(cur_id->type!=AUDIO_INFO || ((cur_id->type==AUDIO_INFO) && (input_id->group_addr==cur_id->group_addr))) {
			*cur_id = *input_id;
			return 1;
		}
	}
	if(input_id->type==POINT_TO_PC) {
		if(cur_id->type==UNKNOWN_TYPE) {	// пакеты не захвачены
			*cur_id = *input_id;
			return 1;
		}
		// ранее захваченный источник
		if(cur_id->group_addr == input_id->group_addr && cur_id->point_addr == input_id->point_addr) {
			*cur_id = *input_id;
			return 1;
		}
		// активный пакет не из родной группы, новый запрос из родной группы, перехватить
		if(cur_id->group_addr != current_group && input_id->group_addr == current_group) {
			*cur_id = *input_id;
			return 1;

		}
		return 1;
	}
	if(input_id->type==POINT_TO_ALL || input_id->type==POINT_CALL) { // точка все
		if(cur_id->type==UNKNOWN_TYPE) {	// пакеты не захвачены
			*cur_id = *input_id;
			return 1;
		}
		// ранее захваченный источник
		if(cur_id->group_addr == input_id->group_addr && cur_id->point_addr == input_id->point_addr) {
			*cur_id = *input_id;
			return 1;
		}
		// активный пакет не из родной группы, новый запрос из родной группы, перехватить
		if(cur_id->group_addr != current_group && input_id->group_addr == current_group) {
			*cur_id = *input_id;
			return 1;
		}
		return 0;
	}
	return 0;
}


uint8_t static check_id_priority(uint32_t packet_id) {
	id_field *input_id = (id_field*)(&packet_id);
	id_field *cur_id = (id_field*)(&can_caught_id);
	if(is_sentence_ready_to_speak()) return 0;
	if(cur_mode == MODE_PC_TO_ALL) {
		return compare_id(input_id, cur_id);
	}else if(cur_mode == MODE_PC_TO_GROUP) {
		if(input_id->group_addr == destination_group) {
			return compare_id(input_id, cur_id);
		}
	}else {
		if(input_id->cmd==AUDIO_PACKET && input_id->type==AUDIO_INFO) {
			return compare_id(input_id, cur_id);
		}
		if(input_id->group_addr == destination_group && input_id->point_addr == destination_point) {
			return compare_id(input_id, cur_id);
		}
	}
	return 0;
}

void check_can_rx(uint8_t can_num) {
	CAN_HandleTypeDef *hcan;
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t cur_num = 0;
	uint8_t cnt = 0;
	id_field *p_id;
	tx_stack_data packet;
	struct point_data point;
	struct group_data group;
	if(can_num==1) hcan = &hcan1; else hcan = &hcan2;
	if(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0)) {
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
			if(alarm_led) {
				if(can_num==2) toggle_second_led(RED);
				else toggle_first_led(RED);
			}else {
				if(can_num==2) toggle_second_led(GREEN);
				else toggle_first_led(GREEN);
			}

			p_id = (id_field*)(&(RxHeader.ExtId));
			if(p_id->group_addr==0) return;
			if(p_id->cmd==AUDIO_PACKET) { // аудиопоток
				if(check_id_priority(RxHeader.ExtId))
				{
					packet_tmr = 0;
					if(p_id->type==POINT_CALL) {
						call_flag=1;
						call_tmr=0;
					}
					cur_num = p_id->param & 0x0F;
					cnt = (p_id->param & 0xFF)>> 4;
					if(cur_num) {
						if(cur_num==cnt) {
						  if(p_id->type==POINT_TO_ALL || p_id->type==POINT_TO_PC || p_id->type==POINT_CALL || p_id->type==AUDIO_INFO) { // точка все
							  j = (cur_num-1)*8;
							  for(i=0;i<RxHeader.DLC;i++) {
								  if(j+i<OPUS_PACKET_MAX_LENGTH) can_priority_frame[j+i]=RxData[i];
							  }
							  can_pckt_length = (cnt-1)*8+RxHeader.DLC;
							  if(can_pckt_length>OPUS_PACKET_MAX_LENGTH) can_pckt_length = OPUS_PACKET_MAX_LENGTH;
							  for(i=0;i<can_pckt_length;i++) {
								  can_frame[i]=can_priority_frame[i];
							  }
							  add_can_frame(&can_frame[0],can_pckt_length);
							  rx_group = p_id->group_addr;
							  rx_point = p_id->point_addr;
						  }
						}else {
						  j = (cur_num-1)*8;
						  for(i=0;i<8;i++) { if(j+i<OPUS_PACKET_MAX_LENGTH) can_priority_frame[j+i]=RxData[i]; }
						}
					}
				}
			}else if(p_id->cmd==POINT_STATE) {
				if(p_id->point_addr>0 && p_id->point_addr<=64) {
					if(p_id->group_addr==current_group && can_num==2) {
						// неправильное подключение
					}
					else
					{
						if(micr_check.activated==0) {
							RxData[0] &= 0b11111100;
						}
						if(p_id->group_addr==current_group) {
							discrInp[16+(p_id->point_addr-1)*11] = RxData[0]&0x01;		// исправность микрофона/динамика
							discrInp[16+(p_id->point_addr-1)*11+1] = RxData[1]&0x01;	// di1
							discrInp[16+(p_id->point_addr-1)*11+2] = RxData[1]&0x02;	// обрыв
							discrInp[16+(p_id->point_addr-1)*11+3] = RxData[1]&0x04;	// кз
							discrInp[16+(p_id->point_addr-1)*11+4] = RxData[1]&0x08;	// di2
							discrInp[16+(p_id->point_addr-1)*11+5] = RxData[1]&0x10;	// обрыв
							discrInp[16+(p_id->point_addr-1)*11+6] = RxData[1]&0x20;	// кз
							discrInp[16+(p_id->point_addr-1)*11+7] = RxData[1]&0x40;		// do1
							discrInp[16+(p_id->point_addr-1)*11+8] = RxData[1]&0x80;		// do2
							discrInp[16+(p_id->point_addr-1)*11+9] = RxData[0]&0x02;		// проверка испрвн динамиков
							discrInp[16+(p_id->point_addr-1)*11+10] = RxData[0]&0x04;		// концевик
							inpReg[16+(p_id->point_addr-1)] = (((uint16_t)RxData[2])<<8) | RxData[3];
						}
						if(p_id->group_addr!=0 && p_id->point_addr!=0) {
							point.battery = RxData[2];
							point.power = RxData[3];
							point.gr_num = p_id->group_addr-1;
							point.point_num = p_id->point_addr-1;
							point.version = RxData[4];
							point.bits = (((uint16_t)RxData[0])<<8) | RxData[1];
							point.gain = RxData[5];
							point.inp_filters = RxData[6];
							add_point_data(&point);
						}
					}
				}
			}else if(p_id->cmd==LAST_POINT && p_id->point_addr<=64 && p_id->point_addr>0 && can_num==1) {
				if(p_id->type == BREAK_FINISH) {
					p_cnt = p_id->point_addr;
				}else if(p_id->type == NORMAL_FINISH) {
					if(p_id->point_addr != supposed_point_cnt) {
						holdReg[12] = supposed_point_cnt = p_id->point_addr;
						EE_WriteVariable(VirtAddVarTab[13],  supposed_point_cnt);
					}
					p_cnt = p_id->point_addr;
				}
				inpReg[0] = p_cnt;
				if(prev_p_cnt!=p_cnt) send_get_state();
				prev_p_cnt = p_cnt;
				if(p_id->group_addr==current_group) can_tmr = 0;
			}else if(p_id->cmd==GATE_STATE) {
				group.num = p_id->group_addr;
				group.bits=(((uint16_t)RxData[1])<<8) | RxData[2];;
				group.point_cnt=RxData[0];
				group.version=1;
				group_tmr[p_id->group_addr-1]=0;
				add_group_data(p_id->group_addr-1,&group);
			}else if(p_id->cmd==BOOT) {
				if(p_id->type==BOOT_WRITE_ACK) {
					send_boot_ack(RxData[0]);
				}else if(p_id->type==BOOT_ERASE_PAGE_ACK) {
					send_erase_ack(p_id->param);
				}
			}


			packet.id = RxHeader.ExtId;
			packet.priority = LOW_PACKET_PRIORITY;
			packet.length = RxHeader.DLC;
			for(i=0;i<packet.length;++i) packet.data[i] = RxData[i];
			if(can_num==1 && is_cmd_through_up(p_id->cmd)) {
				add_tx_can_packet(&can2_tx_stack,&packet);
			}
			else if(can_num==2 && is_cmd_through_down(p_id->cmd,p_id->param)){add_tx_can_packet(&can1_tx_stack,&packet);}
		}
	}
}

