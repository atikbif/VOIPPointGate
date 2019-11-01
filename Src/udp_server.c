/*
 * udp_server.c
 *
 *  Created on: 13 янв. 2019 г.
 *      Author: User
 */

#include "udp_server.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "crc.h"

#include "stm32f4xx_hal.h"
#include <string.h>
#include "button_led.h"
#include "frame_stack.h"
#include "can_tx_stack.h"
#include "modbus.h"
#include "can_cmd.h"
#include "dyn_data.h"
#include "button_led.h"
#include "cmsis_os.h"
#include "can_protocol.h"

#define UDP_SERVER_PORT    12145


uint8_t cur_mode = MODE_PC_TO_ALL;

static char answer[1324];	// буфер для ответа запросов по порту UDP_SERVER_PORT (общение с диспетчером)
static char lanswer[1324];	// буфер для ответа запросов по порту UDP_SERVER_PORT+1 (общение с загрузчиком)

volatile uint32_t start_wait_time = 0;	// таймер для ожидания ответов от точек по кан и формирования ethernet ответа

uint8_t destination_group = 1;		// активный номер группы для передачи аудио от ПК к точке (получен в запросе по ethernet)
uint8_t destination_point = 255;	// активный номер точки для передачи аудио от ПК к точке (получен в запросе по ethernet)

extern uint8_t rx_group;	// активный номер группы для передачи аудио от точки к ПК (получен в запросе по can)
extern uint8_t rx_point;	// активный номер точки для передачи аудио от точки к ПК (получен в запросе по can)

extern uint16_t adc_data[3];	// данные АЦП (состояние дискретных входов)

extern unsigned short inpReg[InputRegistersLimit];
extern unsigned short holdReg[HoldingRegistersLimit];
extern unsigned char discrInp[DiscreteInputsLimit];

extern uint8_t modbus_di_array[DiscreteInputsLimit/8];	// массив для упаковки состояния дискретных входов в байты

extern uint8_t call_flag;

// флаги для ответа на команды загрузчика
static uint8_t boot_ack = 0;
static uint8_t boot_id = 0;
static uint8_t erase_ack = 0;
static uint8_t erase_num = 0;


static void udp_server_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
static void udp_loader_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
static void inline send_udp_data(struct udp_pcb *upcb,const ip_addr_t *addr,u16_t port,u16_t length);
static void inline send_udp_data_loader(struct udp_pcb *upcb,const ip_addr_t *addr,u16_t port,u16_t length);

// вызывается при обработке can протокола (подтверждение записи)
void send_boot_ack(uint8_t id) {
	boot_id = id;
	boot_ack=1;
}

// вызывается при обработке can протокола (подтверждение стирания)
void send_erase_ack(uint8_t num) {
	erase_num = num;
	erase_ack=1;
}

void udp_server_init(void) {
	struct udp_pcb *upcb;
	struct udp_pcb *upcb_loader;
	err_t err;

	/* Create a new UDP control block  */
	upcb = udp_new();

	if (upcb)
	{
	 /* Bind the upcb to the UDP_PORT port */
	 /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
	  err = udp_bind(upcb, IP_ADDR_ANY, UDP_SERVER_PORT);

	  if(err == ERR_OK)
	  {

		/* Set a receive callback for the upcb */
		udp_recv(upcb, udp_server_receive_callback, NULL);
	  }
	  else
	  {
		udp_remove(upcb);
	  }
	}

	upcb_loader = udp_new();

	if (upcb_loader)
	{
	 /* Bind the upcb to the UDP_PORT port */
	 /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
	  err = udp_bind(upcb_loader, IP_ADDR_ANY, UDP_SERVER_PORT+1);

	  if(err == ERR_OK)
	  {

		/* Set a receive callback for the upcb */
		udp_recv(upcb_loader, udp_loader_receive_callback, NULL);
	  }
	  else
	  {
		udp_remove(upcb_loader);
	  }
	}
}

// функция обратного для загрузчика (порт UDP_SERVER_PORT+1)
void udp_loader_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
	unsigned char *data;
	unsigned short crc;

	data = (unsigned char*)(p->payload);
	crc = GetCRC16(data,p->len);
	if(crc==0)
	{
	  switch(data[2]){
	  case 0x10:	// запись данных
		  send_write_boot_data(data[3], data[4], data[5],data[6], data[7], data[8], data[9],data[10], &data[11]);
		  boot_ack = 0;
		  // первые 4 байта - адрес записи,
		  // группа, точка, id, длина пакета (до 56 байт - 8 посылок по 7 байт)
		  // непосредственно данные
		  start_wait_time = 0;
		  while(start_wait_time<100) {
			  osDelay(1);start_wait_time++;
			  if(boot_ack) {
				  lanswer[0] = data[0];	// id high
				  lanswer[1] = data[1];	// id low
				  lanswer[2] = 0x10;		// cmd
				  lanswer[3] = boot_id;
				  crc = GetCRC16((unsigned char*)lanswer,4);
				  lanswer[4] = crc>>8;
				  lanswer[5] = crc&0xFF;
				  send_udp_data_loader(upcb, addr, port, 6);
				  break;
			  }
		  }
		  break;
	  case 0x11:	// стирание данных
		  send_erase_page(data[3], data[4], data[5]);
		  // номер страницы
		  // группа, точка
		  erase_ack = 0;
		  start_wait_time = 0;
		  while(start_wait_time<100) {
			  osDelay(1);start_wait_time++;
			  if(erase_ack) {
				  lanswer[0] = data[0];	// id high
				  lanswer[1] = data[1];	// id low
				  lanswer[2] = 0x11;		// cmd
				  lanswer[3] = erase_num;
				  crc = GetCRC16((unsigned char*)lanswer,4);
				  lanswer[4] = crc>>8;
				  lanswer[5] = crc&0xFF;
				  send_udp_data_loader(upcb, addr, port, 6);
				  break;
			  }
		  }
		  break;
	  case 0x12:	// проверка наличия точки
		  lanswer[0] = data[0];	// id high
		  lanswer[1] = data[1];	// id low
		  lanswer[2] = 0x12;		// cmd
		  struct point_data* point = is_point_created(data[3]-1,data[4]-1);
		  if(point) {
			  lanswer[3] = 1;
			  lanswer[4] = point->version;
		  }else {
			  lanswer[3] = 0;
			  lanswer[4] = 0;
		  }
		  crc = GetCRC16((unsigned char*)lanswer,5);
		  lanswer[5] = crc>>8;
		  lanswer[6] = crc&0xFF;
		  send_udp_data_loader(upcb, addr, port, 7);
		  break;
	  case 0x13:	// переключение точки в режим загрузчика
		  send_switch_to_boot(data[3],data[4]);
		  send_switch_to_boot(data[3],data[4]);
		  send_switch_to_boot(data[3],data[4]);
		  lanswer[0] = data[0];	// id high
		  lanswer[1] = data[1];	// id low
		  lanswer[2] = 0x13;		// cmd
		  crc = GetCRC16((unsigned char*)lanswer,3);
		  lanswer[3] = crc>>8;
		  lanswer[4] = crc&0xFF;
		  send_udp_data_loader(upcb, addr, port, 5);
		  break;
	  case 0x14:	// перезапуск загрузчика после программирования
		  send_reset_bootloder(data[3],data[4]);
		  send_reset_bootloder(data[3],data[4]);
		  send_reset_bootloder(data[3],data[4]);
		  lanswer[0] = data[0];	// id high
		  lanswer[1] = data[1];	// id low
		  lanswer[2] = 0x14;		// cmd
		  crc = GetCRC16((unsigned char*)lanswer,3);
		  lanswer[3] = crc>>8;
		  lanswer[4] = crc&0xFF;
		  send_udp_data_loader(upcb, addr, port, 5);
		  break;
	  }
	}
	pbuf_free(p);
	p = NULL;
}

// функция обратного для диспетчера (порт UDP_SERVER_PORT)
void udp_server_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
	unsigned char *data;
	unsigned short crc;

	unsigned short length = 0;
	unsigned short offset = 0;
	unsigned short answer_offset = 0;
	unsigned short i=0;
	static unsigned char pckt_cnt = 0;
	static unsigned char pckt_length[20];

	data = (unsigned char*)(p->payload);
	crc = GetCRC16(data,p->len);
	if(crc==0)
	{
	  switch(data[2]){
		  case 0xA0:	// идентификатор типа устройства
			  answer[0] = data[0];
			  answer[1] = data[1];
			  answer[2] = 0xA0;
			  answer[3] = 0x01;	// type of device identificator
			  answer[4] = 0xEF;
			  answer[5] = 0x35;
			  answer[6] = 0x7A;
			  crc = GetCRC16((unsigned char*)answer,7);
			  answer[7]=crc>>8;
			  answer[8]=crc&0xFF;
			  send_udp_data(upcb, addr, port,9);
			  break;
		  case 0xD0:	// чтение модбас входных регистров и дискретных входов
			  answer[0] = data[0];	// id high
			  answer[1] = data[1];	// id low
			  answer[2] = 0xD0;		// cmd
			  for(i=0;i<116;i++) {
				  answer[3+i*2] = inpReg[i]>>8;
				  answer[3+i*2+1] = inpReg[i]&0xFF;
			  }
			  for(i=0;i<DiscreteInputsLimit/8;i++) {
				  answer[235+i] = modbus_di_array[i];
			  }
			  crc = GetCRC16((unsigned char*)answer,235+128);
			  answer[363]=crc>>8;
			  answer[364]=crc&0xFF;
			  send_udp_data(upcb, addr, port,365);
			  break;
		  case 0xD1:	// команда на проверку микрофона/динамиков точек
			  answer[0] = data[0];	// id high
			  answer[1] = data[1];	// id low
			  answer[2] = 0xD0;		// cmd
			  crc = GetCRC16((unsigned char*)answer,3);
			  answer[3]=crc>>8;
			  answer[4]=crc&0xFF;
			  send_udp_data(upcb, addr, port,5);
			  send_scan_cmd_from_pc();
			  break;
		  case 0xEF:	// пересброс шлюза
			  SCB->AIRCR = 0x05FA0004;
			  break;
		  case 0x01:	// аудио данные от диспетчера в режиме передачи
			  answer[0] = data[0];	// id high
			  answer[1] = data[1];	// id low
			  answer[2] = 0x01;		// cmd

			  destination_group = data[3];
			  destination_point = data[4];
			  if(destination_point==0xFF) cur_mode = MODE_PC_TO_ALL;
			  else if(destination_point==0x00) {cur_mode = MODE_PC_TO_GROUP;}
			  else cur_mode = MODE_PC_TO_POINT;

			  answer[3] = rx_group;		// group from
			  answer[4] = rx_point;  	// point from

			  pckt_cnt = data[5];if(pckt_cnt>=20) pckt_cnt=0;
			  answer[5] = pckt_cnt;
			  for(i=0;i<pckt_cnt;i++) pckt_length[i] = data[6+i];
			  offset = 6 + pckt_cnt;
			  answer_offset = 6 + pckt_cnt;

			  for(i=0;i<pckt_cnt;i++) {
				  divide_to_packets_and_send_to_can(destination_group, destination_point, pckt_length[i], &data[offset]);
				  offset+=pckt_length[i];
				  length=0;
				  answer[6+i] = length;
				  if(length) { answer_offset+=length;}
			  }
			  crc = GetCRC16((unsigned char*)answer,answer_offset);
			  answer[answer_offset++] = crc>>8;
			  answer[answer_offset++] = crc&0xFF;
			  send_udp_data(upcb, addr, port,answer_offset);
			  break;
		  case 0x02:	// запрос аудио дааных у шлюза диспетчером в режиме прослушивания
			  answer[0] = data[0];	// id high
			  answer[1] = data[1];	// id low
			  if(call_flag) answer[2] = 0x82;		// cmd
			  else answer[2] = 0x02;				// cmd

			  destination_group = data[3];
			  destination_point = data[4];
			  if(destination_point==0xFF) cur_mode = MODE_PC_TO_ALL;
			  else if(destination_point==0x00) cur_mode = MODE_PC_TO_GROUP;
			  else cur_mode = MODE_PC_TO_POINT;

			  answer[3] = rx_group;		// group from
			  answer[4] = rx_point;  	// point from

			  pckt_cnt = data[5];if(pckt_cnt>=20) pckt_cnt=0;
			  answer[5] = pckt_cnt;
			  for(i=0;i<pckt_cnt;i++) pckt_length[i] = data[6+i];
			  offset = 6 + pckt_cnt;
			  answer_offset = 6 + pckt_cnt;

			  for(i=0;i<pckt_cnt;i++) {
				  offset+=pckt_length[i];
				  length=get_can_frame((unsigned char*)&answer[answer_offset]);
				  answer[6+i] = length;
				  if(length) {answer_offset+=length;}
			  }

			  crc = GetCRC16((unsigned char*)answer,answer_offset);
			  answer[answer_offset++] = crc>>8;
			  answer[answer_offset++] = crc&0xFF;
			  send_udp_data(upcb, addr, port,answer_offset);
			  break;
		  case 0x03:	// запрос данных по обнаруженным шлюзам
			  answer[0] = data[0];	// id high
			  answer[1] = data[1];	// id low
			  answer[2] = 0x03;		// cmd
			  length =3 + write_group_data_to_buf((unsigned char*)&answer[3]);
			  crc = GetCRC16((unsigned char*)answer,length);
			  answer[length++] = crc>>8;
			  answer[length++] = crc&0xFF;
			  send_udp_data(upcb, addr, port, length);
			  break;
		  case 0x04:	// запрос данных по обнаруженным точкам
			  answer[0] = data[0];	// id high
			  answer[1] = data[1];	// id low
			  answer[2] = 0x04;		// cmd
			  answer[3] = data[3]; // part_num
			  length = 4 + write_point_data_to_buf(data[3],(unsigned char*)&answer[4]);
			  crc = GetCRC16((unsigned char*)answer,length);
			  answer[length++] = crc>>8;
			  answer[length++] = crc&0xFF;
			  send_udp_data(upcb, addr, port, length);
			  break;
		  case 0x05:	// настройка громкости точки
			  answer[0] = data[0];	// id high
			  answer[1] = data[1];	// id low
			  answer[2] = 0x05;		// cmd
			  crc = GetCRC16((unsigned char*)answer,3);
			  answer[3] = crc>>8;
			  answer[4] = crc&0xFF;
			  send_udp_data(upcb, addr, port, 5);
			  if(data[5]<4) {
				  send_set_volume(data[3],data[4],data[5]);
				  send_set_volume(data[3],data[4],data[5]);
				  send_set_volume(data[3],data[4],data[5]);
			  }

			  break;
	  }
	}

	/* Free the p buffer */
	pbuf_free(p);
	p = NULL;
}

void send_udp_data(struct udp_pcb *upcb,const ip_addr_t *addr,u16_t port,u16_t length) {
	struct pbuf *p_answer;
	udp_connect(upcb, addr, port);
	p_answer = pbuf_alloc(PBUF_TRANSPORT,length, PBUF_POOL);
	if (p_answer != NULL)
	{
	  pbuf_take(p_answer, answer, length);
	  udp_send(upcb, p_answer);
	  pbuf_free(p_answer);
	  p_answer=NULL;
	}
	udp_disconnect(upcb);
}

void send_udp_data_loader(struct udp_pcb *upcb,const ip_addr_t *addr,u16_t port,u16_t length) {
	struct pbuf *p_answer;
	udp_connect(upcb, addr, port);
	p_answer = pbuf_alloc(PBUF_TRANSPORT,length, PBUF_POOL);
	if (p_answer != NULL)
	{
	  pbuf_take(p_answer, lanswer, length);
	  udp_send(upcb, p_answer);
	  pbuf_free(p_answer);
	  p_answer=NULL;
	}
	udp_disconnect(upcb);
}
