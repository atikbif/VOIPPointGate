#include "modbus.h"
#include "crc.h"
#include "button_led.h"
#include "can_cmd.h"
#include "eeprom.h"

#define ReadCoils               0x01
#define ReadDiscreteInputs      0x02
#define ReadHoldingRegisters    0x03
#define ReadInputRegisters      0x04
#define WriteSingleCoil         0x05
#define WriteSingleRegister     0x06
#define WriteMultipleCoils      0x0F
#define WriteMultipleRegisters  0x10
#define ReportSlaveID           0x11


unsigned short inpReg[InputRegistersLimit]={0};
unsigned short holdReg[HoldingRegistersLimit]={0};
unsigned char coils[CoilsLimit];
unsigned char discrInp[DiscreteInputsLimit];

// назначение входов/выходов шлюза
// дискретный вход 1 Ц разрешение с предыдущего конвейера;
// дискретный вход 2 Ц запуск предстартовой сигнализации;
// дискретный вход 3 Ц подтверждение запуска конвейера;
// дискретный выход 1 Ц прохождение предстартовой сигнализации;
// дискретный выход 2 Ц готовность к запуску;

// INPUT REGISTERS
// 0 - число реально подключенных точек
// 1 - состо€ние шлюза по алгоритму
// состо€ние питани€ точек в цепочке подключенной непосредственно к шлюзу
// начина€ с 16 в старшем байте напр€жение аккумул€тора точки, в младшем напр€жение питание (16 - точка 1, 17 - точка 2)

// COILS не используютс€

// DISCRETE INPUTS
// 0 - вход 1 вкл
// 1 - вход 1 обрыв
// 2 - вход 1 кз
// 3 - вход 2 вкл
// 4 - вход 2 обрыв
// 5 - вход 2 кз
// 6 - вход 3 вкл
// 7 - вход 3 обрыв
// 8 - вход 3 кз
// 9 - выход 1
// 10 - выход 2
// 11 - концевик

// начина€ с 16 по 11 битов на точку непосредственно подключенную к шлюзу (до 64 точек)
// 16 - результат проверки микрофона точки 1
// 17 - вход 1 точки 1 вкл
// 18 - вход 1 точки 1 обрыв
// 19 - вход 1 точки 1 кз
// 20 - вход 2 точки 1 вкл
// 21 - вход 2 точки 1 обрыв
// 22 - вход 2 точки 1 кз
// 23 - выход 1 точки 1
// 24 - выход 2 точки 1
// 25 - факт проверки микрофона в точке 1
// 26 - концевик

// HOLDING REGISTERS
// [0..3] ip дрес
// [4..7] ip маска
// [8..11] ip шлюз
// 12 - число предполагаемых к подключению точек
// 13 - сетевой адрес шлюза



extern uint16_t VirtAddVarTab[NB_OF_VAR];

extern void write_data(buf* data);      //  send data to uart
unsigned char typeOfDevice = 0x01;

static modbusCmd cmd;

modbusCmd* searchCmd(buf* rx_data)
{
    unsigned short tmp;
    cmd.isCmd = 0;
    switch(rx_data->ptr[1])
    {
    case ReadCoils:
        if(rx_data->cnt==8)
        {
            cmd.netAddr = rx_data->ptr[0];
            cmd.code = ReadCoils;
            cmd.memAddr = (((unsigned short)rx_data->ptr[2]) << 8) | rx_data->ptr[3];
            cmd.cnt = (((unsigned short)rx_data->ptr[4]) << 8) | rx_data->ptr[5];
            cmd.isCmd = 1;
            cmd.err = 0;
            if(cmd.cnt > CoilsLimit) cmd.err = 0x03;
            else
            {
                if(cmd.memAddr + cmd.cnt > CoilsLimit) cmd.err = 0x02;
            }
        }
        break;
    case ReadDiscreteInputs:
        if(rx_data->cnt==8)
        {
            cmd.netAddr = rx_data->ptr[0];
            cmd.code = ReadDiscreteInputs;
            cmd.memAddr = (((unsigned short)rx_data->ptr[2]) << 8) | rx_data->ptr[3];
            cmd.cnt = (((unsigned short)rx_data->ptr[4]) << 8) | rx_data->ptr[5];
            cmd.isCmd = 1;
            cmd.err = 0;
            if(cmd.cnt > DiscreteInputsLimit) cmd.err = 0x03;
            else
            {
                if(cmd.memAddr + cmd.cnt > DiscreteInputsLimit) cmd.err = 0x02;
            }
        }
        break;
    case ReadHoldingRegisters:
        if(rx_data->cnt==8)
        {
            cmd.netAddr = rx_data->ptr[0];
            cmd.code = ReadHoldingRegisters;
            cmd.memAddr = (((unsigned short)rx_data->ptr[2]) << 8) | rx_data->ptr[3];
            cmd.cnt = (((unsigned short)rx_data->ptr[4]) << 8) | rx_data->ptr[5];
            cmd.isCmd = 1;
            cmd.err = 0;
            if(cmd.cnt > HoldingRegistersLimit) cmd.err = 0x03;
            else
            {
				if(cmd.memAddr + cmd.cnt > HoldingRegistersLimit) cmd.err = 0x02;
            }
        }
        break;
    case ReadInputRegisters:
        if(rx_data->cnt==8)
        {
            cmd.netAddr = rx_data->ptr[0];
            cmd.code = ReadInputRegisters;
            cmd.memAddr = (((unsigned short)rx_data->ptr[2]) << 8) | rx_data->ptr[3];
            cmd.cnt = (((unsigned short)rx_data->ptr[4]) << 8) | rx_data->ptr[5];
            cmd.isCmd = 1;
            cmd.err = 0;
            if(cmd.cnt > InputRegistersLimit) cmd.err = 0x03;
            else
            {
                if(cmd.memAddr + cmd.cnt > InputRegistersLimit) cmd.err = 0x02;
            }
        }
        break;
    case WriteSingleCoil:
        if(rx_data->cnt==8)
        {
            cmd.netAddr = rx_data->ptr[0];
            cmd.code = WriteSingleCoil;
            cmd.memAddr = (((unsigned short)rx_data->ptr[2]) << 8) | rx_data->ptr[3];
            cmd.cnt = (((unsigned short)rx_data->ptr[4]) << 8) | rx_data->ptr[5];
            cmd.isCmd = 1;
            cmd.err = 0;
            if((cmd.cnt != 0x00)&&(cmd.cnt != 0xFF00)) cmd.err = 0x03;
            else
            {
                if(cmd.memAddr >= CoilsLimit) cmd.err = 0x02;
            }
        }
        break;
    case WriteSingleRegister:
        if(rx_data->cnt==8)
        {
            cmd.netAddr = rx_data->ptr[0];
            cmd.code = WriteSingleRegister;
            cmd.memAddr = (((unsigned short)rx_data->ptr[2]) << 8) | rx_data->ptr[3];
            cmd.cnt = (((unsigned short)rx_data->ptr[4]) << 8) | rx_data->ptr[5];
            cmd.isCmd = 1;
            cmd.err = 0;
			if(cmd.memAddr >= HoldingRegistersLimit) cmd.err = 0x02;
        }
        break;
    case WriteMultipleCoils:
        cmd.cnt = (((unsigned short)rx_data->ptr[4]) << 8) | rx_data->ptr[5];
        tmp = (cmd.cnt%8)?cmd.cnt/8+1:cmd.cnt/8;
        if(rx_data->cnt == 9 + tmp)
        {
            cmd.netAddr = rx_data->ptr[0];
            cmd.code = WriteMultipleCoils;
            cmd.memAddr = (((unsigned short)rx_data->ptr[2]) << 8) | rx_data->ptr[3];
            cmd.isCmd = 1;
            cmd.err = 0;
            cmd.ptr = &rx_data->ptr[7];
            if(cmd.cnt > CoilsLimit) cmd.err = 0x03;
            else
            {
                if(cmd.memAddr + cmd.cnt > CoilsLimit) cmd.err = 0x02;
            }
        }
        break;
    case WriteMultipleRegisters:
        cmd.cnt = (((unsigned short)rx_data->ptr[4]) << 8) | rx_data->ptr[5];
        if(rx_data->cnt == 9 + cmd.cnt*2)
        {
            cmd.netAddr = rx_data->ptr[0];
            cmd.code = WriteMultipleRegisters;
            cmd.memAddr = (((unsigned short)rx_data->ptr[2]) << 8) | rx_data->ptr[3];
            cmd.isCmd = 1;
            cmd.err = 0;
            cmd.ptr = &rx_data->ptr[7];
			if(cmd.cnt > HoldingRegistersLimit) cmd.err = 0x03;
            else
			{
				if(cmd.memAddr + cmd.cnt > HoldingRegistersLimit) cmd.err = 0x02;
			}
        }
        break;
    case ReportSlaveID:
        if(rx_data->cnt == 4)
        cmd.netAddr = rx_data->ptr[0];
        cmd.code = ReportSlaveID;
        cmd.isCmd = 1;
        cmd.err = 0;
        break;
    }
    return &cmd;
}

void sendAnswer(modbusCmd* cmd, buf* tx_data)
{
    unsigned short tmp;
    if(cmd->isCmd)
    {
        if(cmd->err)
        {
            if(tx_data->cnt >= 5)
            {
                tx_data->ptr[0] = cmd->netAddr;
                tx_data->ptr[1] = cmd->code | 0x80;
                tx_data->ptr[2] = cmd->err;
                tmp=GetCRC16(tx_data->ptr,3);
                tx_data->ptr[3]=tmp>>8;
                tx_data->ptr[4]=tmp&0xFF;
                tx_data->cnt = 5;
                write_data(tx_data);
            }
        }
        else
        {
        	inpReg[InputRegistersLimit-1]++;
            switch(cmd->code)
            {
            case ReadCoils:
                tmp = (cmd->cnt % 8)?cmd->cnt/8+1:cmd->cnt/8;
                if(tx_data->cnt >= 5 + tmp)
                {
                    tx_data->ptr[0] = cmd->netAddr;
                    tx_data->ptr[1] = cmd->code;
                    tx_data->ptr[2] = tmp;
                    for(tmp=0;tmp<tx_data->ptr[2];tmp++) tx_data->ptr[3+tmp] = 0;
                    for(tmp=0;tmp<cmd->cnt;tmp++)
                    {
                        if(coils[cmd->memAddr + tmp]) tx_data->ptr[3+tmp/8] |= 1<<(tmp%8);
                    }
                    tmp=GetCRC16(tx_data->ptr,3 + tx_data->ptr[2]);
                    tx_data->ptr[3 + tx_data->ptr[2]]=tmp>>8;
                    tx_data->ptr[4 + tx_data->ptr[2]]=tmp&0xFF;
                    tx_data->cnt = 5 + tx_data->ptr[2];
                    write_data(tx_data);
                }
                break;
            case ReadDiscreteInputs:
                tmp = (cmd->cnt % 8)?cmd->cnt/8+1:cmd->cnt/8;
                if(tx_data->cnt >= 5 + tmp)
                {
                    tx_data->ptr[0] = cmd->netAddr;
                    tx_data->ptr[1] = cmd->code;
                    tx_data->ptr[2] = tmp;
                    for(tmp=0;tmp<tx_data->ptr[2];tmp++) tx_data->ptr[3+tmp] = 0;
                    for(tmp=0;tmp<cmd->cnt;tmp++)
                    {
                        if(discrInp[cmd->memAddr + tmp]) tx_data->ptr[3+tmp/8] |= 1<<(tmp%8);
                    }
                    tmp=GetCRC16(tx_data->ptr,3 + tx_data->ptr[2]);
                    tx_data->ptr[3 + tx_data->ptr[2]]=tmp>>8;
                    tx_data->ptr[4 + tx_data->ptr[2]]=tmp&0xFF;
                    tx_data->cnt = 5 + tx_data->ptr[2];
                    write_data(tx_data);
                }
                break;
            case ReadInputRegisters:
                if(tx_data->cnt >= 5 + cmd->cnt*2)
                {
                    tx_data->ptr[0] = cmd->netAddr;
                    tx_data->ptr[1] = cmd->code;
                    tx_data->ptr[2] = cmd->cnt * 2;
                    for(tmp=0;tmp<cmd->cnt;tmp++)
                    {
                        tx_data->ptr[3+tmp*2] = inpReg[cmd->memAddr + tmp] >> 8;
                        tx_data->ptr[4+tmp*2] = inpReg[cmd->memAddr + tmp] & 0xFF;
                    }
                    tmp=GetCRC16(tx_data->ptr,3+cmd->cnt*2);
                    tx_data->ptr[3+cmd->cnt*2]=tmp>>8;
                    tx_data->ptr[4+cmd->cnt*2]=tmp & 0xFF;
                    tx_data->cnt = 5+cmd->cnt*2;
                    write_data(tx_data);
                }
                break;
            case ReadHoldingRegisters:
                if(tx_data->cnt >= 5 + cmd->cnt*2)
                {
                    tx_data->ptr[0] = cmd->netAddr;
                    tx_data->ptr[1] = cmd->code;
                    tx_data->ptr[2] = cmd->cnt * 2;
                    for(tmp=0;tmp<cmd->cnt;tmp++)
                    {
						tx_data->ptr[3+tmp*2] = holdReg[cmd->memAddr + tmp] >> 8;
						tx_data->ptr[4+tmp*2] = holdReg[cmd->memAddr + tmp] & 0xFF;
                    }
                    tmp=GetCRC16(tx_data->ptr,3+cmd->cnt*2);
                    tx_data->ptr[3+cmd->cnt*2]=tmp>>8;
                    tx_data->ptr[4+cmd->cnt*2]=tmp & 0xFF;
                    tx_data->cnt = 5+cmd->cnt*2;
                    write_data(tx_data);
                }
                break;
            case WriteSingleCoil:
                if(tx_data->cnt >= 8)
                {
                    tx_data->ptr[0] = cmd->netAddr;
                    tx_data->ptr[1] = cmd->code;
                    tx_data->ptr[2] = cmd->memAddr >> 8;
                    tx_data->ptr[3] = cmd->memAddr & 0xFF;
                    tx_data->ptr[4] = cmd->cnt >> 8;
                    tx_data->ptr[5] = cmd->cnt & 0xFF;
                    tmp=GetCRC16(tx_data->ptr,6);
                    tx_data->ptr[6]=tmp>>8;
                    tx_data->ptr[7]=tmp & 0xFF;
                    tx_data->cnt = 8;
                    write_data(tx_data);
                    if(cmd->cnt) coils[cmd->memAddr] = 0x01; else coils[cmd->memAddr] = 0x00;
                    /*if(cmd->memAddr>=16) {
                    	manage_relay((cmd->memAddr-16)/2,(cmd->memAddr-16)%2,cmd->cnt?1:0);
                    }*/

                }
                break;
            case WriteSingleRegister:
                if(tx_data->cnt >= 8)
                {
                    tx_data->ptr[0] = cmd->netAddr;
                    tx_data->ptr[1] = cmd->code;
                    tx_data->ptr[2] = cmd->memAddr >> 8;
                    tx_data->ptr[3] = cmd->memAddr & 0xFF;
                    tx_data->ptr[4] = cmd->cnt >> 8;
                    tx_data->ptr[5] = cmd->cnt & 0xFF;
                    tmp=GetCRC16(tx_data->ptr,6);
                    tx_data->ptr[6]=tmp>>8;
                    tx_data->ptr[7]=tmp & 0xFF;
                    tx_data->cnt = 8;
                    write_data(tx_data);
                    holdReg[cmd->memAddr] = cmd->cnt;
                    if(cmd->memAddr<NB_OF_VAR-1) {
                    	EE_WriteVariable(VirtAddVarTab[cmd->memAddr+1],holdReg[cmd->memAddr]);
					}
                }
                break;
            case WriteMultipleCoils:
                if(tx_data->cnt >= 8)
                {
                    tx_data->ptr[0] = cmd->netAddr;
                    tx_data->ptr[1] = cmd->code;
                    tx_data->ptr[2] = cmd->memAddr >> 8;
                    tx_data->ptr[3] = cmd->memAddr & 0xFF;
                    tx_data->ptr[4] = cmd->cnt >> 8;
                    tx_data->ptr[5] = cmd->cnt & 0xFF;
                    tmp=GetCRC16(tx_data->ptr,6);
                    tx_data->ptr[6]=tmp>>8;
                    tx_data->ptr[7]=tmp & 0xFF;
                    tx_data->cnt = 8;
                    write_data(tx_data);
                    for(tmp=0;tmp<cmd->cnt;tmp++)
                    {
                        if(cmd->ptr[tmp/8] & (1<<(tmp%8))) coils[cmd->memAddr + tmp] = 0x01;
                        else coils[cmd->memAddr + tmp] = 0x00;
                    }
                }
                break;
            case WriteMultipleRegisters:
                if(tx_data->cnt >= 8)
                {
                    tx_data->ptr[0] = cmd->netAddr;
                    tx_data->ptr[1] = cmd->code;
                    tx_data->ptr[2] = cmd->memAddr >> 8;
                    tx_data->ptr[3] = cmd->memAddr & 0xFF;
                    tx_data->ptr[4] = cmd->cnt >> 8;
                    tx_data->ptr[5] = cmd->cnt & 0xFF;
                    tmp=GetCRC16(tx_data->ptr,6);
                    tx_data->ptr[6]=tmp>>8;
                    tx_data->ptr[7]=tmp & 0xFF;
                    tx_data->cnt = 8;
                    write_data(tx_data);
                    for(tmp=0;tmp<cmd->cnt;tmp++)
                    {
						holdReg[cmd->memAddr + tmp] = (((unsigned short)cmd->ptr[tmp*2])<<8) | cmd->ptr[tmp*2+1];
						if(cmd->memAddr+tmp<NB_OF_VAR-1) {
							EE_WriteVariable(VirtAddVarTab[cmd->memAddr+1+tmp],holdReg[cmd->memAddr+tmp]);
						}
                    }
                }
                break;
            case ReportSlaveID:
                if(tx_data->cnt >= 11)
                {
                    tx_data->ptr[0] = cmd->netAddr;
                    tx_data->ptr[1] = cmd->code;
                    tx_data->ptr[2] = 6;     // byte count
                    tx_data->ptr[3] = typeOfDevice;     // slave id
                    tx_data->ptr[4] = 0xFF; // run indicator status
                    // additional data
                    tx_data->ptr[5] = 14;   // code word high byte
                    tx_data->ptr[6] = 10;   // code word low byte
                    tx_data->ptr[7] = 0x02; // version high byte
                    tx_data->ptr[8] = 0x00; // version low byte
                    tmp=GetCRC16(tx_data->ptr,9);
                    tx_data->ptr[9]=tmp>>8;
                    tx_data->ptr[10]=tmp & 0xFF;
                    tx_data->cnt = 11;
                    write_data(tx_data);
                }
                break;
            }
        }
    }
}
