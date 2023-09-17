#ifndef __MODEBUS_H__
#define __MODEBUS_H__

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

//本机modebus地址
#define MODEBUSSLAVE 0x01

//寄存器数据类型
#define REGISTER_TYPE uint16_t

//寄存器数量
#define REGISTER_COUNT 5

//读寄存器报文长度
#define CTLREADLEN 8

//读取修改寄存器数据
void ModeBusSetRegData_BIT(uint8_t reg, uint8_t bit, bool state);
void ModeBusSetRegData_Reg(uint8_t reg, REGISTER_TYPE value);

bool ModeBusReadRegData_BIT(uint8_t reg, uint8_t bit);
REGISTER_TYPE ModeBusReadRegData_REG(uint8_t reg);

//初始化寄存器数据
void ModeBusInitData(void);

void ModeBusRecData(char* data, uint8_t dataLen);

#endif
