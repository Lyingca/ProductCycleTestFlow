//
// Created by 陈骏骏 on 2023/5/13.
//

#ifndef PRODUCTCYCLETESTFLOW_LIN_USART2_H
#define PRODUCTCYCLETESTFLOW_LIN_USART2_H

#include "main.h"

//一个同步间隔段 + 1个同步字节（0x55） + PID（1个字节） + （数据帧 （8个字节）  +  校验和（1个字节））
#define LIN_RX_MAXSIZE 12
//PID（1个字节） + （数据帧 （8个字节）  +  校验和（1个字节））
#define LIN_TX_MAXSIZE 10
//不算PID，只有8个数据
#define LIN_CHECK_STD_NUM 8
//PID+8个数据
#define LIN_CHECK_EN_NUM 9
//最大重试次数
#define MAX_RETRY_NUM 10
//最大步长
#define MAX_STEP 480
//电机错误信号
#define EXV_ERROR 0
//电机正确信号
#define EXV_OK 1

extern uint8_t pLINRxBuff[LIN_RX_MAXSIZE];

void Data_To_LIN(uint16_t step,uint16_t cycles,uint8_t init_enable);
void Finished_LIN(uint8_t send,uint8_t read);
void Send_LIN_Data(UART_HandleTypeDef *huart);
void LIN_Data_Process(void);

#endif //PRODUCTCYCLETESTFLOW_LIN_USART2_H
