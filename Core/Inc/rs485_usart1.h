/*
 * rs485_usart3.h
 *
 *  Created on: 2023年4月10日
 *      Author: 陈骏骏
 */

#ifndef INC_RS485_USART3_H_
#define INC_RS485_USART3_H_

#include "main.h"

//设备地址（1个字节），功能码（1个字节），数据（最多20字节），CRC校验（2个字节）
#define RS485_MAXSIZE 24
//读取气体流量计的次数
#define MAX_READ_FLOW_NUM 10

extern uint8_t pRS485RxBuff[RS485_MAXSIZE];

void read_flow(UART_HandleTypeDef *huart);
void RS485_Data_Process(uint8_t rxlen);
void Send_RS485_Data(UART_HandleTypeDef *huart);
void StartUp_Flow_Calculation();

#endif /* INC_RS485_USART3_H_ */
