//
// Created by 陈骏骏 on 2023/5/13.
//
#include <string.h>
#include "LIN_usart2.h"
#include "CH455.h"
#include "rs485_usart1.h"

//LIN校验模式
typedef enum
{
    //标准校验
    LIN_CK_STANDARD = 0x00U,
    //增强校验
    LIN_CK_ENHANCED = 0x01U
}LIN_CK_Mode;

//LIN_PID+P0+P1,LSB,PID列表
typedef enum
{
    LIN_PID_00_0x00 = 0x80,
    LIN_PID_01_0x01 = 0xC1,
    LIN_PID_02_0x02 = 0x42,
    LIN_PID_03_0x03 = 0x03,
    LIN_PID_04_0x04 = 0xC4,
    LIN_PID_05_0x05 = 0x85,
    LIN_PID_06_0x06 = 0x06,
    LIN_PID_07_0x07 = 0x47,
    LIN_PID_08_0x08 = 0x08,
    LIN_PID_09_0x09 = 0x49,
    LIN_PID_10_0x0A = 0xCA,
    LIN_PID_11_0x0B = 0x8B,
    LIN_PID_12_0x0C = 0x4C,
    LIN_PID_13_0x0D = 0x0D,
    LIN_PID_14_0x0E = 0x8E,
    LIN_PID_15_0x0F = 0xCF,
    LIN_PID_16_0x10 = 0x50,
    LIN_PID_17_0x11 = 0x11,
    LIN_PID_18_0x12 = 0x92,
    LIN_PID_19_0x13 = 0xD3,
    LIN_PID_20_0x14 = 0x14,
    LIN_PID_21_0x15 = 0x55,
    LIN_PID_22_0x16 = 0xD6,
    LIN_PID_23_0x17 = 0x97,
    LIN_PID_24_0x18 = 0xD8,
    LIN_PID_25_0x19 = 0x99,
    LIN_PID_26_0x1A = 0x1A,
    LIN_PID_27_0x1B = 0x5B,
    LIN_PID_28_0x1C = 0x9C,
    LIN_PID_29_0x1D = 0xDD,
    LIN_PID_30_0x1E = 0x5E,
    LIN_PID_31_0x1F = 0x1F,
    LIN_PID_32_0x20 = 0x20,
    LIN_PID_33_0x21 = 0x61,
    LIN_PID_34_0x22 = 0xE2,
    LIN_PID_35_0x23 = 0xA3,
    LIN_PID_36_0x24 = 0x64,
    LIN_PID_37_0x25 = 0x25,
    LIN_PID_38_0x26 = 0xA6,
    LIN_PID_39_0x27 = 0xE7,
    LIN_PID_40_0x28 = 0xA8,
    LIN_PID_41_0x29 = 0xE9,
    LIN_PID_42_0x2A = 0x6A,
    LIN_PID_43_0x2B = 0x2B,
    LIN_PID_44_0x2C = 0xEC,
    LIN_PID_45_0x2D = 0xAD,
    LIN_PID_46_0x2E = 0x2E,
    LIN_PID_47_0x2F = 0x6F,
    LIN_PID_48_0x30 = 0xF0,
    LIN_PID_49_0x31 = 0xB1,
    LIN_PID_50_0x32 = 0x32,
    LIN_PID_51_0x33 = 0x73,
    LIN_PID_52_0x34 = 0xB4,
    LIN_PID_53_0x35 = 0xF5,
    LIN_PID_54_0x36 = 0x76,
    LIN_PID_55_0x37 = 0x37,
    LIN_PID_56_0x38 = 0x78,
    LIN_PID_57_0x39 = 0x39,
    LIN_PID_58_0x3A = 0xBA,
    LIN_PID_59_0x3B = 0xFB,
    LIN_PID_60_0x3C = 0x3C,
    LIN_PID_61_0x3D = 0x7D,
    LIN_PID_62_0x3E = 0xFE,
    LIN_PID_63_0x3F = 0xBF
} LIN_PID_List;

//LIN通信故障反馈
typedef enum
{
    EXV_F_RESP_NO_ERROR = 0x00,
    EXV_F_RESP_ERROR = 0x01
}EXV_F_Response;

//初始化状态反馈
typedef enum
{
    EXV_ST_INIT_NOT = 0x00,
    EXV_ST_INIT_PROCESS = 0x04,
    EXV_ST_INIT_SUCCESS = 0x08
}EXV_St_Initial;

//运行状态反馈
typedef enum
{
    EXV_ST_RUN_NOT_MOVE = 0x00,
    EXV_ST_RUN_MOVING = 0x10
}EXV_St_Running;

//故障状态
typedef enum
{
    EXV_ST_FAULT_NOT = 0x00,
    EXV_ST_FAULT_SHORTED = 0x01,
    EXV_ST_FAULT_OPENLOAD = 0x02,
    EXV_ST_FAULT_OVERTEMP = 0x03,
    EXV_ST_FAULT_ACTUATORFAULT = 0x05
}EXV_St_Fault;

//电压状态
typedef enum
{
    EXV_ST_VOLTAGE_OK = 0x00,
    EXV_ST_VOLTAGE_OVER = 0x10,
    EXV_ST_VOLTAGE_UNDER = 0x20
}EXV_St_Voltage;

//温度状态
typedef enum
{
    EXV_OVERTEMP_OK = 0x00,
    EXV_OVERTEMP_OVER = 0x40
}EXV_W_OverTemp;

//电机信息比较值
typedef enum
{
    EXV_F_RESP_COMP = 0x01,
    EXV_ST_INIT_COMP = 0x0C,
    EXV_ST_RUN_COMP = 0x10,
    EXV_ST_FAULT_COMP = 0x0F,
    EXV_ST_VOLTAGE_COMP = 0x30,
    EXV_OVERTEMP_COMP = 0xC0
}EXV_St_Comp;

//LIN同步帧字节
uint8_t  SYNC_Frame = 0x55;

//LIN接收数据缓存
uint8_t pLINRxBuff[LIN_RX_MAXSIZE];
//LIN发送数据缓存
uint8_t pLINTxBuff[LIN_TX_MAXSIZE];
//当前测试的电机步长
uint16_t EXV_Test_Step;
//当前测试的循环次数
uint16_t EXV_Test_Cycles;
//发送读取帧的标志位
uint8_t LIN_Read_Flag = DISABLE;
//发送写帧的标志位
uint8_t LIN_Send_Flag = DISABLE;
//指令重复发送计数器
uint8_t retries = 0;
//保存LIN芯片的信息
struct LIN_Chip_Msg
{
    //读PID
    uint8_t read_PID;
    //写PID
    uint8_t write_PID;
    //电机运动使能
    uint8_t EXV_Move_Enable;
    //初始化请求
    uint8_t EXV_Init_Request;
    //非初始化请求
    uint8_t EXV_Not_Init_Request;
};
//初始化LIN芯片信息
struct LIN_Chip_Msg chip[3] = {
        {LIN_PID_53_0x35,LIN_PID_52_0x34,0xFF,0xFD,0xFC},
        {LIN_PID_55_0x37,LIN_PID_54_0x36,0xFF,0xFD,0xFC},
        {LIN_PID_32_0x20,LIN_PID_16_0x10,0xFF,0xFD,0xFC}
};
//默认芯片编号
uint8_t chip_Num = 0;

/****************************************************************************************
** 函数名称: LINCheckSum----标准校验
** 功能描述: 计算并返回LIN校验值
** 参    数:  uint8_t *buf：需要计算的数组
			        uint8_t lens：数组长度
** 返 回 值:   uint8_t ckm: 计算结果
****************************************************************************************/
uint8_t LIN_Check_Sum(uint8_t *buf, uint8_t lens)
{
    uint8_t i, ckm = 0;
    uint16_t chm1 = 0;
    for(i = 1; i < lens; i++)
    {
        chm1 += *(buf+i);
    }
    ckm = chm1 / 256;
    ckm = ckm + chm1 % 256;
    ckm = 0xFF - ckm;
    return ckm;
}
/****************************************************************************************
** 函数名称: LINCheckSumEn----增强校验
** 功能描述: 计算并返回LIN校验值
** 参    数:  uint8_t *buf：需要计算的数组
			        uint8_t lens：数组长度
** 返 回 值:   uint8_t ckm: 计算结果
****************************************************************************************/
uint8_t LIN_Check_Sum_En(uint8_t *buf, uint8_t lens)
{
    uint8_t i, ckm = 0;
    uint16_t chm1 = 0;
    for(i = 0; i < lens; i++)
    {
        chm1 += *(buf+i);
    }
    ckm = ~(chm1 % 255);
    return ckm;
}
/****************************************************************************************
** 函数名称: Lin_Tx_PID_Data
** 功能描述: LIN发送数据帧
** 参    数: *buf:数组地址；buf[0]=PID
			       lens:数据长度,不含校验字节
			       CK_Mode: 校验类型增强型LIN_CK_ENHANCED=1：基本LIN_CK_STANDARD=0
             Timeout (0xffff)不做时间限制
** 返 回 值: 无
****************************************************************************************/
void LIN_Tx_PID_Data(UART_HandleTypeDef *huart, uint8_t *buf, uint8_t lens, LIN_CK_Mode CK_Mode)
{
    if(CK_Mode == LIN_CK_STANDARD)
    {
        //arr[i] = *(arr + i)
        //计算标准型校验码，不计算PID
        *(buf + lens) = LIN_Check_Sum(buf, LIN_CHECK_STD_NUM);
    }
    else
    {
        //计算增强型校验码,连PID一起校验
        *(buf + lens) = LIN_Check_Sum_En(buf, LIN_CHECK_EN_NUM);
    }

    //发送同步间隔段
    HAL_LIN_SendBreak(huart);
    //发送同步段
    HAL_UART_Transmit(huart,&SYNC_Frame,1,HAL_MAX_DELAY);
    //发送PID,数据内容和校验
    HAL_UART_Transmit(huart,buf,LIN_TX_MAXSIZE,HAL_MAX_DELAY);
}
/****************************************************************************************
** 函数名称: Lin_Tx_PID
** 功能描述: LIN发送报文头，PID，读取从机状态信息
** 参    数: PID, Timeout (0xffff)不做时间限制
** 返 回 值: 无
****************************************************************************************/
void LIN_Tx_PID(UART_HandleTypeDef *huart, uint8_t PID)
{
    //发送间隔帧
    HAL_LIN_SendBreak(huart);
    //发送同步帧
    HAL_UART_Transmit(huart,&SYNC_Frame,1,HAL_MAX_DELAY);
    HAL_UART_Transmit(huart,&PID,1,HAL_MAX_DELAY);
}

/**
 * 拼接LIN通信数据格式
 * @param step
 * @param cycles
 * @param init_enable
 */
void Data_To_LIN(uint16_t step,uint16_t cycles,uint8_t init_enable)
{


    uint8_t index = 0;
    EXV_Test_Step = step;
    EXV_Test_Cycles = cycles;

    pLINTxBuff[index++] = chip[chip_Num].write_PID;
    pLINTxBuff[index++] = step & 0xFF;
    pLINTxBuff[index++] = step >> 8;
    pLINTxBuff[index++] = chip[chip_Num].EXV_Move_Enable;
    if(init_enable)
    {
        pLINTxBuff[index++] = chip[chip_Num].EXV_Init_Request;
    }
    else
    {
        pLINTxBuff[index++] = chip[chip_Num].EXV_Not_Init_Request;
    }
    LIN_Send_Flag = ENABLE;
}

/**
 * 按下结束按钮执行的方法，不读取数据，不发送数据
 * @param send
 * @param read
 */
void Finished_LIN(uint8_t send,uint8_t read)
{
    LIN_Send_Flag = send;
    LIN_Read_Flag = read;
}

/**
 * 发送LIN数据，包括读取帧和写帧
 */
void Send_LIN_Data(UART_HandleTypeDef *huart)
{
    if(LIN_Send_Flag)
    {
        LIN_Tx_PID_Data(huart,pLINTxBuff,LIN_TX_MAXSIZE - 1,LIN_CK_ENHANCED);
        LIN_Send_Flag = DISABLE;
        LIN_Read_Flag = ENABLE;
        HAL_Delay(20);
    }
    if(LIN_Read_Flag)
    {
        LIN_Tx_PID(huart, chip[chip_Num].read_PID);
        HAL_Delay(100);
    }
}

/**
 * 反转发送的指令
 */
void Reverse_Instruction(uint16_t cycles,uint16_t test_step,uint16_t reset_step)
{
    if (cycles)
    {
        int step = 0;
        if (test_step)
        {
            step = 0;
            cycles--;
        }
        else
        {
            step = reset_step;
        }
        Data_To_LIN(step,cycles,0);
    }
}

/**
 * 反馈电机信号
 * @param signal
 */
void Feedback_Signal(uint8_t signal)
{
    //读取标志位置为不发送读取数据帧
    LIN_Read_Flag = DISABLE;
    //发送标志置为不发送写数据帧
    LIN_Send_Flag = DISABLE;
    //重置重试计数器为0
    retries = 0;
    //发送响应数据后表示本次测试结束，清空发送数据缓存
    memset(pLINTxBuff,0,LIN_TX_MAXSIZE);

    if (signal)
    {
        //启动流量计算
        StartUp_Flow_Calculation();
        //反转发送的指令
        Reverse_Instruction(EXV_Test_Cycles,EXV_Test_Step,currentStepSize);
    }
    else
    {
        //不亮绿灯
        HAL_GPIO_WritePin(LED_EXV_GPIO_Port,LED_EXV_Pin,GPIO_PIN_RESET);
    }
}

/**
 * 数据处理函数
 */
void LIN_Data_Process()
{
    //电机转动步长
    uint16_t EXV_Run_Step = 0;
    //通过校验位-校验数据
    uint8_t ckm = 0;
    //pLINRxBuff + 2表示从接收的第3个数据开始，因为接收数组第1个是同步间隔段，第2个是同步段（0x55）
    ckm = LIN_Check_Sum_En(pLINRxBuff + 2,LIN_CHECK_EN_NUM);
    //如果校验不通过，丢弃这帧数据
    if(pLINRxBuff[2] == LIN_PID_52_0x34 || ckm != pLINRxBuff[LIN_RX_MAXSIZE - 1])
    {
        return;
    }
    //解析数据具有优先级：LIN通信故障->电机故障->电压异常->温度异常->电机停止标志->判断步长
    //校验LIN通信故障反馈
    if((pLINRxBuff[3] & EXV_F_RESP_COMP) == EXV_F_RESP_ERROR)
    {
        Feedback_Signal(EXV_ERROR);
    }
    //检查初始化状态，解决反馈数据中以E2，E3开始的数据帧
    else if((pLINRxBuff[3] & EXV_ST_INIT_COMP) == EXV_ST_INIT_NOT || (pLINRxBuff[3] & EXV_ST_INIT_COMP) == EXV_ST_INIT_PROCESS)
    {
        return;
    }
    //校验故障状态
    else if((pLINRxBuff[4] & EXV_ST_FAULT_COMP) > 0)
    {
        uint8_t fault_index = pLINRxBuff[4] & EXV_ST_FAULT_COMP;
        switch(fault_index)
        {
            case EXV_ST_FAULT_SHORTED:
                Feedback_Signal(EXV_ERROR);
                break;
            case EXV_ST_FAULT_OPENLOAD:
                Feedback_Signal(EXV_ERROR);
                break;
            case EXV_ST_FAULT_OVERTEMP:
                Feedback_Signal(EXV_ERROR);
                break;
            case EXV_ST_FAULT_ACTUATORFAULT:
                Feedback_Signal(EXV_ERROR);
                break;
        }
    }
    //校验电压状态
    else if((pLINRxBuff[4] & EXV_ST_VOLTAGE_COMP) > 0)
    {
        uint8_t voltage_index = pLINRxBuff[4] & EXV_ST_VOLTAGE_COMP;
        switch(voltage_index)
        {
            case EXV_ST_VOLTAGE_OVER:
                Feedback_Signal(EXV_ERROR);
                break;
            case EXV_ST_VOLTAGE_UNDER:
                Feedback_Signal(EXV_ERROR);
                break;
        }
    }
    //校验温度状态
    else if((pLINRxBuff[4] & EXV_OVERTEMP_COMP) == EXV_OVERTEMP_OVER)
    {
        Feedback_Signal(EXV_ERROR);
    }
    //电机停止转动
    else if((pLINRxBuff[3] & EXV_ST_RUN_COMP) == EXV_ST_RUN_NOT_MOVE)
    {
        //计算电机转动步长，步长低字节在前高字节在后
        EXV_Run_Step = (pLINRxBuff[6] << 8) | pLINRxBuff[5];
        if(EXV_Run_Step == EXV_Test_Step)
        {
            Feedback_Signal(EXV_OK);
        }
        //重试10次发送电机运动使能
        else
        {
            LIN_Send_Flag = ENABLE;
            retries++;
            //当10次电机运动使能后，电机转动步长与测试步长不一致，发送错误信息
            if(retries > MAX_RETRY_NUM)
            {
                Feedback_Signal(EXV_ERROR);
            }
        }
    }
    //这帧数据解析完成，清空接收缓存数据
    memset(pLINRxBuff,0,LIN_RX_MAXSIZE);
}

