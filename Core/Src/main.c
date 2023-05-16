/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "key.h"
#include "CH455.h"
#include "LIN_usart2.h"
#include "rs485_usart1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
    __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
    //启动DMA接收
    HAL_UART_Receive_DMA(&huart1,pRS485RxBuff,RS485_MAXSIZE);
    HAL_UART_Receive_DMA(&huart2,pLINRxBuff,LIN_RX_MAXSIZE);
    //初始化数码管
    CH455G_Init(&hi2c1);
    CH455G_Init(&hi2c2);
    //使能系统运行指示灯
    HAL_GPIO_WritePin(LED_System_GPIO_Port,LED_System_Pin,GPIO_PIN_SET);
    //电机正常指示灯
    HAL_GPIO_WritePin(LED_EXV_GPIO_Port,LED_EXV_Pin,GPIO_PIN_SET);
    //使能TJA1028LIN芯片的EN
    HAL_GPIO_WritePin(TJA1028_EN_GPIO_Port,TJA1028_EN_Pin,GPIO_PIN_SET);
    //使能TJA1028LIN芯片的RSTN
    HAL_GPIO_WritePin(TJA1028_RSTN_GPIO_Port,TJA1028_RSTN_Pin,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    //检测加减按键
    Operation_Key_Scan(Step_Add_GPIO_Port,Step_Add_Pin,1,STEP_DIGITAL_TUBE);
    Operation_Key_Scan(Step_Sub_GPIO_Port,Step_Sub_Pin,0,STEP_DIGITAL_TUBE);
    Operation_Key_Scan(Loop_Add_GPIO_Port,Loop_Add_Pin,1,LOOP_DIGITAL_TUBE);
    Operation_Key_Scan(Loop_Sub_GPIO_Port,Loop_Sub_Pin,0,LOOP_DIGITAL_TUBE);
    //检测初始化按钮
    if (General_Key_Scan(Init_Key_GPIO_Port,Init_Key_Pin))
    {
      Data_To_LIN(0,0,1);
    }
    //检测开始按钮
    if (General_Key_Scan(Start_Key_GPIO_Port,Start_Key_Pin))
    {
      Data_To_LIN(currentStepSize,currentCycleCount,0);
    }
    //检测结束按钮
    if (General_Key_Scan(Finished_Key_GPIO_Port,Finished_Key_Pin))
    {
      Finished_LIN(DISABLE,DISABLE);
    }
    //循环发送LIN数据
    Send_LIN_Data(&huart2);
    //循环发送RS485数据
    Send_RS485_Data(&huart1);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void USAR_UART_IDLECallback(UART_HandleTypeDef *huart,uint8_t rxlen )
{
    //接收气体流量串口
    if(huart == &huart1)
    {
        RS485_Data_Process(rxlen);
        //清除数据长度计数
        rxlen = 0;
        //重新打开DMA接收
        HAL_UART_Receive_DMA(&huart1,pRS485RxBuff,RS485_MAXSIZE);
    }
    //LIN数据串口
    else if (huart == &huart2)
    {
        //LIN数据处理
        LIN_Data_Process();
        //清除数据长度计数
        rxlen = 0;
        //重新打开DMA接收
        HAL_UART_Receive_DMA(&huart2,pLINRxBuff,LIN_RX_MAXSIZE);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
