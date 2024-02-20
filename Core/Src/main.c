/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "interrupt.h"
#include "lcd.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern struct keys key[4];
extern uint8_t rxbuffer[30];
extern uint8_t uart_flag;
extern uint8_t rx_data;
extern uint8_t rx_pointer;
void key_pro(void);
void view_pro(void);
void led_control(void);
void rx_pro(void);
uint8_t isparked(uint8_t* car_type , uint8_t* car_data);
uint8_t isempty(void);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	uint8_t view=0;
	uint8_t CNBR=0;
	uint8_t VNBR=0;
	uint8_t IDLE=8;
	float cnbr_price=3.5;
	float	vnbr_price=2.0;
  uint8_t control=0;
  uint8_t car_type[5];
  uint8_t car_data[5];
  uint8_t car_time[13];
typedef struct Park
  {
    uint8_t car_type[5];
    uint8_t car_data[5];
    uint8_t car_time[13];
    bool ifPark;
  }Park;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
Park park[8];
void Park_Init(Park* park);
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
    MX_TIM3_Init();
    MX_TIM17_Init();
    MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
		LCD_Init();
		LCD_Clear(Black);
		LCD_SetBackColor(Black);
		LCD_SetTextColor(White);
    MX_GPIO_Init();
    MX_TIM3_Init();
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
    HAL_UART_Receive_IT(&huart1 , &rx_data , 1);
    Park_Init(park);
    //HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			key_pro();
      view_pro();
      led_control();
      if(uart_flag==1)
      {
        //HAL_UART_Transmit(&huart1 , (uint8_t*)rxbuffer , strlen(rxbuffer),50);
        rx_pro();
        uart_flag=0;
      }
          

      //HAL_UART_Transmit(&huart1,(uint8_t *)"Error",strlen("Error"),50);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void key_pro(void)
{
	if(key[0].single_flag==1)
	{
		key[0].single_flag=0;
    view+=1;
    if(view>1){
      view = 0;
    }
	}
	if(key[1].single_flag==1)
	{
		key[1].single_flag=0;
    if(view==1)
    {
      cnbr_price+=0.5f;
      vnbr_price+=0.5f;
    }
	}
	if(key[2].single_flag==1)
	{
		key[2].single_flag=0;
    if(view==1)
    {
      cnbr_price-=0.5f;
      vnbr_price-=0.5f;
    }
	}
	if(key[3].single_flag==1)
	{
		key[3].single_flag=0;
    control+=1;
    if(control>1)
    {
      control=0;
    }
	}
}

void view_pro(void)
{
	if(view==0)
	{
		LCD_DisplayStringLine(Line1,(uint8_t *)"       Data         ");
		uint8_t text[30];
		sprintf(text , "   CNBR:%d           ",CNBR);
		LCD_DisplayStringLine(Line3,(uint8_t *)text);
		sprintf(text , "   VNBR:%d           ",VNBR);
		LCD_DisplayStringLine(Line5,(uint8_t *)text);
		sprintf(text , "   IDLE:%d           ",IDLE);
		LCD_DisplayStringLine(Line7,(uint8_t *)text);
	}

	if(view==1)
	{
		LCD_DisplayStringLine(Line1,(uint8_t *)"       Para         ");
		uint8_t text[30];
		sprintf(text,"   CNBR:%.2f      ",cnbr_price);
		LCD_DisplayStringLine(Line3,(uint8_t *)text);
		sprintf(text,"   VNBR:%.2f      ",vnbr_price);
		LCD_DisplayStringLine(Line5,(uint8_t *)text);
		LCD_DisplayStringLine(Line7,(uint8_t *)"                   ");		
	}
}

void control_PA7()
{
    if(control==0)
    {
      __HAL_TIM_SET_PRESCALER(&htim17,400-1);
      __HAL_TIM_SET_AUTORELOAD(&htim17,100-1);
      __HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,20);
      HAL_TIM_GenerateEvent(&htim17,TIM_EVENTSOURCE_UPDATE);

    }else{
      __HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,0);
      HAL_TIM_GenerateEvent(&htim17,TIM_EVENTSOURCE_UPDATE);
    }
}

void led_control()
{
  if(IDLE)
  {
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
  }else{
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
  }
  if(control==0)
  {
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
  }else{
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
  }
}

void rx_pro()
{
  uint8_t numok;
  
  for(int i=10;i<22;i++)
  {
    if(rxbuffer[i]<='9'&&rxbuffer[i]>='0'){
      numok = 1;
      //HAL_UART_Transmit(&huart1 , (uint8_t*)rxbuffer , strlen(rxbuffer),50);
    }else{
      numok = 0;
      HAL_UART_Transmit(&huart1,(uint8_t *)"Error\r\n",strlen("Error\r\n"),50);
    }
  }
  if(numok && (strncmp("VNBR",rxbuffer,4)==0||strncmp("CNBR",rxbuffer,4)==0))
  {
    sscanf(rxbuffer,"%4s:%4s:%12s", car_type , car_data , car_time);
    // char text[10];
    // sprintf(text , "%d\r\n" , rx_pointer);
    // HAL_UART_Transmit(&huart1 , (uint8_t*)text , strlen(text),50);
    // HAL_UART_Transmit(&huart1 , (uint8_t*)car_type , strlen(car_type),50);
    // HAL_UART_Transmit(&huart1 , "\r\n" , strlen("\r\n"),50);
    // HAL_UART_Transmit(&huart1 , (uint8_t*)car_data , strlen(car_data),50);
    // HAL_UART_Transmit(&huart1 , "\r\n" , strlen("\r\n"),50);
    // HAL_UART_Transmit(&huart1 , (uint8_t*)car_time , strlen(car_time),50);
    int ifpark = isparked(car_type , car_data);
    if(ifpark==8)
    {
      //入库函数
      uint8_t emptyPark = isempty();
      if(strncmp("CNBR" , car_type , 4)==0)
      {
        CNBR++;
        IDLE--;
      }else if(strncmp("VNBR" , car_type , 4)==0)
      {
        VNBR++;
        IDLE--;
      }
      strcpy(park[emptyPark].car_type , car_type);
      strcpy(park[emptyPark].car_data , car_data);
      strcpy(park[emptyPark].car_time , car_time);
      park[emptyPark].ifPark = 0;
          // HAL_UART_Transmit(&huart1 , (uint8_t*)park[emptyPark].car_type , strlen(park[emptyPark].car_type),50);
          // HAL_UART_Transmit(&huart1 , "\r\n" , strlen("\r\n"),50);
          // HAL_UART_Transmit(&huart1 , (uint8_t*)park[emptyPark].car_data , strlen(park[emptyPark].car_data),50);
          // HAL_UART_Transmit(&huart1 , "\r\n" , strlen("\r\n"),50);
          // HAL_UART_Transmit(&huart1 , (uint8_t*)park[emptyPark].car_time , strlen(park[emptyPark].car_time),50);
          // HAL_UART_Transmit(&huart1 , "\r\n" , strlen("\r\n"),50);
          // char text[10];
          // sprintf(text , "%d\r\n" , park[emptyPark].ifPark);
          // HAL_UART_Transmit(&huart1 , (uint8_t*)text , strlen(text),50);
    }
    else
    {
      //出库函数
      int dayin,dayout;
      int hourin,hourout;
      int minin,minout;
      dayin = (park[ifpark].car_time[4]-'0')*10 + (park[ifpark].car_time[5] - '0');
      dayout = (car_time[4]-'0')*10 + (car_time[5]-'0');
          // char text[10];
          // sprintf(text , "dayin:%d\r\n" , dayin);
          // HAL_UART_Transmit(&huart1 , (uint8_t*)text , strlen(text),50);
          // sprintf(text , "dayout:%d\r\n" , dayout);
          // HAL_UART_Transmit(&huart1 , (uint8_t*)text , strlen(text),50);
      int daylong = dayout - dayin;
      hourin = (park[ifpark].car_time[6]-'0')*10 + (park[ifpark].car_time[7] - '0');
      hourout = (car_time[6]-'0')*10 + (car_time[7]-'0');
      int hourlong = hourout - hourin + 24*daylong;
      minin = (park[ifpark].car_time[8]-'0')*10 + (park[ifpark].car_time[9] - '0');
      minout = (car_time[8]-'0')*10 + (car_time[9]-'0');
      if(minout - minin>0)
      {
        hourlong++;
      }
      if(strncmp("CNBR" , car_type , 4)==0)
      {
        uint8_t result[30];
        // uint8_t price[10];
        // sprintf(price , "%.2f\n" , cnbr_price*hourlong);
        // HAL_UART_Transmit(&huart1 , (uint8_t *)price , strlen(price) , 50);
        // HAL_UART_Transmit(&huart1 , (uint8_t *)car_type , strlen(car_type) , 50);
        sprintf(result , "%s:%s:%.2f\r\n" , car_type , car_data , cnbr_price*hourlong);
        HAL_UART_Transmit(&huart1 , (uint8_t *)result , strlen(result) , 50);
        IDLE++;
        CNBR--;
        strcpy(park[ifpark].car_type , "0");
        strcpy(park[ifpark].car_data , "0");
        strcpy(park[ifpark].car_time , "0");
        park[ifpark].ifPark = 1;
      }
      if(strncmp("VNBR" , car_type , 4)==0)
      {
        uint8_t result[30];
        sprintf(result , "%s:%s:%.2f\r\n" , car_type , car_data , vnbr_price*hourlong);
        HAL_UART_Transmit(&huart1 , (uint8_t *)result , strlen(result) , 50);
        IDLE++;
        VNBR--;
        strcpy(park[ifpark].car_type , "0");
        strcpy(park[ifpark].car_data , "0");
        strcpy(park[ifpark].car_time , "0");
        park[ifpark].ifPark = 1;
      }
    }
  }else{
    HAL_UART_Transmit(&huart1 , (uint8_t *)"Error\r\n" , strlen("Error\r\n") , 50);
  }
  rx_pointer=0;
  memset(rxbuffer , 0 , 30);
}
uint8_t isparked(uint8_t* car_type , uint8_t* car_data)
{
    uint8_t i;
    for(i=0;i<8;i++)
    {
      if(park[i].ifPark ==1)    //ifPark为1表示没停车
          continue;
      if(strncmp(car_type , park[i].car_type , 4)==0 && strncmp(car_data , park[i].car_data , 4)==0)
      {
        break;
      }
    }
    return i;
}

uint8_t isempty()
{
  uint8_t i=0;
  for(i=0;i<8;i++)
  {
    if(park[i].ifPark)
        return i;
  }
}

void Park_Init(Park* park)
{
    for(int i=0;i<8;i++)
    {
      strcpy(park[i].car_data , "");
      strcpy(park[i].car_time , "");
      strcpy(park[i].car_type , "");
      park[i].ifPark = 1;
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
