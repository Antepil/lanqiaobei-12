#include "interrupt.h"
#include "usart.h"



struct keys key[4]={0,0,0,0};



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3)
	{
		key[0].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
		key[1].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
		key[2].key_sta=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
		key[3].key_sta=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
		
		for(int i=0;i<4;i++)
		{
			switch(key[i].key_judage)
			{
				case 0:
				{
					if(key[i].key_sta==0)
					{
						key[i].key_judage=1;
						key[i].key_time=0;
					}
				}
				break;
				case 1:
				{
					if(key[i].key_sta ==0)
					{
						key[i].key_judage=2;
					}
					else
						key[i].key_judage=0;
				}
				break;
				case 2:
				{
					if(key[i].key_sta==1)
					{
						key[i].key_judage=0;
						if(key[i].key_time<=80)
							key[i].single_flag=1;
					}
					else
					{
						key[i].key_time++;
						if(key[i].key_time>80)
						{
							key[i].key_judage=0;
							key[i].long_flag=1;
						}
					}
				}
				break;
			}
		}
	}
}


uint8_t rxbuffer[30];
uint8_t uart_flag=0;
uint8_t rx_data=0;
uint8_t rx_pointer=0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		rxbuffer[rx_pointer++] = rx_data;
		HAL_UART_Receive_IT(&huart1 , &rx_data , 1);
		if(rx_pointer==22)
		{
			uart_flag = 1;
		}
		
	}
}


