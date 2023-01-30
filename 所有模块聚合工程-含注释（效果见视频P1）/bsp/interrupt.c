#include "interrupt.h"
#include "usart.h"
#include "lcd.h"
uchar judge_state[4]={0},double_click_time[4]={0},key_state[4]={0},double_click_timerEN[4]={0},
							single_key_flag[4]={100},double_key_flag[4]={0},long_key_flag[4]={0};
uint key_time[4]={0};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//回调函数
{
	if(htim->Instance==TIM3)//定时器3的事件
	{
		key_state[0]=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
		key_state[1]=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
		key_state[2]=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
		key_state[3]=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
		for(int i=0;i<4;i++)
		{
			switch (judge_state[i])
			{
				case 0:
				{
					if(key_state[i]==0)//按键按下
					{
						judge_state[i]=1;
						key_time[i]=0;
					}
					break;
				}
				case 1://消抖过程
				{
					if(key_state[i]==0)
					{
						judge_state[i]=2;
					}
					else judge_state[i]=0;//未按下
					break;
				}
				case 2:
				{
					if((key_state[i]==1)&&key_time[i]<70)//等待松开过程,且非长按键
					{	
						if(double_click_timerEN[i]==0) //可能双击按键的第一次，进入计时
						{
							double_click_timerEN[i]=1;
							double_click_time[i]=0;
						}
						else //在计时范围内又按了一次
						{
							double_key_flag[i]=1;//双击情况
							double_click_timerEN[i]=0;
						}
						judge_state[i]=0;
					}
					else if(key_state[i]==1&&key_time[i]>=70) judge_state[i]=0;//松开且是长按键
					else  
					{
						if (key_time[i]>=70)long_key_flag[i]=1;//长按键
						key_time[i]++;//长按键计时 还没松开
					}
					break;
				}
			}
			if(double_click_timerEN[i]==1)//延时确认是否双击
			{
				double_click_time[i]++;
				if(double_click_time[i]>=35) 
				{
					single_key_flag[i]=1;//按键1单次按下
					double_click_timerEN[i]=0;
				}
			}
		}
	}
}

uint8_t rx[100];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart1, rx, 1);
	LCD_DisplayStringLine(Line9,(uint8_t *)rx);//显示接收到的一个字符
}


double  tim_val1 = 0,tim_val2=0;  									// TIMx_CCR1 的值 
uint  frq = 0,pulse=0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//捕获计数器 频率测量
{
	if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)//中断消息来源 选择直接输入的通道
	{
		tim_val1= HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);//获取计数器1的值
		tim_val2= HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);//获取计数器2的值
		__HAL_TIM_SetCounter(htim,0);//计数器归零
		frq = 1000000/tim_val1; //frq=时钟（80m）/prescaler（80）/tim_val1
		pulse=(tim_val2/tim_val1)*100;
		HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
		HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_2);
	}
}
