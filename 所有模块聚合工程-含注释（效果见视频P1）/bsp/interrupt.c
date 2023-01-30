#include "interrupt.h"
#include "usart.h"
#include "lcd.h"
uchar judge_state[4]={0},double_click_time[4]={0},key_state[4]={0},double_click_timerEN[4]={0},
							single_key_flag[4]={100},double_key_flag[4]={0},long_key_flag[4]={0};
uint key_time[4]={0};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//�ص�����
{
	if(htim->Instance==TIM3)//��ʱ��3���¼�
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
					if(key_state[i]==0)//��������
					{
						judge_state[i]=1;
						key_time[i]=0;
					}
					break;
				}
				case 1://��������
				{
					if(key_state[i]==0)
					{
						judge_state[i]=2;
					}
					else judge_state[i]=0;//δ����
					break;
				}
				case 2:
				{
					if((key_state[i]==1)&&key_time[i]<70)//�ȴ��ɿ�����,�ҷǳ�����
					{	
						if(double_click_timerEN[i]==0) //����˫�������ĵ�һ�Σ������ʱ
						{
							double_click_timerEN[i]=1;
							double_click_time[i]=0;
						}
						else //�ڼ�ʱ��Χ���ְ���һ��
						{
							double_key_flag[i]=1;//˫�����
							double_click_timerEN[i]=0;
						}
						judge_state[i]=0;
					}
					else if(key_state[i]==1&&key_time[i]>=70) judge_state[i]=0;//�ɿ����ǳ�����
					else  
					{
						if (key_time[i]>=70)long_key_flag[i]=1;//������
						key_time[i]++;//��������ʱ ��û�ɿ�
					}
					break;
				}
			}
			if(double_click_timerEN[i]==1)//��ʱȷ���Ƿ�˫��
			{
				double_click_time[i]++;
				if(double_click_time[i]>=35) 
				{
					single_key_flag[i]=1;//����1���ΰ���
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
	LCD_DisplayStringLine(Line9,(uint8_t *)rx);//��ʾ���յ���һ���ַ�
}


double  tim_val1 = 0,tim_val2=0;  									// TIMx_CCR1 ��ֵ 
uint  frq = 0,pulse=0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//��������� Ƶ�ʲ���
{
	if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)//�ж���Ϣ��Դ ѡ��ֱ�������ͨ��
	{
		tim_val1= HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);//��ȡ������1��ֵ
		tim_val2= HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);//��ȡ������2��ֵ
		__HAL_TIM_SetCounter(htim,0);//����������
		frq = 1000000/tim_val1; //frq=ʱ�ӣ�80m��/prescaler��80��/tim_val1
		pulse=(tim_val2/tim_val1)*100;
		HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
		HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_2);
	}
}
