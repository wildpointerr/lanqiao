#include "led.h"

void LED_Disp(uchar dsLED)
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);//开锁存器
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_All,GPIO_PIN_SET);//所有LED熄灭（l小写）
	HAL_GPIO_WritePin(GPIOC,dsLED<<8,GPIO_PIN_RESET);//左移8位：控制C8-15引脚，值为1的点亮
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);//关锁存器
}

