#include "badc.h"

double getADC(ADC_HandleTypeDef *pin)
{
	uint16_t adc=0;
	HAL_ADC_Start(pin);
	adc=HAL_ADC_GetValue(pin);
	return adc*3.3/4096;
}
