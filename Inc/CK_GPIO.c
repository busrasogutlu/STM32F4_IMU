
#include "CK_GPIO.h"

void CK_GPIO_Init(GPIO_TypeDef* GPIOx, uint16_t GPIOx_Pin, CK_GPIOx_Mode GPIOx_Mode, CK_GPIOx_AFx GPIOx_AF,
				  CK_GPIOx_Speed GPIOx_Speed, CK_GPIOx_PUPD GPIOx_PUPD)
{
	GPIO_InitTypeDef GPIO_InitStruct 	= {0};
	GPIO_InitStruct.Pin 				= 1u << GPIOx_Pin;
	GPIO_InitStruct.Mode 				= GPIOx_Mode;
	GPIO_InitStruct.Pull 				= GPIOx_PUPD;
	GPIO_InitStruct.Speed 				= GPIOx_Speed;
	if(GPIOx_AF != CK_GPIO_NOAF){
		GPIO_InitStruct.Alternate 		= GPIOx_AF;
	}


	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

}

void CK_GPIO_DeInit(GPIO_TypeDef* GPIOx, uint16_t GPIOx_Pin){

	HAL_GPIO_DeInit(GPIOx, 1u << GPIOx_Pin);

}

void CK_GPIO_ClockEnable(GPIO_TypeDef* GPIOx){

	uint16_t port_clk = ((uint32_t)GPIOx - (GPIOA_BASE)) / ((GPIOB_BASE) - (GPIOA_BASE));

	RCC->AHB1ENR |= 1u<<port_clk;
}

void CK_GPIO_ClockDisable(GPIO_TypeDef* GPIOx){

	uint16_t port_clk = ((uint32_t)GPIOx - (GPIOA_BASE)) / ((GPIOB_BASE) - (GPIOA_BASE));

	RCC->AHB1ENR &= ~(1u << port_clk);
}

void CK_GPIO_SetPin(GPIO_TypeDef* GPIOx, uint16_t GPIOx_Pin){

	GPIOx->BSRR |= 1u << GPIOx_Pin;

	//HAL_GPIO_WritePin(GPIOx, 1u << GPIOx_Pin, GPIO_PIN_SET);

}

void CK_GPIO_ClearPin(GPIO_TypeDef* GPIOx, uint16_t GPIOx_Pin){

	GPIOx->BSRR |= 1u << (GPIOx_Pin + 16);

	//HAL_GPIO_WritePin(GPIOx, 1u << GPIOx_Pin, GPIO_PIN_RESET);

}

void CK_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIOx_Pin){

	HAL_GPIO_TogglePin(GPIOx, 1u << GPIOx_Pin);
}

uint8_t CK_GPIO_ReadPin(GPIO_TypeDef* GPIOx ,uint16_t GPIOx_Pin){

	uint16_t temp = (GPIOx->IDR & (uint16_t)(1u << GPIOx_Pin)) >> GPIOx_Pin;

	if(temp == 0x01){
		return 0x01;
	}
	return 0x00;

}
