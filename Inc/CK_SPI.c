
#include "CK_GPIO.h"
#include "CK_SPI.h"

#define CK_SPIx_CR1_MSTR               1u<<2
#define CK_SPIx_CR1_CPOL               1u<<1
#define CK_SPIx_CR1_CPHA               1u<<0
#define CK_SPIx_CR1_SPE                1u<<6
#define CK_SPIx_CR1_SSI                1u<<8
#define CK_SPIx_CR1_SSM                1u<<9

#define CK_SPIx_CR2_TXDMAEN            1u<<1
#define CK_SPIx_CR2_RXDMAEN            1u<<0

#define CK_SPIx_SR_TXE                 1u<<1
#define CK_SPIx_SR_RXNE                1u<<0
#define CK_SPIx_SR_BSY                 1u<<7

#define CK_RCC_SPI1_EN                 1u<<12
#define CK_RCC_SPI2_EN                 1u<<14
#define CK_RCC_SPI3_EN                 1u<<15

#define SPI_TIMEOUT                    250 // 250 makes around 25 usec which is enough.

typedef struct{

    uint32_t timeout;

    uint32_t spi1_timeout;
    int spi1_init;

    uint32_t spi2_timeout;
    int spi2_init;

    uint32_t spi3_timeout;
    int spi3_init;

}SPI_t;

SPI_t spi_variables = {

    .timeout      = 0,

    .spi1_timeout = 0,
    .spi1_init    = 0,

    .spi2_timeout = 0,
    .spi2_init    = 0,

    .spi3_timeout = 0,
    .spi3_init    = 0,

};

void CK_SPI_Init(SPI_TypeDef* spi_n, CK_SPIx_CR1_Fclk_Div spi_clock){

	SPI_TypeDef* SPIx = spi_n;
	GPIO_TypeDef* GPIOx;
	CK_GPIOx_AFx AFx;
	uint16_t miso_pin, mosi_pin, sck_pin;

	/*
	 *	SPI1,4,5 have APB2 = 90MHz
	 *	SPI2 have APB1 = 45MHz
	 *	so prescaler must be selected to meet 10MHZ max spi clock speed
	 *
	 */

	if(SPIx == SPI1){

		GPIOx = GPIOA;

		sck_pin     = 5;
		miso_pin    = 6;
		mosi_pin    = 7;

		AFx = CK_GPIO_AF5;
		RCC->APB2ENR |= CK_RCC_SPI1_EN; // Enable related SPI clock

		spi_variables.spi1_init    = 1;
		spi_variables.spi1_timeout = 0;
	}
	else if(SPIx == SPI2){

		GPIOx = GPIOB;

		sck_pin     = 13;
		miso_pin    = 14;
		mosi_pin    = 15;

		AFx = CK_GPIO_AF5;
		RCC->APB1ENR |= CK_RCC_SPI2_EN; // Enable related SPI clock

		spi_variables.spi2_init    = 1;
		spi_variables.spi2_timeout = 0;
	}
	else if(SPIx == SPI3){

		GPIOx = GPIOC;

		sck_pin     = 10;
		miso_pin    = 11;
		mosi_pin    = 12;

		AFx = CK_GPIO_AF6;
		RCC->APB1ENR |= CK_RCC_SPI3_EN; // Enable related SPI clock

		spi_variables.spi3_init    = 1;
		spi_variables.spi3_timeout = 0;
	}

	CK_GPIO_ClockEnable(GPIOx);
	CK_GPIO_Init(GPIOx, sck_pin, CK_GPIO_AF_PP, AFx, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
	CK_GPIO_Init(GPIOx, miso_pin, CK_GPIO_AF_PP, AFx, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
	CK_GPIO_Init(GPIOx, mosi_pin, CK_GPIO_AF_PP, AFx, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);

	/*
	 * Default: full duplex, (cpha=0,cpol=0),8bit data,No CRC,MSB First
	 */

	SPIx->CR1 |= CK_SPIx_CR1_MSTR | CK_SPIx_CR1_SSM | CK_SPIx_CR1_SSI | spi_clock;
	SPIx->CR1 |= CK_SPIx_CR1_SPE; // SPI Enable


}

void CK_SPI_Enable(SPI_TypeDef* spi_n){

	spi_n->CR1 |= CK_SPIx_CR1_SPE; // SPI Enable

}
void CK_SPI_Disable(SPI_TypeDef* spi_n){

	spi_n->CR1 &= ~CK_SPIx_CR1_SPE; // SPI Disable

}

void CK_SPI_EnableTXDMA(SPI_TypeDef* spi_n){

	spi_n->CR2 |= CK_SPIx_CR2_TXDMAEN; // | CK_SPIx_CR2_RXDMAEN;

}

void CK_SPI_EnableRXDMA(SPI_TypeDef* spi_n){

	spi_n->CR2 |= CK_SPIx_CR2_RXDMAEN;

}

void CK_SPI_DisableTXDMA(SPI_TypeDef* spi_n){

	spi_n->CR2 &= ~(CK_SPIx_CR2_TXDMAEN); // | CK_SPIx_CR2_RXDMAEN);

}

void CK_SPI_DisableRXDMA(SPI_TypeDef* spi_n){

	spi_n->CR2 &= ~(CK_SPIx_CR2_RXDMAEN);

}

void CK_SPI_ChangeClock(SPI_TypeDef* spi_n, CK_SPIx_CR1_Fclk_Div clk){

	spi_n->CR1 &= ~(7u<<3);
	spi_n->CR1 |= clk;

}

uint8_t CK_SPI_WriteRegister(uint8_t reg, uint8_t data, SPI_TypeDef* SPIn, GPIO_TypeDef* GPIOx_CS, uint16_t cs_pin){
	uint8_t val = 0;
	CK_GPIO_ClearPin(GPIOx_CS, cs_pin);

	CK_SPI_Transfer(SPIn, reg);
	val = CK_SPI_Transfer(SPIn, data);

	CK_GPIO_SetPin(GPIOx_CS, cs_pin);
	return val;
}

void CK_SPI_ReadRegisterMulti(uint8_t reg, SPI_TypeDef* SPIn, GPIO_TypeDef* GPIOx_CS, uint16_t cs_pin, uint8_t* dataIn, int count){

    CK_GPIO_ClearPin(GPIOx_CS, cs_pin);

    CK_SPI_Transfer(SPIn, reg);

    while (count--) {

        *dataIn++ = CK_SPI_Transfer(SPIn, 0);
    }

    CK_GPIO_SetPin(GPIOx_CS, cs_pin);

}

uint8_t CK_SPI_Transfer(SPI_TypeDef* SPIn, uint8_t data){

    spi_variables.timeout = SPI_TIMEOUT;
	while(((SPIn)->SR & (CK_SPIx_SR_TXE | CK_SPIx_SR_RXNE)) == 0 || ((SPIn)->SR & CK_SPIx_SR_BSY)){
		if(--spi_variables.timeout == 0x00){
		    CK_SPI_TimeOutCounter(SPIn);
		    return 1;
		}
	}

	SPIn->DR = data;

	spi_variables.timeout = SPI_TIMEOUT;
	while(((SPIn)->SR & (CK_SPIx_SR_TXE | CK_SPIx_SR_RXNE)) == 0 || ((SPIn)->SR & CK_SPIx_SR_BSY)){
		if(--spi_variables.timeout == 0x00){
		    CK_SPI_TimeOutCounter(SPIn);
		    return 1;
		}
	}
	return SPIn->DR;
}

uint8_t CK_SPI_WaitTransfer(SPI_TypeDef* SPIn){

    spi_variables.timeout = SPI_TIMEOUT;
    while(((SPIn)->SR & (CK_SPIx_SR_TXE | CK_SPIx_SR_RXNE)) == 0 || ((SPIn)->SR & CK_SPIx_SR_BSY)){
        if(--spi_variables.timeout == 0x00){
            CK_SPI_TimeOutCounter(SPIn);
            return 1;
        }
    }

    return 0;
}

int CK_SPI_CheckInitialized(SPI_TypeDef* SPIn){

    int res;
    if(SPIn == SPI1){
        res = spi_variables.spi1_init;
    }
    else if(SPIn == SPI2){
        res = spi_variables.spi2_init;
    }
    else if(SPIn == SPI3){
        res = spi_variables.spi3_init;
    }
    else{
        res = 2; // Error
    }

	return res;

}

void CK_SPI_TimeOutCounter(SPI_TypeDef* spi){
    if(spi == SPI1){
        spi_variables.spi1_timeout++;
    }
    else if(spi == SPI2){
        spi_variables.spi2_timeout++;
    }
    else if(spi == SPI3){
        spi_variables.spi3_timeout++;
    }
}

uint32_t CK_SPI_GetTimeOut(SPI_TypeDef* spi){

    uint32_t res;

    if(spi == SPI1){
        res = spi_variables.spi1_timeout;
    }
    else if(spi == SPI2){
        res = spi_variables.spi2_timeout;
    }
    else if(spi == SPI3){
        res = spi_variables.spi3_timeout;
    }

    return res;
}

void CK_SPI_ResetTimeOut(SPI_TypeDef* spi){

    if(spi == SPI1){
        spi_variables.spi1_timeout = 0;
    }
    else if(spi == SPI2){
        spi_variables.spi2_timeout = 0;
    }
    else if(spi == SPI3){
        spi_variables.spi3_timeout = 0;
    }

}

CK_SPIx_CR1_Fclk_Div CK_SPI_GetClockRate(SPI_TypeDef* spi, uint32_t spi_clock){

	uint16_t clock_rate = 0;

	if(spi == SPI1){
		clock_rate = 84000000 / spi_clock;
	}
	else if(spi == SPI2){
		clock_rate = 42000000 / spi_clock;
	}
	else if(spi == SPI3){
		clock_rate = 42000000 / spi_clock;
	}

	CK_SPIx_CR1_Fclk_Div clock = 0;

	if(clock_rate <= 2){
		clock = CK_SPIx_CR1_Fclk_Div2;
	}
	else if(clock_rate > 2 && clock_rate <= 4){
		clock = CK_SPIx_CR1_Fclk_Div4;
	}
	else if(clock_rate > 4 && clock_rate <= 8){
		clock = CK_SPIx_CR1_Fclk_Div8;
	}
	else if(clock_rate > 8 && clock_rate <= 16){
		clock = CK_SPIx_CR1_Fclk_Div16;
	}
	else if(clock_rate > 16 && clock_rate <= 32){
		clock = CK_SPIx_CR1_Fclk_Div32;
	}
	else if(clock_rate > 32 && clock_rate <= 64){
		clock = CK_SPIx_CR1_Fclk_Div64;
	}
	else if(clock_rate > 64 && clock_rate <= 128){
		clock = CK_SPIx_CR1_Fclk_Div128;
	}
	else if(clock_rate > 128 && clock_rate <= 256){
		clock = CK_SPIx_CR1_Fclk_Div256;
	}

	return clock;
}








