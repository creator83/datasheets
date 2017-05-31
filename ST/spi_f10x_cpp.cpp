#include "stm32f10x.h"                 // Device header
#include "spi_f10x_cpp.h"


uint8_t Spi::transfer (uint8_t data)
{
  GPIOB->ODR &= ~GPIO_ODR_ODR12;
  SPI2->DR = data; //Пишем в буфер передатчика SPI1. После этого стартует обмен данными
  while(SPI2->SR & SPI_SR_BSY); 
	GPIOB->ODR |= GPIO_ODR_ODR12;
  return SPI2->DR;

}


  Spi::Spi(uint8_t num, uint8_t sp, int8_t role, int8_t cpol, int8_t cpha)
{
  if (num == spi1) 
{
  spi1_init (sp, role, cpol, cpha);
 // n = num;
}
  if (num == spi2) spi2_init (sp, role, cpol, cpha);
  
}

void Spi::spi2_init (uint8_t sp, int8_t role, int8_t cpol, int8_t cpha)
{
    RCC->APB1ENR |=RCC_APB1ENR_SPI2EN;
	RCC->APB2ENR |=RCC_APB2ENR_IOPBEN|RCC_APB2ENR_AFIOEN;
	//B12 - CS; B13 - SCK; B14 - MISO; B15 - MOSI
	GPIOB->CRH &= ~(GPIO_CRH_MODE13|GPIO_CRH_MODE14|GPIO_CRH_MODE15|GPIO_CRH_CNF13|GPIO_CRH_CNF14|GPIO_CRH_CNF15);
	GPIOB->CRH |=	GPIO_CRH_MODE13|GPIO_CRH_MODE15|GPIO_CRH_CNF13_1 |GPIO_CRH_CNF14_1|GPIO_CRH_CNF15_1;
	
	//настройка CS
	GPIOB->CRH &= ~(GPIO_CRH_MODE12|GPIO_CRH_CNF12);
	GPIOB->CRH |= GPIO_CRH_MODE12;	
  GPIOB->ODR |= GPIO_ODR_ODR12;

	SPI2->CR2 =0;
	SPI2->CR1 =0;

	SPI2->CR2 |= SPI_CR2_SSOE;

  if (cpol == Pos) SPI2->CR1 = SPI_CR1_CPOL;
  if (cpha == Rising) SPI2->CR1 = SPI_CR1_CPHA;
  if (sp == low)SPI2->CR1 |= SPI_CR1_BR_2; //Baud rate = Fpclk/32  = 1.125Mhz
  if (sp == med) SPI2->CR1 |= SPI_CR1_BR_1; // Baud rate = Fpclk/8 = 4.5Mhz

	SPI2->CR1 |= SPI_CR1_BR_1; //Baud rate = Fpclk/8
  SPI2->CR1 |= SPI_CR1_MSTR; //Режим Master
  SPI2->CR1 |= SPI_CR1_SPE; //Включаем SPI1
}


void Spi::spi1_init (uint8_t sp, int8_t role, int8_t cpol, int8_t cpha)
{
  RCC->APB2ENR |=RCC_APB2ENR_SPI1EN|RCC_APB2ENR_IOPAEN|RCC_APB2ENR_AFIOEN;
	//A4 - CS; A5 - SCK; A6 - MISO; A7 - MOSI
	GPIOA->CRL &= ~(GPIO_CRL_MODE5|GPIO_CRL_MODE7|GPIO_CRL_MODE6|GPIO_CRL_CNF5|GPIO_CRL_CNF6|GPIO_CRL_CNF7);
	GPIOA->CRL |=	GPIO_CRL_MODE5|GPIO_CRL_MODE7|GPIO_CRL_CNF5_1 |GPIO_CRL_CNF7_1|GPIO_CRL_CNF6_1;
	
	//настройка CS
	GPIOA->CRL &= ~(GPIO_CRL_MODE4|GPIO_CRL_CNF4);
	GPIOA->CRL |= GPIO_CRL_MODE4;
	GPIOA->ODR |= GPIO_ODR_ODR4;
	SPI1->CR2 =0;
	SPI1->CR1 =0;
	SPI1->CR2 |= SPI_CR2_SSOE;
  if (cpol == Pos) SPI1->CR1 = SPI_CR1_CPOL;
  if (cpha == Rising) SPI1->CR1 = SPI_CR1_CPHA;
  if (sp == low)SPI1->CR1 |= SPI_CR1_BR_0|SPI_CR1_BR_2; //Baud rate = Fpclk/64
  if (sp == med) SPI1->CR1 |= SPI_CR1_BR_0|SPI_CR1_BR_1; // Baud rate = Fpclk/16
  if (sp == max) SPI1->CR1 |= SPI_CR1_BR_0; // Baud rate = Fpclk/4
  SPI1->CR1 |= SPI_CR1_MSTR; //Режим Master
  SPI1->CR1 |= SPI_CR1_SPE; //Включаем SPI1
}

