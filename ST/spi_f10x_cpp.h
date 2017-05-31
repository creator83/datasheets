#include "stm32f10x.h"  

#ifndef SPI_F10X_CPP_H
#define SPI_F10X_CPP_H


class Spi
{
public:
  Spi (uint8_t num = spi2, uint8_t speed=med, int8_t role=master, int8_t cpol=Neg, int8_t cpha=Faling);
  enum Num_spi {spi1,spi2,spi3};
enum Speed {low,med,max};
  uint8_t transfer (uint8_t);


private:
void spi1_init (uint8_t speed, int8_t role, int8_t cpol, int8_t cpha);
void spi2_init (uint8_t speed, int8_t role, int8_t cpol, int8_t cpha);

 Num_spi n;
enum Role {master,slave};
enum Cpol {Neg, Pos};
enum Cpha {Faling, Rising};
};

uint8_t transfer (uint8_t data);

//настройки линий CS
#define CS1_ON GPIOA->ODR|= 1<<4
#define CS1_OFF GPIOA->ODR&= ~(1<<4)
#define CS2_ON GPIOB->ODR|= 1<<12
#define CS2_OFF GPIOB->ODR&= ~(1<<12)
#define CS3_ON GPIOB->ODR|= 1<<1
#define CS3_OFF GPIOB->ODR&= ~(1<<1)

void init_spi1 (void);
void init_spi2 (void);
void init_spi3 (void);

uint8_t spi1_transfer_byte (uint8_t m);
uint8_t spi2_transfer_byte (uint8_t m);
uint8_t spi_transfer3 (uint8_t m);
uint8_t spi1_transfer(uint8_t m);
uint8_t spi2_transfer(uint8_t m);

uint32_t spi2_receive (void);

uint8_t spi_exchange(uint8_t send_data);

#endif
