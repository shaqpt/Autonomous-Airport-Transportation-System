/*
 * SPI.h
 *
 * Created: 10/18/2017 10:24:47 AM
 * Authors: Shaqueir Tardif
 *		    Anthony Mitchell
 *		    Devaj Ramsamooj
 */ 


#ifndef SPI_H_
#define SPI_H_

void SPI_init(uint8_t dord, uint8_t mode, uint8_t div);
uint8_t SPI_transmit(uint8_t data);
void SPI_setCS(uint8_t val);
void MAX7221_send(uint8_t cmd, uint8_t data);
void MAX7221_init();
void MAX7221_DisplayVal(uint8_t val[]);
void setDigit(uint8_t digit, uint8_t val);
void blinkDigit(uint8_t digit, uint8_t val);
void stopBlink(uint8_t digit, uint8_t val);
void MAX7221_refresh();




#endif /* SPI_H_ */