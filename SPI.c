/*
 * SPI.c
 *
 * Created: 10/18/2017 10:24:47 AM
 * Authors: Shaqueir Tardif
 *		    Anthony Mitchell
 *		    Devaj Ramsamooj
 */ 
#define F_CPU 16000000
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "SPI.h"


#define DDR DDRB
#define CS 2
#define MOSI 3
#define MISO 4
#define SCK 5

void SPI_init(uint8_t dord, uint8_t mode, uint8_t div)
{
	DDRB = (1<<CS)|(1<<MOSI)|(1<<SCK);
	PORTB |=(1<<CS);
	//SPSR = (0<<SPI2X);
	//no interrupts, /4, LSB first, mode 0
	SPCR=(0<<SPIE)|(1<<SPE)|((dord&1)<<DORD)|(1<<MSTR)|((mode&3)<<CPHA)|((div&3)<<SPR0);
}
void SPI_setCS(uint8_t val)
{
	if(val) PORTB |= 1<<CS;
	else PORTB &= ~(1<<CS);
}

uint8_t SPI_transmit(uint8_t data)
{
	SPDR = data;
	while((SPSR & (1<<SPIF))==0);
	return SPDR;
}

/****** LCD Functions *****/
#define MAX7221_NOP 0
#define MAX7221_DIG0 1
#define MAX7221_DIG1 2
#define MAX7221_DIG2 3
#define MAX7221_DIG3 4
#define MAX7221_DIG4 5
#define MAX7221_DIG5 6
#define MAX7221_DIG6 7
#define MAX7221_DIG7 8
#define MAX7221_DECODE 9
#define MAX7221_INTENSITY 10
#define MAX7221_SCAN 11
#define MAX7221_ENABLE 12
#define MAX7221_TEST 15


uint8_t DigLUT[]={1,7,3,5,4,8,6,2};

void MAX7221_init()
{
	//MSB first, mode 0, divider
	SPI_init(0,0,0);
	MAX7221_send(MAX7221_ENABLE,1);
	MAX7221_send(MAX7221_DECODE,0xFF);
	MAX7221_send(MAX7221_SCAN,7);
	MAX7221_send(MAX7221_INTENSITY,0xF);
	//for(int i = 0; i<8; i++)
		//MAX7221_send(i+1,0xF);
	//MAX7221_send(MAX7221_TEST,0xFF);
}
void MAX7221_send(uint8_t cmd, uint8_t data)
{
	SPI_setCS(0);
	SPI_transmit(cmd&0x0F);
	SPI_transmit(data);
	SPI_setCS(1);
	
}
void setDigit(uint8_t digit, uint8_t val)
{
	MAX7221_send(DigLUT[digit],val);
}

void MAX7221_DisplayVal(uint8_t val[])
{
	
	for (int i=0; i<8; i++)
	{
		
		setDigit(i,val[i]);
	}
	
}
void MAX7221_refresh() //LCD Screen
{
	uint8_t mon,date,hr,min;
	while(1)
	{
		DS3231_getTime2(&mon,&date,&hr,&min);
		uint8_t digit[8]={0xff,0xff,date>>4,date,hr>>4,hr,min>>4,min}; //mon>>4, mon
		MAX7221_DisplayVal(digit);
	}
}
void blinkDigit(uint8_t digit, uint8_t val) //Seven-Segment Display
{
	while(1)
	{
		setDigit(digit,val);
		_delay_ms(500);
		setDigit(digit,0x0F);
		_delay_ms(500);
	}
}
void stopBlink(uint8_t digit, uint8_t val)
{
	setDigit(digit,val);
}
//48:48