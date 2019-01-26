/*
* Design Prototype Source Code
*
* Created: 10/18/2017 10:24:47 AM
* Authors :Shaqueir Tardif
*		   Anthony Mitchell
*		   Devaj Ramsamooj
*
* This source code is for control of a mimic autonomous airport transportation system controlled by an Atmel328p microcontroller. This microcontroller
* is connected to a perfboard leading to connections to the following components:
*
* -- DC motor (to control the movement of the train) 
* -- Servo motor (to control door movement),
* -- Distance sensor (to interrupt door closing routine if interference is detected in order to avoid human injury) 
* -- Contact switches (to trigger a slow down routine to allow the train to stop at its upcoming destination safely)
*
*/

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "SPI.h"
uint8_t CSW = 3; //contact switch temp var
uint8_t sta = 0, IR, stp;

int IRsensDR();
int IRsensFWD();
void servopen();
void servclose();
void initdcmtr();
void initservmtr();
void initADC();
void initbtn();
void drive();
void stop();
void approach();

int main()
{
	int FIR, DIR;
	//Door Servo
	initservmtr();
	ICR1=40000;  //fPWM=50Hz (Period = 20ms Standard). //4999
	//DC Motor
	initdcmtr();
	//ADComparator
	initADC();
	//ADMUX = (1<<7)|(1<<6);
	IRsensFWD();
	//Call distance sensor
	IRsensDR();
	//Call distance sensor
	initbtn();
	initdcmtr();
	SPI_init(0,0,0);
	MAX7221_init();
	setDigit(0,9);
	initADC();
	sei();
	while(1)
	{
		FIR = IRsensFWD(); //initialize IR distance sensor
		stop();
		while (FIR < 0x15C)
		{
			FIR = IRsensFWD();
			drive();
 			if (CSW == 0)
			{
				approach();
			}
			else if (CSW == 1)
			{
				stop();
				if(sta==0) //sta = station #
				{
					setDigit(0,0); //display station number
					servopen(); //open doors
					_delay_ms(3000); //hold for 3 seconds unless interrupted
				}
				else if(sta ==1) 
				{
					setDigit(0,1);
					servopen();
					_delay_ms(3000); //hold for 3 seconds unless interrupted
				}
				DIR = IRsensDR();
				while (DIR>=0x15C)
				{
					//_delay_ms(1000);
					DIR = IRsensDR();
					
				}
				
				servclose();
				_delay_ms(1500);
				
			}
			servclose();
			drive();
		}
		FIR = IRsensFWD();
	}
}
int IRsensFWD()
{
	//int temp;
	//ADMUX |= (0b000<MUX0);  //vcc ref and adc0
	//ADCSRA = (1<<ADSC);
	//while (!(ADCSRA&(1<<ADIF)));
	//if(ADC >= 0x15C) // 15C-> 15cm
	//temp = 1;
	//else
	//temp = 0; //ACO
	//ADCSRA&=~(1<<ADIF);
	//return temp;
	
	//ADCSRC = 10<<ADSUT0;   // set start-up time
	//ADCSRB = 0<<MUX5;    // set MUX5 first
	ADMUX = (1<<REFS0) + (0<<MUX0); // 1.6V AREF - channel 0
	
	// switch ADC on, set prescaler (32 -> 500kHz), start conversion
	ADCSRA = (1<<ADEN) | (1<<ADSC) | (5<<ADPS0);
	do
	{} while( (ADCSRA & (1<<ADSC))); // wait for conversion end
	ADCSRA = 0; // disable the ADC
	
	return (ADC);
	
}
int IRsensDR() //initialize IR distance sensor
{
	ADMUX = (1<<REFS0) + (1<<MUX0); // 1.6V AREF - channel 0
	
	// switch ADC on, set prescaler (32 -> 500kHz), start conversion
	ADCSRA = (1<<ADEN) | (1<<ADSC) | (5<<ADPS0);
	do
	{} while( (ADCSRA & (1<<ADSC))); // wait for conversion end
	ADCSRA = 0; // disable the ADC
	
	return (ADC);
}
void servopen() //open doors (servo)
{
	OCR1A = ICR1 - 100; //open
	_delay_ms(1000);
}
void servclose() //close doors (servo)
{
	OCR1A = ICR1 - 3500; //close
	_delay_ms(1000);
}
void initdcmtr()
{
	DDRD = (1<<5); //PD5 OC0B as output (PCINT21/OC0B/T1) PD5
	//PORTD|=(1<<7); //0x80; // pull-up resistor D7
	TCCR0A=(0b00<<COM0A0)|(0b10<<COM0B0)|(1<<WGM01)|(1<<WGM00);//Timer0
	//Mode3-Fast PWM, Top=0xFF, Update of OCRx = BOTTOM, TOV flag set = MAX
	TCCR0B|=(0<<FOC0A)|(0<<FOC0B)|(0<<WGM02)|(0b001<<CS00);//Timer0
}
void initservmtr() //initialize servo motor (configure times and correct PWM modes)
{
	TCCR1A|=(1<<COM1A1)|(1<<COM1A0)|(1<<WGM11); //Configure TIMER1
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11);  //NON Inverted PWM
	DDRB|=(1<<1);
}
void initbtn() //initiate button interrupt
{
	DDRC &= ~(0b11<<DDC2);
	//PORTC &= ~(0b11<<PORTC2);
	PCICR |=(1<<PCIE1);
	PCMSK1 |=(0b11<<PCINT10); //Button interrupt PC2 and 3 = A2 and 3
	
	//DDRB &= ~(1<<DDB7);
	//PORTB |= (1<<PORTB7);
	//PCICR |=(1<<PCIE0);
	//PCMSK0 |=(1<<PCINT7);
	
}
void initADC() //initialize analog to digital converter
{
	//DIDR1 =(1<<AIN0D)|(1<<AIN1D);//digital input buffers disabled
	//DDRC &= ~(1<<0); //1=output pin 23 PC0(ADC0) = A0
	//ADCSRB |= (1<<ACME)|(0<<ADTS2)|(0<<ADTS1)|(1<<ADTS0);//MUX EN, Analog Comparator
	//ADCSRA = (0<<ADEN);
	//ACSR |= (0<<ACD)|(1<<ACBG)|(1<<ACIE)|(1<<ACIS1)|(1<<ACIS0);
	// ADC en, AIN0 en, Interrupt en, Rising Edge
	
	ADCSRA |=(1<<ADEN|0<<ADIE|0b100<<ADPS0);
	ADMUX |= (0b01<<REFS0);
}
void drive() //move train, only moves in forward direction
{
	//OCR0B = 100; //40%
	//_delay_ms(1500);
	//OCR0B = 127;//50%
	//_delay_ms(1500);
	OCR0B = 155; //60%
}
void stop() //halt the train
{
	OCR0B = 0;//0%
}
void approach() //decreasing speed function
{
	//OCR0B = 127;//50%
	//_delay_ms(1500);
	OCR0B = 63; //25%
}
ISR(PCINT1_vect) //interrupt handler for Contact Switch
{
	
	if (!(PINC&(1<<PINC2))) //Contact Switch interrupts, slow down train
	{
		CSW =0;
		approach();
	}
	
	
	if (!(PINC&(1<<PINC3)))	//when specific contact switch initiated, display which station train is at onto LCD
	{
		CSW = 1;
		//stop();
		
	}
}
ISR(ADC_vect) //interrupt handler for distance sensor
{
	if(IR==0)// FIR
	{
		if(ADC >= 0x15C)
		{ // 15C-> 15cm
			stp = 1;
		}
		else
		stp = 0; //ACO
	}
	else if(IR ==1)
	{
		if(ADC >= 0x15C)
		{ // 15C-> 15cm
			stp = 1;
		}
		else
		stp = 0;
		
	}
		
}
