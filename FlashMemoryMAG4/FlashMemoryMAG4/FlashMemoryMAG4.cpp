/*
 * FlashMemoryMAG4.cpp
 *
 * Created: 21.03.2019 19:32:41
 *  Author: goout
 */ 


#include <avr/io.h>

#include "USART.h"
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "spilib.h"

#define WRITEEnable 0x06
#define WRITEDisable 0x04
#define ChipEraseInst 0xC7
#define PageProgramInst 0x02 
#define ReadInst 0x03
void writeEn()
{
	PORTD &= ~(1UL<<SPI_SS);
	spi_send_recv(WRITEEnable);
	PORTD |= (1UL<<SPI_SS);
}
void writeDis()
{
	PORTD &= ~(1UL<<SPI_SS);
	spi_send_recv(WRITEDisable);
	PORTD |= (1UL<<SPI_SS);
}
void ChipErase(){
	writeEn();
	PORTD &= ~(1UL<<SPI_SS);
	spi_send_recv(ChipEraseInst);
	PORTD |= (1UL<<SPI_SS);
}

uint8_t address = 0;
uint8_t address2 = 0;
uint8_t address3 = 0;

void WriteTheStuffIndefinitely(int32_t ayedata[], int sizee) //ayedata is array of 256  entries, last one is 0x00
{
	writeEn();
	PORTD &= ~(1UL<<SPI_SS);
	spi_send_recv(PageProgramInst);
	spi_send_recv(address);
	spi_send_recv(address2);
	spi_send_recv(address3);
	if (address == 255){
		address = 0;
		address2 += 1;
		
	}
	if (address2 == 255){  // every entry is new page
		address2 = 0;
		address3 += 1;
	}
	address +=1;
	for (uint8_t i = 0; i < sizee; i++)
	{
		spi_send_recv(ayedata[i]);
	}
	PORTD |= (1UL<<SPI_SS);
	
	
}
void ReadTheShitIndefinitely(){
	writeEn();
	char hi[120];
	PORTD &= ~(1UL<<SPI_SS);
	spi_send_recv(ReadInst);
	spi_send_recv(address);
	spi_send_recv(address2);
	spi_send_recv(address3);
	for (int i=0; i<16; i++) {
		uint32_t data = spi_send_recv(0);
		sprintf(hi, "%ld",data);
		Uart_tr(hi);
	}
	PORTD |= (1UL<<SPI_SS);
	
}
int main(void)
{
	init_USART();
	spi_init();
	ChipErase();
	//int telemetry[256] = { };
	//telemetry =
	int32_t data[] = {0x48,0x45,0x4c,0x4c,0x4f,0x2c,0x20,0x53,0x4f,0x4c,0x54,0x41,0x55,0x2e,0x52,0x55}; 
	WriteTheStuffIndefinitely(data, sizeof(data));
    
	int32_t daa;
	while(1)
    {
         address = 0;
         address2 = 0;
         address3 = 0;
		 //ReadTheShitIndefinitely();
		 char gi[120];
		 daa = 156;
		 sprintf(gi, "why hey %ld",daa);
		 Uart_tr(gi);
		 address +=1;
		 _delay_ms(1000);
	 }
 }