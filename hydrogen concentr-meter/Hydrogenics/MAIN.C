/*
 * Hydrogenics.c
 *
 * Created: 26.09.2018 22:24:22
 * Author : space
 */ 

#include <avr/io.h>
#define F_CPU 8000000
#define BAUD 9600L
#define UBRRL_value (F_CPU/(BAUD*16))-1

#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "i2c.h"

void init_adc()
{
	ADMUX = 0b01000000; //�������� �������� ������� ���
	ADCSRA |=(1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ������������� ������������
	ADCSRA |=(1 << ADEN); // �������� ���
}

// ������� ������� �������������� ��� � ������ ����������
unsigned int read_adc (unsigned char adc_input)
{
	ADMUX |=0x07 & adc_input; //���������� �� ����� ����� �������� V
	//0?07-����������� (����� ������ ���������� �� ����� �� ������ �����)
	//0?07-��� ������ ������������ ������������ ���������������� �����
	//���� �������� ADMUX|=0x1F&adc_input; -��������� ���. ��� �������� ��� � �� �������� �����
	_delay_us(10);                       // �������� �� ������������ �������� ����������
	ADCSRA |= (1 << ADSC);              // ������ �������������� ���
	while ((ADCSRA & (1 << ADIF))==0); // �������� ��������� ��������������
	return ADCW;
}
void init_USART()
{
	UBRRL= UBRRL_value;
	(UBRRH) = (UBRRL_value) >> 8;
	UCSRB |= (1<<RXEN)|(1<<TXEN);
	UCSRC = (1<<URSEL)|(1<<USBS)|(3<<UCSZ0);

}
void Uart_tr (char s[])
{
	unsigned char k;
	for (k=0;s[k]!=0;k++)
	{
		UDR =s[k];
		while (!(UCSRA&(1<<UDRE)));
	}
}


int32_t hydroconcent;

int main(void)
{
	char hey[120];
	char hydro[120];
	i2c_init();
	_delay_ms(500);
	init_adc();
	_delay_ms(500);
	init_USART();
    /* Replace with your application code */
    while (1) 
    {
		
		hydroconcent = read_adc(0);
		sprintf(hydro, "Hydrogen concentration: %ld", hydroconcent);
		Uart_tr(hydro);
		_delay_ms(1000);
    }
}

