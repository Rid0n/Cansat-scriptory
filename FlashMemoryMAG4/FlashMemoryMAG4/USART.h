#include <avr/io.h>
#define F_CPU 8000000
#define BAUD 9600L
#define UBRRL_value (F_CPU/(BAUD*16))-1
/*void init_USART()
{
	UBRR0L= UBRRL_value;
	(UBRR0H) = (UBRRL_value) >> 8;
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (0<<UMSEL0)|(1<<USBS0)|(3<<UCSZ00);

}
void Uart_tr (char s[])
{
	unsigned char k;
	for (k=0;s[k]!=0;k++)
	{
		UDR0=s[k];
		while (!(UCSR0A&(1<<UDRE0)));
	}
}*/



#include <inttypes.h>


void init_USART()
{
	UBRRL= UBRRL_value;
	(UBRRH) = ((UBRRL_value)>> 8);
	UCSRB |= (1<<RXEN)|(1<<TXEN);
	UCSRC = (1<<UMSEL)|(1<<USBS)|(3<<UCSZ0);

}
void Uart_tr (char s[])
{
	unsigned char k;
	for (k=0;s[k]!=0;k++)
	{
		while (!(UCSRA&(1<<UDRE)));
		UDR =s[k];
		
	}
}
