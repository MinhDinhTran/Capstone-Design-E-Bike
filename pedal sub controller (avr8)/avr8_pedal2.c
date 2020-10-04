#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned char flag_tx;

unsigned int adc[3];
unsigned int adc_avg;

unsigned char data_tx;
unsigned char data_rx;

ISR(USART_RXC_vect)
{
	data_rx = UDR;
}

void uart_putch(unsigned char data)
{
	while(!(UCSRA & (1 << UDRE)));
	UDR = data;
}

unsigned char uart_getch(void)
{
	while(!(UCSRA & (1 << RXC)));

	return UDR;
}

int main(void)
{
	UCSRA = 0x00;
	UCSRB = 0x98;
	UCSRC = 0x06;
	UBRRH = 0x00;	// 115200
	UBRRL = 0x08;	// 115200

	ADMUX = 0x00;
	ADCSRA = 0x87;	

	sei();

	while(1)
	{
		if(data_rx == '#')
		{
			flag_tx = 1;

			ADMUX = 0x00;
			ADCSRA |= 0x40;
			while((ADCSRA & 0x10) == 0x00);
			adc[0] = ADC;
			ADCSRA &= ~0x10;

			ADMUX = 0x01;
			ADCSRA |= 0x40;
			while((ADCSRA & 0x10) == 0x00);
			adc[1] = ADC;
			ADCSRA &= ~0x10;

			ADMUX = 0x02;
			ADCSRA |= 0x40;
			while((ADCSRA & 0x10) == 0x00);
			adc[2] = ADC;
			ADCSRA &= ~0x10;

			adc_avg = (adc[0] + adc[1] + adc[2]) / 3;

			data_tx = (adc_avg >> 3);

			data_tx &= ~0x80;			// Left Pedal Mark
//			data_tx |= 0x80;			// Right Pedal Mark

			uart_putch(data_tx);
		}
		else if(data_rx == '$')
		{
			flag_tx = 0;
		}	
		
		_delay_ms(10);	
	}
}
