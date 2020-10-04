#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// user-defined headers
#include "clcd.h"
#include "AbsEncoder.h"
#include "uart2.h"
#include "spi.h"
#include "moment.h"				// unit: milimeter

#define OFF			0
#define ON			1

#define HALLMAX		1100

#define N_GRAV		9.8
#define CONST_PR	0.68		// Raw to Kg 환산 계수
#define CONST_PL	0.782		// Raw to Kg 환산 계수

unsigned int i;
/////////////////////////////////////////////
unsigned char flagMode = 0;
unsigned char flagLCD = 0;		// flag for Character LCD page

unsigned int flagTimer = 0;
unsigned int tcnt;

unsigned char btTXmode = 0;
unsigned char btConn[2];
unsigned char btConn_old[2];

unsigned char uart0TXmode = 0;
unsigned long cntUART0 = 0;

unsigned char uart3TXmode = 0;
unsigned long cntUART3 = 0;
/////////////// Volt & Amp //////////////////
unsigned long adc0;
float volt;
float voltPC;

unsigned long adc1;
unsigned int adc1_ref;
float amp;
float ampPC;
/////////////// Power ///////////////////////
float PowerMotor;
float PowerHuman;
///////////////// Encoder ///////////////////
float ANGLE_DIV = 2.844;

unsigned int encoder_new = 0;
unsigned int encoder_old = 0;
int encoder_diff = 0;
int encoder_diff_old = 0;
int encoder_vel[20];
int encoder_velMean;

int encoder_angle = 0;
int angle_calib;

unsigned int angleR;
unsigned int angleR_old;
unsigned int angleL;
unsigned int angleL_old;
int angle4;

float pedalSpeed;
/////////////// UART Buffer /////////////////
unsigned char udr0;
unsigned char udr1;
unsigned char udr2;
unsigned char udr3;
/////////////////////////////////////////////
unsigned char pedal[2];

unsigned int moment_R;
unsigned int moment_L;

int forceR;
int forceL;

int torqueR;
int torqueL;
int torqueA;
int torque4;

float gain = 0.015;
float gain4 = 1.8;

unsigned int memTorque[360];

unsigned int offset = 1080;

unsigned char flagDAC = 0;
unsigned int valDAC = 0;
/////////////////////////////////////////////

////////////// Hall Sensor //////////////////
unsigned char hallCnt;
unsigned int hallPeriod;
unsigned char hallDiff;
unsigned int hallRes;

float hallSpeed;
/////////////////////////////////////////////

/////////// Serial Comm Buffers with Android /////////////
unsigned char txBikeSp;
unsigned char txVolt;
unsigned char txHPow;
unsigned char txMPow;

unsigned char rxCMD[6];
unsigned char rxCMD1_old;
///////////////////////////////////////////////////////////

ISR(TIMER1_COMPA_vect);
ISR(USART0_RX_vect);
ISR(USART1_RX_vect);

// Moving Average Filter
int16_t MAF(uint8_t arraySize, volatile int16_t* arrayData, int16_t newData)
{
	int32_t avgData = 0;	
 
	// Rearrange Data(d[n] -> d[n+1])
	for(i = arraySize - 1; i > 0; i--)
	{
		arrayData[i] = arrayData[i - 1];
		avgData += arrayData[i];
	}
	
	// insert new data
	arrayData[0] = newData;
	avgData += arrayData[0];
 
	// calculate average
	avgData = avgData / arraySize;
 
	return (int16_t)avgData;
}

void GetPedalLValue(void)
{
	PORTB &= ~UART1_CTS;					// UART1 Request Send, Active Low
	if((UCSR1A & RXC_FLAG) == 0x80)
	{
		udr1 = UDR1;
		if(btTXmode == 0)
		{
			#ifdef COM_MONITOR
			uart0_putch(udr1);
			#endif
		}
		else if(btTXmode == 1 && btConn[0] == ON)
		{
			pedal[0] = udr1;
		}
	}
	PORTB |= UART1_CTS;						// UART1 Stop Send
}

void GetPedalRValue(void)
{
	PORTH &= ~UART2_CTS;					// UART2 Request Send, Active Low
	if((UCSR2A & RXC_FLAG) == 0x80)				
	{
		udr2 = UDR2;
		if(btTXmode == 0)
		{
			#ifdef COM_MONITOR
			uart0_putch(udr2);
			#endif
		}
		else if(btTXmode == 1 && btConn[1] == ON)
		{
			pedal[1] = udr2;
		}
	}
	PORTH |= UART2_CTS;						// UART2 Stop Send	
}

int GetCrankVelocity(void)
{
	int encoder_dff;
	encoder_old = encoder_new;
	encoder_new = AbsEncoder_Get();
	encoder_dff = encoder_new - encoder_old;

	if(encoder_dff <= -600)				// 엔코더값이 overflow될때 속도값처리
	{
		encoder_dff += 1023;
	}

	return encoder_dff;
}

/*
void MCP4901_Write(unsigned char buff, unsigned char gain, unsigned char shutdown, unsigned char volt)
{	// DAC Value = 0 ~ 255 (8bit)
	SP_CS1_LOW;

	spi_txrx((buff << 6) | (gain << 5) | (shutdown << 4) | (volt >> 4));
	spi_txrx((volt << 4) & 0xFF);

	SP_CS1_HIGH;
}
*/

void MCP4921_Write(unsigned char buff, unsigned char gain, unsigned char shutdown, unsigned int volt)
{	// DAC Value = 0 ~ 4095 (12bit)
	SP_CS1_LOW;

	spi_txrx((buff << 6) | (gain << 5) | (shutdown << 4) | ((volt >> 8) & 0x0F));
	spi_txrx(volt & 0xFF);

	SP_CS1_HIGH;
}

void GetHallSensor(void)
{
	if(hallCnt != TCNT0)
	{
		hallCnt = TCNT0;
		if(hallPeriod > 20)
		{
			hallRes = hallPeriod;
			hallPeriod = 0;
		}
	}
	else					// hallCnt == TCNT0
	{
		if(hallPeriod <= HALLMAX)
		{
			hallPeriod++;
		}
		else
		{
			hallRes = HALLMAX;
		}
	}
}

ISR(TIMER1_COMPA_vect)
{
/////////////////////////////////////////////////////////////////////////////////////
	// 위쪽 블루투스 모듈 => 왼쪽 페달
	GetPedalLValue();
/////////////////////////////////////////////////////////////////////////////////////
	// 아래 블루투스모듈 => 오른쪽 페달
	GetPedalRValue();
/////////////////////////////////////////////////////////////////////////////////////
	if(flagTimer == 0)
	{
		btConn_old[0] = btConn[0];
		// Bluetooth 접속 상태 확인
		if((PINB & UART1_DCD) != 0)
		{
			btConn[0] = OFF;
		}
		else
		{
			btConn[0] = ON;
		}

		btConn_old[1] = btConn[1];
		if((PINB & UART2_DCD) != 0)
		{
			btConn[1] = OFF;
		}
		else
		{
			btConn[1] = ON;
		}

		// 한쪽이 연결되어 있는데, 다른쪽이 꺼져있다 켜질 경우, 자동으로 센서값 수신시작
		if(btConn_old[0] == OFF && btConn[0] == ON && btConn[1] == ON)		
		{
			btTXmode = 1;
			uart1_putch('#');
			uart2_putch('#');			
		}
		else if(btConn_old[1] == OFF && btConn[1] == ON && btConn[0] == ON)
		{
			btTXmode = 1;
			uart1_putch('#');
			uart2_putch('#');	
		}
	}
	else if(flagTimer == 1)
	{
		// Encoder 데이터 받음
		encoder_diff_old = encoder_diff;
		encoder_diff = GetCrankVelocity();
		encoder_angle = (int)(encoder_new / ANGLE_DIV);	

		encoder_velMean = MAF(5, encoder_vel, encoder_diff);
	}
	else if(flagTimer == 2)
	{
		// error exception
		angleR_old = angleR;
		angleL_old = angleL;

		// calculating current moment length
		angle_calib = (int)encoder_angle - 163;
		if(angle_calib < 0)
		{
			angle_calib = 360 + angle_calib;
		}
		angleR = angle_calib;

		angle_calib += 90;		
		if(angle_calib >= 360)
		{
			angle_calib = angle_calib - 360;
		}
		angleL = angle_calib;

		// Error exception
		if(angleR > 0 && angleR < 360)
		{
			moment_R = moment[angleR];
		}

		if(angleL > 0 && angleL < 360)
		{
			moment_L = moment[angleL];
		}
	}
	else if(flagTimer == 3)
	{
		// 페달 힘값 및 토크 계산
		if(btTXmode == 1 && btConn[1] == ON && btConn[0] == ON)
		{
			// Dimension => Kg
			forceR = (pedal[1] * 0.68) - 3;
			forceL = (pedal[0] * 0.782) - 3;		// 0.68 * 1.15(보정계수)

			if(forceR < 0)
			{
				forceR = 0;
			}

			if(forceL < 0)
			{
				forceL = 0;
			}
		}
		else
		{
			forceR = 1;
			forceL = 1;
		}

		// Error exception
		if(forceR > 70)
		{
			forceR = 0;
		}
		if(forceL > 70)
		{
			forceL = 0;
		}
		
		torqueR = forceR * moment_R;
		if(torqueR < 0)
		{
			torqueR = 0;
		}
		torqueL = forceL * moment_L;
		if(torqueL < 0)
		{
			torqueL = 0;
		}

		torqueA = torqueR + torqueL;
	}
	else if(flagTimer == 4)
	{
		// Virtual 4 cycling system processing
		if((flagMode == 2) || (flagMode == 3))
		{
			// Error exception
			if(angleR > 0 && angleR < 360)
			{
				memTorque[angleR] = torqueA;
			}
			else
			{
				memTorque[angleR_old] = torqueA;
			}


			if(angleR-1 > 0)
			{
				memTorque[angleR-1] = torqueA;
			}
			if(angleR+1 < 360)
			{
				memTorque[angleR+1] = torqueA;
			}
			
			angle4 = angleR - 45;
			if(angle4 < 0)
			{
				angle4 += 360;
			}

			torque4 = memTorque[angle4];
		}
		else
		{
			for(i = 0; i < 360; i++)
			{
				memTorque[i] = 0;
			}
		}
	}
	else if(flagTimer == 5)
	{
		// Driving Motor using DAC
		if(flagMode == 0)				// no Motor
		{
			valDAC = 0;
		}
		else if(flagMode == 1)			// PAS
		{
			valDAC = torqueA * gain + offset;
		}
		else if(flagMode == 2)			// Virtual 4
		{
			valDAC = torque4 * (gain4 * gain) + offset;
		}
		else if(flagMode == 3)			// PAS + Virtual 4
		{
			valDAC = (torqueA * gain) + (torque4 * (gain4 * gain)) + offset;
		}

		if(valDAC > 4095)				// DAC value saturation
		{
			valDAC = 4095;
		}

		if((flagDAC == 1) && flagMode != 0 && (encoder_velMean >= 1))
		{
			MCP4921_Write(1, 1, 1, valDAC);
		}
		else
		{
			valDAC = 0;
			MCP4921_Write(1, 1, 0, valDAC);
		}
	}
	else if(flagTimer == 6)
	{
		ADMUX = 0x40;		// VRef, 10bit resolution, AD Channel 0
		ADCSRA |= 0x40;
		while((ADCSRA & 0x10) == 0x00);
		adc0 = ADC;
		ADCSRA &= ~0x10;
		volt = ((float)(adc0 * (float)0.04878));

		voltPC = volt * 10.0;
		txVolt = (unsigned char)volt;
	}
	else if(flagTimer == 7)
	{
		ADMUX = 0x41;		// VRef, 8bit resolution, AD Channel 1
		ADCSRA |= 0x40;
		while((ADCSRA & 0x10) == 0x00);
		adc1 = ADC;
		ADCSRA &= ~0x10;
		amp = (float)(adc1);
//		amp = ((amp -  512) * (float)4.8828) / (float)60.0;			// (XmV - (Vcc/2)) / 60mV = Current(A)
		amp = amp * (float)4.8828;									// mV
		if(amp < 0)
		{
			amp = 0;
		}

		ampPC = amp;
	}
	else if(flagTimer == 8)
	{
		hallSpeed = (float)932.58 / hallRes;									// km/h
		txBikeSp = (unsigned char)hallSpeed;
	}
	else if(flagTimer == 9)
	{
		pedalSpeed = encoder_diff * (float)2.9296;								// RPM	
	}
	else if(flagTimer == 10)
	{
		PowerHuman = ((torqueA * (float)0.00981) * pedalSpeed) * (float)0.1047;	// torque * 9.81/1000 => N*m,   
		if(PowerHuman > 255)
		{
			txHPow = 255;
		}
		else if(PowerHuman < 0)
		{
			txHPow = 0;
		}
		else
		{
			txHPow = (unsigned char)PowerHuman;
		}

		PowerMotor = volt * ((amp - 2510) / 60.0);
		if(PowerMotor > 255)
		{
			txMPow = 255;
		}
		else if(PowerMotor < 0)
		{
			PowerMotor = 0;
			txMPow = 0;
		}
		else
		{
			txMPow = (unsigned char)PowerMotor;
		}
	}
	else if(flagTimer == 11)
	{
		if(uart3TXmode == 1)
		{
			uart3_putch((unsigned int)(voltPC) >> 8);
			uart3_putch((unsigned int)(voltPC));
			uart3_putch((unsigned int)(ampPC) >> 8);
			uart3_putch((unsigned int)(ampPC));
			uart3_putch(forceR);
			uart3_putch(forceL);
			uart3_putch(angleR >> 8);
			uart3_putch(angleR);
		}
	}
	else if(flagTimer == 12)
	{
		if(uart3TXmode == 1)
		{
			uart3_putch((unsigned char)(hallRes >> 8));
			uart3_putch((unsigned char)(hallRes));
			uart3_putch((unsigned char)encoder_diff);
			uart3_putch(torqueA >> 8);
			uart3_putch(torqueA);
			uart3_putch(torque4 >> 8);
			uart3_putch(torque4);
			uart3_putch('E');
			uart3_putch('D');
		}
	}
	else if(flagTimer == 13)
	{
		// Currently Nothing to do
	}
	else if(flagTimer == 14)
	{
		// Currently Nothing to do
	}
	else if(flagTimer == 15)
	{
		// Currently Nothing to do
	}
	else if(flagTimer == 16)
	{
		if(uart0TXmode == 1)
		{
			uart0_putch('<');
			uart0_putch('S');
			uart0_putch(txBikeSp);
		}
	}
	else if(flagTimer == 17)
	{
		if(uart0TXmode == 1)
		{
			uart0_putch(',');
			uart0_putch('V');
			uart0_putch(txVolt);
			uart0_putch(',');
		}	
	}
	else if(flagTimer == 18)
	{
		if(uart0TXmode == 1)
		{
			uart0_putch('H');
			uart0_putch(txHPow);
			uart0_putch(',');
		}
	}
	else if(flagTimer == 19)
	{
		if(uart0TXmode == 1)
		{
			uart0_putch('M');
			uart0_putch(txMPow);
			uart0_putch('>');
		}
	}
	else if(flagTimer == 20)
	{
		if(btTXmode == 0)
		{
			pedal[0] = 1;
			pedal[1] = 1;
		}
	}
/////////////////////////////////////////////////////////////////////////////////////
	GetHallSensor();
/////////////////////////////////////////////////////////////////////////////////////
	if(flagTimer == 20)
	{
		flagTimer = 0;
	}
	else
	{
		flagTimer++;
	}
	tcnt = TCNT1;

//	uart3_putch(tcnt);
}

ISR(USART0_RX_vect)
{
	udr0 = UDR0;

	if(udr0 == '#')
	{
		btTXmode = 1;
		uart1_putch('#');
		uart2_putch('#');
	}
	else if(udr0 == '$')
	{
		btTXmode = 0;
		uart1_putch('$');
		uart2_putch('$');
	}
	else if(udr0 == '%')
	{
		flagDAC ^= 1;
		PORTE ^= (1 << 3);
	}
	else
	{
		uart1_putch(udr0);
		uart2_putch(udr0);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
void init_timer(void)
{
	TCCR0A = 0x00;
	TCCR0B = 0x07;		// External clock source on T0(PD7) pin(rising edge)

	TCCR1A = 0x00;
	TCCR1B = 0x0B;		// CTC Mode, prescaler 64, 1clk = 4us

	OCR1A = 250;		// 2500 = 10ms, 250 = 1ms
	TCNT1 = 0;

	TIMSK1 = 0x02;		// T1 Compare Match A Enable
}

void init_adc(void)
{
	DDRF = 0x00;

	ADMUX = 0x00;		// VRef, 8bit resolution, AD Channel 0
	ADCSRA = 0x87;		// ADC enable, prescaler 128
}

void init_uart1_flowctrl(void)
{
	DDRB |= UART1_CTS;	
	DDRB &= ~UART1_RTS;

	PORTB |= UART1_CTS;
}

void init_uart2_flowctrl(void)
{
	DDRH |= UART2_CTS;	
	DDRH &= ~UART2_RTS;

	PORTH |= UART2_CTS;
}

void Button(void)
{
	if((PINL & 0x01) != 0x00)	// SW0
	{
		_delay_ms(5);
		while((PINL & 0x01) != 0x00);
		_delay_ms(5);

		if(btTXmode == 0)
		{
			btTXmode = 1;
			uart1_putch('#');
			uart2_putch('#');
		}
		else if(btTXmode == 1)
		{
			btTXmode = 0;
			uart1_putch('$');
			uart2_putch('$');
		}
	}
	if((PINL & 0x02) != 0x00)	// SW1
	{
		_delay_ms(5);
		while((PINL & 0x02) != 0x00);
		_delay_ms(5);

		uart0TXmode++;
		if(uart0TXmode == 2)
		{
			uart0TXmode = 0;
		}
	}
	if((PINL & 0x04) != 0x00)	// SW2
	{
		_delay_ms(5);
		while((PINL & 0x04) != 0x00);
		_delay_ms(5);

		if(flagDAC == 0)
		{
			flagDAC = 1;
		}
		else if(flagDAC == 1)
		{
			valDAC = 0;
			flagDAC = 0;
		}
	}
	if((PINL & 0x08) != 0x00)	// SW3
	{
		_delay_ms(5);
		while((PINL & 0x08) != 0x00);
		_delay_ms(5);

		flagMode++;
		if(flagMode == 4)
		{
			flagMode = 0;
		}
	}
	if((PINL & 0x10) != 0x00)	// SW4
	{
		_delay_ms(5);
		while((PINL & 0x10) != 0x00);
		_delay_ms(5);

		gain += 0.001;

//		offset += 1;
	}
	if((PINL & 0x20) != 0x00)	// SW5
	{
		_delay_ms(5);
		while((PINL & 0x20) != 0x00);
		_delay_ms(5);

		gain -= 0.001;

//		offset -= 1;
	}
	if((PINL & 0x40) != 0x00)	// SW6
	{
		_delay_ms(5);
		while((PINL & 0x40) != 0x00);
		_delay_ms(5);

		if(flagLCD != 0)
		{
			clear_LCD();
			flagLCD -= 1;
		}
	}
	if((PINL & 0x80) != 0x00)	// SW7
	{
		_delay_ms(5);
		while((PINL & 0x80) != 0x00);
		_delay_ms(5);

		if(flagLCD != 3)
		{
			clear_LCD();
			flagLCD += 1;
		}
	}
}

void displayCLCD(void)
{
	write_num1(19, 3, flagLCD);

	if(flagLCD == 0)
	{
		write_string(0, 0, "Enc:");
		write_num1000(4, 0, encoder_new);
		write_string(8, 0, ">");
		write_num100(9, 0, encoder_angle);
		write_string(12, 0, ">");
		write_num100(13, 0, angleR);

		write_string(0, 1, "PL:");
		write_num100(3, 1, pedal[0]);
		write_string(6, 1, ">");
		write_num100(7, 1, forceL);
		write_string(10, 1, "kg");
		if(btConn[0] == 1)
		{
			write_string(13, 1, "L-ON ");
		}
		else
		{
			write_string(13, 1, "L-OFF");
		}

		write_string(0, 2, "PR:");
		write_num100(3, 2, pedal[1]);
		write_string(6, 2, ">");
		write_num100(7, 2, forceR);
		write_string(10, 2, "kg");
		if(btConn[1] == 1)
		{
			write_string(13, 2, "R-ON ");
		}
		else
		{
			write_string(13, 2, "R-OFF");
		}

		write_string(0, 3, "T0:");
		write_num100(3, 3, hallCnt);
		write_num1000(7, 3, hallRes);
		write_string(12, 3, "Mode:");
		write_num1(17, 3, flagMode);
	}
	if(flagLCD == 1)
	{
		write_string(0, 0, "DAC:");
		write_num1(4, 0, flagDAC);
		write_string(6, 0, "Val:");
		write_num1000(10, 0, valDAC);
		write_num100(17, 0, tcnt);

		write_string(0, 1, "Deg:");
		write_num100(4, 1, angleR);
		write_string(8, 1, "Gain:");
		write_num100(13, 1, gain*1000);

		write_string(0, 2, "M(L,R):");
		write_snum1000(7, 2, moment_L);
		write_snum1000(12, 2, moment_R);

		write_string(0, 3, "Tor:");
		write_num1000(4, 3, torqueL);
		write_string(8, 3, "+"); 
		write_num1000(9, 3, torqueR);
		write_string(13, 3, "=");
		write_num1000(14, 3, torqueA);
	}
	if(flagLCD == 2)
	{
		write_string(0, 0, "Volt:");
		write_num1000(6, 0, adc0);
		write_string(10, 0, "=>"); 
		write_num100(12, 0, (unsigned int)(volt * 10)); 

		write_string(0, 1, "Amp:");
		write_num1000(6, 1, adc1);
		write_string(10, 1, "=>"); 
		write_num1000(12, 1, (unsigned int)(amp)); 

		write_string(0, 2, "HS:");
		write_num100(3, 2, (unsigned int)(hallSpeed));
		write_string(8, 2, "PS:");
		write_num100(11, 2, (unsigned int)(pedalSpeed));

		write_string(0, 3, "PM:");
		write_num1000(3, 3, PowerMotor);
		write_string(8, 3, "PH:");
		write_num1000(11, 3, PowerHuman);
	}
	if(flagLCD == 3)
	{
		write_string(0, 0, "UDR:");
		write_hex(4, 0, udr0);

		write_num1(0, 1, rxCMD[0]);
		write_num1(2, 1, rxCMD[1]);
		write_num1(4, 1, rxCMD[2]);
		write_num1(6, 1, rxCMD[3]);

		write_string(0, 2, "EncDiff:");
		write_snum100(8, 2, encoder_diff);

		write_string(0, 3, "EncMean:");
		write_snum100(8, 3, encoder_velMean);
	}
}

void RecvFromPhone(void)
{
	if((UCSR0A & RXC_FLAG) == 0x80)	
	{
		udr0 = UDR0;

		rxCMD[0] = (udr0 & 0x70) >> 4;		// Motor Mode

		rxCMD1_old = rxCMD[1];
		rxCMD[1] = (udr0 & 0x80) >> 7;		// Bluetooth On/Off
		
		rxCMD[2] = (udr0 & 0x0F);			// Acc Sensor

		if(rxCMD[0] == 1)
		{
			flagMode = 0;
		}
		else if(rxCMD[0] == 2)
		{
			flagMode = 1;
		}
		else if(rxCMD[0] == 3)
		{
			flagMode = 3;
		}

		if(rxCMD[1] == 1 && rxCMD1_old == 0)
		{
			btTXmode = 1;
			uart1_putch('#');
			uart2_putch('#');
		}
		else if(rxCMD[1] == 0 && rxCMD1_old == 1)
		{
			btTXmode = 0;
			uart1_putch('$');
			uart2_putch('$');
		}
	}
}

void SendToPhone(void)
{
	if(cntUART0 < 2)
	{
		cntUART0++;
	}
	else
	{
		cntUART0 = 0;

		uart0_putch('<');
		uart0_putch('S');
		uart0_putch(txBikeSp);
		uart0_putch(',');
		uart0_putch('V');
		uart0_putch(txVolt);
		uart0_putch(',');
		uart0_putch('H');
		uart0_putch(txHPow);
		uart0_putch(',');
		uart0_putch('M');
		uart0_putch(txMPow);
		uart0_putch('>');		
	}
}

void RecvFromPC(void)
{
	if((UCSR3A & RXC_FLAG) == 0x80)	
	{
		udr3 = UDR3;
		if(udr3 == '@')
		{
			udr3 = 0;
			if(uart3TXmode == 1)
			{
				uart3TXmode = 0;
			}
			else
			{
				uart3TXmode = 1;
			}
		}
	}
}

void SendToPC(void)
{
	if(cntUART3 < 2)
	{
		cntUART3++;
	}
	else
	{
		cntUART3 = 0;
		if(uart3TXmode == 1)
		{
			uart3_putch((unsigned int)(voltPC) >> 8);
			uart3_putch((unsigned int)(voltPC));
			uart3_putch((unsigned int)(ampPC) >> 8);
			uart3_putch((unsigned int)(ampPC));
			uart3_putch(forceR);
			uart3_putch(forceL);
			uart3_putch(angleR >> 8);
			uart3_putch(angleR);
			uart3_putch((unsigned char)(hallRes >> 8));
			uart3_putch((unsigned char)(hallRes));
			uart3_putch((unsigned char)encoder_diff);
			uart3_putch(torqueA >> 8);
			uart3_putch(torqueA);
			uart3_putch(torque4 >> 8);
			uart3_putch(torque4);
			uart3_putch('E');
			uart3_putch('D');
		}
	}
}

int main(void)
{
	_delay_ms(100);

	DDRE |= 0x08;
	DDRL = 0x00;			// Button0 ~ Button7

	init_LCD();
	init_AbsEncoder();
	init_spi();
	init_timer();
	init_adc();

	// 보드에 장착된 블루투스 모듈이 왼쪽 페달, 외부기판에 장착된 블루투스 모듈이 오른쪽 페달
	init_uart0();	// Phone
	init_uart1();	// BT1
	init_uart2();	// BT2
	init_uart3();	// PC

	init_uart1_flowctrl();
	init_uart2_flowctrl();

	write_string(0, 0, "Power Assist e-Bike");
	_delay_ms(2000);
	clear_LCD();

/*
	// Get Current Reference
	ADMUX = 0x41;		// VRef, 8bit resolution, AD Channel 1
	ADCSRA |= 0x40;
	while((ADCSRA & 0x10) == 0x00);
	adc1_ref = ADC;
	ADCSRA &= ~0x10;
*/

	sei();

	flagDAC = 1;			// DAC Enable
	flagMode = 0;			// Motor Mode None

	uart0TXmode = 1;		// to Android Phone
	uart3TXmode = 0;		// to PC

	while(1)
	{
		if(uart0TXmode == 1)
		{
			RecvFromPhone();
//			SendToPhone();
		}

		RecvFromPC();
//		SendToPC();
/////////////////////////////////////////////////////////////////////////////////////
		displayCLCD();
		Button();				
	}
}
