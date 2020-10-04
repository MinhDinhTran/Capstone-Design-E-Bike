#define DDR_LCD 	DDRC
#define PORT_LCD 	PORTC
#define RS			0x04
#define E			0x08

typedef unsigned char BYTE;
typedef unsigned int WORD;

#ifndef _USER_DELAY_
#define _USER_DELAY_
void delay_us(unsigned char time_us);
void delay_ms(unsigned int time_ms);

void delay_us(unsigned char time_us)	
{
	register unsigned char i;

	for(i = 0; i < time_us; i++)	        // 4 cycle +
    {
		asm volatile(" PUSH  R0 ");		// 2 cycle +
      	asm volatile(" POP   R0 ");		// 2 cycle +
      	asm volatile(" PUSH  R0 ");		// 2 cycle +
      	asm volatile(" POP   R0 ");		// 2 cycle +
      	asm volatile(" PUSH  R0 ");		// 2 cycle +
      	asm volatile(" POP   R0 ");		// 2 cycle = 16 cycle = 1 us for 16MHz
    }
}

void delay_ms(unsigned int time_ms)		
{
	register unsigned int i;

  	for(i = 0; i < time_ms; i++)
    {
		delay_us(250);
      	delay_us(250);
    	delay_us(250);
      	delay_us(250);
    }
}
#endif

void LCD_command(unsigned char command);
void init_LCD(void);
void clear_LCD(void);
void shift_display(BYTE dir, BYTE x);
void write_string(BYTE x, BYTE y, char *str);
void write_word(BYTE x, BYTE y, char data);
void set_coordinate(BYTE x, BYTE y);
void LCD_data(unsigned char data);
void write_num1000(BYTE x, BYTE y, WORD num);
void write_snum1000(BYTE x, BYTE y, int num);
void write_num100(BYTE x, BYTE y, WORD num);
void write_num10(BYTE x, BYTE y, BYTE num);
void write_num1(BYTE x, BYTE y, BYTE num);
void write_hex(BYTE x, BYTE y, BYTE hex);

void LCD_command(unsigned char command)		/* write a command(instruction) to text LCD */
{
  	PORT_LCD = (command & 0xF0) | E; // 상위 명령 | Enable
	delay_us(1);
	PORT_LCD = 0x00;
	delay_us(1);

	PORT_LCD = ((command<<4) & 0xF0) | E; // 하위 명령 | Enable 0x84
	delay_us(1);
	PORT_LCD = 0x00;
	delay_us(1);

	delay_us(50);
}

void init_LCD(void)			/* initialize text LCD module */
{
	DDR_LCD |= (RS | E);
	DDR_LCD |= 0xF0;

	LCD_command(0x28);			// 0010 1000b function set(funcion, 4 bit, 2 line, 5x8 dot)
	delay_ms(2);
	LCD_command(0x28);			// function set(funcion, 4 bit, 2 line, 5x8 dot)
	delay_ms(2);
	LCD_command(0x0C);			// display control(display ON, cursor OFF, blink OFF)
	delay_ms(2);
	LCD_command(0x06);			// entry mode set(increment, not shift)
	delay_ms(2);
	LCD_command(0x01);			// clear display
	delay_ms(2);
}

void clear_LCD(void)
{
	LCD_command(0x01);			// clear display
	delay_ms(2);
}

void shift_display(BYTE dir, BYTE x)
{
	BYTE i;
	for(i=0;i<x;i++){
		if(dir == 0)
			LCD_command(0x10 | 0x0C); 	//오른쪽
		else if(dir == 1)
			LCD_command(0x10 | 0x08);  	// 왼쪽
		delay_us(20);
		delay_ms(500);
	}
}

void write_string(BYTE x, BYTE y, char *str)
{
	set_coordinate(x, y); 				//좌표설정
    while(*str) LCD_data(*str++); 		// 데이터 쓰기
}

void write_word(BYTE x, BYTE y, char data)
{
	set_coordinate(x, y); 				//좌표설정
	LCD_data(data); 					// 데이터 쓰기
}

void set_coordinate(BYTE x, BYTE y)		/* character의 좌표설정*/
{ 	
	// DDRAM Address Setting
	if (y == 0)
	{
		LCD_command(0x80 + x); 			// 1째줄 시작 좌표 + 사용자 좌표
	}
	else if (y == 1)
	{
		LCD_command(0x80 + 0x40 + x); 	// 2째줄 시작 좌표 + 사용자 좌표
	}
	else if (y == 2)
	{
		LCD_command(0x80 + 0x14 + x);	// 3째줄 시작 좌표 + 사용자 좌표
	}
	else if (y == 3)
	{
		LCD_command(0x80 + 0x54 + x);	// 4째줄 시작 좌표 + 사용자 좌표
	}

//	LCD_command(0x80 | ((y % 2) * 0x40) | (x % 0x40));
	delay_us(40);
}

void LCD_data(unsigned char data)		/* display a character on text LCD */
{
	PORT_LCD = (data & 0xF0) | E | RS; 		//상위 데이터 | Enable | RS
	delay_us(1);
	PORT_LCD = 0x00;
	delay_us(1);

	PORT_LCD = ((data<<4) & 0xF0) | E | RS; 	//하위 데이터 | Enable | RS
	delay_us(1);
	PORT_LCD = 0x00;
	delay_us(1);

	delay_us(50);
}

void write_num1000(BYTE x, BYTE y, WORD num)
{
	char temp[5];
	
	if(num > 9999)
	{
		num = 9999;
	}

	temp[0] = num / 1000 + '0';
	num = num % 1000;

	temp[1] = num / 100 + '0';
	num = num % 100;

	temp[2] = num / 10 + '0';
	num = num % 10;

	temp[3] = num + '0';

	temp[4] = 0;

	write_string(x, y, temp);
}

void write_snum1000(BYTE x, BYTE y, int num)
{
	char temp[6];
	
	if(num >= 0)
	{
		temp[0] = '+';
	}
	else
	{
		temp[0] = '-';
		num = -num;
	}

	if(num > 9999)
	{
		num = 9999;
	}

	temp[1] = num / 1000 + '0';
	num = num % 1000;

	temp[2] = num / 100 + '0';
	num = num % 100;

	temp[3] = num / 10 + '0';
	num = num % 10;

	temp[4] = num + '0';

	temp[5] = 0;

	write_string(x, y, temp);
}

void write_num100(BYTE x, BYTE y, WORD num)
{
	char temp[4];

	if(num > 999)
	{
		num = 999;
	}
		
	temp[0] = num / 100 + '0';
	num = num % 100;

	temp[1] = num / 10 + '0';
	num = num % 10;

	temp[2] = num + '0';

	temp[3] = 0;

	write_string(x, y, temp);
}

void write_snum100(BYTE x, BYTE y, int num)
{
	char temp[5];
	
	if(num >= 0)
	{
		temp[0] = '+';
	}
	else
	{
		temp[0] = '-';
		num = -num;
	}

	if(num > 9999)
	{
		num = 9999;
	}

	temp[1] = num / 100 + '0';
	num = num % 100;

	temp[2] = num / 10 + '0';
	num = num % 10;

	temp[3] = num + '0';

	temp[4] = 0;

	write_string(x, y, temp);
}

void write_num10(BYTE x, BYTE y, BYTE num)
{
	char temp[3];

	if(num > 99)
	{
		num = 99;
	}
		
	temp[0] = num / 10 + '0';
	num = num % 10;

	temp[1] = num + '0';

	temp[2] = 0;

	write_string(x, y, temp);
}

void write_num1(BYTE x, BYTE y, BYTE num)
{
	char temp[2];

	if(num > 9)
	{
		num = 9;
	}

	temp[0] = num + '0';

	temp[1] = 0;

	write_string(x, y, temp);
}

void write_hex(BYTE x, BYTE y, BYTE hex)
{
	char hexa_h, hexa_l;
	char temp_hexa[3];

	hexa_h = hex >> 4;
	if (hexa_h >= 10) 
	{
		temp_hexa[0] = hexa_h - 10 + 'A';
	}
	else
	{			
		temp_hexa[0] = hexa_h + '0';
	}

	hexa_l = hex & 0x0F;
	if (hexa_l >= 10) 
	{
		temp_hexa[1] = hexa_l - 10 + 'A';
	}
	else			
	{
		temp_hexa[1] = hexa_l + '0';
	}

	temp_hexa[2]=0;

	write_string(x, y,temp_hexa);
}
