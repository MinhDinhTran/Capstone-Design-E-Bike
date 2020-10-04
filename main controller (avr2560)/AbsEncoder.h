void init_AbsEncoder(void);
unsigned int AbsEncoder_Get(void);

void init_AbsEncoder(void)
{
	DDRA &= ~0xFF;		// 0 ~ 7bit
	DDRC &= ~0x03;		// 8 ~ 9bit
}

unsigned int AbsEncoder_Get(void)
{
	unsigned char encL;
	unsigned char encH;
	unsigned int enc;

	encL = PINA;
	encH = (PINC & 0x03);
	enc = encL | ((unsigned int)(encH << 8));

	return enc;
}
