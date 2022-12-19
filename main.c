#define F_CPU 8000000UL		//������� ������ ��
#define BAUDRATE 9600L

#include <avr/io.h>			//������������ ����������
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define false 0
#define true 1

#define LCD_DDR		DDRB				//���� �������
#define LCD_PORT	PORTB
#define E1			PORTB |= 0b00001000 //��������� ����� E � 1
#define E0			PORTB &= 0b11110111 //��������� ����� E � 0
#define RS1			PORTB |= 0b00000100 //��������� ����� RS � 1 (������)
#define RS0			PORTB &= 0b11111011 //��������� ����� RS � 0 (�������)

// ���� ������
#define BUT_DDR		DDRA
#define BUT_PORT	PORTA
#define BUT_PIN		PINA
#define BUT_NEXT	PA0

#define slaveF_SCL			400000	//400 ��� ��� ������� ������
#define TW_START			0x08
#define TW_REP_START		0x10
#define slaveAddressConst	0b1010	//���������� ����� ������ �������� ����������
#define slaveAddressVar		0b000	//���������� ����� ������ �������� ����������

//������� ����������� �������� ������
#define READFLAG	1	//������
#define WRITEFLAG	0	//������

//��������� ��� TWI
//master transmitter
#define TW_MT_SLA_ACK               0x18
#define TW_MT_DATA_ACK              0x28
//master receiver
#define TW_MR_SLA_ACK               0x40
#define TW_MR_DATA_NACK             0x58

char	rx_data[32];			//������ �������� ��� ������ �� usart
char	buffer[32];				//������ ��� ������ �������� �� �������

uint8_t		flag_next	= false;// ���� ������������ �������
										
const char	cmd_set_route[]	= "set\r";	//������� ��������� ��������
const char	cmd_get_route[]	= "get\r";	//������� ��������� ��������

static int usart_put_char(char c, FILE *stream);
static int lcd_put_char(char c, FILE *stream);
static FILE usart = FDEV_SETUP_STREAM(usart_put_char, NULL, _FDEV_SETUP_WRITE);
static FILE lcd = FDEV_SETUP_STREAM(lcd_put_char, NULL, _FDEV_SETUP_WRITE);

//������������� TWI
void ee_init(void)
{
	TWBR = (F_CPU / slaveF_SCL - 16) / (2 * /* TWI_Prescaler= 4^TWPS */1);
	if (TWBR < 10)
	TWBR = 10;
	TWSR &= (~((1<<TWPS1)|(1<<TWPS0)));
}

//������ ����� � ������ �������
uint8_t ee_write_byte(uint16_t address, uint8_t data)
{
	do
	{
		TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
		while(!(TWCR & (1<<TWINT)));
		
		if((TWSR & 0xF8) != TW_START)
		{
			return false;
		}
		TWDR = (slaveAddressConst<<4) + (slaveAddressVar<<1) + (WRITEFLAG);
		TWCR = (1<<TWINT) | (1<<TWEN);
		while(!(TWCR & (1<<TWINT)));
	} while((TWSR & 0xF8) != TW_MT_SLA_ACK);
	
	TWDR = (address>>8);
	TWCR = (1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

	if((TWSR & 0xF8) != TW_MT_DATA_ACK)
	{
		return false;
	}

	TWDR=(address);
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

	if((TWSR & 0xF8) != TW_MT_DATA_ACK)
	return false;

	TWDR=(data);
	TWCR=(1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

	if((TWSR & 0xF8) != TW_MT_DATA_ACK)
	return false;

	TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	while(TWCR & (1<<TWSTO));

	return true;
}

//������ ����� �� �������� ������
uint8_t ee_read_byte(uint16_t address)
{
	uint8_t data;
	
	do
	{
		TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
		while(!(TWCR & (1<<TWINT)));

		if((TWSR & 0xF8) != TW_START)
		return false;

		TWDR = (slaveAddressConst<<4) + (slaveAddressVar<<1) + WRITEFLAG;
		TWCR = (1<<TWINT)|(1<<TWEN);

		while(!(TWCR & (1<<TWINT)));
		
	}while((TWSR & 0xF8) != TW_MT_SLA_ACK);
	
	/*****�������� ����� ������********/
	TWDR = (address>>8);
	TWCR = (1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

	if((TWSR & 0xF8) != TW_MT_DATA_ACK)
	return false;

	TWDR = (address);
	TWCR = (1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

	if((TWSR & 0xF8) != TW_MT_DATA_ACK)
	return false;

	/*****������� � ����� ������********/
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

	if((TWSR & 0xF8) != TW_REP_START)
	return false;

	TWDR = (slaveAddressConst<<4) + (slaveAddressVar<<1) + READFLAG;

	TWCR = (1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

	if((TWSR & 0xF8) != TW_MR_SLA_ACK)
	return false;

	/*****��������� ���� ������********/
	TWCR = (1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

	if((TWSR & 0xF8) != TW_MR_DATA_NACK)
	return false;

	data = TWDR;
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	while(TWCR & (1<<TWSTO));
	
	return data;
}

//������������� usart
void usart_init()
{
	stderr = &usart;
	
	UBRRL = F_CPU / BAUDRATE / 16 - 1;	//8 000 000 / 9600 / 16 - 1 = 51
	UCSRB = (1<<TXEN)|(1<<RXEN);			//���������� ������ � ��������
	UCSRC =	(1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1);		//9 ���
	UCSRB |= (1<<RXCIE)|(1<<UCSZ2);		//���������� ���������� ��� ��������
}

//�������� ������� �� usart
static int usart_put_char(char c, FILE *stream)
{
	if (c == '\n')
	{
		usart_put_char('\r', stream);
	}
	
	while(!(UCSRA & (1<<UDRE)));
	UDR = c;
	
	return 0;
}

//����� ������ �� usart
void receiving_usart()
{
	memset(rx_data, 0, sizeof rx_data);
	
	int i = 0;
	do
	{
		while(!(UCSRA&(1<<RXC)));
		rx_data[i] = UDR;
		if (rx_data[i] == '\b')
		{
			i--;
			continue;
		}
		i++;
		
	} while (rx_data[i-1] != '\r');
}

int8_t get_number()
{
	int8_t number = 0;
	
	for (uint8_t element = 0; element < 2; element++)
	{
		if ((rx_data[element] > '9') || (rx_data[element] < '0'))
		{
			return -1;
		}
	}
	number = ((rx_data[0] & 0b00001111) * 10) + (rx_data[1] & 0b00001111);
	
	if (number > 20)
	{
		return -1;
	}
	return number;
}

//��������� ��������
void set_route()
{
	fprintf(stderr, "Enter the count of stantions (00-20): ");
	
	receiving_usart();
	
	//fprintf(stderr, "check_number: %d\n", get_number());

	int8_t count;
	count = get_number();
	
	if (count == -1)
	{
		fprintf(stderr, "Wrong count!\n");
	}
	else
	{
		ee_write_byte(0, count);
		_delay_ms(5);
		
		for (int stantion = 0; stantion < ee_read_byte(0); stantion++)
		{
			fprintf(stderr, "Enter stantions #%d:\n", stantion + 1);
			fprintf(stderr, "--------------------\n");
			receiving_usart();
			
			for (int letter = 0; letter < 20; letter++)
			{
				ee_write_byte(1 + (stantion * 20 + letter), rx_data[letter]);
				_delay_ms(5);
			}
		}
	}
}

//��������� ��������
void get_route()
{
	for (int stantion = 0; stantion < ee_read_byte(0); stantion++)
	{
		fprintf(stderr, "Stantion #%d: ", stantion + 1);
		for (int letter = 0; letter < 20; letter++)
		{
			fprintf(stderr, "%c", ee_read_byte(1 + (stantion * 20 + letter)));
		}
		fprintf(stderr, " \n");
	}
}

//���������� usart
ISR(USART_RXC_vect)
{
	receiving_usart();

	if (strcmp(rx_data, cmd_set_route) == 0)
	{
		set_route();
	}
	else if (strcmp(rx_data, cmd_get_route) == 0)
	{
		get_route();
	}
}

//�������� �������� � �������
void lcd_send_halfbyte(unsigned char c)
{
	c <<= 4;
	E1;						//��������� ����� �
	_delay_us(50);
	LCD_PORT &= 0b00001111; //������� ���������� �� ������ DB4-DB7, ��������� �� �������
	LCD_PORT |= c;
	E0;						//���������� ����� �
	_delay_us(50);
}

//�������� ����� � �������
void lcd_send_byte(unsigned char c, unsigned char mode)
{
	if (mode == 0)
	{
		RS0;
	}
	else
	{
		RS1;
	}
	
	unsigned char hc = 0;
	hc = c >> 4;
	
	lcd_send_halfbyte(hc);
	lcd_send_halfbyte(c);
}

//�������� ������� � �������
void lcd_send_char(unsigned char c)
{
	lcd_send_byte(c, 1);
}

//�������� ������� �� usart
static int lcd_put_char(char c, FILE *stream)
{
	if (c == '\n')
	{
		lcd_put_char('\r', stream);
	}
	
	lcd_send_char(c);
	
	return 0;
}

//��������� ��������� ������� �� �������
void lcd_set_pos(unsigned char x, unsigned y)
{
	switch(y)
	{
		case 0:
		lcd_send_byte(x | 0x80, 0);
		break;
		case 1:
		lcd_send_byte((0x40 + x) | 0x80, 0);
		break;
		case 2:
		lcd_send_byte((0x14 + x) | 0x80, 0);
		break;
		case 3:
		lcd_send_byte((0x54 + x) | 0x80, 0);
		break;
	}
}

//������������� �������
void lcd_init(void)
{
	stdout = &lcd;
	
	_delay_ms(15); //���� 15 �� (��� 45)
	lcd_send_halfbyte(0b00000011);
	_delay_ms(4);
	lcd_send_halfbyte(0b00000011);
	_delay_us(100);
	lcd_send_halfbyte(0b00000011);
	_delay_ms(1);
	lcd_send_halfbyte(0b00000010);
	_delay_ms(1);
	lcd_send_byte(0b00101000, 0); //4���-����� (DL=0) � 2 ����� (N=1)
	_delay_ms(1);
	lcd_send_byte(0b00001100, 0); //�������� ����������� �� ������� (D=1), ������� ������� �� �������� (C=0, B=0)
	_delay_ms(1);
	lcd_send_byte(0b00000110, 0); //������ (���� �� � ��� � ���������) ����� ��������� �����
	_delay_ms(1);
}

//�������� �������
void lcd_clear(void)
{
	lcd_send_byte(0b00000001, 0);
	_delay_us(1500);
}

//����� ������ � �������
void lcd_str (char str1[])
{
	wchar_t n;
	
	for (n=0; str1[n] != '\0'; n++)
	{
		lcd_send_char(str1[n]);
	}
}

//���������� �������
void lcd_update()
{
	static uint8_t current_stantion = 0;
	
	if (flag_next)
	{	
		lcd_clear();
		
		lcd_set_pos(0, 0);
		fprintf(stdout, "Next station:");
		lcd_set_pos(0, 1);
		
		for (int letter = 0; letter < 20; letter++)
		{
			fprintf(stdout, "%c", ee_read_byte(1 + (current_stantion * 20 + letter)));
		}
		
		current_stantion++;
		if (current_stantion >= ee_read_byte(0))
		{
			current_stantion = 0;
		}
		
		flag_next = false;
	}
}

//������� ������ ������
void ask_button()
{
	if (!(BUT_PIN & (1<<BUT_NEXT)))
	{
		while (!(BUT_PIN & (1<<BUT_NEXT)));
		flag_next = true;
	}
}

//������������� ���� ���������
void init()
{
	LCD_DDR	= 0xff;
	LCD_PORT = 0x00;
	
	BUT_DDR &= ~(1<<BUT_NEXT);
	BUT_PORT |= (1<<BUT_NEXT);
	
	usart_init();
	lcd_init();
	ee_init();
	
	sei();
}

//����� ����� � ���������
int main(void)
{
	init();

    while (1) 
    {
		ask_button();
		lcd_update();
    }
}

