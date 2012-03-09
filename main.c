#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include "main.h"
#include "lcd.h"
#include "usart.h"
#include "timer.h"






int main (void)
{


	// enable pull ups for the 4 keys
	PORTA |= (1<<PORTA4)|(1<<PORTA5)|(1<<PORTA6)|(1<<PORTA7);

	
	// set LED Pins to output

	DDRC |= (1<<DDC2); // BL-LED1
	DDRD |= (1<<DDD6); // BL-LED2
	DDRD |= (1<<DDD7); // BL-LED3
	DDRC |= (1<<DDC6); // buzzer


	// disable bootloader LED
	DDRC |= (1<<DDC3);
	PORTC |= (1<<PORTC3);




	PORTD |= (1<<PORTD6);
	PORTC |= (1<<PORTC2);
	PORTD |= (1<<PORTD7);


	LCD_Init ();
	USART_Init ();
	USART1_Init ();
	
	TIMER0_Init ();
	
	sei ();


	lcd_cls ();
	lcd_printp (PSTR("Hallo"), 0);
	
	
	
	for (;;)
	{
	}
}
