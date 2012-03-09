/*****************************************************************************
 *   Copyright (C) 2009 Peter "woggle" Mack, mac@denich.net                  *
 *   based on the key handling by Peter Dannegger                            *
 *     see www.mikrocontroller.net                                           *
 *                                                                           *
 *   This program is free software; you can redistribute it and/or modify    *
 *   it under the terms of the GNU General Public License as published by    *
 *   the Free Software Foundation; either version 2 of the License.          *
 *                                                                           *
 *   This program is distributed in the hope that it will be useful,         *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *   GNU General Public License for more details.                            *
 *                                                                           *
 *   You should have received a copy of the GNU General Public License       *
 *   along with this program; if not, write to the                           *
 *   Free Software Foundation, Inc.,                                         *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.               *
 *                                                                           *
 *****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "main.h"
#include "timer.h"

volatile uint16_t timer;


uint8_t key_state = 0;	// debounced and inverted key state:
												// bit = 1: key pressed
uint8_t key_press = 0;	// key press detect
uint8_t key_rpt;				// key long press and repeat


//*****************************************************************************
// 

volatile uint8_t beepmode = 0; //1 with LED, 2 without LED , 3 single beep (+128 == active but in off period)
volatile uint8_t beeplength = 0;
volatile uint8_t beeptimer = 0;


#if defined (__AVR_ATmega32__)
ISR(TIMER0_COMP_vect)						// Timer-Interrupt (100 Hz)
#else
ISR(TIMER0_COMPA_vect)					// Timer-Interrupt (100 Hz)
#endif
{
  static uint8_t ct0 = 0;
	static uint8_t ct1 = 0;
	static uint8_t rpt = 0;
	uint8_t i;
	
	// Key handling by Peter Dannegger
	// see www.mikrocontroller.net
	i = key_state ^ ~KEY_PIN;	// key changed ?
	ct0 = ~(ct0 & i);						// reset or count ct0
	ct1 = ct0 ^ (ct1 & i);			// reset or count ct1
	i &= (ct0 & ct1);						// count until roll over ?
	key_state ^= i;							// then toggle debounced state
	key_press |= (key_state & i);	// 0->1: key press detect
	
	if ((key_state & REPEAT_MASK) == 0)	// check repeat function
	{
		rpt = REPEAT_START;	// start delay
	}
	if (--rpt == 0)
	{
		rpt = REPEAT_NEXT;	// repeat delay
		key_rpt |= (key_state & REPEAT_MASK);
	}

	
	if (timer > 0)
	{
		timer --;
	}

	
	// beep code
	
	if(beeptimer > 0)
	{
		beeptimer --;
		
		if(beeptimer == 0)
		{
			beeptimer = beeplength;
			
			//toggle buzzer
		    PORTC ^= (1<<PORTC6);
 
        	if(beepmode == 1)
			{
				toggle_light();
				beepmode = 129;
			}
			else if (beepmode == 2)
			{
				beepmode = 130;
			}
			else if (beepmode == 3)
			{
				beepmode = 131;
			}
			else if (beepmode == 129)
			{
				toggle_light();
				beepmode = 1;
			}
			else if (beepmode == 130)
			{
				beepmode = 2;
			}
			else if (beepmode == 131)
			{
				beep(0,0);
			}
			
		}
	
	}	

}

void beep(uint8_t mode,uint8_t length)
{
	restore_light();
	// restore buzzer
	PORTC &= ~(1<<PORTC6);
	
	if(mode == 0)
	{
		beepmode = 0;
		beeptimer = 0;
		beeplength = 0;
		
	}
	else
	{
		beepmode = mode;
		beeplength = length;
		beeptimer = length;
	}

}



//*****************************************************************************
// 
void TIMER0_Init (void)
{
	timer = 0;
	
#if defined (__AVR_ATmega32__)
	TCCR0 = (1 << CS02) | (1 << CS00) | (1 << WGM01);		// Prescaler 1024
	OCR0 = (F_CPU / (100L * 1024L)) ;

	TIMSK |= (1 << OCIE0);	// enable interrupt for OCR
#else
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS02) | (1 << CS00);
	OCR0A = (F_CPU / (100L * 1024L)) ;

	TIMSK0 |= (1 << OCIE0A);	// enable interrupt for OCR
#endif
}


//*****************************************************************************
// 
uint8_t get_key_press (uint8_t key_mask)
{
	uint8_t sreg = SREG;
	
	// disable all interrupts
	cli();
	
  key_mask &= key_press;	// read key(s)
  key_press ^= key_mask;	// clear key(s)
	
	SREG = sreg;	// restore status register
	
  return key_mask;
}


//*****************************************************************************
// 
uint8_t get_key_rpt (uint8_t key_mask)
{
	uint8_t sreg = SREG;

	// disable all interrupts
	cli();
	
  key_mask &= key_rpt;	// read key(s)
  key_rpt ^= key_mask;	// clear key(s)
	
	SREG = sreg;	// restore status register
	
  return key_mask;
}


//*****************************************************************************
// 
uint8_t get_key_short (uint8_t key_mask)
{
	uint8_t ret;
	uint8_t sreg = SREG;
	
	// disable all interrupts
	cli();
	
  ret = get_key_press (~key_state & key_mask);
	
	SREG = sreg;	// restore status register
	
  return ret;
}


//*****************************************************************************
// 
uint8_t get_key_long (uint8_t key_mask)
{
  return get_key_press (get_key_rpt (key_mask));
}


//*****************************************************************************
// 
uint8_t get_key_long2 (uint8_t key_mask)
{
  return get_key_press (get_key_rpt (key_press^key_mask));
}


//*****************************************************************************
// 
uint8_t get_key_long_rpt (uint8_t key_mask)
{
  return get_key_rpt (~key_press^key_mask);
}



void toggle_light(void)
{
	PORTD ^= (1<<PORTD6);
    PORTC ^= (1<<PORTC2);
    PORTD ^= (1<<PORTD7);
}

void restore_light(void)
{
	PORTD |= (1<<PORTD6);
	PORTC |= (1<<PORTC2);
	PORTD |= (1<<PORTD7);
}
