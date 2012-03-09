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

#ifndef _TIMER_H
#define _TIMER_H

#define KEY_PIN			PINA
#define KEY_ENTER		PA7
#define KEY_ESC			PA6
#define KEY_PLUS		PA5
#define KEY_MINUS		PA4
#define KEY_ALL			((1 << KEY_PLUS) | (1 << KEY_MINUS) | (1 << KEY_ENTER) | (1 << KEY_ESC))
#define REPEAT_MASK	((1 << KEY_PLUS) | (1 << KEY_MINUS) | (1 << KEY_ENTER) | (1 << KEY_ESC))	// repeat: MODE

#define REPEAT_START	50	// after 500ms
#define REPEAT_NEXT		10	// every 100ms


extern volatile uint16_t timer;
extern volatile uint8_t beepmode; //1 with LED, 2 without LED , 3 single beep (+128 == active but in off period)
extern volatile uint8_t beeplength;


void TIMER0_Init (void);

uint8_t get_key_press (uint8_t key_mask);
uint8_t get_key_rpt (uint8_t key_mask);
uint8_t get_key_short (uint8_t key_mask);
uint8_t get_key_long (uint8_t key_mask);
uint8_t get_key_long2 (uint8_t key_mask);
uint8_t get_key_long_rpt (uint8_t key_mask);

void toggle_light(void);
void restore_light(void);
void beep(uint8_t,uint8_t);

#endif
