/*****************************************************************************
 *   Copyright (C) 2009 Peter "woggle" Mack, mac@denich.net                  *
 *   taken some ideas from the C-OSD code from CaScAdE                       *
 *   the MK communication routines are taken from the MK source              *
 *    (killagreg version)                                                    *
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
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdarg.h>

#include "main.h"
#include "usart.h"
#include "lcd.h"

uint8_t buffer[30];

volatile uint8_t txd_buffer[TXD_BUFFER_LEN];
volatile uint8_t txd_complete = TRUE;
volatile uint8_t txd1_buffer[TXD_BUFFER_LEN];
volatile uint8_t txd1_complete = TRUE;
volatile uint8_t rxd_buffer[RXD_BUFFER_LEN];
volatile uint8_t rxd_buffer_locked = FALSE;
volatile uint8_t ReceivedBytes = 0;
volatile uint8_t *pRxData = 0;
volatile uint8_t RxDataLen = 0;

volatile uint16_t stat_crc_error = 0;
volatile uint16_t stat_overflow_error = 0;

volatile uint8_t rx_byte;
volatile uint8_t rxFlag = 0;


#define UART_RXBUFSIZE 64

volatile static uint8_t rxbuf[UART_RXBUFSIZE];
volatile static uint8_t rx1buf[UART_RXBUFSIZE];
volatile static uint8_t *volatile rxhead, *volatile rxtail;
volatile static uint8_t *volatile rx1head, *volatile rx1tail;


//*****************************************************************************
// USART1 transmitter ISR
/*
ISR (USART1_TX_vect)
{
	static uint16_t ptr_txd1_buffer = 0;
	uint8_t tmp_tx1;
	
	if(!txd1_complete) // transmission not completed
	{
		ptr_txd1_buffer++;                    // [0] was already sent
		tmp_tx1 = txd1_buffer[ptr_txd1_buffer];
		// if terminating character or end of txd buffer was reached
		if((tmp_tx1 == '\r') || (ptr_txd1_buffer == TXD_BUFFER_LEN))
		{
			ptr_txd1_buffer = 0;		// reset txd pointer
			txd1_complete = TRUE;	// stop transmission
		}
		UDR1 = tmp_tx1; // send current byte will trigger this ISR again
	}
	// transmission completed
	else ptr_txd1_buffer = 0;
}
*/


#ifdef USART_INT
//*****************************************************************************
// USART0 transmitter ISR
ISR (USART_TXC_vect)
{
	static uint16_t ptr_txd_buffer = 0;
	uint8_t tmp_tx;

	if(!txd_complete) // transmission not completed
	{
		ptr_txd_buffer++;                    // [0] was already sent
		tmp_tx = txd_buffer[ptr_txd_buffer];
		// if terminating character or end of txd buffer was reached
		if((tmp_tx == '\r') || (ptr_txd_buffer == TXD_BUFFER_LEN))
		{
			ptr_txd_buffer = 0;		// reset txd pointer
			txd_complete = TRUE;	// stop transmission
		}
		UDR = tmp_tx; // send current byte will trigger this ISR again
	}
	// transmission completed
	else ptr_txd_buffer = 0;
}
#endif

//*****************************************************************************
// 


uint8_t uart_getc_nb(uint8_t *c)
{
	if (rxhead==rxtail) return 0;
	*c = *rxtail;
	if (++rxtail == (rxbuf + UART_RXBUFSIZE)) rxtail = rxbuf;
	return 1;
}


ISR (USART_RXC_vect)
{
	uint8_t c;





                                                                                                        	
		int diff;
		c=UDR;
		diff = rxhead - rxtail;
		if (diff < 0) diff += UART_RXBUFSIZE;
		if (diff < UART_RXBUFSIZE -1) 
		{
			*rxhead = c;
			++rxhead;
			if (rxhead == (rxbuf + UART_RXBUFSIZE)) rxhead = rxbuf;
		};
		
		
		
		

}

//*****************************************************************************
// 
void USART_Init (void)
{
	// set clock divider
	#undef BAUD
	#define BAUD USART_BAUD
	#include <util/setbaud.h>
	UBRRH = UBRRH_VALUE;
	UBRRL = UBRRL_VALUE;
	
#if USE_2X
	UCSRA |= (1 << U2X);	// enable double speed operation
#else
	UCSRA &= ~(1 << U2X);	// disable double speed operation
#endif
	
	// set 8N1
#if defined (__AVR_ATmega8__) || defined (__AVR_ATmega32__)
	UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
#else
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0);
#endif
	UCSRB &= ~(1 << UCSZ2);

	// flush receive buffer
	while ( UCSRA & (1 << RXC) ) UDR;

	UCSRB |= (1 << RXEN) | (1 << TXEN);
#ifdef USART_INT
	UCSRB |= (1 << RXCIE) | (1 << TXCIE);
#else
	UCSRB |= (1 << RXCIE);
#endif

	rxhead = rxtail = rxbuf;

}



uint8_t uart1_getc_nb(uint8_t *c)
{
	if (rx1head==rx1tail) return 0;
	*c = *rx1tail;
	if (++rx1tail == (rx1buf + UART_RXBUFSIZE)) rx1tail = rx1buf;
	return 1;
}


ISR (USART1_RX_vect)
{

                                                                                                        	
		int diff;
		uint8_t c;
		c=UDR1;
		diff = rx1head - rx1tail;
		if (diff < 0) diff += UART_RXBUFSIZE;
		if (diff < UART_RXBUFSIZE -1) 
		{
			*rx1head = c;
			++rx1head;
			if (rx1head == (rx1buf + UART_RXBUFSIZE)) rx1head = rx1buf;
		};
		
}


//*****************************************************************************
// 
void USART1_Init (void)
{
	// set clock divider
#undef BAUD
#define BAUD USART_BAUD
#include <util/setbaud.h>
	UBRR1H = UBRRH_VALUE;
	UBRR1L = UBRRL_VALUE;
	
#if USE_2X
	UCSR1A |= (1 << U2X1);	// enable double speed operation
#else
	UCSR1A &= ~(1 < U2X1);	// disable double speed operation
#endif
	
	// set 8N1
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
	UCSR1B &= ~(1 << UCSZ12);
	
	// flush receive buffer
	while ( UCSR1A & (1 << RXC1) ) UDR1;
	
	//	UCSR1B |= (1 << RXEN1) | (1 << TXEN1);
	//	UCSR1B |= (1 << RXCIE1) | (1 << TXCIE1);
	UCSR1B |= (1 << RXEN1) | (1 << TXEN1);
//	UCSR1B |= (1 << RXCIE1) | (1 << TXCIE1);
	UCSR1B |= (1 << RXCIE1);

	rx1head = rx1tail = rx1buf;
	
}

//*****************************************************************************
// disable the txd pin of usart
void USART_DisableTXD (void)
{
#ifdef USART_INT
	UCSRB &= ~(1 << TXCIE);		// disable TX-Interrupt
#endif
	UCSRB &= ~(1 << TXEN);		// disable TX in USART
	DDRB  &= ~(1 << DDB3);		// set TXD pin as input
	PORTB &= ~(1 << PORTB3);	// disable pullup on TXD pin
}

//*****************************************************************************
// enable the txd pin of usart
void USART_EnableTXD (void)
{
	DDRB  |=  (1 << DDB3);		// set TXD pin as output
	PORTB &= ~(1 << PORTB3);	// disable pullup on TXD pin
	UCSRB |=  (1 << TXEN);		// enable TX in USART
#ifdef USART_INT
	UCSRB |=  (1 << TXCIE);		// enable TX-Interrupt
#endif
}


//*****************************************************************************
// 
void USART_putc (char c)
{
#ifdef USART_INT
#else
	loop_until_bit_is_set(UCSRA, UDRE);
	UDR = c;
#endif
}

void USART1_putc (char c)
{
#ifdef USART_INT
#else
	loop_until_bit_is_set(UCSR1A, UDRE1);
	UDR1 = c;
#endif
}

void USART1_puts (char *s)
{
#ifdef USART_INT
#else
	while (*s)
	{
		USART1_putc (*s);
		s++;
	}
#endif
}


//*****************************************************************************
// 
void USART_puts (char *s)
{
#ifdef USART_INT
#else
	while (*s)
	{
		USART_putc (*s);
		s++;
	}
#endif
}

//*****************************************************************************
// 
void USART_puts_p (const char *s)
{
#ifdef USART_INT
#else
	while (pgm_read_byte(s))
	{
		USART_putc (pgm_read_byte(s));
		s++;
	}
#endif
}

