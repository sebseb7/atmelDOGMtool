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

#ifndef _USART_H
#define _USART_H

//*****************************************************************************
// 
#ifndef FALSE
#define FALSE	0
#endif
#ifndef TRUE
#define TRUE	1
#endif


// must be at least 4('#'+Addr+'CmdID'+'\r')+ (80 * 4)/3 = 111 bytes
#define TXD_BUFFER_LEN  60
#define RXD_BUFFER_LEN  180

// Baud rate of the USART
#define USART_BAUD 57600	
//#define USART_BAUD 125000	

//*****************************************************************************
// 
extern uint8_t buffer[30];

extern volatile uint8_t txd_buffer[TXD_BUFFER_LEN];
extern volatile uint8_t txd_complete;
extern volatile uint8_t rxd_buffer[RXD_BUFFER_LEN];
extern volatile uint8_t rxd_buffer_locked;
extern volatile uint8_t ReceivedBytes;
extern volatile uint8_t *pRxData;
extern volatile uint8_t RxDataLen;



//*****************************************************************************
// 
void USART_Init (void);
void USART1_Init (void);
void USART1_putc (char c);
void USART1_puts (char *s);
uint8_t uart1_getc_nb(uint8_t*);

void USART_putc (char c);
void USART_puts (char *s);
void USART_puts_p (const char *s);


uint8_t uart_getc_nb(uint8_t*);

//*****************************************************************************
//Anpassen der seriellen Schnittstellen Register
#define USART_RXC_vect USART0_RX_vect
//-----------------------
#define UCSRA		UCSR0A
#define UCSRB		UCSR0B
#define UCSRC		UCSR0C
#define UDR			UDR0
#define UBRRL		UBRR0L
#define UBRRH		UBRR0H

// UCSRA
#define	RXC			RXC0
#define TXC			TXC0
#define UDRE		UDRE0
#define FE			FE0
#define UPE			UPE0
#define U2X			U2X0
#define MPCM		MPCM0

// UCSRB
#define RXCIE		RXCIE0
#define TXCIE		TXCIE0
#define UDRIE		UDRIE0
#define TXEN		TXEN0
#define RXEN		RXEN0
#define UCSZ2		UCSZ02
#define RXB8		RXB80
#define TXB8		TXB80

// UCSRC
#define UMSEL1	UMSEL01
#define UMSEL0	UMSEL00
#define UPM1		UPM01
#define UPM0		UPM00
#define USBS		USBS0
#define UCSZ1		UCSZ01
#define UCSZ0		UCSZ00
#define UCPOL		UCPOL0
//-----------------------


#endif

