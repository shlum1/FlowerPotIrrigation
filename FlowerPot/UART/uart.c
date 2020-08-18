/*************************************************************************
Title:     Interrupt UART library with receive/transmit circular buffers
Author:    Peter Fleury <pfleury@gmx.ch>   http://jump.to/fleury
File:      $Id: uart.c,v 1.5.2.2 2004/02/27 22:00:28 peter Exp $
Software:  AVR-GCC 3.3
Hardware:  any AVR with built-in UART,
           tested on AT90S8515 at 4 Mhz and ATmega at 1Mhz
Extension: uart_puti, uart_puthex by M.Thomas 9/2004

DESCRIPTION:
    An interrupt is generated when the UART has finished transmitting or
    receiving a byte. The interrupt handling routines use circular buffers
    for buffering received and transmitted data.

    The UART_RX_BUFFER_SIZE and UART_TX_BUFFER_SIZE variables define
    the buffer size in bytes. Note that these variables must be a
    power of 2.

USAGE:
    Refere to the header file uart.h for a description of the routines.
    See also example test_uart.c.

NOTES:
    Based on Atmel Application Note AVR306

*************************************************************************/
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "uart.h"


T_UART_Status volatile UART_status;

/*
 *  constants and macros
 */

 // see Vector names: http://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html


/* size of RX/TX buffers */
#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1)
#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1)

#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
#error RX buffer size is not a power of 2
#endif
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
#error TX buffer size is not a power of 2
#endif

#if defined(__AVR_AT90S2313__) \
 || defined(__AVR_AT90S4414__) || defined(__AVR_AT90S4434__) \
 || defined(__AVR_AT90S8515__) || defined(__AVR_AT90S8535__)
 /* old AVR classic with one UART */
 #warning  "__AVR_AT90S2313__"
 #define AT90_UART 
 #define UART0_RECEIVE_INTERRUPT   SIG_UART_RECV
 #define UART0_TRANSMIT_INTERRUPT  SIG_UART_DATA
 #define UART0_STATUS   USR
 #define UART0_CONTROL  UCR
 #define UART0_DATA     UDR
 #define UART0_UDRIE    UDRIE
 #define UART0_RX_EMPTY UDRE

#elif defined(__AVR_AT90S2333__) || defined(__AVR_AT90S4433__)
 /* old AVR classic with one UART */
 #warning  "__AVR_AT90S2333__"
 #define AT90_UART
 #define UART0_RECEIVE_INTERRUPT   SIG_UART_RECV
 #define UART0_TRANSMIT_INTERRUPT  SIG_UART_DATA
 #define UART0_STATUS   UCSRA
 #define UART0_CONTROL  UCSRB
 #define UART0_DATA     UDR
 #define UART0_UDRIE    UDRIE

#elif  defined(__AVR_ATmega8__)  || defined(__AVR_ATmega16__) || defined(__AVR_ATmega32__) || defined(__AVR_ATmega8515__)\
    || defined(__AVR_ATmega8535__) || defined(__AVR_ATmega323__) || defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny4313__)

  /* ATMega with one USART */
 #warning  "__AVR_ATmega8__ / __AVR_ATtiny4313__ / __AVR_ATmega32__"

 #define ATMEGA_USART

 #if defined(__AVR_ATtiny4313__)  || defined(__AVR_ATtiny2313__)
   #define UART0_RECEIVE_INTERRUPT   USART_RX_vect
 #else
   #define UART0_RECEIVE_INTERRUPT   USART_RXC_vect
 #endif

 #define UART0_TRANSMIT_INTERRUPT  USART_UDRE_vect
 #define UART0_STATUS   UCSRA
 #define UART0_CONTROL  UCSRB
 #define UART0_DATA     UDR
 #define UART0_UDRIE    UDRIE

#elif defined(__AVR_ATmega163__)
  /* ATMega163 with one UART */
 #warning  "__AVR_ATmega163__"
 #define ATMEGA_UART
 #define UART0_RECEIVE_INTERRUPT   SIG_UART_RECV
 #define UART0_TRANSMIT_INTERRUPT  SIG_UART_DATA
 #define UART0_STATUS   UCSRA
 #define UART0_CONTROL  UCSRB
 #define UART0_DATA     UDR
 #define UART0_UDRIE    UDRIE

#elif defined(__AVR_ATmega162__)
 /* ATMega with two USART */
 #define ATMEGA_USART0
 #define ATMEGA_USART1
 #define UART0_RECEIVE_INTERRUPT   SIG_USART0_RECV
 #define UART1_RECEIVE_INTERRUPT   SIG_USART1_RECV
 #define UART0_TRANSMIT_INTERRUPT  SIG_USART0_DATA
 #define UART1_TRANSMIT_INTERRUPT  SIG_USART1_DATA
 #define UART0_STATUS   UCSR0A
 #define UART0_CONTROL  UCSR0B
 #define UART0_DATA     UDR0
 #define UART0_UDRIE    UDRIE0
 #define UART1_STATUS   UCSR1A
 #define UART1_CONTROL  UCSR1B
 #define UART1_DATA     UDR1
 #define UART1_UDRIE    UDRIE1

#elif defined(__AVR_ATmega64__)  || defined(__AVR_ATmega644__) || defined(__AVR_ATmega128__)  || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega328P__)
 /* ATMega with two USART */

 #warning  "__AVR_ATmega64__ or __AVR_ATmega328P__"
 #define ATMEGA_USART0
 #define ATMEGA_USART1
 #define UART0_RECEIVE_INTERRUPT   USART_RX_vect
 #define UART1_RECEIVE_INTERRUPT   USART1_RX_vect
 #define UART0_TRANSMIT_INTERRUPT  USART_UDRE_vect
 #define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
 #define UART0_STATUS   UCSR0A
 #define UART0_CONTROL  UCSR0B
 #define UART0_DATA     UDR0
 #define UART0_UDRIE    UDRIE0
 #define UART1_STATUS   UCSR1A
 #define UART1_CONTROL  UCSR1B
 #define UART1_DATA     UDR1
 #define UART1_UDRIE    UDRIE1
 #define UART0_RX_EMPTY UDRE0

#elif defined(__AVR_ATmega103__)
 /* ATMega with one UART */
 #error "AVR ATmega103 currently not supported by this libaray !"
#elif defined(__AVR_ATmega161__)
 /* ATmega with UART */
 #error "AVR ATmega161 currently not supported by this libaray !"
#elif defined(__AVR_ATmega169__)
 #error "AVR ATmega169 currently not supported by this libaray !"
#endif


//
//  module global variables
//
static volatile unsigned char UART_TxBuf[UART_TX_BUFFER_SIZE]={0};
static volatile unsigned char UART_RxBuf[UART_RX_BUFFER_SIZE]={0};
static volatile unsigned char UART_TxHead=0;
static volatile unsigned char UART_TxTail=0;
static volatile unsigned char UART_RxHead=0;
static volatile unsigned char UART_RxTail=0;

//static volatile unsigned char UART_LastRxError;



SIGNAL(UART0_RECEIVE_INTERRUPT)
/*************************************************************************
Function: UART Receive Complete interrupt
Purpose:  called when the UART has received a character
**************************************************************************/
{
    unsigned char tmphead;
    unsigned char data;
    unsigned char usr;
    unsigned char lastRxError;

    /* read UART status register and UART data register */
    usr  = UART0_STATUS;
    data = UART0_DATA;

    /* */
#if defined( AT90_UART )
    lastRxError = (usr & (_BV(FE)|_BV(DOR)) );
#elif defined( ATMEGA_USART )
    lastRxError = (usr & (_BV(FE)|_BV(DOR)) );
#elif defined( ATMEGA_USART0 )
    lastRxError = (usr & (_BV(FE0)|_BV(DOR0)) );
#elif defined ( ATMEGA_UART )
    lastRxError = (usr & (_BV(FE)|_BV(DOR)) );
#endif

    /* calculate buffer index */
    tmphead = ( UART_RxHead + 1) & UART_RX_BUFFER_MASK;

    if ( tmphead == UART_RxTail )
    {
        /* error: receive buffer overflow */
        lastRxError = UART_BUFFER_OVERFLOW >> 8;
    }else
    {
        /* store new index */
        UART_RxHead = tmphead;
        /* store received data in buffer */
        UART_RxBuf[tmphead] = data;

				UART_status.numByte++;
				if((data=='\r') || (data=='\n') )
						UART_status.numLines++;

    }
    UART_status.UART_LastRxError = lastRxError;
}





SIGNAL(UART0_TRANSMIT_INTERRUPT)
/*************************************************************************
Function: UART Data Register Empty interrupt
Purpose:  called when the UART is ready to transmit the next byte
**************************************************************************/
{
    unsigned char tmptail;


    if ( UART_TxHead != UART_TxTail)
    {
        /* calculate and store new buffer index */
        tmptail = (UART_TxTail + 1) & UART_TX_BUFFER_MASK;
        UART_TxTail = tmptail;
        /* get one byte from buffer and write it to UART */
        UART0_DATA = UART_TxBuf[tmptail];  /* start transmission */
    }
    else
    {
        /* tx buffer empty, disable UDRE interrupt */
        UART0_CONTROL &= ~_BV(UART0_UDRIE);
    }
}





//FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);





/*************************************************************************
Function: uart_init()
Purpose:  initialize UART and set baudrate
Input:    baudrate using macro UART_BAUD_SELECT()
Returns:  none
**************************************************************************/
void uart_init(unsigned char baudrate)  //convert baudrate to propper UBR Value usig UART_BAUD_SELECT()
//void uart_init(unsigned long baud)
{
//    unsigned int baudrate;
  uart_flush();

//###  stderr = stdout = stdin = &uart_str;

//	baudrate=(UART_BAUD_SELECT((baud),F_CPU));
#if defined( AT90_UART )
    /* set baud rate */
    UBRR = (unsigned char)baudrate;

    /* enable UART receiver and transmmitter and receive complete interrupt */
    UART0_CONTROL = _BV(RXCIE)|_BV(RXEN)|_BV(TXEN);

#elif defined (ATMEGA_USART)
    /* Set baud rate */
 //   UBRRH = (unsigned char)(baudrate>>8);
    UBRRH=0; // used only 1 byte for baudrate
    UBRRL = (unsigned char) baudrate;

    /* Enable USART receiver and transmitter and receive complete interrupt */
    UART0_CONTROL = _BV(RXCIE)|(1<<RXEN)|(1<<TXEN);

    /* Set frame format: asynchronous, 8data, no parity, 1stop bit */
    #ifdef URSEL
    UCSRC = (1<<URSEL)|(3<<UCSZ0);
    #else
    UCSRC = (3<<UCSZ0);
    #endif

#elif defined (ATMEGA_USART0 )
    /* Set baud rate */
    UBRR0H = (unsigned char)(baudrate>>8);
    UBRR0L = (unsigned char) baudrate;

    /* Enable USART receiver and transmitter and receive complete interrupt */
    UART0_CONTROL = _BV(RXCIE0)|(1<<RXEN0)|(1<<TXEN0);

    /* Set frame format: asynchronous, 8data, no parity, 1stop bit */
    #ifdef URSEL0
    UCSR0C = (1<<URSEL0)|(3<<UCSZ00);
    #else
    UCSR0C = (3<<UCSZ00);
    #endif

#elif defined ( ATMEGA_UART )
    /* set baud rate */
    UBRRHI = (unsigned char)(baudrate>>8);
    UBRR   = (unsigned char) baudrate;

    /* Enable UART receiver and transmitter and receive complete interrupt */
    UART0_CONTROL = _BV(RXCIE)|(1<<RXEN)|(1<<TXEN);

#endif


}/* uart_init */



/*************************************************************************
Function: uart_getc()
Purpose:  return byte from ringbuffer
Returns:  lower byte:  received byte from ringbuffer
          higher byte: last receive error
**************************************************************************/
unsigned int uart_getc(void)
{
    unsigned char tmptail;
    unsigned char data;



    if ( UART_RxHead == UART_RxTail ) {
        return UART_NO_DATA;   /* no data available */
    }

    /* calculate /store buffer index */
    tmptail = (UART_RxTail + 1) & UART_RX_BUFFER_MASK;
    UART_RxTail = tmptail;

    /* get data from receive buffer */
    data = UART_RxBuf[tmptail];

		UART_status.numByte--;
		if(data=='\r')
		  UART_status.numLines--;

    return (UART_status.UART_LastRxError << 8) + data;

}/* uart_getc */


/*************************************************************************
Function: uart_putc()
Purpose:  write byte to ringbuffer for transmitting via UART
Input:    byte to be transmitted
Returns:  none
**************************************************************************/
void uart_putc(unsigned char data)
{
    unsigned char tmphead;

    tmphead  = (UART_TxHead + 1) & UART_TX_BUFFER_MASK;

    while ( tmphead == UART_TxTail )
    {
        ;/* wait for free space in buffer */
    }

    UART_TxBuf[tmphead] = data;
    UART_TxHead = tmphead;

    /* enable UDRE interrupt */
    UART0_CONTROL    |= _BV(UART0_UDRIE);

}/* uart_putc */


/*************************************************************************
Function: uart_puts()
Purpose:  transmit string to UART
Input:    string to be transmitted
Returns:  none
**************************************************************************/
void uart_puts(const char *s )
{
    while (*s)
      uart_putc(*s++);

}/* uart_puts */

/*************************************************************************
Function: uart_puts_p()
Purpose:  transmit string from program memory to UART
Input:    program memory string to be transmitted
Returns:  none
**************************************************************************/
void uart_puts_p(const char *progmem_s )
{
    register char c;

    while ( (c = pgm_read_byte(progmem_s++)) )
      uart_putc(c);

}/* uart_puts_p */


/*************************************************************************
Function: uart_puti()
Purpose:  transmit integer as ASCII to UART
Input:    integer value
Returns:  none
This functions has been added by Martin Thomas <eversmith@heizung-thomas.de>
Don't blame P. Fleury if it doesn't work ;-)
**************************************************************************/
void uart_puti( const int val )
{
    char buffer[8];  //"-32768\0" .. "32767\0"

    uart_puts( itoa(val, buffer, 10) );

}/* uart_puti */



/*************************************************************************
Function: uart_puthex_nibble()
Purpose:  transmit lower nibble as ASCII-hex to UART
Input:    byte value
Returns:  none
This functions has been added by Martin Thomas <eversmith@heizung-thomas.de>
Don't blame P. Fleury if it doesn't work ;-)
**************************************************************************/
void uart_puthex_nibble(const unsigned char b)
{
    unsigned char  c = b & 0x0f;
    if (c>9) c += 'A'-10;
    else c += '0';
    uart_putc(c);
} /* uart_puthex_nibble */

/*************************************************************************
Function: uart_puthex_byte()
Purpose:  transmit upper and lower nibble as ASCII-hex to UART
Input:    byte value
Returns:  none
This functions has been added by Martin Thomas <eversmith@heizung-thomas.de>
Don't blame P. Fleury if it doesn't work ;-)
**************************************************************************/
void uart_puthex_byte(const unsigned char  b)
{
    uart_puthex_nibble(b>>4);
    uart_puthex_nibble(b);
} /* uart_puthex_byte */


/*************************************************************************
Function: uart_flush()
Purpose:  flush the UART
Input:    none
Returns:  none
This functions has been added by Shlum
**************************************************************************/
void uart_flush(void)
{
  cli();
  UART_RxHead = 0;
  UART_RxTail = 0;

  memset((void*)(&UART_status), 0, sizeof(UART_status));
  sei();
}


#ifndef SMALL_FOOTPRINT


/*************************************************************************
Function: uart_write( const char* s, ... )
Purpose:
Input:    format string to be transmitted, any number of arguments...
Returns:  none
**************************************************************************/
void uart_write( const char* s, ... )
{
	char buffer[256]={0};
	va_list aptr;

	va_start(aptr, s);
	vsprintf(buffer, s, aptr);
	va_end(aptr);
//	uart_puts("\n\r");
	uart_puts(buffer);
} //void uart_write( const char* s, ... )



/*************************************************************************
Function: _uartwrite_p()
Purpose:  transmit string from program memory to UART
Input:    program memory string to be transmitted
Returns:  none
**************************************************************************/
void _uart_write_p(const char *progmem_s, ... )
{
  register char c; 
  register uint8_t i;
	char buffer[200]={0}, s[128]={0};
	va_list aptr;

  i=0;
  while ( (c = pgm_read_byte(progmem_s++)) && (i<sizeof(s)-1))
    s[i++]=c;

	va_start(aptr, progmem_s);
	vsnprintf(buffer, sizeof(buffer), s, aptr);
	va_end(aptr);

	uart_puts(buffer);
  }/* _uartwrite_p */


/*************************************************************************
* Send / receive one character to the UART  / USED BY printf.
**************************************************************************/
int uart_putchar(char c, FILE *stream)
{
	if(c=='\n')
  	uart_putc('\r');

	uart_putc(c);
	return 0;
}


int	uart_getchar(FILE *stream)
{
	return uart_getc();
}


#endif  //#ifndef SMALL_FOOTPRINT
