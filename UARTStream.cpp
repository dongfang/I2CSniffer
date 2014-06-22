#include "UARTStream.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#define INBUFSIZ 64
#define OUTBUFSIZ 16
#define INMASK (INBUFSIZ-1)
#define OUTMASK (OUTBUFSIZ-1)

#define BAUD 115200

#include <util/setbaud.h>

 struct UART_INBUFFER {
	volatile uint8_t head;
	volatile uint8_t tail;
	volatile uint8_t data[INBUFSIZ];
};

 volatile static struct UART_INBUFFER inbuffer;

 struct UART_OUTBUFFER {
	volatile uint8_t head;
	volatile uint8_t tail;
	volatile uint8_t data[OUTBUFSIZ];
};

volatile static struct UART_OUTBUFFER outbuffer;

int uart_putchar(char c, FILE *stream) {
	uint8_t next = (outbuffer.head+1) & OUTMASK;
	while (next == outbuffer.tail) { /*wdt_reset();*/ }
	outbuffer.data[outbuffer.head] = c;
	// Now there definitely is data in the buffer.
	outbuffer.head = next;
	UCSR0B |= (1 << UDRIE0);
	UCSR0A |= (1 << TXC0);
	return c;
}

ISR(USART_RX_vect) {
	uint8_t next = (inbuffer.head+1) & INMASK;
	if (next != inbuffer.tail) {
		inbuffer.data[inbuffer.head] = UDR0;
		inbuffer.head = next;
	}
}

ISR(USART_UDRE_vect) {
	if (outbuffer.head == outbuffer.tail) {
		UCSR0B &= ~(1<<UDRIE0);
	} else {
		uint8_t data = outbuffer.data[outbuffer.tail];
		outbuffer.tail = (outbuffer.tail+1) & OUTMASK;
		UDR0 = data;
	}
}

int uart_getchar(FILE *stream) {
	if (inbuffer.head == inbuffer.tail)
		return -1;
	int result = inbuffer.data[inbuffer.tail];
	inbuffer.tail = (inbuffer.tail+1) & INMASK;
	return result;
}

size_t uart_available() {
	return (inbuffer.head-inbuffer.tail) & INMASK;
}

size_t uart_txSpace() {
	return (inbuffer.tail-inbuffer.head - 1) & INMASK;
}

void uart_waitFlushed() {
	while(outbuffer.head != outbuffer.tail)
		;
}

void uart_init() {
	UBRR0 = UBRR_VALUE;
	UCSR0A = (USE_2X<<1);
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}
