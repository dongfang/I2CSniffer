#ifndef __UARTSTREAM_H
#define __UARTSTREAM_H

#include <stdio.h>

void uart_init();
int uart_getchar(FILE *stream);
int uart_putchar(char c, FILE *stream);
size_t uart_available();
size_t uart_txSpace();
void uart_waitFlushed();

#endif
