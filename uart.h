
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#define DECIMAL_COUNT 5
#define BAUD_RATE 115200
unsigned int uart_data;
signed int num_digit=0;

void uart_init(unsigned int b_rate)
{
    P4SEL = BIT4+BIT5;                        // P3.4,5 = USCI_A0 TXD/RXD
    UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL_2;
switch(b_rate)
{
case 9600:
   UCA0BR0 = 6;
    UCA0BR1 = 0;
    UCA0MCTL = UCBRS_0 + UCBRF_13 + UCOS16;
break;
case 14400:
   UCA0BR0 = 4;
    UCA0BR1 = 0;
    UCA0MCTL = UCBRS_0 + UCBRF_13 + UCOS16;
   break;
case 19200:
   UCA0BR0 = 52;
    UCA0BR1 = 0;
    UCA0MCTL |= UCBRS_1 + UCBRF_0 ;
   break;
case 38400:
   UCA0BR0 = 26;
    UCA0BR1 = 0;
    UCA0MCTL |= UCBRS_1 + UCBRF_0 ;
   break;
case 57600:
   UCA0BR0 = 17;
    UCA0BR1 = 0;
    UCA0MCTL |= UCBRS_1 + UCBRF_0 ;
   break;
default:                                        // 115200
    UCA1BR0 = 9;                                // 1MHz 115200
    UCA1BR1 = 0;                                // 1MHz 115200
    UCA1MCTL |= UCBRS_1 + UCBRF_0 ;             // Modulation UCBRSx=1, UCBRFx=0
}
UCA1CTL1 &= ~UCSWRST;
}

void uart_tx(char data)
{
   while (!(UCA1IFG&UCTXIFG));
   UCA1IFG &= ~UCTXIFG;
   UCA1TXBUF = data;
}

unsigned char uart_rx()
{
   while (!(UCA1IFG&UCRXIFG));
   return UCA1RXBUF ;
}

void reverse(uint32_t data)
{
 unsigned int temp_data;
 while(data)
 {
     temp_data=data%10;
     uart_data=uart_data*10+temp_data;
     data=data/10;
 }
}
void uart_data_send(uint32_t data)
{
reverse(data);
while(uart_data)
{
uart_tx(uart_data%10 + 48);
uart_data=uart_data/10;
}
}

void uart_string(unsigned char *str)
{
   while(*str)
   {
       uart_tx(*str);
       str++;
   }
}

void uart_float_send(float data)
{
unsigned int temp_data,count=DECIMAL_COUNT;
temp_data=data;
reverse(temp_data);
while(uart_data)
{
uart_tx((uart_data%10)+48);
uart_data=uart_data/10;
}
uart_tx('.');
data=data-(int)data;
while(data && count)
{
   uart_tx((int)(data*10)+48);
   data=data*10;
   data=data-(int)data;
   count--;
}
}
