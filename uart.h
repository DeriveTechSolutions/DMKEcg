#define DECIMAL_COUNT 5
#define BAUD_RATE 115200
unsigned int adc_data=0,uart_data=0;

void delay_ms(unsigned int count1)
{
  unsigned int j=0,i=0;
  for(j=0;j<count1;j++)
  {
    for(i=0;i<3000;i++);
  }
}


void uart_init(unsigned int b_rate)
{
    P4SEL |= BIT4 | BIT5;
    UCA1CTLW0 &= ~UCSYNC;
    UCA1CTLW0 |= UCMODE_3;
    UCA1CTL1 |= UCSWRST;                    
    UCA1CTL1 |= UCSSEL_2;                   

switch(b_rate)
{
case 9600:
    UCA1BR0 = 6;
    UCA1BR1 = 0;
    UCA0MCTL = UCBRS_0 + UCBRF_13 + UCOS16;
break;
case 14400:
    UCA1BR0 = 4;
    UCA1BR1 = 0;
    UCA1MCTL = UCBRS_0 + UCBRF_13 + UCOS16;
   break;
case 19200:
    UCA1BR0 = 52;
    UCA1BR1 = 0;
    UCA1MCTL |= UCBRS_1 + UCBRF_0 ;
   break;
case 38400:
    UCA1BR0 = 26;
    UCA1BR1 = 0;
    UCA1MCTL |= UCBRS_1 + UCBRF_0 ;
   break;
case 57600:
    UCA1BR0 = 17;
    UCA1BR1 = 0;
    UCA1MCTL |= UCBRS_1 + UCBRF_0 ;
   break;
default:
    UCA1BR0 = 9;                              // 1MHz 115200
    UCA1BR1 = 0;                              // 1MHz 115200
    UCA1MCTL |= UCBRS_1 + UCBRF_0;            // Modulation UCBRSx=1, UCBRFx=0
    UCA1MCTL |= UCBRS_1 + UCBRF_0;            // Modulation UCBRSx=1, UCBRFx=0
}
UCA1CTL1 &= ~UCSWRST;

__no_operation();
}
void uart_tx(unsigned char data)
{

  while (!(UCA1IFG&UCTXIFG));
 //  UCA1IFG &=~UCTXIFG;

  UCA1TXBUF = data;

}


unsigned char uart_rx()
{

   while (!(UCA1IFG&UCRXIFG));
return UCA1RXBUF ;
}




void reverse(unsigned int data)
{
 unsigned int temp_data=0;
 uart_data=0;
 while(data)
 {

     temp_data=data%10;
     uart_data=uart_data*10+temp_data;
  data=data/10;

 }


}

void uart_data_send(unsigned int data)
{
reverse(data);
while(uart_data)
{

uart_tx((uart_data%10)+48);
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


