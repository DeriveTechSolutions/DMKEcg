#include<msp430.h>
#include"uart.h"
int count=0;
int main()
{
unsigned char _data;
   WDTCTL = WDTPW + WDTHOLD;
   uart_init(BAUD_RATE);
//------------------------------ADC---------------------------------
   ADC12CTL0 = ADC12SHT02 + ADC12ON;
   ADC12CTL1 = ADC12SHP;
   ADC12IE = 0x01;
   ADC12CTL0 |= ADC12ENC;
   P6SEL |= BIT0;
   P1DIR |= BIT0;
   P6DIR |= BIT2;
   P6OUT &= ~BIT2;
//----------------------------ADC END---------------------------------------
   while(1){
   ADC12CTL0 |= ADC12SC;
   __bis_SR_register(LPM0_bits + GIE);
   __no_operation();
 //-----------------------------TX/RX--------------------------------
  _data=uart_rx();
 //---------------------------------------------------------------------
           }
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(ADC12IV,34))
  {

  case  0: break;
  case  2: break;
  case  4: break;
  case  6:
uart_data_send(ADC12MEM0);
uart_string(" ");
if (ADC12MEM0 >= 512)
      {
P1OUT |= BIT0;
      }
else  {
P1OUT &= ~BIT0;
      }
  __bic_SR_register_on_exit(LPM0_bits);
  case  8: break;
  case 10: break;
  case 12: break;
  case 14: break;
  case 16: break;
  case 18: break;
  case 20: break;
  case 22: break;
  case 24: break;
  case 26: break;
  case 28: break;
  case 30: break;
  case 32: break;
  case 34: break;
  default: break;
  }
}

