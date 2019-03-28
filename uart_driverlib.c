//******************************************************************************
#include "driverlib.h"
#include "msp430.h"
#include "uart_driverlib.h"
//*****************************************************************************
#define A1
unsigned char status;
//unsigned int uart_data;
void uart_init()
{
    //P3.4,5 = USCI_A0 TXD/RXD
#ifdef A0
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P3,
        GPIO_PIN3 + GPIO_PIN4
        );
#endif
#ifdef A1
    GPIO_setAsPeripheralModuleFunctionInputPin(
           GPIO_PORT_P4,
           GPIO_PIN4 + GPIO_PIN5
           );
#endif
    //Baudrate = 9600, clock freq = 1.048MHz
    //UCBRx = 109, UCBRFx = 0, UCBRSx = 2, UCOS16 = 0
    USCI_A_UART_initParam param = {0};
    param.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = 138;
    param.firstModReg = 0;
    param.secondModReg = 0;
    param.parity = USCI_A_UART_NO_PARITY;
    param.msborLsbFirst = USCI_A_UART_LSB_FIRST;
    param.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
    param.uartMode = USCI_A_UART_MODE;
    param.overSampling = USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;

#ifdef A0
    if (STATUS_FAIL == USCI_A_UART_init(USCI_A0_BASE, &param)){
        return;
    }
    USCI_A_UART_enable(USCI_A0_BASE);
#endif
#ifdef A1
    if (STATUS_FAIL == USCI_A_UART_init(USCI_A1_BASE, &param)){
           return;
       }
    USCI_A_UART_enable(USCI_A1_BASE);
/*    //Enable Receive Interrupt
      USCI_A_UART_clearInterrupt(USCI_A1_BASE,
        USCI_A_UART_RECEIVE_INTERRUPT);
    USCI_A_UART_enableInterrupt(USCI_A1_BASE,
        USCI_A_UART_RECEIVE_INTERRUPT);*/
#endif
    //Enable UART module for operation
    __enable_interrupt();
}
//*****************************************************************************
//
// trasnmit data
//
//*****************************************************************************

void uart_tx(unsigned char data)
{
#ifdef A0
    while(USCI_A_UART_TRANSMIT_INTERRUPT_FLAG !=  USCI_A_UART_getInterruptStatus(USCI_A0_BASE,
                                                                                              USCI_A_UART_TRANSMIT_INTERRUPT_FLAG));
    USCI_A_UART_transmitData(USCI_A0_BASE,
                             data);
#endif
#ifdef A1
    while(USCI_A_UART_TRANSMIT_INTERRUPT_FLAG !=  USCI_A_UART_getInterruptStatus(USCI_A1_BASE,
                                                                                                 USCI_A_UART_TRANSMIT_INTERRUPT_FLAG));
    USCI_A_UART_transmitData(USCI_A1_BASE,
                                data);
#endif
}
//*****************************************************************************
//
//function to receive data
//
//*****************************************************************************
unsigned char uart_rx(void)
{
#ifdef A0
    while( USCI_A_UART_RECEIVE_INTERRUPT_FLAG !=  USCI_A_UART_getInterruptStatus(USCI_A0_BASE,
                                                                                 USCI_A_UART_RECEIVE_INTERRUPT_FLAG));

    return USCI_A_UART_receiveData(USCI_A0_BASE);
#endif
#ifdef A1
    while( USCI_A_UART_RECEIVE_INTERRUPT_FLAG !=  USCI_A_UART_getInterruptStatus(USCI_A1_BASE,
                                                                                USCI_A_UART_RECEIVE_INTERRUPT_FLAG));
     status = USCI_A_UART_receiveData(USCI_A1_BASE);
    return USCI_A_UART_receiveData(USCI_A1_BASE);
#endif
}
//*****************************************************************************
//
//to transmit string
//
//*****************************************************************************
void uart_string(unsigned char * str)
{
    while(*str)
    {
        uart_tx(*str);
        str++;
    }
}
void rev_str(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}
int int2Str(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x % 10) + '0';
        x = x/10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    rev_str(str, i);
    //str[i] = '\0';
    return i;
}

void F2A(float n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (int)ipart;

    // convert integer part to string
    int i = int2Str(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart*pow(10, afterpoint);

        int2Str(( int )fpart, res + i + 1, afterpoint);
    }
}

//*****************************************************************************
//
//to send float value
//
//*****************************************************************************

void uart_float_send(float data)
{
    unsigned int count=DECIMAL_COUNT,temp_data,uart_data=0;
    int zeros;
    temp_data=data;
    zeros=  reverse(temp_data,&uart_data);
    while(uart_data)
    {
        uart_tx((uart_data%10)+48);
        uart_data=uart_data/10;
        zeros--;
    }
    while(zeros-- > 0)
    uart_tx('0');
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
//*****************************************************************************
//
//function to reverse number
//
//*****************************************************************************

unsigned int reverse(uint32_t data,unsigned int *uart_data)
{
    unsigned int temp_data;
    int zeros=0;
    *uart_data=0;
    while(data)
    {

        temp_data=data%10;
        *uart_data=(*uart_data)*10+temp_data;
        data=data/10;
        zeros++;
    }
    return zeros;
}

//*****************************************************************************
//
//function to send the integer data
//
//*****************************************************************************
void uart_data_send(uint32_t data)
{
    int zeros=0;
    unsigned int uart_data=0;
   zeros = reverse(data,&uart_data);
    while(uart_data)
    {
        uart_tx(uart_data%10 + 48);
        uart_data=uart_data/10;
        zeros--;
    }
    while(zeros-- > 0)
       uart_tx('0');
}
