#ifndef UART_DRIVERLIB_H_
#define UART_DRIVERLIB_H_

#define DECIMAL_COUNT 1


/****************************************function declaration***********************************/
void uart_init();                         //function to initialize uart
void uart_tx(unsigned char data);              //function to send single byte
unsigned char uart_rx(void);                   //function to receive single byte
void uart_string(unsigned char * str);         //function to send string
void rev_str(char *str, int len);              //function to receive string
int int2Str(int x, char str[], int d);
void F2A(float n, char *res, int afterpoint);   //function to convert float to ascci
void uart_float_send(float data);               //function to send float data
unsigned int reverse(uint32_t data,unsigned int *);            //function to reverse the number
void uart_data_send(uint32_t data);             //function to send integer data
double      pow(double x, double y);

#endif /* UART_DRIVERLIB_H_ */


