/*
 * ECG.h
 *
 *  Created on: Feb 11, 2019
 *      Author: Derive01
 */

/*
 * ECG.h
 *
 *  Created on: Feb 11, 2019
 *      Author: Derive01
 */

/*
 * ECG.h
 *
 *  Created on: Feb 5, 2019
 *      Author: Derive01
 */

#ifndef ECG_H_
#define ECG_H_
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/**********************************
 * Macros
 **********************************/
#define DUMMY           0xFF
/*
#define SLAVE_CS_OUT    P2OUT
#define SLAVE_CS_DIR    P2DIR
#define SLAVE_CS_PIN    BIT0
*/
#define TYPE_0_LENGTH   1
#define TYPE_1_LENGTH   2
#define TYPE_2_LENGTH   6

#define MAX_BUFFER_SIZE 20

#define SMCLK_11500     0
#define SMCLK_9600      1
#define ACLK_9600       2

#define UART_MODE       SMCLK_115200
#define WREG            0x00
#define RREG            0x01
#define ECG_FIFO_BURST  0x20
#define ECG_FIFO        0x21
#define RTOR            0x25
#define NO_OP1          0x7F



int i;
uint32_t DataPacketHeader[20];

signed long ecgdata;
unsigned long data;
unsigned char SPI_temp_32b[4];
unsigned char SPI_temp_32b1[4];
unsigned long ECG_FIFOType0[TYPE_1_LENGTH];
unsigned long RTORType0[TYPE_1_LENGTH];

//unsigned char Detection_Done=0;



typedef enum SPI_ModeEnum{
    IDLE_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    TIMEOUT_MODE
} SPI_Mode;

SPI_Mode MasterMode;
unsigned char TransmitRegAddr;
uint32_t ReceiveBuffer[MAX_BUFFER_SIZE];
uint32_t RXByteCtr;
uint32_t ReceiveIndex;
long double TransmitBuffer[MAX_BUFFER_SIZE];
uint32_t TXByteCtr;
uint32_t TransmitIndex;


#define DECIMAL_COUNT 5
#define BAUD_RATE 115200
unsigned int uart_data;
//signed int num_digit=0;


/*****************************************
 * Function decleration
 *****************************************/
void initUART();
void SendUCA0Data(unsigned char val);
uint8_t SPI_transfer(uint8_t data);
SPI_Mode SPI_Master_WriteReg(unsigned char *reg_addr, unsigned long reg_data, unsigned char count);
SPI_Mode SPI_Master_ReadReg(unsigned long reg_addr, unsigned long count);
SPI_Mode SPI_Master_ReadReg1(unsigned long reg_addr, unsigned long count);
void initClockTo16MHz();
void initUART();
void initSPI();
void initGPIO();
void TimerZero();
void plot(int DATA);
void F2A(float n, char *res, int afterpoint);
void uart_string(unsigned char *str);
void ECG_init();
void ECG_Start();
#endif /* ECG_H_ */

