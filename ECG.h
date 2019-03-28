#ifndef ECG_H_
#define ECG_H_
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/**********************************
 * Macros
 **********************************/
#define DUMMY           0xFF
#define TYPE_0_LENGTH   1
#define TYPE_1_LENGTH   2
#define TYPE_2_LENGTH   6
#define MAX_BUFFER_SIZE 20
#define WREG            0x00
#define RREG            0x01
#define LEAD_STATUS     0x01
#define ECG_FIFO_BURST  0x20
#define ECG_FIFO        0x21
#define RTOR            0x25
#define NO_OP1          0x7F

static char LEAD_OFF_POS_FLAG, LEAD_OFF_NEG_FLAG, LEAD_ON_POS_FLAG, LEAD_ON_NEG_FLAG;
static int mandar;
static uint32_t DataPacketHeader[20];
static signed long ecgdata;
static unsigned long DATA;
static unsigned char SPI_temp_32b[4];
static unsigned char SPI_temp_32b1[4];
static unsigned char SPI_temp_32b2[4];
static unsigned long ECG_FIFOType0[TYPE_1_LENGTH];
static unsigned long RTORType0[TYPE_1_LENGTH];

typedef enum SPI_ModeEnum{
    IDLE_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    TIMEOUT_MODE
} SPI_Mode;

SPI_Mode MasterMode;
static unsigned char TransmitRegAddr;
static uint32_t ReceiveBuffer[MAX_BUFFER_SIZE];
static uint32_t RXByteCtr;
static uint32_t ReceiveIndex;
static long double TransmitBuffer[MAX_BUFFER_SIZE];
static uint32_t TXByteCtr;
static uint32_t TransmitIndex;

/*****************************************
 * Function decleration
 *****************************************/
void SendUCA0Data(unsigned char val);
uint8_t SPI_transfer( uint8_t data);
void CopyArray(uint32_t *source, uint32_t *dest, uint32_t count);
SPI_Mode SPI_Master_WriteReg( unsigned char *reg_addr, unsigned long reg_data, unsigned char count);
SPI_Mode SPI_Master_ReadReg( unsigned long reg_addr, unsigned long count);
SPI_Mode SPI_Master_ReadReg1( unsigned long reg_addr, unsigned long count);
SPI_Mode SPI_Master_ReadReg2( unsigned long reg_addr, unsigned long count);
void initClockTo16MHz();
void initSPI();
void SPI_start();
void initGPIO();
void plot( char wavetype);
void ECG_init();
void ECG_Start();
void ECG_Stop();
#endif /* ECG_H_ */
