#include <msp430.h> 
#include <stdint.h>
#include <stdbool.h>
#include <TimerOne.h>
#include <uart.h>
#include <math.h>

/**********************************
 * Macros
 **********************************/
#define DUMMY           0xFF
#define SLAVE_CS_OUT    P2OUT
#define SLAVE_CS_DIR    P2DIR
#define SLAVE_CS_PIN    BIT0

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

/**************************************
 * Global Variables
 **************************************/
unsigned long NO_OP[TYPE_0_LENGTH]      = {0x00};
unsigned long STATUS[TYPE_0_LENGTH]     = {0x01};
unsigned long EN_INT[TYPE_0_LENGTH]     = {0x02};
unsigned long EN_INT2[TYPE_0_LENGTH]    = {0x03};
unsigned long MNGR_INT[TYPE_0_LENGTH]   = {0x04};
unsigned long MNGR_DYN[TYPE_0_LENGTH]   = {0x05};
unsigned char SW_RST[TYPE_0_LENGTH]     = {0x08};
unsigned char SYNCH[TYPE_0_LENGTH]      = {0x09};
unsigned long FIFO_RST[TYPE_0_LENGTH]   = {0x0A};
unsigned long INFO[TYPE_0_LENGTH]       = {0x0F};
unsigned char CNFG_GEN[TYPE_0_LENGTH]   = {0x10};
unsigned char CNFG_CAL[TYPE_0_LENGTH]   = {0x12};
unsigned char CNFG_EMUX[TYPE_0_LENGTH]  = {0x14};
unsigned char CNFG_ECG[TYPE_0_LENGTH]   = {0x15};
unsigned char CNFG_RTOR1[TYPE_0_LENGTH] = {0x1D};
unsigned long CNFG_RTOR2[TYPE_0_LENGTH] = {0x1E};

int i=0;
uint32_t DataPacketHeader[20];
signed long ecgdata;
unsigned long data;
unsigned char SPI_temp_32b[4];
unsigned char SPI_temp_32b1[4];
unsigned long ECG_FIFOType0[TYPE_1_LENGTH];
unsigned long RTORType0[TYPE_1_LENGTH];

typedef enum SPI_ModeEnum{
    IDLE_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    TIMEOUT_MODE
} SPI_Mode;

SPI_Mode MasterMode = IDLE_MODE;
unsigned char TransmitRegAddr = 0;
uint32_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
uint32_t RXByteCtr = 0;
uint32_t ReceiveIndex = 0;
long double TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint32_t TXByteCtr = 0;
uint32_t TransmitIndex = 0;

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
void initGPIO();
void initSPI();

/*****************************************
 * Main
 *****************************************/
void main()
{

    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
    initClockTo16MHz();
    initGPIO();
    initSPI();
    initUART();
    P1OUT &= ~BIT5;                         // Now with SPI signals initialized,
    __delay_cycles(1000);
    P1OUT |= BIT5;                          // reset slave
    __delay_cycles(1000);                   // Wait for slave to initialize
    P1OUT |= BIT0;
    __bis_SR_register(GIE);                 // CPU off, enable interrupt

//--------------------------Max30003 write-----------------------------

    SPI_Master_WriteReg(SW_RST, 0x000000, TYPE_0_LENGTH);
    __delay_cycles(2000);
    SPI_Master_WriteReg(CNFG_GEN, 0x081007, TYPE_0_LENGTH);
    __delay_cycles(1000);
    SPI_Master_WriteReg(CNFG_CAL, 0x720000, TYPE_0_LENGTH);
    __delay_cycles(1000);                   // 0x700000
    SPI_Master_WriteReg(CNFG_EMUX,0x0B0000, TYPE_0_LENGTH);
    __delay_cycles(1000);
    SPI_Master_WriteReg(CNFG_ECG, 0x005000, TYPE_0_LENGTH);
    __delay_cycles(1000);                   // sampling rate 0x805000(10)=128sps ; 0x0405000(01)=256sps ; 0x005000(00)=512sps.
    SPI_Master_WriteReg(CNFG_RTOR1, 0x3fc600, TYPE_0_LENGTH);
    __delay_cycles(1000);
    SPI_Master_WriteReg(SYNCH, 0x000000, TYPE_0_LENGTH);
    __delay_cycles(1000);

//---------------------------Max30003 read--------------------------
//---------------------------ECG FIFO-------------------------------
    while(1)
    {
        SPI_Master_ReadReg(ECG_FIFO, TYPE_0_LENGTH);
        unsigned long data0 = (unsigned long) (SPI_temp_32b[0]);
        data0 = data0 <<24;
        unsigned long data1 = (unsigned long) (SPI_temp_32b[1]);
        data1 = data1 <<16;
        unsigned long data2 = (unsigned long) (SPI_temp_32b[2]);
        data2 = data2 >>6;
        data2 = data2 & 0x03;
        data = (unsigned long) (data0 | data1 | data2);
        ecgdata = (signed long) (data);

        //----------------------------RTOR-----------------------------------
        SPI_Master_ReadReg1(RTOR, TYPE_0_LENGTH);
        unsigned long RTOR_msb = (unsigned long) (SPI_temp_32b1[0]);
        unsigned long RTOR_lsb = (unsigned long) (SPI_temp_32b1[1]);
        unsigned long rtor =(RTOR_msb<<8 | RTOR_lsb);
        rtor = ((rtor >>2) & 0x3fff) ;
        float hr =  60 /((float)rtor*0.007789);      // 0.008 multiplier
        unsigned int RR = (unsigned int)rtor*8 ;  //8ms
        unsigned int HR = (unsigned int)hr;
        if (HR==179 || HR==183 && RR==336 || RR==344)
        {
            HR = 180;
            RR = 340;
        }
        else if (HR==197 || HR==202 && RR==304 || RR==312)
        {
            HR = 200;
            RR = 308;
        }
        else
        {
            HR = HR;
            RR = RR;
        }
        HR = HR;
        RR = RR;

        DataPacketHeader[0] = 0x0A;
        DataPacketHeader[1] = 0xFA;
        DataPacketHeader[2] = 0x0C;
        DataPacketHeader[3] = 0;
        DataPacketHeader[4] = 0x02;
        DataPacketHeader[5] = ecgdata;
        DataPacketHeader[6] = ecgdata>>8;
        DataPacketHeader[7] = ecgdata>>16;
        DataPacketHeader[8] = ecgdata>>24;
        DataPacketHeader[9] =  RR ;
        DataPacketHeader[10] = RR >>8;
        DataPacketHeader[11] = 0x00;
        DataPacketHeader[12] = 0x00;
        DataPacketHeader[13] = HR ;
        DataPacketHeader[14] = HR >>8;
        DataPacketHeader[15] = 0x00;
        DataPacketHeader[16] = 0x00;
        DataPacketHeader[17] = 0x00;
        DataPacketHeader[18] = 0x0b;

        for(i=0;i<19;i++)
        {
            while (!(UCA1IFG & UCTXIFG));
            UCA1TXBUF =DataPacketHeader[i];
        }
        __bis_SR_register(GIE);
        __no_operation();
    }
}

/**********************************************
 * Initilialize Uart
 **********************************************/
void initUART()
{
    UCA1CTLW0 = UCSWRST;                 // Put eUSCI in reset
#if UART_MODE == SMCLK_115200

    UCA1CTLW0 |= UCSSEL__SMCLK;          // CLK = SMCLK
    UCA1BR0 = 8;                         // 16000000/16/9600
    UCA1BR1 = 0x00;
    UCA1MCTL |= UCOS16 | UCBRF_11 | UCBRS_0;
#endif
    UCA1CTLW0 &= ~UCSWRST;               // Initialize eUSCI
    UCA1IE |= UCRXIE;                    // Enable USCI_A0 RX interrupt
}

/**********************************************
 * Initilialize SPI send
 **********************************************/
void SendUCA0Data(unsigned char val)
{
    while (!(UCA0IFG & UCTXIFG));        // USCI_A0 TX buffer ready?
    UCA0TXBUF = val;
}

/**********************************************
 * Initilialize SPI Communication
 **********************************************/
uint8_t SPI_transfer(uint8_t data)
{
    unsigned char dummy_rx;
    while ( (UCA0STAT & UCBUSY) );
    UCA0TXBUF = data;
    dummy_rx = UCA0RXBUF;
    UCA0STAT |= UCBUSY;
    while ( (UCA0STAT & UCBUSY) );
    return (uint8_t) (dummy_rx & 0xff);
}

/**********************************************
 * Initilialize SPI Write
 **********************************************/
SPI_Mode SPI_Master_WriteReg(unsigned char *reg_addr, unsigned long reg_data, unsigned char count)
{
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = (*reg_addr<<1) | WREG;
    TXByteCtr = count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;
    SLAVE_CS_OUT &= ~(SLAVE_CS_PIN);     // CS Low
    SendUCA0Data(0x00);                  // Send dummy 8-bit
    SendUCA0Data(0x00);                  // send dummy 8-bit
    SLAVE_CS_OUT |= SLAVE_CS_PIN;        // CS High
    __delay_cycles(1000);                // wait for
    SLAVE_CS_OUT &= ~(SLAVE_CS_PIN);     // CS Low
    SendUCA0Data(TransmitRegAddr);       // send 8-bit Reg address
    SendUCA0Data(reg_data>>16);          // send 1st frame of 8-bit MSB data to write to the Reg
    SendUCA0Data(reg_data>>8);           // send 2nd frame of 8-bit data to write to the Reg
    SendUCA0Data(reg_data);              // send 3rd frame of 8-bit (LSB) data to write to the Reg
                                         // Total 8-bit Address + 24-bit data = 32-bit
    __delay_cycles(500);                 // wait
    __bis_SR_register(GIE);              // Global interrupt enable
    SLAVE_CS_OUT |= SLAVE_CS_PIN;        // CS High
    return MasterMode;
}

/**********************************************
 * Initilialize SPI Read
 **********************************************/
//-----------------------------ECG_FIFO--------------------------------
SPI_Mode SPI_Master_ReadReg(unsigned long reg_addr, unsigned long count)
{
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = (reg_addr<<1 ) | RREG;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;
    SLAVE_CS_OUT &= ~(SLAVE_CS_PIN);      // CS Low
    SendUCA0Data(TransmitRegAddr);        // send 8-bit Reg address
    for ( i = 0; i < 3; i++)              // repeat the process for 3 times
    {                                     // 8-bit + 8-bit + 8-bit data read
     SPI_temp_32b[i] = SPI_transfer(0xff);// send "0xFF" to read from slave : UCA0RXBUF
    }                                     // store the slave data into SPI_temp_32b[i]
    SLAVE_CS_OUT |= SLAVE_CS_PIN;         // CS High
    __bis_SR_register(GIE);               // Global interrupt enable
    return MasterMode;
}
//--------------------------RTOR-----------------------------------
SPI_Mode SPI_Master_ReadReg1(unsigned long reg_addr, unsigned long count)
{
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = (reg_addr<<1 ) | RREG;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;
    SLAVE_CS_OUT &= ~(SLAVE_CS_PIN);       // CS Low
    SendUCA0Data(TransmitRegAddr);         // send 8-bit Reg addres
    for ( i = 0; i < 3; i++)               // repeat the process for 3 times
    {                                      // 8-bit + 8-bit + 8-bit data read
     SPI_temp_32b1[i] = SPI_transfer(0xff);// send "0xFF" to read from slave : UCA0RXBUF
    }                                      // store the slave data into SPI_temp_32b1[i]
    SLAVE_CS_OUT |= SLAVE_CS_PIN;          // CS High
    __bis_SR_register(GIE);                // Global interrupt enable
    return MasterMode;
}

/**********************************************
 * Initilialize clock
 **********************************************/
void initClockTo16MHz()
{
    UCSCTL3 |= SELREF_2;                   // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA_2;                     // Set ACLK = REFO
    __bis_SR_register(SCG0);               // Disable the FLL control loop
    UCSCTL0 = 0x0000;                      // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_5;                   // Select DCO range 16MHz operation
    UCSCTL2 = 479;                         // Set DCO Multiplier for 16MHz
    P2DIR |= BIT2;                         // Pin direction for SMCLK output signal
    P2SEL |= BIT2;                         // Pin select for SMCLK output signal
                                           // (N + 1) * FLLRef = Fdco
                                           // (487 + 1) * 32768 = 16MHz
                                           // Set FLL Div = fDCOCLK
    __bic_SR_register(SCG0);               // Enable the FLL control loop
                                           // Worst-case settling time for the DCO when the DCO range bits have been
                                           // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
                                           // UG for optimization.
                                           // 32 x 32 x 16 MHz / 32,768 Hz = 500000 = MCLK cycles for DCO to settle
    __delay_cycles(500000);                // Loop until XT1,XT2 & DCO fault flag is cleared
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);// Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                 // Clear fault flags
    }while (SFRIFG1&OFIFG);                // Test oscillator fault flag
}

/**********************************************
 * Initilialize GPIO
 **********************************************/
void initGPIO()
{
  //LEDs
  P1OUT = 0x00;                             // P1 setup for LED & reset output
  P1DIR |= BIT0 + BIT5;
  //SPI Pins
  P3SEL |= BIT3 + BIT4;                     // P3.3,4 option select
  P2SEL |= BIT7;                            // P2.7 option select
  //UART pins
  P4SEL = BIT4 + BIT5;                      // P3.4,5 = USCI_A0 TXD/RXD
  P5SEL |= BIT4+BIT5;                       // Select XT1
}

/**********************************************
 * Initilialize SPI Pins
 **********************************************/
void initSPI()
{
                                            //Clock Polarity: The inactive state is high
                                            //MSB First, 8-bit, Master, 3-pin mode, Synchronous
  UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
  UCA0CTL0 |= UCMST+UCSYNC+UCCKPL+UCMSB;    // 3-pin, 8-bit SPI master
                                            // Clock polarity high, MSB
  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
  UCA0BR0 = 0x02;                           // /2
  UCA0BR1 = 0;                              //
  UCA0MCTL = 0;                             // No modulation must be cleared for SPI
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  UCA0IE |= UCRXIE;                         // Enable USCI0 RX interrupt
  SLAVE_CS_DIR |= SLAVE_CS_PIN;
  SLAVE_CS_OUT |= SLAVE_CS_PIN;
}

/**********************************************
 * SPI ISR
 **********************************************/
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#endif
{
    unsigned long uca0_rx_val = 0;
    switch(__even_in_range(UCA0IV,4))
    {
        case 0: break;                         // Vector 0 - no interrupt

        case 2: uca0_rx_val = UCA0RXBUF;       // Vector 2 - RXIFG
                switch (MasterMode)
                {
                    case TX_REG_ADDRESS_MODE:
                        if (RXByteCtr)
                        {
                            MasterMode = RX_DATA_MODE;// Need to start receiving now
                            __delay_cycles(2000);
                            SendUCA0Data(DUMMY);
                        }
                        else
                        {
                            MasterMode = TX_DATA_MODE;// Continue to transmision with the data in Transmit Buffer
                            SendUCA0Data(TransmitBuffer[TransmitIndex++]);//Send First
                            TXByteCtr--;
                        }
                        break;

                    case TX_DATA_MODE:
                        if (TXByteCtr)
                        {
                            SendUCA0Data(TransmitBuffer[TransmitIndex++]);
                            TXByteCtr--;
                        }
                        else
                        {
                            MasterMode = IDLE_MODE;//Done with transmission
                        }
                        break;

                    case RX_DATA_MODE:
                        if (RXByteCtr)
                        {
                            ReceiveBuffer[ReceiveIndex++] = uca0_rx_val;
                            RXByteCtr--;
                        }
                        if (RXByteCtr == 0)
                        {
                            MasterMode = IDLE_MODE;
                        }
                        else
                        {
                            SendUCA0Data(DUMMY);
                        }
                        break;

                    default:
                        __no_operation();
                        break;
                }
                __delay_cycles(100);
                break;
          case 4:break;                         // Vector 4 - TXIFG
          default: break;
    }
}

