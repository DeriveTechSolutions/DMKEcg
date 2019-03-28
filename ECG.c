#include "driverlib.h"
#include <msp430.h>
#include "ECG.h"
#include "uart_driverlib.h"
/**************************************
 * Global Variables
 **************************************/
static unsigned int HR,RR;
unsigned long status0,status1,status2;

static unsigned char SW_RST[TYPE_0_LENGTH]     = {0x08}; // MAX30003 Register for Software Reset
static unsigned char SYNCH[TYPE_0_LENGTH]      = {0x09}; // MAX30003 Register for Synchronization Master Slave
static unsigned char EN_INT[TYPE_0_LENGTH]     = {0x02}; // MAX30003 Register for Interrupt1
static unsigned char EN_INT2[TYPE_0_LENGTH]    = {0x03}; // MAX30003 Register for Interrupt2
static unsigned char CNFG_GEN[TYPE_0_LENGTH]   = {0x10}; // MAX30003 Register for General Configuration of AFE
static unsigned char CNFG_CAL[TYPE_0_LENGTH]   = {0x12}; // MAX30003 Register for Calculation Configuration of AFE
static unsigned char CNFG_EMUX[TYPE_0_LENGTH]  = {0x14}; // MAX30003 Register for Input Multiplexer Configuration
static unsigned char CNFG_ECG[TYPE_0_LENGTH]   = {0x15}; // MAX30003 Register for ECG channel Configuration
static unsigned char CNFG_RTOR1[TYPE_0_LENGTH] = {0x1D}; // MAX30003 Register for Heart Rate Configuration1
uint32_t LEAD__STATUS_BUFFER[TYPE_2_LENGTH] = {0};

void ECG_init(void)
{

    initClockTo16MHz();                           // Start the function of System main clock
    initSPI();                                    // Configure SPI Pins(3.3-3.4-2.7), Module (UCA0), Clock (2MHz), Interrupt,etc.
    uart_init();
    SPI_start();
    initGPIO();                                   // External sampling frequency for AFE MAX30003


    SPI_Master_WriteReg(SW_RST, 0x000000, TYPE_0_LENGTH);     // Software Reset Register
    __delay_cycles(2000);               //0x081217  0x481317                     // Wait to write the register value in Slave
    //SPI_Master_WriteReg(CNFG_GEN, 0x081007, TYPE_0_LENGTH); // Configure AFE to
    SPI_Master_WriteReg(CNFG_GEN, 0x480A17, TYPE_0_LENGTH);   // 00: ULTRA LOW POWER LEAD ON DETECTATION Disable/01: ULTRA LOW POWER LEAD ON DETECTATION ENABLE
    __delay_cycles(1000);                                     // Wait to write the register value in Slave
    SPI_Master_WriteReg(CNFG_CAL, 0x720000, TYPE_0_LENGTH);   // Calculations
    __delay_cycles(1000);                                     // Wait to write the register value in Slave
    SPI_Master_WriteReg(CNFG_EMUX,0x0B0000, TYPE_0_LENGTH);   // Input Multiplexer Configured to
    __delay_cycles(1000);

    SPI_Master_WriteReg(CNFG_ECG, 0x005000, TYPE_0_LENGTH);   // ECG channel Configured
    __delay_cycles(1000);                                     // Wait to write the register value in Slave          // sampling rate 0x805000(10)=128sps ; 0x0405000(01)=256sps ; 0x005000(00)=512sps.
    //SPI_Master_WriteReg(CNFG_RTOR1, 0x3f9600, TYPE_0_LENGTH); // Heart Rate Configured
    SPI_Master_WriteReg(CNFG_RTOR1, 0x3f9600, TYPE_0_LENGTH);
    __delay_cycles(1000);                                     // Wait to write the register value in Slave
    SPI_Master_WriteReg(SYNCH, 0x000000, TYPE_0_LENGTH);      // Synchronized AFE
    __delay_cycles(1000);                                     // Wait to write the register value in Slave
}
void CopyArray(uint32_t *source, uint32_t *dest, uint32_t count)
{
    uint32_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}
void ECG_Start(void)
{
    static unsigned char first_reading=1;
    static unsigned char RR_previous,HR_previous;

    SPI_Master_ReadReg(ECG_FIFO, TYPE_0_LENGTH);      // Read the data from ECG_FIFO register
     unsigned long data0 = (unsigned long) (SPI_temp_32b[1]); // Received data from slave is stored in data0 variable
    data0 = data0 <<24;                               // Left shift data to Store only last 8-bit
     unsigned long data1 = (unsigned long) (SPI_temp_32b[2]); // Received data from slave is stored in data1 variable
    data1 = data1 <<16;                               // Left shift data to Store only middle 8-bit
     unsigned long data2 = (unsigned long) (SPI_temp_32b[3]); // Received data from slave is stored in data2 variable
    data2 = data2 >>6;                                // Right shift data to Store only Starting 6-bit
    data2 = data2 & 0x03;                             // complete 8-bits by anding last two bits with 1
    DATA = (unsigned long) (data0 | data1 | data2);   // Adding all the 8-bit data and store it to the data variable
    ecgdata = (signed long) (DATA);                   // Type casting from unsigned integer to signed integer and store it into ecgdata variable
    //----------------------------RTOR-----------------------------------
    SPI_Master_ReadReg1(RTOR, TYPE_0_LENGTH);         // Read the data from ECG_FIFO register
     unsigned long RTOR_msb = (unsigned long) (SPI_temp_32b1[1]); // Received data from slave is stored in RTOR_msb variable
     unsigned long RTOR_lsb = (unsigned long) (SPI_temp_32b1[2]); // Received data from slave is stored in RTOR_lsb variable
     unsigned long rtor =(RTOR_msb<<8 | RTOR_lsb);     // Left shift data to Store only last 8-bit
    rtor = ((rtor >>2) & 0x3fff) ;                    // Shift the data and Store it to rtor variable
     float hr =  60 /((float)rtor*0.007789);           // 0.008 multiplier formula to calculate hr
    RR = (unsigned int)rtor*8;     // 8ms timing resolution of R-R interval
    HR = (unsigned int)hr;               // Type casting from unsigned integer to signed integer and store it into HR Buffer


    SPI_Master_ReadReg2(LEAD_STATUS, TYPE_0_LENGTH);
     status0 = (unsigned long) (SPI_temp_32b2[0]); // Received data from slave is stored in data0 variable
    status0 = status0 <<24;                               // Left shift data to Store only last 8-bit
     status1 = (unsigned long) (SPI_temp_32b2[1]); // Received data from slave is stored in data1 variable
    status1 = status1 <<16;                               // Left shift data to Store only middle 8-bit
     status2 = (unsigned long) (SPI_temp_32b2[2]); // Received data from slave is stored in data2 variable
    status2 = status2 >>6;

    if(SPI_temp_32b[2]>175 || SPI_temp_32b[2]<180)
    {

     //   SPI_Master_WriteReg(CNFG_GEN, 0x000000, TYPE_0_LENGTH);   // 00: ULTRA LOW POWER LEAD ON DETECTATION Disable/01: ULTRA LOW POWER LEAD ON DETECTATION ENABLE

        SPI_Master_WriteReg(EN_INT, 0x000000, TYPE_0_LENGTH);
        //__delay_cycles(1000);
        SPI_Master_WriteReg(EN_INT2, 0x000000, TYPE_0_LENGTH);
        //__delay_cycles(1000);

    }
    else
    {
 //       SPI_Master_WriteReg(CNFG_GEN, 0x480A17, TYPE_0_LENGTH);   // 00: ULTRA LOW POWER LEAD ON DETECTATION Disable/01: ULTRA LOW POWER LEAD ON DETECTATION ENABLE
 //       __delay_cycles(1000);
        SPI_Master_WriteReg(EN_INT, 0x000801, TYPE_0_LENGTH);
        //__delay_cycles(1000);
        SPI_Master_WriteReg(EN_INT2, 0x100001, TYPE_0_LENGTH);
        //__delay_cycles(1000);
    }

    DataPacketHeader[0] = 0x0A;                       // Start bit send to uart for ECG graph
    DataPacketHeader[1] = 0xFA;                       //
    DataPacketHeader[2] = 0x0C;                       //
    DataPacketHeader[3] = 0;                          //
    DataPacketHeader[4] = 0x02;                       //
    DataPacketHeader[5] = ecgdata;                    //
    DataPacketHeader[6] = ecgdata>>8;                 //
    DataPacketHeader[7] = ecgdata>>16;                // ECG wave data
    DataPacketHeader[8] = ecgdata>>24;                //
    DataPacketHeader[9] =  RR ;                       // R-to-R value
    DataPacketHeader[10] = RR >>8;                    //
    DataPacketHeader[11] = 0x00;                      //
    DataPacketHeader[12] = 0x00;                      //
    DataPacketHeader[13] = HR ;                       // Heart Rate value
    DataPacketHeader[14] = HR >>8;                    //
    DataPacketHeader[15] = 0x00;                      //
    DataPacketHeader[16] = 0x00;                      //
    DataPacketHeader[17] = 0x00;                      //
    DataPacketHeader[18] = 0x0b;                      // Stop bit send to uart for ECG graph
//------------------------------Averaging-------------------------------------------
    if (first_reading==1)
    {
        HR_previous = HR;
        RR_previous = RR;
        first_reading = 2;
    }
    else if (first_reading==2)
    {
        if (HR > HR_previous)
        {
            HR = HR_previous + 1;
            HR_previous = HR;
        }
        else if (HR < HR_previous)
        {
            HR = HR_previous - 1;
            HR_previous = HR;
        }
        if (RR > RR_previous)
            {
            RR = RR_previous + 1;
            RR_previous = RR;
            }
        else if (RR < RR_previous)
        {
            RR = RR_previous - 1;
            RR_previous = RR;
        }
    }
//    plot('E'); // ECG wave on SerialPlotter
//------------------------------INTR-----------------------------------------
     //Test P1.4status0
     if (GPIO_INPUT_PIN_HIGH == GPIO_getInputPinValue(
             GPIO_PORT_P1,
             GPIO_PIN6
             )){// LEAD OFF Detection POS
         //if P1.4 set, set P1.0
         LEAD_OFF_POS_FLAG = 1;
         GPIO_setOutputHighOnPin(
             GPIO_PORT_P1,
             GPIO_PIN0
             );
     } else   {
         //else reset
         GPIO_setOutputLowOnPin(
             GPIO_PORT_P1,
             GPIO_PIN0
             );
     }
     if (GPIO_INPUT_PIN_HIGH == GPIO_getInputPinValue(
             GPIO_PORT_P2,
             GPIO_PIN2
             )){// LEAD OFF Detection NEG
         LEAD_OFF_NEG_FLAG = 1;
         //if P1.4 set, set P1.0
         GPIO_setOutputHighOnPin(
             GPIO_PORT_P4,
             GPIO_PIN7
             );
     } else   {
         //else reset
         GPIO_setOutputLowOnPin(
             GPIO_PORT_P4,
             GPIO_PIN7
             );
     }
//---------------------------------------------------------------------------
}

/**********************************************
 * Initialize SPI send
 **********************************************/
void SendUCA0Data(unsigned char val)
{
    while (!(USCI_A_SPI_getInterruptStatus(USCI_A0_BASE, USCI_A_SPI_TRANSMIT_INTERRUPT)));// USCI_A0 TX buffer ready?
    USCI_A_SPI_transmitData(USCI_A0_BASE,val);       // Send value
}

/**********************************************
 * Initialize SPI Communication
 **********************************************/
 uint8_t SPI_transfer( uint8_t data)
{

    static unsigned char dummy_rx;
    while (USCI_A_SPI_isBusy(USCI_A0_BASE)) ;        // Check for the SPI State
    USCI_A_SPI_transmitData(
            USCI_A0_BASE,                            // Send data to SPI to Slave
            data
            );
    dummy_rx = USCI_A_SPI_receiveData(USCI_A0_BASE); // Read Slave data over UCA0RXBUF and store it to dummy_rx variable
    USCI_A_SPI_isBusy (USCI_A0_BASE);                // Change the State of the SPI
    while (USCI_A_SPI_isBusy(USCI_A0_BASE)) ;        // Check for the SPI State
    return (uint8_t) (dummy_rx & 0xff);              // Return the value of dummy_rx to the function
}

/**********************************************
 * Initialize SPI Write
 **********************************************/
SPI_Mode SPI_Master_WriteReg( unsigned char *reg_addr,  unsigned long reg_data,  unsigned char count)
{

    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = (*reg_addr<<1) | WREG;
    TXByteCtr = count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);    // CS Low
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);    // CS Low
    SendUCA0Data(0x00);                                 // Send dummy 8-bit
    SendUCA0Data(0x00);                                 // send dummy 8-bit
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);   // CS High
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN4);   // CS High
    __delay_cycles(1000);                               // wait for
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);    // CS Low
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);    // CS Low
    SendUCA0Data(TransmitRegAddr);                      // send 8-bit Reg address
    SendUCA0Data(reg_data>>16);                         // send 1st frame of 8-bit MSB data to write to the Reg
    SendUCA0Data(reg_data>>8);                          // send 2nd frame of 8-bit data to write to the Reg
    SendUCA0Data(reg_data);                             // send 3rd frame of 8-bit (LSB) data to write to the Reg
                                                        // Total 8-bit Address + 24-bit data = 32-bit
    __delay_cycles(500);                                // wait

    //__bis_SR_register(GIE);                             // Global interrupt enable
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);   // CS High
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN4);   // CS High
    return MasterMode;
}

/**********************************************
 * Initialize SPI Read
 **********************************************/
//-----------------------------ECG_FIFO--------------------------------
SPI_Mode SPI_Master_ReadReg( unsigned long reg_addr,  unsigned long count)
{

    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = (reg_addr<<1 ) | RREG;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);    // CS Low
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);    // CS Low
    SendUCA0Data(TransmitRegAddr);                      // send 8-bit Reg address
    for ( mandar = 0; mandar < 3; mandar++)                            // repeat the process for 3 times
    {
     SPI_temp_32b[mandar] = SPI_transfer(0xff);              // send "0xFF" to read from slave : UCA0RXBUF
    }                                                   // store the slave data into SPI_temp_32b[i]
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);   // CS High
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN4);   // CS High
    __bis_SR_register(GIE);                             // Global interrupt enable
    return MasterMode;
}

//--------------------------RTOR-----------------------------------
SPI_Mode SPI_Master_ReadReg1( unsigned long reg_addr,  unsigned long count)
{

    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = (reg_addr<<1 ) | RREG;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);    // CS Low
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);    // CS Low
    SendUCA0Data(TransmitRegAddr);                      // send 8-bit Reg addres
    for ( mandar = 0; mandar < 3; mandar++)                            // repeat the process for 3 times
    {
     SPI_temp_32b1[mandar] = SPI_transfer(0xff);             // send "0xFF" to read from slave : UCA0RXBUF
    }
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);   // CS High
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN4);   // CS High
    __bis_SR_register(GIE);                             // Global interrupt enable
    return MasterMode;
}
//--------------------------Status-----------------------------------
SPI_Mode SPI_Master_ReadReg2( unsigned long reg_addr,  unsigned long count)
{

    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = (reg_addr<<1 ) | RREG;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);    // CS Low
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN4);    // CS Low
    SendUCA0Data(TransmitRegAddr);                      // send 8-bit Reg addres
    for ( mandar = 0; mandar < 3; mandar++)                            // repeat the process for 3 times
    {
     SPI_temp_32b2[mandar] = SPI_transfer(0xff);             // send "0xFF" to read from slave : UCA0RXBUF
    }
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);   // CS High
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN4);   // CS High
    __bis_SR_register(GIE);                             // Global interrupt enable
    return MasterMode;
}
/**********************************************
 * Initialize clock only for TEST purpose
 **********************************************/
void initClockTo16MHz()
{
    UCSCTL3 |= SELREF_2;                          // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA_2;                            // Set ACLK = REFO
    __bis_SR_register(SCG0);                      // Disable the FLL control loop
    UCSCTL0 = 0x0000;                             // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_5;                          // Select DCO range 16MHz operation
    UCSCTL2 = 479;                                // Set DCO Multiplier for 16MHz
    P2DIR |= BIT2;                         // Pin direction for SMCLK output signal
    P2SEL |= BIT2;                                              // (N + 1) * FLLRef = Fdco
                                                  // (487 + 1) * 32768 = 16MHz
                                                  // Set FLL Div = fDCOCLK
    __bic_SR_register(SCG0);                      // Enable the FLL control loop
                                                  // Worst-case settling time for the DCO when the DCO range bits have been
                                                  // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
                                                  // UG for optimization.
                                                  // 32 x 32 x 16 MHz / 32,768 Hz = 500000 = MCLK cycles for DCO to settle
    __delay_cycles(500000);                       // Loop until XT1,XT2 & DCO fault flag is cleared
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);// Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                        // Clear fault flags
    }while (SFRIFG1&OFIFG);                       // Test oscillator fault flag
}
/**********************************************
 * Initialize GPIO
 **********************************************/
void initGPIO()
{
  //INT lead off detection
    //Set P1.0 to output direction
    GPIO_setAsOutputPin(
          GPIO_PORT_P1,
          GPIO_PIN0
          );
    GPIO_setAsOutputPin(
          GPIO_PORT_P4,
          GPIO_PIN7
          );
    //Set P1.4 to input direction
    GPIO_setAsInputPin(
        GPIO_PORT_P1,
        GPIO_PIN6
        );
    GPIO_setAsInputPin(
        GPIO_PORT_P2,
        GPIO_PIN2
        );
  //UART pins
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4 + GPIO_PIN5);   // P3.3-MOSI,P3.4-MISO

  //Crystal
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN4 + GPIO_PIN5);   // P3.3-MOSI,P3.4-MISO

}
/**********************************************
 * Initialize SPI Pins
 **********************************************/
void initSPI()
{
      GPIO_setAsPeripheralModuleFunctionOutputPin(
              GPIO_PORT_P3,
              GPIO_PIN3                  // Configure SPI MISO Pin at P3.3 and MOSI at P3.4
              );
      GPIO_setAsPeripheralModuleFunctionInputPin(
              GPIO_PORT_P3,
             GPIO_PIN4                  // Configure SPI MISO Pin at P3.3 and MOSI at P3.4
              );
      GPIO_setAsPeripheralModuleFunctionOutputPin(
              GPIO_PORT_P2,
              GPIO_PIN7                // Configure SPI Clock Pin SCLK at P2.7
              );

      USCI_A_SPI_disable(USCI_A0_BASE);              // SPI Control and Reset



      USCI_A_SPI_initMasterParam usci_a0_param = {0};


      usci_a0_param.msbFirst = USCI_A_SPI_MSB_FIRST;// MSB First, 8-bit, Master, 3-pin mode, Synchronous
      usci_a0_param.clockPhase = USCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;    //SPI Mode - 2, CPOL=1 & CPHA=0
      usci_a0_param.clockPolarity = USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;               //Clock Polarity: The inactive state is high


      usci_a0_param.selectClockSource = USCI_A_SPI_CLOCKSOURCE_SMCLK;                       // SPI Clock source SMCLK
      usci_a0_param.clockSourceFrequency = UCS_getSMCLK(); // Get Smclk source clock 16mhz
      usci_a0_param.desiredSpiClock = 8000000;      // SPI clock at 8mhz


      USCI_A_SPI_initMaster(
              USCI_A0_BASE,
              &usci_a0_param
              );

      UCA0BR0 = 0x02;                               // /2
      UCA0BR1 = 0;                                  //
      UCA0MCTL = 0;                                 // No modulation must be cleared for SPI

      USCI_A_SPI_enable(USCI_A0_BASE);              // SPI Control and Reset

/*
      USCI_A_SPI_enableInterrupt(
              USCI_A0_BASE,
              USCI_A_SPI_RECEIVE_INTERRUPT
              );                                    // SPI Receive interrupt
*/

      GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0); // Configure Chip select Start/end the communication
      GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN4); // Configure Chip select Start/end the communication
      GPIO_setOutputHighOnPin(
                    GPIO_PORT_P2,
                    GPIO_PIN0
                    );
      GPIO_setOutputHighOnPin(
                    GPIO_PORT_P2,
                    GPIO_PIN4
                    );
}

void plot(char wavetype)
{

    if(wavetype == 'H')
    {
        uart_data_send(HR);
        uart_string("\n\r");
    }
    else
    {
        if(wavetype == 'R')
        {
            uart_data_send(RR);
            uart_string("\n\r");
        }
        else
        {
            if(wavetype == 'E')
            {
                //uart_string("W");
                uart_data_send(SPI_temp_32b[2]);
                uart_string("\n\r");
            }
            else
            {
                for(mandar=0;mandar<19;mandar++)                                 // Send each buffer one by one
                {
                    while (!(UCA1IFG & UCTXIFG));                 // Check the uart flags
                    UCA1TXBUF =DataPacketHeader[mandar];               // Send the buffer on uart
                    }
            }
        }
    }
}

void SPI_start()
{
    static unsigned long uca0_rx_val = 0;
    if(UCA0IFG)
    {
        uca0_rx_val = USCI_A_SPI_receiveData(USCI_A0_BASE);       // Vector 2 - RXIFG
                switch (MasterMode)                   // SPI Master mode enable
                {
                    case TX_REG_ADDRESS_MODE:         // Start the Master in Transmitting Register Address mode
                        if (RXByteCtr)                // Check the data is present or not
                        {
                            MasterMode = RX_DATA_MODE;// Need to start receiving now
                            __delay_cycles(2000);     // Wait for the Slave to initialized in SPI mode
                            SendUCA0Data(DUMMY);      // Send dummy to establish the communication between master and slave device
                        }
                        else
                        {
                            MasterMode = TX_DATA_MODE;// Continue to transmision with the data in Transmit Buffer
                            SendUCA0Data(TransmitBuffer[TransmitIndex++]); // Send the data to SPI UCA0TXBUF
                            TXByteCtr--;              //  Each time after sending data decrease the pointer value
                        }
                        break;

                    case TX_DATA_MODE:                // Start the Master in Transmitting mode
                        if (TXByteCtr)                // Check the data is present or not
                        {
                            SendUCA0Data(TransmitBuffer[TransmitIndex++]); // Send the data to SPI UCA0TXBUF
                            TXByteCtr--;              //  Each time after sending data decrease the pointer value
                        }
                        else
                        {
                            MasterMode = IDLE_MODE;   // Switch to Ideal_mode Done with communication
                        }
                        break;

                    case RX_DATA_MODE:                // Start the Master in Receiving mode
                        if (RXByteCtr)                // Check the data is present or not
                        {
                            ReceiveBuffer[ReceiveIndex++] = uca0_rx_val; // Store the data into ReciveBuffer register
                            RXByteCtr--;              // Each time after receiving data decrease the pointer value
                        }
                        if (RXByteCtr == 0)           // Check for the the whole data Should be send
                        {
                            MasterMode = IDLE_MODE;   // Switch to Ideal_mode Done with communication
                        }
                        else
                        {
                            SendUCA0Data(DUMMY);      // Send DUMMY(0xff) to start receiving from the slave
                        }
                        break;

                    default:
                        __no_operation();             // No operation
                        break;
                }
                __delay_cycles(100);                  // Wait for Communication to end
    }
}

void ECG_Stop()
{
    USCI_A_SPI_disable(USCI_A0_BASE);              // SPI Control and Reset
    //Set all the ECG Port Pins to Output Direction
    GPIO_setAsOutputPin(GPIO_PORT_P1,
                        GPIO_PIN6);                // 1.6 Lead off detection +ve
    GPIO_setAsOutputPin(GPIO_PORT_P2,
                        GPIO_PIN2 + GPIO_PIN7 + GPIO_PIN0 + GPIO_PIN4);// 2.2 Lead off detection -ve, 2.7 SPI SCLK, 2.0 SPI CSB, 2.4 SPI CSB2
    GPIO_setAsOutputPin(GPIO_PORT_P3,
                        GPIO_PIN3 + GPIO_PIN4);    // 3.3 SPI MOSI,  3.4 SPI MISO
    //Set all the ECG Port Pins as Low
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,
                           GPIO_PIN6);             // Disable/Low 1.6 Lead off detection +ve
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,
                           GPIO_PIN2 + GPIO_PIN7 + GPIO_PIN0 + GPIO_PIN4);// Disable/Low 2.2 Lead off detection -ve, 2.7 SPI SCLK, 2.0 SPI CSB, 2.4 SPI CSB2
    GPIO_setOutputLowOnPin(GPIO_PORT_P3,
                           GPIO_PIN3 + GPIO_PIN4); // Disable/Low 3.3 SPI MOSI, 3.4 SPI MISO
}
