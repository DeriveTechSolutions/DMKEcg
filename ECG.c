/*
 * ECG.c
 *
 *  Created on: Feb 11, 2019
 *      Author: Derive01
 */


/*
 * ECG.c
 *
 *  Created on: Feb 11, 2019
 *      Author: Derive01
 */

/*
 * ECG.c
 *
 *  Created on: Feb 5, 2019
 *      Author: Derive01
 */
#include "driverlib.h"
#include <msp430.h>
#include "ECG.h"
#define  H
#define  R
#define  E
unsigned int HR,RR;
//unsigned int HR,RR;
/**************************************
 * Global Variables
 **************************************/
unsigned long NO_OP[TYPE_0_LENGTH]      = {0x00}; // MAX30003 Register for No Operation
unsigned long STATUS[TYPE_0_LENGTH]     = {0x01}; // MAX30003 Register for Status
unsigned long EN_INT[TYPE_0_LENGTH]     = {0x02}; // MAX30003 Register for Interrupt1
unsigned long EN_INT2[TYPE_0_LENGTH]    = {0x03}; // MAX30003 Register for Interrupt2
unsigned long MNGR_INT[TYPE_0_LENGTH]   = {0x04}; // MAX30003 Register for Manage Interrupt mode of AFE
unsigned long MNGR_DYN[TYPE_0_LENGTH]   = {0x05}; // MAX30003 Register for Manage General/Dynamic mode of AFE
unsigned char SW_RST[TYPE_0_LENGTH]     = {0x08}; // MAX30003 Register for Software Reset
unsigned char SYNCH[TYPE_0_LENGTH]      = {0x09}; // MAX30003 Register for Synchronization Master Slave
unsigned long FIFO_RST[TYPE_0_LENGTH]   = {0x0A}; // MAX30003 Register for First In First Out ECG wave
unsigned long INFO[TYPE_0_LENGTH]       = {0x0F}; // MAX30003 Register for Information of Slave ID
unsigned char CNFG_GEN[TYPE_0_LENGTH]   = {0x10}; // MAX30003 Register for General Configuration of AFE
unsigned char CNFG_CAL[TYPE_0_LENGTH]   = {0x12}; // MAX30003 Register for Calculation Configuration of AFE
unsigned char CNFG_EMUX[TYPE_0_LENGTH]  = {0x14}; // MAX30003 Register for Input Multiplexer Configuration
unsigned char CNFG_ECG[TYPE_0_LENGTH]   = {0x15}; // MAX30003 Register for ECG channel Configuration
unsigned char CNFG_RTOR1[TYPE_0_LENGTH] = {0x1D}; // MAX30003 Register for Heart Rate Configuration1
unsigned long CNFG_RTOR2[TYPE_0_LENGTH] = {0x1E}; // MAX30003 Register for Heart Rate Configuration2

void ECG_init(void)
{
    initClockTo16MHz();                           // Start the function of System main clock
    initSPI();                                    // Configure SPI Pins(3.3-3.4-2.7), Module (UCA0), Clock (2MHz), Interrupt,etc.
    initUART();
    initGPIO();// External sampling frequency for AFE MAX30003

    SPI_Master_WriteReg(SW_RST, 0x000000, TYPE_0_LENGTH);     // Software Reset Register
    __delay_cycles(2000);                                     // Wait to write the register value in Slave
    SPI_Master_WriteReg(CNFG_GEN, 0x081007, TYPE_0_LENGTH);     // Configure AFE to
                                                              // 00: ULTRA LOW POWER LEAD ON DETECTATION Disable/01: ULTRA LOW POWER LEAD ON DETECTATION ENABLE
                                                              // 00: Fmstr 32.768Khz ECG 512sps = 0000-0/0100-4
                                                              // 1: ECG channel enable
                                                              // 000: Don't care = 1000-8/1000-8
                                                              // 00: Don't care
                                                              // 00: DC lead off detection disable/1-01: DC lead off detection enable = 0001-1/0001-1
                                                              // 0: ECGP pullup DC lead off polarity
                                                              // 000: DC lead off magnitude 0nAmp/100: 50nAmp magnitude = 0000-0/0100-4
                                                              // 00:
    __delay_cycles(1000);                                     // Wait to write the register value in Slave
    SPI_Master_WriteReg(CNFG_CAL, 0x720000, TYPE_0_LENGTH);   // Calculations
    __delay_cycles(1000);                                     // Wait to write the register value in Slave
    SPI_Master_WriteReg(CNFG_EMUX,0x0B0000, TYPE_0_LENGTH);   // Input Multiplexer Configured to
    __delay_cycles(1000);                                     // Wait to write the register value in Slave
    SPI_Master_WriteReg(CNFG_ECG, 0x005000, TYPE_0_LENGTH);   // ECG channel Configured
    __delay_cycles(1000);                                     // Wait to write the register value in Slave          // sampling rate 0x805000(10)=128sps ; 0x0405000(01)=256sps ; 0x005000(00)=512sps.
    SPI_Master_WriteReg(CNFG_RTOR1, 0x3fc600, TYPE_0_LENGTH); // Heart Rate Configured
    __delay_cycles(1000);                                     // Wait to write the register value in Slave
    SPI_Master_WriteReg(SYNCH, 0x000000, TYPE_0_LENGTH);      // Synchronized Master Slave devices
    __delay_cycles(1000);                                     // Wait to write the register value in Slave
}


void ECG_Start(void)
{
            unsigned char first_reading=0;
            unsigned char RR_previous,HR_previous;


            TimerZero();

            SPI_Master_ReadReg(ECG_FIFO, TYPE_0_LENGTH);      // Read the data from ECG_FIFO register
            unsigned long data0 = (unsigned long) (SPI_temp_32b[0]); // Received data from slave is stored in data0 variable
            data0 = data0 <<24;                               // Left shift data to Store only last 8-bit
            unsigned long data1 = (unsigned long) (SPI_temp_32b[1]); // Received data from slave is stored in data1 variable
            data1 = data1 <<16;                               // Left shift data to Store only middle 8-bit
            unsigned long data2 = (unsigned long) (SPI_temp_32b[2]); // Received data from slave is stored in data2 variable
            data2 = data2 >>6;                                // Right shift data to Store only Starting 6-bit
            data2 = data2 & 0x03;                             // complete 8-bits by anding last two bits with 1
            data = (unsigned long) (data0 | data1 | data2);   // Adding all the 8-bit data and store it to the data variable
            ecgdata = (signed long) (data);                   // Type casting from unsigned integer to signed integer and store it into ecgdata variable
            //----------------------------RTOR-----------------------------------
            SPI_Master_ReadReg1(RTOR, TYPE_0_LENGTH);         // Read the data from ECG_FIFO register
            unsigned long RTOR_msb = (unsigned long) (SPI_temp_32b1[0]); // Received data from slave is stored in RTOR_msb variable
            unsigned long RTOR_lsb = (unsigned long) (SPI_temp_32b1[1]); // Received data from slave is stored in RTOR_lsb variable
            unsigned long rtor =(RTOR_msb<<8 | RTOR_lsb);     // Left shift data to Store only last 8-bit
            rtor = ((rtor >>2) & 0x3fff) ;                    // Shift the data and Store it to rtor variable
            float hr =  60 /((float)rtor*0.007789);           // 0.008 multiplier formula to calculate hr
            RR = (unsigned int)rtor*8;     // 8ms timing resolution of R-R interval
            HR = (unsigned int)hr;               // Type casting from unsigned integer to signed integer and store it into HR Buffer

            //------------------------------Averaging-------------------------------------------


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

            //if (Detection_Done)
             //          {
                           //Max & Min are all found. Calculate SpO2 & Pulse Rate
                           //SpO2_Calculation();             //calculate SpO2
                           //Pulse_Rate_Calculation();       //calculate pulse rate


                           if (first_reading==1)
                           {
                               //SpO2_previous = SpO2;
                               HR_previous = HR;
                               //Pulse_Rate_previous = Pulse_Rate;
                               RR_previous = RR;
                               first_reading = 2;
                           }
                           else if (first_reading==2)
                           {
                               //if (SpO2 > SpO2_previous)
                               if (HR > HR_previous)
                               {
                                   //SpO2 = SpO2_previous + 1;
                                   HR = HR_previous + 1;
                                   //SpO2_previous = SpO2;
                                   HR_previous = HR;
                               }
                               //else if (SpO2 < SpO2_previous)
                               else if (HR < HR_previous)
                               {
                                   //SpO2 = SpO2_previous - 1;
                                   HR = HR_previous - 1;
                                   //SpO2_previous = SpO2;
                                   HR_previous = HR;
                               }
                               //if (Pulse_Rate > Pulse_Rate_previous)
                               if (RR > RR_previous)
                               {
                                   //Pulse_Rate = Pulse_Rate_previous + 1;
                                   RR = RR_previous + 1;
                                   //Pulse_Rate_previous = Pulse_Rate;
                                   RR_previous = RR;
                               }
                               //else if (Pulse_Rate < Pulse_Rate_previous)
                               else if (RR < RR_previous)
                               {
                                   //Pulse_Rate = Pulse_Rate_previous - 1;
                                   RR = RR_previous - 1;
                                   //Pulse_Rate_previous = Pulse_Rate;
                                   RR_previous = RR;
                               }
                           }


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
uint8_t SPI_transfer(uint8_t data)
{

    unsigned char dummy_rx;
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
SPI_Mode SPI_Master_WriteReg(unsigned char *reg_addr, unsigned long reg_data, unsigned char count)
{
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = (*reg_addr<<1) | WREG;
    TXByteCtr = count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);    // CS Low

    SendUCA0Data(0x00);                                 // Send dummy 8-bit
    SendUCA0Data(0x00);                                 // send dummy 8-bit
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);   // CS High

    __delay_cycles(1000);                               // wait for

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);    // CS Low
    SendUCA0Data(TransmitRegAddr);                      // send 8-bit Reg address
    SendUCA0Data(reg_data>>16);                         // send 1st frame of 8-bit MSB data to write to the Reg
    SendUCA0Data(reg_data>>8);                          // send 2nd frame of 8-bit data to write to the Reg
    SendUCA0Data(reg_data);                             // send 3rd frame of 8-bit (LSB) data to write to the Reg
                                                        // Total 8-bit Address + 24-bit data = 32-bit
    __delay_cycles(500);                                // wait
    __bis_SR_register(GIE);                             // Global interrupt enable
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);   // CS High

    return MasterMode;
}

/**********************************************
 * Initialize SPI Read
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
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);    // CS Low

    SendUCA0Data(TransmitRegAddr);                      // send 8-bit Reg address
    for ( i = 0; i < 3; i++)                            // repeat the process for 3 times
    {
     SPI_temp_32b[i] = SPI_transfer(0xff);              // send "0xFF" to read from slave : UCA0RXBUF
    }                                                   // store the slave data into SPI_temp_32b[i]
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);   // CS High

    __bis_SR_register(GIE);                             // Global interrupt enable
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
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);    // CS Low

    SendUCA0Data(TransmitRegAddr);                      // send 8-bit Reg addres
    for ( i = 0; i < 3; i++)                            // repeat the process for 3 times
    {
     SPI_temp_32b1[i] = SPI_transfer(0xff);             // send "0xFF" to read from slave : UCA0RXBUF
    }
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);   // CS High

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
  GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN6 + GPIO_PIN7);   // P1.6-INT1, P1.7-INT2 : Lead off detection interrupt


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


      USCI_A_SPI_enableInterrupt(
              USCI_A0_BASE,
              USCI_A_SPI_RECEIVE_INTERRUPT
              );                                    // SPI Receive interrupt


      GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0); // Configure Chip select Start/end the communication

      GPIO_setOutputHighOnPin(
                    GPIO_PORT_P2,
                    GPIO_PIN0
                    );
}

void plot(int DATA)
{
    char res[15]={0};

    if(DATA == 'H')
    {
        F2A(HR,res,0);
        uart_string(res);
        uart_string("\n\r");
    }
    else
    {
        if(DATA == 'R')
        {
            F2A(RR,res,0);
            uart_string(res);
            uart_string("\n\r");
        }
        else
        {
            if(DATA == 'E')
            {
                F2A(SPI_temp_32b[1],res,0);
                uart_string(res);
                uart_string("\n\r");
            }
            else
            {
                for(i=0;i<19;i++)                                 // Send each buffer one by one
                    {
                        while (!(UCA1IFG & UCTXIFG));                 // Check the uart flags
                        UCA1TXBUF =DataPacketHeader[i];               // Send the buffer on uart
                    }
            }
        }
    }
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
        case 0: break;                                // Vector 0 - no interrupt

        case 2: uca0_rx_val = USCI_A_SPI_receiveData(USCI_A0_BASE);       // Vector 2 - RXIFG
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
                break;
          case 4:break;                               // Vector 4 - TXIFG
          default: break;
    }
}

/**********************************************
 * Initialize Uart
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



void uart_tx(char data)
{
   while (!(UCA1IFG&UCTXIFG));
   UCA1IFG &= ~UCTXIFG;
   UCA1TXBUF = data;
}



reverse(uint32_t data)
{
 unsigned int temp_data;

 while(data)
 {

     temp_data=data%10;
     uart_data=uart_data*10+temp_data;
     data=data/10;
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

//  float to ascii string
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
        fpart = fpart * pow(10, afterpoint);

        int2Str((int)fpart, res + i + 1, afterpoint);
    }

}

/**********************************************
 * Initialize TimerZero
 **********************************************/

void TimerZero()
{
                            // P1.0 output
    TA0CCTL0 = CCIE;                          // CCR0 interrupt enabled
    TA0CCR0 = 5500;
    TA0CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, upmode, clear TAR

}

/**********************************************
 * TimerZero ISR
 **********************************************/

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
#endif
{
    P2OUT ^= BIT5;                            // Toggle P1.0
}





