#include "driverlib.h"
#include <msp430.h>
#include "ECG.h"
/*****************************************
 * Main
 *****************************************/
void main()
{

    WDT_A_hold(WDT_A_BASE);               // Stop watchdog timer
//    TimerZero();
    ECG_init();                           // Initialize ECG
    while(1)
        {
        ECG_Start(); // Start Reading AFE
//      plot('D'); //Graph on Processing GUI
        plot('E'); // ECG wave on SerialPlotter
//        plot('H'); // HeartRate data on UART
//      plot('R'); // R-to-R data on UART
//         __delay_cycles(20000);
        }

}
