#include "driverlib.h"
#include <msp430.h>
#include "ECG.h"

/*****************************************
 * Main
 *****************************************/
void main()
{

    WDT_A_hold(WDT_A_BASE);               // Stop watchdog timer
    ECG_init();                           // Initialize ECG
    while(1){
    ECG_Start(); // Start Reading AFE
   plot('D');
//    plot('E');
//    plot('H');
//    plot('R');
    __bis_SR_register(GIE);
    __no_operation();                               // Do not perform any operation
    }
}
