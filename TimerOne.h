int timer(void)
{
    P2DIR |= BIT5;                            // P1.0 output
    TA0CCTL0 = CCIE;                          // CCR0 interrupt enabled
    TA0CCR0 = 5500;
    TA0CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, upmode, clear TAR
    return(0);
}

// Timer0 A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
#endif
{
    P2OUT &= ~BIT5;                            // Toggle P1.0
    P1OUT &= ~BIT0;
}
