/**
 * @file       usr.c
 * @brief      WISP application-specific code set
 * @details    The WISP application developer's implementation goes here.
 *
 * @author     Aaron Parks, UW Sensor Systems Lab
 * @author     Ivar in 't Veen, TU Delft Embedded Software Group
 *
 */

#include "wisp-base.h"

WISP_dataStructInterface_t wispData;

/**
 * This function is called by WISP FW after a successful RN16 transmission
 *
 */
void my_rn16Callback(void) {
    asm(" NOP");
}

/** 
 * This function is called by WISP FW after a successful ACK reply
 *
 */
void my_ackCallback(void) {
    asm(" NOP");
}

/**
 * This function is called by WISP FW after a successful read command
 *  reception
 *
 */
void my_readCallback(void) {
    asm(" NOP");
}

/**
 * This function is called by WISP FW after a successful write command
 *  reception
 *
 */
void my_writeCallback(void) {
    asm(" NOP");
}

/** 
 * This function is called by WISP FW after a successful BlockWrite
 *  command decode
 *
 */
void my_blockWriteCallback(void) {
    asm(" NOP");
}

/**
 * This implements the user application and should never return
 *
 * Must call WISP_init() in the first line of main()
 * Must call WISP_doRFID() at some point to start interacting with a reader
 */
void main(void) {
    //WISP_init();

    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    __delay_cycles(75 * 16);
    __delay_cycles(75 * 16);
    __delay_cycles(75 * 16);
    __delay_cycles(75 * 16);
    __delay_cycles(75 * 16);

    while (1) {
        BITSET(P3DIR, PIN_AUX1); // ENA -- output
        BITSET(P3DIR, PIN_AUX2); // ENA -- output

        BITSET(P3OUT, PIN_AUX2);
        __delay_cycles(75 * 16);
        BITCLR(P3OUT, PIN_AUX2);

        BITSET(P3OUT, PIN_AUX1);
        __delay_cycles(75 * 16);
        BITCLR(P3OUT, PIN_AUX1);

        BITSET(P3OUT, PIN_AUX2);
        __delay_cycles(75 * 16);
        BITCLR(P3OUT, PIN_AUX2);

        BITCLR(P3DIR, PIN_AUX1); // CTS -- input
        BITSET(P3REN, PIN_AUX1); //     -- pull...
        BITCLR(P3OUT, PIN_AUX1); //     -- ...down

        BITCLR(P3DIR, PIN_AUX2); // CTS -- input
        BITSET(P3REN, PIN_AUX2); //     -- pull...
        BITCLR(P3OUT, PIN_AUX2); //     -- ...down

        setupDflt_IO()
        ;

        //CSCTL6 = 0x00;
        _bic_SR_register(GIE);

        P1SEL0 = 0x00;
        P1SEL1 = 0x00;
        P2SEL0 = 0x00;
        P2SEL1 = 0x00;
        P3SEL0 = 0x00;
        P3SEL1 = 0x00;
        P4SEL0 = 0x00;
        P4SEL1 = 0x00;
        PJSEL0 = 0x00;
        PJSEL1 = 0x00;

        PMMCTL0_H = PMMPW_H;
        PMMCTL0_L |= PMMREGOFF;
        PMMCTL0_L &= ~(SVSHE_L);
        PMMCTL0_H = 0x0000;

        LPM4; // LPM4.5
    }
}
