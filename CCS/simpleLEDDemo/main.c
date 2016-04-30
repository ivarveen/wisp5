/**
 * @file       usr.c
 * @brief      WISP application-specific code set
 * @details    The WISP application developer's implementation goes here.
 *
 * @author     Aaron Parks, UW Sensor Systems Lab
 *
 */

#include "wisp-base.h"

WISP_dataStructInterface_t wispData;

#define CEIL_DIV(A, B)  ((((A) - 1) / (B)) + 1)

#define Blink_Timer_Period  (LP_LSDLY_1S)                    // 32kHz ticks (<= 2sec)
#define PWM_Timer_Period    (LP_LSDLY_5MS + LP_LSDLY_1MS)    // 32kHz ticks (<= 2sec)

void my_rn16Callback(void) {
    asm(" NOP");
}

/** 
 * This function is called by WISP FW after a successful ACK reply
 *
 */
volatile uint16_t nsuccess = 0;
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

void start_blinkClock(void) {
    //TA2CTL &= ~(TAIFG); // reset interrupt? might introduce timing related issues (i.e. calling this function while an interrupt is pending)
    TA2CCTL0 |= (CCIE); // enable interrupt
    TA2CCR0 += Blink_Timer_Period; // periodic interrupt

    __bis_SR_register(GIE); // global interrupt enable
}

void start_pwmClock(void) {
    //TA2CTL &= ~(TAIFG); // reset interrupt? might introduce timing related issues (i.e. calling this function while an interrupt is pending)
    TA2CCTL1 |= (CCIE); // enable interrupt
    TA2CCR1 += PWM_Timer_Period; // periodic interrupt

    __bis_SR_register(GIE); // global interrupt enable
}

void stop_blinkClock(void) {
    TA2CCTL0 &= ~(CCIE); // disable interrupt
}

void stop_pwmClock(void) {
    TA2CCTL1 &= ~(CCIE); // disable interrupt
}

void init_clocks(void) {
    // better safe than...
    stop_blinkClock();
    stop_pwmClock();

    // ACLK(=REFO), upmode, clear TAR
    TA2CTL = (TASSEL__ACLK | MC__CONTINUOUS | TACLR);
}

volatile uint8_t led_on = 0;

#pragma vector=TIMER2_A0_VECTOR // Interval
__interrupt
void INT_Timer2A0(void) {
    TA2CCR0 += Blink_Timer_Period;

    led_on ^= 0x1;
}

#pragma vector=TIMER2_A1_VECTOR // Timeout
__interrupt
void INT_Timer2A1(void) {
    switch (__even_in_range(TA2IV, TA2IV_TAIFG)) {
    case TA2IV_NONE:
        break;
    case TA2IV_TACCR1:
        // Wakeup Timeout Interrupt
        TA2CCR1 += PWM_Timer_Period;

        static int i = 0;

        if (led_on)
            if( i++>3 ) {
                BITSET(PLED1OUT, PIN_LED1);
                i=0;
            } else
                BITCLR(PLED1OUT, PIN_LED1);
        else {
            BITCLR(PLED1OUT, PIN_LED1);
        }

        break;
    case TA2IV_TAIFG:
        break;
    }
}

/**
 * This implements the user application and should never return
 *
 * Must call WISP_init() in the first line of main()
 * Must call WISP_doRFID() at some point to start interacting with a reader
 */
void main(void) {

    WISP_init();

    // Register callback functions with WISP comm routines
    WISP_registerCallback_RN16(&my_rn16Callback);
    WISP_registerCallback_ACK(&my_ackCallback);
    WISP_registerCallback_READ(&my_readCallback);
    WISP_registerCallback_WRITE(&my_writeCallback);
    WISP_registerCallback_BLOCKWRITE(&my_blockWriteCallback);

    // Initialize BlockWrite data buffer.
    uint16_t bwr_array[6] = { 0 };
    RWData.bwrBufPtr = bwr_array;

    // Get access to EPC, READ, and WRITE data buffers
    WISP_getDataBuffers(&wispData);

    // Set up operating parameters for WISP comm routines
    WISP_setMode( MODE_READ | MODE_WRITE); // | MODE_USES_SEL);
    WISP_setAbortConditions(0); //CMD_ID_READ | CMD_ID_WRITE /*| CMD_ID_ACK*/);

    // Set up EPC
    wispData.epcBuf[0] = 0x00; 		// Tag type
    wispData.epcBuf[1] = 0x11;		// Unused data field
    wispData.epcBuf[2] = 0x22;		// Unused data field
    wispData.epcBuf[3] = 0;			// Unused data field
    wispData.epcBuf[4] = 0;			// Unused data field
    wispData.epcBuf[5] = 0;			// Unused data field
    wispData.epcBuf[6] = 0;			// Unused data field
    wispData.epcBuf[7] = 0x00;		// Unused data field
    wispData.epcBuf[8] = 0x00;		// Unused data field
    wispData.epcBuf[9] = 0x51;		// Tag hardware revision (5.1)
    wispData.epcBuf[10] = *((uint8_t*) INFO_WISP_TAGID + 1); // WISP ID MSB: Pull from INFO seg
    wispData.epcBuf[11] = *((uint8_t*) INFO_WISP_TAGID); // WISP ID LSB: Pull from INFO seg

    UART_init();

    init_clocks();
    start_blinkClock();
    start_pwmClock();

    // Talk to the RFID reader.
    while (FOREVER) {
        TA2CCTL0 |= (CCIE); // enable interrupt
        TA2CCTL1 |= (CCIE); // enable interrupt
        __bis_SR_register(GIE); // global interrupt enable

        LPM4; // LPM3 ?
    }
}
