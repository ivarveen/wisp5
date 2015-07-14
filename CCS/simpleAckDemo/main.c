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

#define WINDOWSIZE (10)
volatile uint16_t ackwindow[WINDOWSIZE] = { 0 };
volatile uint16_t rnwindow[WINDOWSIZE] = { 0 };
volatile uint8_t window_index = 0;

void shiftWindow(void) {
    if ((window_index + 1) < WINDOWSIZE)
        window_index++;
    else
        window_index = 0;

    ackwindow[window_index] = 0;
    rnwindow[window_index] = 0;
}

#define CEIL_DIV(A, B)  ((((A) - 1) / (B)) + 1)

// Transmission period is defined by Period and Multiplier, with Period <= 2sec
#define Transmission_Period         (1*LP_LSDLY_1S) // 32kHz ticks
#define Transmission_Timer_Period   (LP_LSDLY_1S)   // 32kHz ticks (<= 2sec)
#define Transmission_Multiplier     CEIL_DIV(Transmission_Period, Transmission_Timer_Period)

// Message size per transmission is defined is defined as usefull bytes per message
// and bytes per transmission, with bytes per message <=12
#define Bytes_per_Message           (12)        // #bytes/EPC (<= 12)
#define Bytes_per_Transmission      (120)       // #bytes/transmission
#define Messages_per_Transmission   CEIL_DIV(Bytes_per_Transmission, Bytes_per_Message)

#define Window_Timer_Period         (LP_LSDLY_100MS)    // 32kHz ticks (<= 2sec)

void my_rn16Callback(void) {
    rnwindow[window_index]++;
}

/** 
 * This function is called by WISP FW after a successful ACK reply
 *
 */
volatile uint16_t nsuccess = 0;
void my_ackCallback(void) {
    static uint16_t count = 0;

    if (++count >= Messages_per_Transmission) {
        rfid.abortFlag = TRUE;
        count = 0;
        nsuccess++;
    }

    ackwindow[window_index]++;
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

 */
void my_blockWriteCallback(void) {
    asm(" NOP");
}

void start_intervalClock(void) {
    //TA2CTL &= ~(TAIFG); // reset interrupt? might introduce timing related issues (i.e. calling this function while an interrupt is pending)
    TA2CCTL0 |= (CCIE); // enable interrupt
    TA2CCR0 += Transmission_Timer_Period; // periodic interrupt

    __bis_SR_register(GIE); // global interrupt enable
}

void start_timeoutClock(void) {
    //TA2CTL &= ~(TAIFG); // reset interrupt? might introduce timing related issues (i.e. calling this function while an interrupt is pending)
    TA2CCTL1 |= (CCIE); // enable interrupt
    TA2CCR1 += Window_Timer_Period; // periodic interrupt

    __bis_SR_register(GIE); // global interrupt enable
}

void stop_intervalClock(void) {
    TA2CCTL0 &= ~(CCIE); // disable interrupt
}

void stop_timeoutClock(void) {
    TA2CCTL1 &= ~(CCIE); // disable interrupt
}

void init_clocks(void) {
    // better safe than...
    stop_intervalClock();
    stop_timeoutClock();

    // ACLK(=REFO), upmode, clear TAR
    TA2CTL = (TASSEL__ACLK | MC__CONTINUOUS | TACLR);
}

volatile uint16_t go = 0; // use counter instead of on/off to detect overrun/overflow

#pragma vector=TIMER2_A0_VECTOR // Interval
__interrupt
void INT_Timer2A0(void) {
    static uint16_t multiplier_count = 0;
    static uint8_t breakpoint_countdown = 0;

    TA2CCR0 += Transmission_Timer_Period;

    if (++multiplier_count >= Transmission_Multiplier) {
        multiplier_count = 0;

        // EVALUATE RFID STATUS
        uint16_t acksum = 0;
        uint16_t rnsum = 0;
        uint8_t i;
        for (i = WINDOWSIZE; i > 0; i--) {
            acksum += ackwindow[i - 1];
            rnsum += rnwindow[i - 1];
        }

        /* REMOVE */
        if (breakpoint_countdown++ >= 10) {
            asm(" NOP");
            breakpoint_countdown = 0;
        }
        /* /REMOVE */

        if (acksum*3 > rnsum)
            BITSET(PLED1OUT, PIN_LED1);
        else
            BITCLR(PLED1OUT, PIN_LED1);

        if (acksum > (10*WINDOWSIZE))
            BITSET(PLED2OUT, PIN_LED2);
        else
            BITCLR(PLED2OUT, PIN_LED2);

        if (go < ((uint16_t) -1))
            go++;

        LPM4_EXIT;
    }
}

#pragma vector=TIMER2_A1_VECTOR // Timeout
__interrupt
void INT_Timer2A1(void) {
    switch (__even_in_range(TA2IV, TA2IV_TAIFG)) {
    case TA2IV_NONE:
        break;
    case TA2IV_TACCR1:
        // Wakeup Timeout Interrupt
        TA2CCR1 += Window_Timer_Period;

        shiftWindow();

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
    WISP_setMode( MODE_READ | MODE_WRITE | MODE_USES_SEL);
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

    init_clocks();
    start_intervalClock();
    stop_timeoutClock();

    // Talk to the RFID reader.
    while (FOREVER) {
        // TRANSMIT RFID
        start_timeoutClock();
        WISP_doRFID();
        stop_timeoutClock();

        // WAIT FOR TIMER
        while (!go) {
            TA2CCTL0 |= (CCIE); // enable interrupt
            __bis_SR_register(GIE); // global interrupt enable

            LPM4; // LPM3 ?
        }
        go = 0;
    }
}
