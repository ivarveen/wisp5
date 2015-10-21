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

#if 1
#undef PIN_LED2
#define PIN_LED2 0x00
#undef PIN_LED1
#define PIN_LED1 0x00
#endif

#define WINDOWSIZE (1) //(10)
volatile uint16_t ackwindow[WINDOWSIZE] = { 0 };
volatile uint16_t rnwindow[WINDOWSIZE] = { 0 };
volatile uint8_t window_index = 0;

volatile uint8_t use_wisp = 1;
volatile uint16_t sensor = 0;

inline void shiftWindow(void) {
    if ((window_index + 1) < WINDOWSIZE)
        window_index++;
    else
        window_index = 0;

    ackwindow[window_index] = 0;
    rnwindow[window_index] = 0;
}

#define CEIL_DIV(A, B)  ((((A) - 1) / (B)) + 1)

// Switch on/off functionality
#define UseRN16                     (1)
#define UseACK                      (1)
#define UseSENSOR                   (1)
#define UseUART                     (1)
#if !UseUART
#define UseGPIO                     (1)
#endif

// Transmission period is defined by Period and Multiplier, with Period <= 2sec
#define Transmission_Period         (LP_LSDLY_1S)       // 32kHz ticks (multiple of Transmission_Timer_Period)
#define Transmission_Timer_Period   (LP_LSDLY_1S)       // 32kHz ticks (<= 2sec)
#define Transmission_Multiplier     CEIL_DIV(Transmission_Period, Transmission_Timer_Period)

// Message size per transmission is defined is defined as usefull bytes per message
// and bytes per transmission, with bytes per message <=12
#define Bytes_per_Message           (12)                // #bytes/EPC (<= 12)
#define Bytes_per_Transmission      (120)               // #bytes/transmission
#define Messages_per_Transmission   CEIL_DIV(Bytes_per_Transmission, Bytes_per_Message)

#define Window_Timer_Period         (LP_LSDLY_1S)       // 32kHz ticks (<= 2sec)
#define Window_ForceWISP_Period     (5)                 // #transmissions

/**
 * This function is called by WISP FW after a successful RN16 transmission
 *
 */
#if UseRN16
void my_rn16Callback(void) {
    rnwindow[window_index]++;
}
#endif

/** 
 * This function is called by WISP FW after a successful ACK reply
 *
 */
#if UseACK
void my_ackCallback(void) {
    static uint16_t count = 0;

    if (++count >= Messages_per_Transmission) {
        rfid.abortFlag = TRUE;
        count = 0;
    }

    ackwindow[window_index]++;
}
#endif

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

    TA2CCR0 += Transmission_Timer_Period;

#if UseSENSOR
    sensor += 1; // next tick for improvised sensor
#endif

    // EVALUATE RFID STATUS
#if UseACK
    uint16_t acksum = 0;
#endif
#if UseRN16
    uint16_t rnsum = 0;
#endif
#if 0
    uint8_t i;
    for (i = WINDOWSIZE; i > 0; i--) {
#if UseACK
        acksum += ackwindow[i - 1];
        ackwindow[i - 1] = 0;
#endif
#if UseRN16
        rnsum += rnwindow[i - 1];
        rnwindow[i - 1] = 0;
#endif
    }
#endif

    acksum = ackwindow[0];
    ackwindow[0] = 0;
    rnsum = ackwindow[0];
    rnwindow[0] = 0;

    uint8_t wisp_quality = 0;

#if 0 && UseRN16 && UseACK
    if ((rnsum > 0) && (acksum >= rnsum)) {
        wisp_quality++;
    }
#endif

#if UseACK
    if (acksum >= Messages_per_Transmission) {
        //wisp_quality++;
        wisp_quality = 1;
    }
#endif

#if UseUART
    uint8_t m[] = "000";
#endif

    if (wisp_quality) {
        // disable BLE
#if UseUART
        m[0] = 'D';
#endif
        BITSET(PLED2OUT, PIN_LED2);

#if UseGPIO
        BITSET(P3OUT, PIN_AUX2);
        __delay_cycles(75 * 16);
        BITCLR(P3OUT, PIN_AUX2);
#endif
    } else {
        // enable BLE
#if UseUART
        m[0] = 'U';
#endif
        BITCLR(PLED2OUT, PIN_LED2);

#if UseGPIO
        BITSET(P3OUT, PIN_AUX1);
        __delay_cycles(75 * 16);
        BITCLR(P3OUT, PIN_AUX1);
#endif
    }

#if UseUART
    m[1] = (sensor >> 8) & 0xFF;
    m[2] = (sensor >> 0) & 0xFF;

    if (!wisp_quality) {
        BITSET(P3OUT, PIN_AUX1); // RTS

        UART_setClock();

        while (!(P3IN & PIN_AUX2))
            ; // CTS

        UART_critSend(m, sizeof(m));

        BITCLR(P3OUT, PIN_AUX1);
    }
#endif

    go = 1;

    LPM4_EXIT;
}

#if 0
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
#endif

/**
 * This implements the user application and should never return
 *
 * Must call WISP_init() in the first line of main()
 * Must call WISP_doRFID() at some point to start interacting with a reader
 */
void main(void) {
    WISP_init();

    // Register callback functions with WISP comm routines
#if UseRN16
    WISP_registerCallback_RN16(&my_rn16Callback);
#endif
#if UseACK
    WISP_registerCallback_ACK(&my_ackCallback);
#endif
    WISP_registerCallback_READ(&my_readCallback);
    WISP_registerCallback_WRITE(&my_writeCallback);
    WISP_registerCallback_BLOCKWRITE(&my_blockWriteCallback);

    // Get access to EPC, READ, and WRITE data buffers
    WISP_getDataBuffers(&wispData);

    // Set up operating parameters for WISP comm routines
    WISP_setMode(0); // MODE_READ | MODE_WRITE | MODE_USES_SEL);
    WISP_setAbortConditions(0); //CMD_ID_READ | CMD_ID_WRITE /*| CMD_ID_ACK*/);

    rfid.epcSize = (Bytes_per_Message >> 1);

    // Set up EPC
    uint8_t epcidx = 0;
    for (epcidx = (rfid.epcSize << 1); epcidx > 0; epcidx--)
        wispData.epcBuf[epcidx - 1] = ((epcidx - 1) & 0x0F) << 4
                | ((epcidx - 1) & 0x0F) << 0;  // Unused data field

#if UseUART
    UART_initCustom(8000000, 115200); //9600

    // DTR -- Data Ready
    BITSET(P3DIR, PIN_AUX1);// ENA -- output
    BITCLR(P3OUT, PIN_AUX1); //     -- low

    // CTS -- Clear To Send
    BITCLR(P3DIR, PIN_AUX2);// CTS -- input
    BITSET(P3REN, PIN_AUX2); //     -- pull...
    BITSET(P3OUT, PIN_AUX2); //     -- ... up
#endif

#if UseGPIO
    // Set
    BITSET(P3DIR, PIN_AUX1);// ENA -- output
    BITCLR(P3OUT, PIN_AUX1);//     -- low

    // Reset
    BITSET(P3DIR, PIN_AUX2);// ENA -- output
    BITCLR(P3OUT, PIN_AUX2);//     -- low
#endif

    init_clocks();
    start_intervalClock();

    // Talk to the RFID reader.
    while (FOREVER) {

        wispData.epcBuf[3] = (sensor >> 8) & 0xFF;
        wispData.epcBuf[4] = (sensor >> 0) & 0xFF;

        // WAIT FOR TIMER
        while (!go) {
            TA2CCTL0 |= (CCIE); // enable interrupt
            __bis_SR_register(GIE); // global interrupt enable

            CSCTL6 &= ~(MODCLKREQEN + SMCLKREQEN + MCLKREQEN);
            LPM4; // LPM3 ?
            CSCTL6 |= (MODCLKREQEN + SMCLKREQEN + MCLKREQEN);
        }
        go = 0;

        // enable WISP
        //start_timeoutClock();
        WISP_doRFID();
    }
}
