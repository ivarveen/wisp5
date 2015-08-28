;/************************************************************************************************************************************/
;/**
; * @file		TxMx.asm
; * @brief		RFID Transmit in Miller
; * @details
; *
; * @author		Ivar in 't Veen, TU Delft Embedded Software Group
; * @created
; * @last rev	
; *
; *	@notes
; *	@todo
; *	@calling	extern void TxM2(volatile uint8_t *data,uint8_t numBytes,uint8_t numBits,uint8_t TRext)
; */
;/************************************************************************************************************************************/

;/INCLUDES----------------------------------------------------------------------------------------------------------------------------
    .cdecls C,LIST, "../globals.h"
    .cdecls C,LIST, "rfid.h"
    .include "../internals/NOPdefs.asm"; Definitions of NOPx MACROs...
    .global TxClock, RxClock

;/PRESERVED REGISTERS-----------------------------------------------------------------------------------------------------------------
R_currByte	.set  R6 
R_prevState .set  R7
R_scratch0  .set  R8
R_scratch1  .set  R9
R_scratch2  .set  R10


;/SCRATCH REGISTERS-------------------------------------------------------------------------------------------------------------------
R_dataPtr   .set    R12             ; Entry: address of dataBuf start is in R_dataPtr
R_byteCt    .set    R13             ; Entry: length of Tx'd Bytes is in R_byteCt
R_bitCt     .set    R14             ; Entry: length of Tx'd Bits is in R_bitCt
R_TRext     .set    R15             ; Entry: TRext? is in R_TRext

;/Timing Notes------------------------------------------------------------------------------------------------------------------------
    ;*   Cycles Between Bits: 10 (for LF=640kHz @ 12MHz CPU)                                                                      */
    ;/** @todo Make sure the proper link frequency is listed here, or give a table of LF vs clock frequency							*/
    ;*   Cycles Before First Bit Toggle: 29 worst case                                                                              */

;/Begin ASM Code----------------------------------------------------------------------------------------------------------------------
    .def  TxM2

TxM2:
    BIS.W #BIT7, &PTXDIR;           ;[1]
    BIS.W #BIT7, &PTXOUT;           ;[1]
    BIC.W #BIT7, &PTXOUT;           ;[1]

    ;Push the Preserved Registers----------------------------------------------------------------------------------------------------
    PUSHM.A #5, R10                 ;[?] Push all preserved registers onto stack R6-R10/** @todo Find out how long this takes */

    CALLA #TxClock                  ;[] Switch to TxClock

M2_Send_Pilot_Tones:
    ;Prep Some Registers to Send (optimized scratch0/2 down below though)------------------------------------------------------------
    MOV.B   #0xFF, R_scratch1       ;[1] preloading our HIGH and LOW Tx bits to save us some cycles for pilot tones and preamble
    MOV.B   #4,    R_scratch2       ;[2] load up numTones=4

    ;Test to see if we should send pilot tones---------------------------------------------------------------------------------------
    TST.B   R_TRext                 ;[1] TRext means that we should send the pilot tones
    JZ      M2_Correct_Pilot_Tone   ;[2] skip 'em if (!TRext)

M2_Send_Extended_Pilot_Tone:
    ADD.B   #12,   R_scratch2       ;[?] Use 16 step pilot instead of 4

M2_Correct_Pilot_Tone:
    RLA.B   R_scratch2              ;[1] Miller 2
    ;RLA.B   R_scratch2             ;[1] Miller 4 -- only changing this line wont change to M4, because of unrolling a lot of
    ;RLA.B   R_scratch2             ;[1] Miller 8 --     other changes are also needed :-(

    ;/Send Pilot Tones if TRext-------------------------------------------------------------------------------------------------------
M2_Send_A_Pilot_Tone:
    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX
    ;Timing Optimization Shoved Here(5 free cycles)
    MOV.B   #0x00, R_scratch0       ;[1] setup R_scratch0 as LOW (note: if this is skipped, make sure to do it in preamble below too)
    NOPx4                           ;[4] 4 timing cycles
    ;End of 5 free cycles
    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX
    NOP                             ;[1] 1 timing cycles
    DEC     R_scratch2              ;[1] decrement the tone count
    TST.B   R_scratch2              ;[1] keep sending until the count is zero
    JNZ     M2_Send_A_Pilot_Tone    ;[2] ""
    
;/************************************************************************************************************************************
;/													SEND PREAMBLE (UNROLLED)                       							         *
;/                                                                                                                                   *
;/ tx sequence: 000...[0/1/0/1/1/1] or encoded as -> [HLHL/HLLH/LHLH/LHHL/HLLH/LHHL]                                                 *
;/************************************************************************************************************************************    
M2_Send_Preamble:
    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX       /*HL HL*/
    NOPx5                           ;[4] 4 timing cycles
    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles
    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles
    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles

    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX       /*HL LH*/
    NOPx5                           ;[4] 4 timing cycles
    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles
    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles
    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles

    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX       /*LH LH*/
    NOPx5                           ;[4] 4 timing cycles
    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles
    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles
    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles

    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX       /*LH HL*/
    NOPx5                           ;[4] 4 timing cycles
    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles
    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles
    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles

    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX       /*HL LH*/
    NOPx5                           ;[4] 4 timing cycles
    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles
    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles
    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles

    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX       /*LH HL*/
    NOPx5                           ;[4] 4 timing cycles
    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles
    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles
    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX
    NOP                             ;[1] 1 timing cycle

    MOV.B   #0x00, R_scratch0       ;[1] load up prevStateLogic to LOW (cause preamble left it that way)

;/************************************************************************************************************************************
;/													SEND A DATA BYTE (UNROLLED)                    							         *
;/************************************************************************************************************************************
M2_Load_Data:
    MOV.B   @R_dataPtr+, R_currByte ;[2] load current byte of data

M2_Send_a_Byte:
    ;/(b0) First Bit -----------------------------------------------------------------------------------------------------------------
    INV     R_scratch0              ;[1] first is HIGH
    MOV.B   R_scratch0, &PTXOUT     ;[4] pin 1
    NOPx4                           ;[4]

    INV     R_scratch0              ;[1] invert
    MOV.B   R_scratch0, &PTXOUT     ;[4] pin 2
    NOPx3                           ;[3]

    INV     R_scratch0              ;[1] invert
    XOR     R_currByte, R_scratch0	;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4] pin 3
    NOPx4                           ;[4]

    INV     R_scratch0              ;[1] invert
    MOV.B   R_scratch0, &PTXOUT     ;[4] pin 4
    RLA     R_currByte              ;[1] next bit
    NOPx3                           ;[3]

    ;/(b1) Second Bit ----------------------------------------------------------------------------------------------------------------
    XOR     R_currByte, R_scratch0  ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx4                           ;[4]

    INV     R_scratch0              ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx3                           ;[3]

    INV     R_scratch0              ;[1]
    XOR     R_currByte, R_scratch0  ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx4                           ;[4]

    INV     R_scratch0              ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    RLA     R_currByte              ;[1]
    NOPx3                           ;[3]

    ;/(b2) Third Bit -----------------------------------------------------------------------------------------------------------------
    XOR     R_currByte, R_scratch0  ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx4                           ;[4]

    INV     R_scratch0              ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx3                           ;[3]

    INV     R_scratch0              ;[1]
    XOR     R_currByte, R_scratch0  ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx4                           ;[4]

    INV     R_scratch0              ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    RLA     R_currByte              ;[1]
    NOPx3                           ;[3]

    ;/(b3) Fourth Bit ----------------------------------------------------------------------------------------------------------------
    XOR     R_currByte, R_scratch0  ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx4                           ;[4]

    INV     R_scratch0              ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx3                           ;[3]

    INV     R_scratch0              ;[1]
    XOR     R_currByte, R_scratch0  ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx4                           ;[4]

    INV     R_scratch0              ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    RLA     R_currByte              ;[1]
    NOPx3                           ;[3]

    ;/(b4) Fifth Bit -----------------------------------------------------------------------------------------------------------------
    XOR     R_currByte, R_scratch0  ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx4                           ;[4]

    INV     R_scratch0              ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx3                           ;[3]

    INV     R_scratch0              ;[1]
    XOR     R_currByte, R_scratch0  ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx4                           ;[4]

    INV     R_scratch0              ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    RLA     R_currByte              ;[1]
    NOPx3                           ;[3]

    ;/(b5) Sixth Bit -----------------------------------------------------------------------------------------------------------------
    XOR     R_currByte, R_scratch0  ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx4                           ;[4]

    INV     R_scratch0              ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx3                           ;[3]

    INV     R_scratch0              ;[1]
    XOR     R_currByte, R_scratch0  ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx4                           ;[4]

    INV     R_scratch0              ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    RLA     R_currByte              ;[1]
    NOPx3                           ;[3]

    ;/(b6) Seventh Bit ---------------------------------------------------------------------------------------------------------------
    XOR     R_currByte, R_scratch0  ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx4                           ;[4]

    INV     R_scratch0              ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx3                           ;[3]

    INV     R_scratch0              ;[1]
    XOR     R_currByte, R_scratch0  ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx4                           ;[4]

    INV     R_scratch0              ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    RLA     R_currByte              ;[1]
    NOPx3                           ;[3]

    ;/(b7) Eight Bit -----------------------------------------------------------------------------------------------------------------
    XOR     R_currByte, R_scratch0  ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx4

    INV     R_scratch0              ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx3                           ;[3]

    INV     R_scratch0              ;[1]
    XOR     R_currByte, R_scratch0	;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]
    NOPx4                           ;[4]

    INV     R_scratch0              ;[1]
    MOV.B   R_scratch0, &PTXOUT     ;[4]

    DEC     R_byteCt                ;[1] decrement the number of bytes sent
    TST.B   R_byteCt                ;[1] test if there are bytes left to send

    ; Done?
    JNZ     M2_Load_Data            ;[2] if (byteCt!=0) Continue Sending Bytes

    ;/Send the Last Bit (EoS Bit)-----------------------------------------------------------------------------------------------------
M2_Send_EoS_Byte:

    INV     R_scratch0              ;[1] invert
    MOV.B   R_scratch0, &PTXOUT     ;[4] pin 1
    NOPx4                           ;[4]

    INV     R_scratch0              ;[1] invert
    MOV.B   R_scratch0, &PTXOUT     ;[4] pin 2
    NOPx3                           ;[3]

    INV     R_scratch0              ;[1] invert
    INV     R_scratch0              ;[1] invert
    MOV.B   R_scratch0, &PTXOUT     ;[4] pin 3
    NOPx4                           ;[4]

    INV     R_scratch0              ;[1] invert
    MOV.B   R_scratch0, &PTXOUT     ;[4] pin 4
    NOPx4                           ;[4]

    POPM.A  #5, R10                 ;[?] Restore preserved registers R6-R10 /** @todo Find out how long this takes */

    BIC.B   #0x81, &PTXOUT          ;[] Clear 2.0 & 2.7 (1.0 is for old 4.1 HW, 2.7 is for current hack...) eventually just 1.0
    ;* End of 16 free cycles. Also note we only put these here to save 3 friggin cycles which prolly won't make a darn difference...*/
    RETA

    .end ;* End of ASM */
