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

;/Begin ASM defines-------------------------------------------------------------------------------------------------------------------

;Macro to send bits Miller-style
_send_bit    .macro data, miller, start

    .if (start > 0)
      MOV     R_scratch1, R_scratch2 ;[1]
    .else
      XOR     data, R_scratch2  ;[1]
    .endif
    MOV.B   R_scratch2, &PTXOUT     ;[4]

    .loop (miller - 1)
    NOPx4                           ;[4]
    INV     R_scratch2              ;[1]
    MOV.B   R_scratch2, &PTXOUT     ;[4]
    .endloop

    NOPx3                           ;[3]
    INV     R_scratch2              ;[1]
    XOR     data, R_scratch2  ;[1]
    MOV.B   R_scratch2, &PTXOUT     ;[4]

    .loop (miller - 1)
    NOPx4                           ;[4]
    INV     R_scratch2              ;[1]
    MOV.B   R_scratch2, &PTXOUT     ;[4]
    .endloop

    ;RLA     R_currByte              ;[1]
    ;NOPx3

            .endm

;Wrapper Macro to hide some constant parameters
send_bit     .macro data, start

    _send_bit data, MILLER_SIZE, start

             .endm

;/Begin Transmission Code------------------------------------------------------------------------------------------------------------
    .def  TxM2

TxM2:
    BIS.W   #BIT7, &PTXDIR;         ;[1]
    BIS.W   #BIT7, &PTXOUT;         ;[1]
    BIC.W   #BIT7, &PTXOUT;         ;[1]

    ;Push the Preserved Registers----------------------------------------------------------------------------------------------------
    PUSHM.A #5,    R10              ;[?] Push all preserved registers onto stack R6-R10/** @todo Find out how long this takes */

    CALLA   #TxClock                ;[] Switch to TxClock

;/************************************************************************************************************************************
;/												CHECK WHICH VERSION TO SEND                    							             *
;/ operation: in order to meet the timing constraint of 10 cycles, we need to eliminate some overhead in transmitting the data (see  *
;/              'why:' below). Thus we split the transmission into three possible modes V0/V1/V2. We establish which mode to use for *
;/              transmit here and then use it.                                                                                       *
;/                                                                                                                                   *
;/ versions:    V0: (bytes>0)  && (bits>0)                                                                                           *
;/              V1: (bytes>0)  && (bits==0)                                                                                          *
;/              V2: (bytes==0) && (bits>0)                                                                                           *
;/              V3: (bytes==0) && (bits==0)  <- D.C. cause no one should ever call this. it is coded to go to V1 in this case.       *
;                                                                                                                                   *
;/ why:         notice that the jump to 'load_byte:' after sending b7 takes JNZ[2],MOV[2],INV[1],MOV[4] == 9 cycles. Note here for b0*
;/              we also optimized out a MOV but left it commented for clarity. Anyways this is 9 cycles, 1 less than our 10 cycle    *
;/              criteria. It would be nice to hit 9 cycles though; then we could drop our CPU speed.                                 *
;/                                                                                                                                   *
;/ entry timing: worst case this code burns 37 cycles before transmitting the first pilot tone bit of V1. At 12.57MHz this is 2.9us..*
;/               I hope this won't be an issue later!                                                                                *
;/                                                                                                                                   *
;/ future optim: one note here is that the pilot tones never really get called in FM0_640, so if that's the case then the cycles are *
;/               32. also not that two PUSH cycles could be optimized, and perhaps the test logic could too. let's hope we never have*
;/               to go there!                                                                                                        *
;/  opt note:    (temp until fixed) I load up scratch0/1 for transmitting tones to save cycles. but i don't need to save those cycles*
;/               and in fact it costs me entry and exit cycles because it requires a pop/push of R_scratch2. FIXME!!!                *
;/************************************************************************************************************************************
;Test_For_Version0:
;    TST.B   R_byteCt                ;[1] Check for bytes
;    JZ      Test_For_Version2       ;[2] JMP if no bytes
;    TST.B   R_bitCt                 ;[1] Check for bytes
;    JZ      Test_For_Version2       ;[2] JMP if no bits
;    JMP     V0_Send_Pilot_Tones     ;[2] if we got here then (bytes>0) && (bits>0). JMP to V0.

;Test_For_Version2:
;    TST.B   R_byteCt                ;[1] Check for bytes
;    JNZ     Go_To_Version1          ;[2] JMP if bytes
;    TST.B   R_bitCt                 ;[1] Check For bits
;    JZ      Go_To_Version1          ;[2] JMP if no bits
;    JMP     tramp_V2_Send_Pilot_Tones ;[2] if we got here then (bytes==0) && (bits>0). JMP to V2

Go_To_Version1:                     ; Else just go to Version1
;*************************************************************************************************************************************
;                                                                                                                                    *
;  Version 1:       bytes>0 && bits==0         todo make NOPdefs.h NOPx4 something more optimized.                                   *
;                                                                                                                                    *
;  Optimizations:   For Each Byte sent, we need to do the following tasks:                                                           *
;                    - decrement byte count (R_byteCt)                                                                               *
;                    - test for more bytes                                                                                           *
;************************************************************************************************************************************/
V1_Send_Pilot_Tones:
    ;Prep Some Registers to Send (optimized scratch0/2 down below though)------------------------------------------------------------
    ;MOV.B   #0x00, R_scratch0       ;[1] preloading our HIGH and LOW Tx bits to save us some cycles for pilot tones and preamble
    MOV.B   #0xFF, R_scratch1       ;[1] preloading our HIGH and LOW Tx bits to save us some cycles for pilot tones and preamble
    MOV.B   #4,    R_scratch2       ;[2] load up numTones=4

    ;Test to see if we should send pilot tones---------------------------------------------------------------------------------------
    TST.B   R_TRext                 ;[1] TRext means that we should send the pilot tones
    JZ      V1_Correct_Pilot_Tone   ;[2] skip 'em if (!TRext)

V1_Send_Extended_Pilot_Tone:
    ADD.B   #12,   R_scratch2       ;[?] Use 16 step pilot instead of 4

V1_Correct_Pilot_Tone:
    RLA.B   R_scratch2              ;[1] Miller 2
    .if (MILLER_SIZE > 2)
        RLA.B   R_scratch2             ;[1] Miller 4 -- only changing this line wont change to M4, because of unrolling a lot of
    .endif
    .if (MILLER_SIZE > 4)
        RLA.B   R_scratch2             ;[1] Miller 8 --     other changes are also needed :-(
    .endif

    ;/Send Pilot Tones if TRext-------------------------------------------------------------------------------------------------------
V1_Send_A_Pilot_Tone:
    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX
    ;Timing Optimization Shoved Here(5 free cycles)
    MOV.B   #0x00, R_scratch0       ;[1] setup R_scratch0 as LOW (note: if this is skipped, make sure to do it in preamble below too)
    NOPx4                           ;[4] 4 timing cycles
    ;End of 5 free cycles
    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX
    NOP                             ;[1] 1 timing cycles
    DEC     R_scratch2              ;[1] decrement the tone count
    TST.B   R_scratch2              ;[1] keep sending until the count is zero
    JNZ     V1_Send_A_Pilot_Tone    ;[2] ""
    
;/************************************************************************************************************************************
;/													SEND PREAMBLE (UNROLLED)                       							         *
;/                                                                                                                                   *
;/ tx sequence: 000...[0/1/0/1/1/1] or encoded as -> [HLHL/HLLH/LHLH/LHHL/HLLH/LHHL]                                                 *
;/************************************************************************************************************************************    
V1_Send_Preamble:
    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX       /*HL HL*/
    NOPx5                           ;[4] 4 timing cycles
    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles
    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX
    NOPx5                           ;[5] 5 timing cycles
    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX

    NOPx4
    send_bit R_scratch1, 0

    NOPx4
    send_bit R_scratch0, 0

    NOPx5

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

    MOV.B   #0x00, R_scratch2       ;[1] load up prevStateLogic to LOW (cause preamble left it that way)

;    MOV     #0xFF, R_scratch2       ;[1]
;    send_bit R_scratch0             ;[14 + (18*(miller-1))]
;    NOPx4                           ;[4]
;
;    send_bit R_scratch1             ;[14 + (18*(miller-1))]
;    NOPx4                           ;[4]
;
;    send_bit R_scratch0             ;[14 + (18*(miller-1))]
;    NOPx4                           ;[4]
;
;    send_bit R_scratch1             ;[14 + (18*(miller-1))]
;    NOPx4                           ;[4]
;
;    send_bit R_scratch1             ;[14 + (18*(miller-1))]
;    NOPx4                           ;[4]
;
;    send_bit R_scratch1             ;[14 + (18*(miller-1))]
;    NOP                             ;[1] 1 timing cycle
;
;    ;MOV.B   #0x00, R_scratch2       ;[2] load up prevStateLogic to LOW (cause preamble left it that way)
;    NOPx2                           ;[2]

;/************************************************************************************************************************************
;/													SEND A DATA BYTE (UNROLLED)                    							         *
;/************************************************************************************************************************************
V1_Load_Data:
    MOV.B   @R_dataPtr+, R_currByte ;[2] load current byte of data

V1_Send_a_Byte:
    ;/(b0) First Bit -----------------------------------------------------------------------------------------------------------------
    send_bit R_currByte, 1
    RLA     R_currByte              ;[1] next bit
    NOPx3                           ;[3]

    .loop 6
    send_bit R_currByte, 0
    RLA      R_currByte
    NOPx3                           ;[3]
    .endloop

    ;/(b7) Eight Bit -----------------------------------------------------------------------------------------------------------------
    send_bit R_currByte, 0

    DEC     R_byteCt                ;[1] decrement the number of bytes sent
    TST.B   R_byteCt                ;[1] test if there are bytes left to send

    ; Done?
    JNZ     V1_Load_Data            ;[2] if (byteCt!=0) Continue Sending Bytes

    ;/Send the Last Bit (EoS Bit)-----------------------------------------------------------------------------------------------------
V1_Send_EoS_Byte:

    send_bit R_scratch1, 0          ;[14 + (18*(miller-1))]
    NOPx4                           ;[4]

    POPM.A  #5, R10                 ;[?] Restore preserved registers R6-R10 /** @todo Find out how long this takes */

    BIC.B   #0x81, &PTXOUT          ;[] Clear 2.0 & 2.7 (1.0 is for old 4.1 HW, 2.7 is for current hack...) eventually just 1.0
    ;* End of 16 free cycles. Also note we only put these here to save 3 friggin cycles which prolly won't make a darn difference...*/
    RETA

;; trampoline to avoid a very long jump to V2_Send_Pilot_Tones
;; XXX does this extra redirection take too long?
;tramp_V2_Send_Pilot_Tones:
;    JMP V2_Send_Pilot_Tones
;
;;*************************************************************************************************************************************
;;                                                                                                                                    *
;;  Version 0:       bytes>0 && bits>0                                                                                                *
;;                                                                                                                                    *
;;  Notes: This is the same as version 1 above, but it does a bits transmit loop too.                                                 *
;;                                                                                                                                    *
;;************************************************************************************************************************************/
;V0_Send_Pilot_Tones:
;    ;Prep Some Registers to Send (optimized scratch0/2 down below though)------------------------------------------------------------
;    MOV.B   #0xFF, R_scratch1       ;[1] preloading our HIGH and LOW Tx bits to save us some cycles for pilot tones and preamble
;    MOV.B   #4,    R_scratch2       ;[2] load up numTones=4
;
;    ;Test to see if we should send pilot tones---------------------------------------------------------------------------------------
;    TST.B   R_TRext                 ;[1] TRext means that we should send the pilot tones
;    JZ      V0_Correct_Pilot_Tone   ;[2] skip 'em if (!TRext)
;
;V0_Send_Extended_Pilot_Tone:
;    ADD.B   #12,   R_scratch2       ;[?] Use 16 step pilot instead of 4
;
;V0_Correct_Pilot_Tone:
;    RLA.B   R_scratch2              ;[1] Miller 2
;    ;RLA.B   R_scratch2             ;[1] Miller 4 -- only changing this line wont change to M4, because of unrolling a lot of
;    ;RLA.B   R_scratch2             ;[1] Miller 8 --     other changes are also needed :-(
;
;    ;/Send Pilot Tones if TRext-------------------------------------------------------------------------------------------------------
;V0_Send_A_Pilot_Tone:
;    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX
;    ;Timing Optimization Shoved Here(5 free cycles)
;    MOV.B   #0x00, R_scratch0       ;[1] setup R_scratch0 as LOW (note: if this is skipped, make sure to do it in preamble below too)
;    MOV.B   #0xFF, R_scratch1       ;[1] setup R_scratch1 as HIGH (note: if this is skipped, make sure to do it in preamble below too)
;    ;NOPx4                           ;[4] 4 timing cycles
;    NOPx3                           ;[3] 3 timing cycles
;    ;End of 5 free cycles
;    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX
;    NOP                             ;[1] 1 timing cycles
;    DEC     R_scratch2              ;[1] decrement the tone count
;    TST.B   R_scratch2              ;[1] keep sending until the count is zero
;    JNZ     V0_Send_A_Pilot_Tone    ;[2] ""
;
;;/************************************************************************************************************************************
;;/													SEND PREAMBLE (UNROLLED)                       							         *
;;/                                                                                                                                   *
;;/ tx sequence: 000...[0/1/0/1/1/1] or encoded as -> [HLHL/HLLH/LHLH/LHHL/HLLH/LHHL]                                                 *
;;/************************************************************************************************************************************
;V0_Send_Preamble:
;    send_bit R_scratch0             ;[14 + (18*(miller-1))]
;    NOPx4                           ;[4]
;
;    send_bit R_scratch1             ;[14 + (18*(miller-1))]
;    NOPx4                           ;[4]
;
;    send_bit R_scratch0             ;[14 + (18*(miller-1))]
;    NOPx4                           ;[4]
;
;    send_bit R_scratch1             ;[14 + (18*(miller-1))]
;    NOPx4                           ;[4]
;
;    send_bit R_scratch1             ;[14 + (18*(miller-1))]
;    NOPx4                           ;[4]
;
;    send_bit R_scratch1             ;[14 + (18*(miller-1))]
;    NOP                             ;[1] 1 timing cycle
;
;    MOV.B   #0x00, R_scratch2       ;[2] load up prevStateLogic to LOW (cause preamble left it that way)
;
;;/************************************************************************************************************************************
;;/													SEND A DATA BYTE (UNROLLED)                    							         *
;;/************************************************************************************************************************************
;V0_Load_Data:
;    MOV.B   @R_dataPtr+, R_currByte ;[2] load current byte of data
;
;V0_Send_a_Byte:
;    send_byte R_currByte            ;[148 + (144*(miller-1))]
;
;    DEC     R_byteCt                ;[1] decrement the number of bytes sent
;    TST.B   R_byteCt                ;[1] test if there are bytes left to send
;
;    ; Done?
;    JNZ     V0_Load_Data            ;[2] if (byteCt!=0) Continue Sending Bytes
;
;;/************************************************************************************************************************************
;;/													SEND LAST BITS                     							                     *
;;/ operation: same procedure as looping on bytes. number of bits to Tx is in R_bitCt.                                                *
;;/************************************************************************************************************************************
;V0_Send_Last_Bits:
;    MOV.B   @R_dataPtr+, R_currByte ;[2] load current byte of data
;
;V0_Send_A_Bit:
;    send_bit R_currByte             ;[14 + (18*(miller-1))]
;
;    DEC     R_bitCt                 ;[1]
;    TST.B   R_bitCt                 ;[1]
;
;    ; Done?
;    JNZ     V0_Send_A_Bit           ;[2]
;
;    ;/Send the Last Bit (EoS Bit)-----------------------------------------------------------------------------------------------------
;V0_Send_EoS_Byte:
;
;    send_bit R_scratch1             ;[14 + (18*(miller-1))]
;    NOPx4                           ;[4]
;
;    POPM.A  #5, R10                 ;[?] Restore preserved registers R6-R10 /** @todo Find out how long this takes */
;
;    BIC.B   #0x81, &PTXOUT          ;[] Clear 2.0 & 2.7 (1.0 is for old 4.1 HW, 2.7 is for current hack...) eventually just 1.0
;    ;* End of 16 free cycles. Also note we only put these here to save 3 friggin cycles which prolly won't make a darn difference...*/
;    RETA
;
;;*************************************************************************************************************************************
;;                                                                                                                                    *
;;  Version 2:       bytes==0 && bits>0                                                                                               *
;;                                                                                                                                    *
;;  Notes: This is the same as version 0 above, but it omits the byte loop                                                            *
;;                                                                                                                                    *
;;************************************************************************************************************************************/
;V2_Send_Pilot_Tones:
;    ;Prep Some Registers to Send (optimized scratch0/2 down below though)------------------------------------------------------------
;    MOV.B   #0xFF, R_scratch1       ;[1] preloading our HIGH and LOW Tx bits to save us some cycles for pilot tones and preamble
;    MOV.B   #4,    R_scratch2       ;[2] load up numTones=4
;
;    ;Test to see if we should send pilot tones---------------------------------------------------------------------------------------
;    TST.B   R_TRext                 ;[1] TRext means that we should send the pilot tones
;    JZ      V2_Correct_Pilot_Tone   ;[2] skip 'em if (!TRext)
;
;V2_Send_Extended_Pilot_Tone:
;    ADD.B   #12,   R_scratch2       ;[?] Use 16 step pilot instead of 4
;
;V2_Correct_Pilot_Tone:
;    RLA.B   R_scratch2              ;[1] Miller 2
;    ;RLA.B   R_scratch2             ;[1] Miller 4 -- only changing this line wont change to M4, because of unrolling a lot of
;    ;RLA.B   R_scratch2             ;[1] Miller 8 --     other changes are also needed :-(
;
;    ;/Send Pilot Tones if TRext-------------------------------------------------------------------------------------------------------
;V2_Send_A_Pilot_Tone:
;    MOV.B   R_scratch1, &PTXOUT     ;[4] HIGH on PTXOUT.PIN_TX
;    ;Timing Optimization Shoved Here(5 free cycles)
;    MOV.B   #0x00, R_scratch0       ;[1] setup R_scratch0 as LOW (note: if this is skipped, make sure to do it in preamble below too)
;    MOV.B   #0xFF, R_scratch1       ;[1] setup R_scratch1 as HIGH (note: if this is skipped, make sure to do it in preamble below too)
;    ;NOPx4                           ;[4] 4 timing cycles
;    NOPx3                           ;[3] 3 timing cycles
;    ;End of 5 free cycles
;    MOV.B   R_scratch0, &PTXOUT     ;[4] LOW on PTXOUT.PIN_TX
;    NOP                             ;[1] 1 timing cycles
;    DEC     R_scratch2              ;[1] decrement the tone count
;    TST.B   R_scratch2              ;[1] keep sending until the count is zero
;    JNZ     V2_Send_A_Pilot_Tone    ;[2] ""
;
;;/************************************************************************************************************************************
;;/													SEND PREAMBLE (UNROLLED)                       							         *
;;/                                                                                                                                   *
;;/ tx sequence: 000...[0/1/0/1/1/1] or encoded as -> [HLHL/HLLH/LHLH/LHHL/HLLH/LHHL]                                                 *
;;/************************************************************************************************************************************
;V2_Send_Preamble:
;    send_bit R_scratch0             ;[14 + (18*(miller-1))]
;    NOPx4                           ;[4]
;
;    send_bit R_scratch1             ;[14 + (18*(miller-1))]
;    NOPx4                           ;[4]
;
;    send_bit R_scratch0             ;[14 + (18*(miller-1))]
;    NOPx4                           ;[4]
;
;    send_bit R_scratch1             ;[14 + (18*(miller-1))]
;    NOPx4                           ;[4]
;
;    send_bit R_scratch1             ;[14 + (18*(miller-1))]
;    NOPx4                           ;[4]
;
;    send_bit R_scratch1             ;[14 + (18*(miller-1))]
;    NOP                             ;[1] 1 timing cycle
;
;    MOV.B   #0x00, R_scratch2       ;[2] load up prevStateLogic to LOW (cause preamble left it that way)
;
;;/************************************************************************************************************************************
;;/													SEND LAST BITS                     							                     *
;;/ operation: same procedure as looping on bytes. number of bits to Tx is in R_bitCt.                                                *
;;/************************************************************************************************************************************
;V2_Send_Last_Bits:
;    MOV.B   @R_dataPtr+, R_currByte ;[2] load current byte of data
;
;V2_Send_A_Bit:
;    send_bit R_currByte             ;[14 + (18*(miller-1))]
;
;    DEC     R_bitCt                 ;[1]
;    TST.B   R_bitCt                 ;[1]
;
;    ; Done?
;    JNZ     V2_Send_A_Bit           ;[2]
;
;    ;/Send the Last Bit (EoS Bit)-----------------------------------------------------------------------------------------------------
;V2_Send_EoS_Byte:
;
;    send_bit R_scratch1             ;[14 + (18*(miller-1))]
;    NOPx4                           ;[4]
;
;    POPM.A  #5, R10                 ;[?] Restore preserved registers R6-R10 /** @todo Find out how long this takes */
;
;    BIC.B   #0x81, &PTXOUT          ;[] Clear 2.0 & 2.7 (1.0 is for old 4.1 HW, 2.7 is for current hack...) eventually just 1.0
;    ;* End of 16 free cycles. Also note we only put these here to save 3 friggin cycles which prolly won't make a darn difference...*/
;    RETA

    .end ;* End of ASM */

