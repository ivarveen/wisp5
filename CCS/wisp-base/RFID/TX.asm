;/************************************************************************************************************************************/
;/**
; * @file		TX.asm
; * @brief		RFID Transmit
; * @details
; *
; * @author		Ivar in 't Veen, TU Delft Embedded Software Group
; * @created
; * @last rev	
; *
; *	@notes
; *	@todo
; *	@calling	extern void TX(volatile uint8_t *data,uint8_t numBytes,uint8_t numBits,uint8_t TRext)
; */
;/************************************************************************************************************************************/

;/INCLUDES----------------------------------------------------------------------------------------------------------------------------
    .cdecls C,LIST, "../globals.h"
    .cdecls C,LIST, "rfid.h"
    .global TxFM0, TxM2

;/SCRATCH REGISTERS-------------------------------------------------------------------------------------------------------------------
R_dataPtr   .set    R12             ; Entry: address of dataBuf start is in R_dataPtr
R_byteCt    .set    R13             ; Entry: length of Tx'd Bytes is in R_byteCt
R_bitCt     .set    R14             ; Entry: length of Tx'd Bits is in R_bitCt
R_TRext     .set    R15             ; Entry: TRext? is in R_TRext

    .def  TX

TX:
    ;CALLA #TxFM0
    CALLA #TxM2
    
    RETA

    .end ;* End of ASM */
