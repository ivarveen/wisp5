/**
 * @file       usr.c
 * @brief      WISP application-specific code set
 * @details    The WISP application developer's implementation goes here.
 *
 * @author     Aaron Parks, UW Sensor Systems Lab
 * @author     Ivar in 't Veen, TUD Embedded Software Group
 */

#include "wisp-base.h"

WISP_dataStructInterface_t wispData;

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
	int8_t i;
	for (i = 3; i >= 0; i--)
		wispData.readBufPtr[i] = (uint8_t) FRAM_read_infoA_char(i * 8);

	//wispData.readBufPtr[0] = (uint8_t) FRAM_read_infoA_char(0 * 8);
	//wispData.readBufPtr[1] = (uint8_t) FRAM_read_infoA_char(1 * 8);
	//wispData.readBufPtr[2] = (uint8_t) FRAM_read_infoA_char(2 * 8);
	//wispData.readBufPtr[3] = (uint8_t) FRAM_read_infoA_char(3 * 8);
}

/**
 * This function is called by WISP FW after a successful write command
 *  reception
 *
 */
void my_writeCallback(void) {
	int8_t i;
	for (i = 3; i >= 0; i--) {
		/*
		 * Using XOR to create desired pattern.
		 *
		 * i	i^0x2	i^0x1
		 * 3	1		1
		 * 2	1		0
		 * 1	0		1
		 * 0	0		0
		 */
		uint8_t write_data = wispData.writeBufPtr[(i ^ 0x2)] >> (i ^ 0x1)
				& 0xFF;

		FRAM_write_infoA_char(i * 8, 1, &write_data);
	}

	//uint8_t write_data1 = wispData.writeBufPtr[0] >> 8;
	//uint8_t write_data2 = wispData.writeBufPtr[0] & 0xFF;
	//uint8_t write_data3 = wispData.writeBufPtr[1] >> 8;
	//uint8_t write_data4 = wispData.writeBufPtr[1] & 0xFF;

	//FRAM_write_infoA_char(0 * 8, 1, &write_data1);
	//FRAM_write_infoA_char(1 * 8, 1, &write_data2);
	//FRAM_write_infoA_char(2 * 8, 1, &write_data3);
	//FRAM_write_infoA_char(3 * 8, 1, &write_data4);
}

/** 
 * This function is called by WISP FW after a successful BlockWrite
 *  command decode

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

	WISP_init();

	// Register callback functions with WISP comm routines
	WISP_registerCallback_ACK(&my_ackCallback);
	WISP_registerCallback_READ(&my_readCallback);
	WISP_registerCallback_WRITE(&my_writeCallback);
	WISP_registerCallback_BLOCKWRITE(&my_blockWriteCallback);

	// Get access to EPC, READ, and WRITE data buffers
	WISP_getDataBuffers(&wispData);

	// Set up operating parameters for WISP comm routines
	WISP_setMode( MODE_READ | MODE_WRITE | MODE_USES_SEL);
	WISP_setAbortConditions(CMD_ID_READ | CMD_ID_WRITE /*| CMD_ID_ACK*/);

	// Enable FRAM
	FRAM_init();

	// Enable LED
	//BITSET(PLED1OUT, PIN_LED1);

	// Set up EPC
	wispData.epcBuf[0] = 0x05; // WISP version
	wispData.epcBuf[1] = *((uint8_t*) INFO_WISP_TAGID + 1); // WISP ID MSB
	wispData.epcBuf[2] = *((uint8_t*) INFO_WISP_TAGID); // WISP ID LSB
	wispData.epcBuf[3] = 0x33;
	wispData.epcBuf[4] = 0x44;
	wispData.epcBuf[5] = 0x55;
	wispData.epcBuf[6] = 0x66;
	wispData.epcBuf[7] = 0x77;
	wispData.epcBuf[8] = 0x88;
	wispData.epcBuf[9] = 0x99;
	wispData.epcBuf[10] = 0xAA;
	wispData.epcBuf[11] = 0xBB;

	// Talk to the RFID reader.
	while (FOREVER) {
		WISP_doRFID();

		// Switch LED based on FRAM value.
		if (FRAM_read_infoA_char(0) == 0xFF)
			// ENABLE if 0th char has value 0xFF
			BITSET(PLED1OUT, PIN_LED1);
		else
			// DISABLE otherwise
			BITCLR(PLED1OUT, PIN_LED1);
	}
}
