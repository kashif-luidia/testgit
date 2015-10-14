/*----------------------------------------------------------------------
	Module:  colldata.c
	Collect one pen sample worth of ultrasound after getting a valid IR.
	
	This module exposes two main functions - BeginDataCollection and 
	WaitForNewDataAvailable.  Data collection takes place in the background
	using the timers, DMA, and interrupts.  It begins with the code
	looking at infrared.  Once the IR signal has been found, ultrasound
	data is collected.  WaitForNewDataAvailable won't return until the 
	ultrasound data is ready.
		
	Copyright (C) 2003 Luidia, Inc
----------------------------------------------------------------------*/
#include "sysdef.h"
#include "dspdef.h"
#include "structs.h"
#include "colldata.h"
#include "asmfunc.h"
#include "error.h"
#include "gain_control.h"
#include "usdef.h"
#include "profile.h"
#include "simple_filters.h"
#include "irpll.h"


extern void InitDMAISR();                         

void DMA3IsrAsm();
#pragma C54X_CALL(DMA3IsrAsm)
//void DMA4IsrAsm(int, int);
void DMA4IsrAsm(int);
#pragma C54X_CALL(DMA4IsrAsm)
                        
extern int irThreshold;		
extern int firstIrState;	
extern int numSegmentsCollected;
extern int saturatedCh1, saturatedCh2;
extern int avoidIRRebound;				// Control flag for IR state machine
extern int wd_cntr;

int * current_ir_state;
int countSinceFirstIrDetected;
int secondPulseWidth;
int firstPulseWidth;
int firstPulseMaxAmp;				// Maximum amplitude of first pulse
int secPulseMaxAmp;
int lastIrHigh;						// Flag for last IR above threshold
int secPulseRisingEdge;
int lastIR;
int dmaOn;
extern volatile int disableDataCollection;

extern	Int16	irPrjPenFlg;

/*  Status of projection pen  */
int projectionPenStatus;
int previousProjectionPenStatus;
int usDmaLastFrame = 0;

/*  Threshold adaptation */
UInt32  irHistory;		// RMS history in Q23.8 fixed point format
UInt16	irRms;			// Fractional part of irHistory; redundant, but really good for debug
UInt16 	irAverage;

extern	Int16	irState;

void configADC_IR()
{
	int cr0, cr1;  //  Control registers
	

	/*  CR0
		|9 8|7 6|5|4|3 2|1 0|
		|0 0|1 1|1|0|0 0|0 1|
		  |   |  | |  |   |
	  	  |   |  | |  |   Channel Select: Ch2
		  |   |  | |  Conversion Mode Select: Mono interrupt
		  |   |  | Input Type: Single end
		  |   |  Conversion Clock Select: External
		  |   Offset Calibration: Operate without calibration
		  Register address: CR0
	 */
	cr0 = 0x0e1;

	/*  CR1
		|9 8|7|6|5 4|3|2|1|0|
		|0 1|0|1|0 0|0|1|0|0|
		  |  | |  |  | | | |
		  |  | |  |  | | | SW Power Down: Normal
		  |  | |  |  | | Interrupt_Mode Auto Power Down: Disabled.  Not allowed for our sampling rate
		  |  | |  |  | Output Format: binary
		  |  | |  |  0, Reserved
		  |  | |  Resolution Select: 10-Bit
		  |  | Interrupt-Mode Conversion Started by: CSTART
		  |  0, Reserved
		  Register address: CR1
	*/
	cr1 = 0x144;		
	far_poke(ADC_ADDRESS, cr0);
	far_poke(ADC_ADDRESS, cr1);
}

void configADC_US()
{
	int cr0, cr1;  //  Control registers
	
	/*  CR0
		|9 8|7 6|5|4|3 2|1 0|
		|0 0|1 1|1|0|0 1|0 0|
		  |   |  | |  |   |
	  	  |   |  | |  |   Channel Select: pair A
		  |   |  | |  Conversion Mode Select: Dual interrupt
		  Same as IR	
	 */
	cr0 = 0x0e4;
	
	cr1 = 0x144;  //  Same as IR		

	far_poke(ADC_ADDRESS, cr0);
	far_poke(ADC_ADDRESS, cr1);
}

void configADC_PowerDown()
{
	far_poke(ADC_ADDRESS, 0);
	far_poke(ADC_ADDRESS, 1);
}

int ReadADC()
{
	return far_peek(ADC_ADDRESS);
}

#ifndef COPY2
void DMA4Config_IR()
{

	//  Channel 4 Used for IR
	//  DMA Channel Control Register
	/* 		| 15 14   | 13 12  | 11    | 10 | 9  | 8      | 7 | 6  | 5 | 4  0 |
	   		|DSTAMODE |SRCAMODE|ENDPROG| R  |REPT|AUTOINIT|En |PRIO|FS | SYNC |
	   CR4  |   11    |  00    |  1    | 0  | 1  | 1      | 0 | 1  | 0 | 10000|
		        |        |        |      |    |    |        |   |    |     |
	     	    |        |        |      |    |    |        |   |    |     Sync with XINT1
		        |        |        |      |    |    |        |   |    One element on each sync event
	   	  	    |        |        |      |    |    |        |   High priority on DMA port
	 	       	|        |        |      |    |    |        Disable
	       		|        |        |      |    |    auto-restart
	       		|        |        |      |    always repeat regardless of end of programming bit
	       		|        |        |    
	       		|        |        End of programming bit, be 1, since dma is disabled.
           		|        Constant address
           		Double indexing
	*/
	*(ioport unsigned int *)DMACCR4 = 0xCB50;//sync with int1, double indexing

	/*  DMA Interrupt Control Register
			| 15     9|8|7     |6| 5     | 4   | 3     | 2    | 1    | 0       |
            | Reserved|R|AERRIE|R|BLOCKIE|LSTIE|FRAMEIE|HALFIE|DROPIE|TIMEOUTIE|
	  ICR4  |0000 0000|0|  0   |0| 0     | 0   | 1     | 1    | 0    | 0       |
			Interrupts sent at half and full frame points	    
	*/
	*(ioport unsigned int *)DMACICR4 = 0x001C;  // half, full frame,  and last frame flags
	//  Checked CSR in testing code.  Never see error bits.  So, turn off error bits.

	/*  DMA Source and Data Parameters
			| 15 14 | 13    | 12 9 | 8 7  | 6     | 5 2  | 1 0    |
			|DSTBEN |DSTPACK|DST   |SRCBEN|SRCPACK| SRC  |DATATYPE|
	  SDP4  | 0   0 |  0    | 0001 | 0 0  | 0     | 0010 | 0 1    |
			  |        |      |      |      |       |      |
			  |        |      |      |      |       |      16-bit
			  |        |      |      |      |       External memory
			  |        |      |      |      No packing
			  |        |      |      Source no burst
			  |        |      DARAM
			  |        No packing
			  Destination burst disabled
	*/	
	*(ioport unsigned int *)DMACSDP4 = 0x0209;	
	
	//  Set source to address of ADC
	*(ioport unsigned int *)DMACSSAL4 = (ADC_ADDRESS*2 & 0xffff);  // source address in bytes, lower
	*(ioport unsigned int *)DMACSSAU4 = ((ADC_ADDRESS*2 >> 16) & 0xffff);  // source address in bytes, upper

	//  Set destination to dataArray2Seg
	*(ioport unsigned int *)DMACDSAL4 = ((unsigned int)(dataArray2Seg)+IR_BUFFER_LEN)*2;  // destination address in bytes, lower
	*(ioport unsigned int *)DMACDSAU4 = 0x0000;	 // destination address in bytes, upper

	//  Configure the shifts so that the data are repeatedly filled into the same buffer
	*(ioport 		  int *)DMACDEI4 = 1;  //  destination address shift for the next element
	*(ioport          int *)DMACDFI4 = -(2*IR_BUFFER_LEN - 1); //  destination address shift for the next frame

	//  DMA Element Number Register
	*(ioport unsigned int *)DMACEN4 = IR_BUFFER_LEN;    // # elements per frame

	//  DMA Frame Number Register
	*(ioport unsigned int *)DMACFN4 = 0xFFFF;  // # of frames, maximum possible
}

/*-----------------------------------------------------------------------------
	Read IR Data into a specified buffer
	void ReadIRDataIntoBuffer(unsigned int *pBuffer, unsigned int numPointsToRead)
		unsigned int *pBuffer - points to buffer where data should be loaded
		unsigned int numPointsToRead - length of buffer in 16-bit words
		
	This function is self-explanatory.  However the "features" in the DMA confuse 
	the data a bit.  The first DMA_QUEUE_LENGTH elements are corrupted, as are the
	last DMA_QUEUE_LENGTH elements.  That should be taken into account when using 
	this data.

-----------------------------------------------------------------------------*/
void ReadIRDataIntoBuffer(unsigned int *pBuffer, unsigned int numPointsToRead)
{
	volatile int counter = 0;
	configADC_IR();			//  Set up to read from IR channel

	//  Channel 4 Used for IR
	//  DMA Channel Control Register
	/* 		| 15 14   | 13 12  | 11    | 10 | 9  | 8      | 7 | 6  | 5 | 4  0 |
	   		|DSTAMODE |SRCAMODE|ENDPROG| WP |REPT|AUTOINIT|En |PRIO|FS | SYNC |
	   CR4  |   01    |  00    |  1    | 0  | 0  | 0      | 0 | 1  | 0 | 10000|
		        |        |        |      |    |    |        |   |    |     |
	     	    |        |        |      |    |    |        |   |    |     Sync with XINT1
		        |        |        |      |    |    |        |   |    One element on each sync event
	   	  	    |        |        |      |    |    |        |   High priority on DMA port
	 	       	|        |        |      |    |    |        Disable
	       		|        |        |      |    |    No repeating
	       		|        |        |      |    No repeating
	       		|        |        |      Write posting is off
	       		|        |        End of programming bit ignored since DMA disabled
           		|        Source address is constant
           		Destination address is incremented after each transfer
	*/
	*(ioport unsigned int *)DMACCR4 = 0x4850;

	/*  DMA Interrupt Control Register
			| 15     9|8|7     |6| 5     | 4   | 3     | 2    | 1    | 0       |
            | Reserved|R|AERRIE|R|BLOCKIE|LSTIE|FRAMEIE|HALFIE|DROPIE|TIMEOUTIE|
	  ICR4  |0000 0000|0|  0   |0| 0     | 0   | 0     | 0    | 0    | 0       |
			Interrupts sent at half and full frame points	    
	*/
	*(ioport unsigned int *)DMACICR4 = 0x0000;  // No interrupts

	/*  DMA Source and Data Parameters
			| 15 14 | 13    | 12 9 | 8 7  | 6     | 5 2  | 1 0    |
			|DSTBEN |DSTPACK|DST   |SRCBEN|SRCPACK| SRC  |DATATYPE|
	  SDP4  | 0   0 |  0    | 0001 | 0 0  | 0     | 0010 | 0 1    |
			  |        |      |      |      |       |      |
			  |        |      |      |      |       |      16-bit
			  |        |      |      |      |       External memory
			  |        |      |      |      No packing
			  |        |      |      Source no burst
			  |        |      DARAM
			  |        No packing
			  Destination burst disabled
	*/	
	*(ioport unsigned int *)DMACSDP4 = 0x0209;	
	
	//  Set source to address of ADC
	*(ioport unsigned int *)DMACSSAL4 = (ADC_ADDRESS*2 & 0xffff);  // source address in bytes, lower
	*(ioport unsigned int *)DMACSSAU4 = ((ADC_ADDRESS*2 >> 16) & 0xffff);  // source address in bytes, upper

	//  Set destination to dataArray2Seg
	*(ioport unsigned int *)DMACDSAL4 = 2*(unsigned int)(pBuffer);  // destination address in bytes, lower
	*(ioport unsigned int *)DMACDSAU4 = 0x0000;	 // destination address in bytes, upper

	//  DMA Element Number Register
	*(ioport unsigned int *)DMACEN4 = numPointsToRead;    // # elements per frame

	//  DMA Frame Number Register
	*(ioport unsigned int *)DMACFN4 = 0x1;  //  One frame

	//  Start data collection
	*(ioport unsigned int *)DMACCR4 |= 0x0080;  //  Enable DMA4 to start collecting data

	//  Wait for data collection to complete
	//  Due to a collection of "features" in the DMA, the last 16 points are lost
	while(*(ioport unsigned int *)DMACDAC4 < 2*(unsigned int)(pBuffer + numPointsToRead - DMA_QUEUE_LENGTH))
		counter++;
	//  Shut down the DMA manually
	*(ioport unsigned int *)DMACCR4 = 0x0000;
	//  Read once from the ADC to force a sync event to help the DMA reset itself
	ReadADC();
	
}

void DMA5Config_Timer()
{
	//  Channel 5 used as a timer - dump some points between IR and US
	//  DMA Channel Control Register
	/* 		| 15 14   | 13 12  | 11    | 10 | 9  | 8      | 7 | 6  | 5 | 4  0 |
	   		|DSTAMODE |SRCAMODE|ENDPROG| R  |REPT|AUTOINIT|En |PRIO|FS | SYNC |
	   CR5  |   00    |  00    |  1    | 0  | 0  | 0      | 0 | 1  | 1 | 01110|
		        |        |        |           |    |        |   |    |     |
	     	    |        |        |           |    |        |   |    |     Sync with TIM1
		        |        |        |           |    |        |   |    One frame on each sync event
	   	  	    |        |        |           |    |        |   High priority on DMA port
	 	       	|        |        |           |    |        Disable
	       		|        |        |           Run once
	       		|        |        |    
	       		|        |        End of programming bit, be 1, since dma is disabled.
           		|        Constant address
           		Constant Address  */
	*(ioport unsigned int *)DMACCR5 = 0x084E;
	
	/*  DMA Interrupt Control Register
			| 15        6 | 5     | 4   | 3     | 2    | 1    | 0       |
            | Reserved    |BLOCKIE|LSTIE|FRAMEIE|HALFIE|DROPIE|TIMEOUTIE|
	  ICR5  |0000 0000 00 | 0     | 0   | 1     | 0    | 0    | 0       |
			Only interrupt when complete	*/
	*(ioport unsigned int *)DMACICR5 = 0x0020;

	/*  DMA Source and Data Parameters
			| 15 14 | 13    | 12 9 | 8 7  | 6     | 5 2  | 1 0    |
			|DSTBEN |DSTPACK|DST   |SRCBEN|SRCPACK| SRC  |DATATYPE|
	  SDP5  | 0   0 |  0    | 0001 | 0 0  | 0     | 0000 | 0 1    |
			  |        |      |      |      |       |      |
			  |        |      |      |      |       |      16-bit
			  |        |      |      |      |       Internal memory
			  |        |      |      |      No packing
			  |        |      |      Source no burst
			  |        |      DARAM
			  |        No packing
			  Destination burst disabled
	*/	
	*(ioport unsigned int *)DMACSDP5 = 0x0201;
	
	
	//  Set source to address of ADC
//	*(ioport unsigned int *)DMACSSAL5 = (ADC_ADDRESS*2 & 0xffff);  // source address in bytes, lower
//	*(ioport unsigned int *)DMACSSAU5 = ((ADC_ADDRESS*2 >> 16) & 0xffff);  // source address in bytes, upper
	//  Use internal source instead, to avoid sync errors.  Any address is fine
	*(ioport unsigned int *)DMACSSAL5 = 0x2000;  // source address in bytes, lower
	*(ioport unsigned int *)DMACSSAU5 = 0;  // source address in bytes, upper
	//  Set destination to dataArray2Seg
	*(ioport unsigned int *)DMACDSAL5 = (unsigned int)dataArray2Seg*2;  // destination address in bytes, lower
	*(ioport unsigned int *)DMACDSAU5 = 0x0000;	 // destination address in bytes, upper
	//  DMA Element Number Register
	*(ioport unsigned int *)DMACEN5 = 1;  // # elements per frame, temporary, the proper time will be set in IR isr.
	//  DMA Frame Number Register
	*(ioport unsigned int *)DMACFN5 = 1;  // # of frames

}

void DMA2Config_US1()
{
			
	/*  ***DMA 3:  Ultrasound DMA:  Collects two channels into two circular buffers***  */
	/* 		| 15 14   | 13 12  | 11    | 10 | 9  | 8      | 7 | 6  | 5 | 4  0 |
	   		|DSTAMODE |SRCAMODE|ENDPROG| R  |REPT|AUTOINIT|En |PRIO|FS | SYNC |
	   CR3  |   11    |  00    |  1    | 0  | 0  | 0      | 0 | 1  | 0 | 10000|
		        |        |        |      |    |    |        |   |    |     |
	     	    |        |        |      |    |    |        |   |    |     Sync with XINT1
		        |        |        |      |    |    |        |   |    One element on each sync event
	   	  	    |        |        |      |    |    |        |   High priority on DMA port
	 	       	|        |        |      |    |    |        Disable
	       		|        |        |      |    |    no auto-restart
	       		|        |        |      |    for auto-restart
	       		|        |        |    
	       		|        |        Don't wait to update value
           		|        Constant source address
           		Auto double increment dest address - used to implement ping-pong buffer  */
	*(ioport unsigned int *)DMACCR2 = 0xC850;

	/*		| 15        6 | 5     | 4   | 3     | 2    | 1    | 0       |
            | Reserved    |BLOCKIE|LSTIE|FRAMEIE|HALFIE|DROPIE|TIMEOUTIE|
	  ICR3  |0000 0000 00 | 1     | 1   | 0     | 0    | 0    | 0       |
			Interrupts sent at only full block points  */
	*(ioport unsigned int *)DMACICR2 = 0x0;

	/*  DMA Source and Data Parameters
			| 15 14 | 13    | 12 9 | 8 7  | 6     | 5 2  | 1 0    |
			|DSTBEN |DSTPACK|DST   |SRCBEN|SRCPACK| SRC  |DATATYPE|
	  SDP3  | 0   0 |  0    | 0001 | 0 0  | 0     | 0010 | 0 1    |
			  |        |      |      |      |       |      |
			  |        |      |      |      |       |      16-bit
			  |        |      |      |      |       External memory
			  |        |      |      |      No packing
			  |        |      |      Source no burst
			  |        |      DARAM
			  |        No packing
			  Destination burst disabled
	*/	
	*(ioport unsigned int *)DMACSDP2 = 0x0209;
	
	//  Set source to address of ADC
	*(ioport unsigned int *)DMACSSAL2 = (ADC_ADDRESS*2 & 0xffff);  // source address in bytes, lower
	*(ioport unsigned int *)DMACSSAU2 = ((ADC_ADDRESS*2 >> 16) & 0xffff);  // source address in bytes, upper

	//  Set destination to dataArray2Seg
	*(ioport unsigned int *)DMACDSAL2 = 2*(unsigned int)(us1dma);  // destination address in bytes, lower
//	*(ioport unsigned int *)DMACDSAL2 = 2*(unsigned int)(dataArray1Seg);  // destination address in bytes, lower
	*(ioport unsigned int *)DMACDSAU2 = 0x0000;	 // destination address in bytes, upper

	//  Set up counts
	*(ioport unsigned int *)DMACEN2 = 2*SEGMENT_SIZE;    // # elements per frame
	*(ioport unsigned int *)DMACFN2 = TOTAL_SEGMENTS_TO_READ/2;  // # of frames
	
	//  Configure the shifts so that the interleaved data from the two channels are read into two parallel buffers.
	*(ioport 		  int *)DMACDEI2 = 1;  //  destination address shift for the next element
	*(ioport          int *)DMACDFI2 = -(4*SEGMENT_SIZE - 1); //  destination address shift for the next frame

}

/*  The only differences between DMA2 and DMA3 are:
	1.  Priority;
	2.  Destination address;
	3.  DMA2 doesn't generate interrupt.
*/
void DMA3Config_US2()
{
	/*  ***DMA 3:  Ultrasound DMA:  Collects two channels into two circular buffers***  */
	/* 		| 15 14   | 13 12  | 11    | 10 | 9  | 8      | 7 | 6  | 5 | 4  0 |
	   		|DSTAMODE |SRCAMODE|ENDPROG| R  |REPT|AUTOINIT|En |PRIO|FS | SYNC |
	   CR3  |   11    |  00    |  1    | 0  | 0  | 0      | 0 | 0  | 0 | 10000|
		        |        |        |      |    |    |        |   |    |     |
	     	    |        |        |      |    |    |        |   |    |     Sync with XINT1
		        |        |        |      |    |    |        |   |    One element on each sync event
	   	  	    |        |        |      |    |    |        |   Low priority on DMA port
	 	       	|        |        |      |    |    |        Disable
	       		|        |        |      |    |    no auto-restart
	       		|        |        |      |    for auto-restart
	       		|        |        |    
	       		|        |        Don't wait to update registers
           		|        Constant source address
           		Auto double increment dest address - used to implement ping-pong buffer  */
	*(ioport unsigned int *)DMACCR3 = 0xC810;

	/*		| 15        6 | 5     | 4   | 3     | 2    | 1    | 0       |
            | Reserved    |BLOCKIE|LSTIE|FRAMEIE|HALFIE|DROPIE|TIMEOUTIE|
	  ICR3  |0000 0000 00 | 1     | 1   | 0     | 0    | 0    | 0       |
			Interrupts sent at only full block points  */
	*(ioport unsigned int *)DMACICR3 = 0x003C;

	/*  DMA Source and Data Parameters
			| 15 14 | 13    | 12 9 | 8 7  | 6     | 5 2  | 1 0    |
			|DSTBEN |DSTPACK|DST   |SRCBEN|SRCPACK| SRC  |DATATYPE|
	  SDP3  | 0   0 |  0    | 0001 | 0 0  | 0     | 0010 | 0 1    |
			  |        |      |      |      |       |      |
			  |        |      |      |      |       |      16-bit
			  |        |      |      |      |       External memory
			  |        |      |      |      No packing
			  |        |      |      Source no burst
			  |        |      DARAM
			  |        No packing
			  Destination burst disabled
	*/	
	*(ioport unsigned int *)DMACSDP3 = 0x0209;
	
	//  Set source to address of ADC
	*(ioport unsigned int *)DMACSSAL3 = (ADC_ADDRESS*2 & 0xffff);  // source address in bytes, lower
	*(ioport unsigned int *)DMACSSAU3 = ((ADC_ADDRESS*2 >> 16) & 0xffff);  // source address in bytes, upper
	//  Set destination to dataArray2Seg
	*(ioport unsigned int *)DMACDSAL3 = 2*(unsigned int)(us2dma);  // destination address in bytes, lower
//	*(ioport unsigned int *)DMACDSAL3 = 2*(unsigned int)(dataArray2Seg);  // destination address in bytes, lower
	*(ioport unsigned int *)DMACDSAU3 = 0x0000;	 // destination address in bytes, upper

	//  Set up counts
	*(ioport unsigned int *)DMACEN3 = 2*SEGMENT_SIZE;    // # elements per frame
	*(ioport unsigned int *)DMACFN3 = TOTAL_SEGMENTS_TO_READ/2;  // # of frames
	
	//  Configure the shifts so that the interleaved data from the two channels are read into two parallel buffers.
	*(ioport 		  int *)DMACDEI3 = 1;  //  destination address shift for the next element
	*(ioport          int *)DMACDFI3 = -(4*SEGMENT_SIZE - 1); //  destination address shift for the next frame

}

/* void ConfigDMAforDataCollection()
    Init DMA channel 4 to collect Inferrad data into a circular buffer in dataArraySeg2.  
    The DMA will interrupt 	the CPU when the buffer is half-full and completely full.
    The ISR will examine the IR data and detect valid pen signals.  
    The buffer length is set at 128, and the interrupt occurs every 64 samples.
	Init DMA channel 5 as a timer.  After a valid pen signal is detected, DMA5 is used to wait 
	till IR_TO_US_SPACING time passes.  DMA5Isr starts ultrasound data collection in DMA2 and DMA3.
*/
void ConfigDMAforDataCollection()
{
//	*(ioport unsigned int *)DMAGCR = 0x0004;  // DMA not stopped when emulation breakpoint is encountered.
	DMA4Config_IR();
	DMA5Config_Timer();
	DMA2Config_US1();
	DMA3Config_US2();
#ifdef NEW_US_FILTER
	memset( usFilterDelayBuff1, 0, sizeof( usFilterDelayBuff1 ) );
	memset( usFilterDelayBuff2, 0, sizeof( usFilterDelayBuff2 ) );
#endif
	avoidIRRebound = 0;		// init IR state machine variable -- should be done local to state machine
}


void InitDataCollectionVariables()
{
	current_ir_state = &firstIrState;
	lastIrHigh = 0;
	previousProjectionPenStatus = projectionPenStatus;
	projectionPenStatus = 0;
	usDmaLastFrame = 0;

	/* Init the IRPLL state for data collection.  */
	IRPLL_OnDataCollection();

	/*  Initialize ISR to handle DMA interrupts  */
	InitDMAISR();
}

/* int BeginDataCollection()
    This function starts DMA channel 4 to collect IR data.  The ISR for dma4 will
	detect valid IR pulse and start dma to collect ultrasound data.  Once it is done,
	the global flag UsCollected will be set.
*/
extern Int16 irPls12Dis, irPls13Dis;
Int16 cycleCount = 0;
BOOL dropCycle = FALSE;
void BeginDataCollection() {


	if(disableDataCollection)
		return;
		
	dmaOn = 1;

	InitDataCollectionVariables();

	configADC_IR();

	INT_DISABLE;
	*(ioport unsigned int *)DMACCR4 |= 0x0080;  //  Enable DMA4 to start collecting data
	ENA_INT_DMA4;		//  Enable interrupt for IR
	ENA_INT_DMA5;		//  Enable interrupt for IR-to-US handoff timer	
	INT_ENABLE;

	irPls12Dis = 100;
	irPls13Dis = 100;

}

/*  ERROR WaitForNewDataAvailable()
	After BeginDataCollection has initialized the interrupt-based data collection, 
	this function can be called to wait for the data collection to finish and thus 
	for new data to be available.
	Return value:  NOERR or error encountered during data collection.
*/
ERROR WaitForNewDataAvailable()
{
	/*  Wait for data acquisition to complete  */
	wd_cntr = 0;
	while((numSegmentsCollected < TOTAL_SEGMENTS_TO_READ) && dmaOn && !disableDataCollection)
	{

		if (usDmaLastFrame) {
			//  At the end of DMA (block flag) the ISR will clear this usDmaLastFrame flag.
			//  Due to prefetch problem, after it reads (total-16) counts, DMA thinks it reads 
			//  enough to stop, but doesn't feel it reads enough to raise "end of block" flag.
			//  We need to use CPU to do extra reads which clear ADC interupts, and let ADC to
			//  generate more sync events to help DMA count.
			if (*(ioport unsigned int *)DMACDAC3 > ((unsigned int)us2dma+(2*SEGMENT_SIZE)-DMA_QUEUE_LENGTH)*2) 
				ReadADC();
		}
		if (wd_cntr >= 2) {
			wd_cntr = 0;
///			return (ERROR_DMA_TIMEOUT);
		}
	}

	if ((numSegmentsCollected < TOTAL_SEGMENTS_TO_READ)) 
	{
		usDmaLastFrame = 0;  //  If dma is stopped, reset this flag.
		
		if(!disableDataCollection)
			return(ERROR_DMA_OFF);		
			
		while(disableDataCollection);	
			return (ERROR_CMD_PREEMPTED_DATA_COLLECTION);
	}
	
	return(NOERR);
}


void TerminateDataCollection() 
{
	int i;
	TimerStop(GPTIMER1_BASE);	//  Stop ADC, so there is no more DMA sync event. 
	//  Read from the ADC several times.  If the DMA is accessing the ADC when it 
	//  is stopped, the EMIF can get messed up and the system will crash.  It is 
	//  possible that, worst cast, both channels 2 and 3 are currently filling 
	//  their queues.  If that is happening, one access should be in progress.  The
	//  first ReadADC will wait until the EMIF is free, then when it completes the
	//  second access should start.  The loop waits for the worst case where two
	//  queues must both be filled.  I've also added a couple for luck.
	for(i=0; i < 2*DMA_QUEUE_LENGTH+2; ++i)
		ReadADC();  
	//  At this point, no DMA should have any interest in accessing the EMIF since
	//  no sync events have come in for a while and the EMIF has been verified not
	//  to be busy enough times for all sync-independent queue filling to be done.
	//  Now stop all data collection DMA channels
	*(ioport unsigned int *)DMACCR2 &= 0xff7f;	//  stop DMA2.
	*(ioport unsigned int *)DMACCR3 &= 0xff7f;	//  stop DMA3.
	*(ioport unsigned int *)DMACCR4 &= 0xff7f;	//  stop DMA4.
	*(ioport unsigned int *)DMACCR5 &= 0xff7f;	//  stop DMA5.
	Timers0and1SyncStart();		//  Keep clock on.
	dmaOn = 0;
	disableDataCollection = 1;
	/*  Interrupts should be disabled.  It's possible, however, that an interrupt
	was generated by the DMA before DMA was disabled.  Clear that interrupt  */
	*(volatile unsigned short *)IFR0 = (DMA4_INT_BIT | DMA5_INT_BIT);
	*(volatile unsigned short *)IFR1 = (DMA3_INT_BIT);
}

//  Interrupt for IR - just call assembly function after using C keyword for 
//  Saving and restoring registers
int dynamicPeak = IR_PEAK_FACTOR;			// multiplier that gives peak amplitude from RMS
int dynamicFracBits = IR_RMS_FRAC_BITS;		// time constant for the RMS calculation
#pragma DATA_ALIGN( segmentTime, 4 );
UInt32 segmentTime;
interrupt void DMA4ISR_C()
{
	int csr; 
	int *src, *dst;
	BOOL detectionEnabled;

	wd_cntr = 0;
	IRPLL_GetTime( &segmentTime );

	//  DMACSR is automatically cleared when read.  So, we read it once 
	//	into a local variable.  
	csr = *(ioport unsigned int *)DMACSR4;

	//GPIO21_SET;
	PROFILE_SUMMED_INTERVAL_BEGIN( PROFILE_DMA4ISR );

	if (csr & 0x0010) { //  Last frame.
		//  DMA4 should on all the time to scan for possible IR signals.  When it completes
		//  one transfer (supposedly at receiving block flag), it should be restarted.  
		//  Due to C5502 DMA prefectching problem, the block flag can never come.  
		//  Use last-frame flag instead.  Upon receiving the last-frame flag, just stops 
		//  the DMA and restart it.
		//  During this transition period, it is blind.  The last 16 points in the prefetch 
		//  buffer for the previous DMA transfer are lost.  The first 16 points output of the new 
		//  DMA transfer are invalid.  But, there is only a slight chance that the IR signal 
		//  arrives in this blind period, and it only happens to IR of the first pendown.
		//  Therefore it is an acceptable solution.
		//  The chance of an IR edge is in this 16 point blind period is 16/(num_frames*frame_size)
		//  = 2^4/(2^16*2^6) = 2^(-18).
		//  The max duration of IR (projection pen) is 2^7.  So the chance of missing it is 2^(-11).

		//  Going to stop and restart DMA4.  It is trickly because stopping DMA4 at
		//  wrong time may freeze DMA.  Stop its sync event first, then do a CPU access 
		//  to EMIF to make sure DMA finishes its operation on EMIF.  The 
		TimerStop(GPTIMER1_BASE);	//  Stop ADC, so there is no more DMA sync event. 
		ReadADC();  //  CPU access to EMIF.  It waits here until DMA finishes on EMIF.
		*(ioport unsigned int *)DMACCR4 &= 0xff7f;	//  stop DMA4.
		Timers0and1SyncStart();		//  restart ADC
		*(ioport unsigned int *)DMACCR4 |= 0x0080;  //  re-enable DMA4.
		goto DMA4ISR_Exit;		
	}

	// Make sure this is the right kind of interrupt
	if (!(csr&0x000C)) {
		goto DMA4ISR_Exit;
	}

	IRPLL_OnSegment( &segmentTime, &detectionEnabled );
	if (detectionEnabled) {
		// Run IR detection processing
//		GPIO21_SET;
		// Move the IR segment to the processing buffer from the DMA buffer
		if (csr & 0x0004) {
			// bottom half just transferred by DMA
			src = 	dataArray2Seg+IR_BUFFER_LEN; 
			dst =	dataArray2Seg;
		} else {
			// top half just transferred by DMA
			src = 	dataArray2Seg+IR_BUFFER_LEN*3/2; 
			dst =	dataArray2Seg+IR_BUFFER_LEN/2;
		}
		memcpy( dst, src, IR_BUFFER_LEN/2);

	{
		extern Int16 pktOk;
		extern UInt32 irNextPls1Time, irPls1Time;
		pktOk = 0;

		// Run the IR detection state machine
		DMA4IsrAsm(csr);

		if(pktOk && (irState != IR_TRACKING)){	// to start US collection for old marker pen
			irNextPls1Time = irPls1Time;
			irPrjPenFlg = 0;
			IRPLL_TriggerUS();
		}
	}


		if (!(IRPLL_irPredetect || IRPLL_irDetect)) {
#if 0
			// Adjust IR detection threshold
			irRms = Rms(dst, IR_BUFFER_LEN/2, irAverage, dynamicFracBits, &irHistory);
			irThreshold = ((dynamicPeak*irRms)>>IR_PEAK_FRAC_BITS) + irAverage + IR_PEAK_OFFSET;
			if (irThreshold < (NOMINAL_IR_THRESHOLD + irAverage)) {
				irThreshold = NOMINAL_IR_THRESHOLD + irAverage;
			} 
			if(irThreshold > 900){
				irThreshold = 800;
			}
#endif
			irThreshold = 500;	//fix the threshold level
		} 
//		GPIO21_RESET;
	}

DMA4ISR_Exit:
	PROFILE_SUMMED_INTERVAL_END( PROFILE_DMA4ISR );
	//GPIO21_RESET;
}

//  Interrupt for IR-to-US handoff timer.
unsigned int timerValue;
unsigned int saveTimerValue;
interrupt void DMA5ISR_C()
{
	int interruptStatus = *(ioport unsigned int *)DMACSR5;

	//PROFILE_SUMMED_INTERVAL_BEGIN( PROFILE_DMA5ISR );

	//  Read DMA status to clear the bits.  Necessary for the future
	//  interrupts to occur.
	if (!(interruptStatus & 0x0020))  // if not the wanted bit, quit.
		goto DMA5ISR_Exit;
	
	DIS_INT_DMA4;
	DIS_INT_DMA5;
	*(ioport unsigned int *)DMACCR5 &= 0xFF7F;  //  Disable DMA5

	//  Wait for something.
	saveTimerValue = *(ioport unsigned int *)(GPTIMER1_BASE+GPTCNT1);
	do
	{
		timerValue = *(ioport unsigned int *)(GPTIMER1_BASE+GPTCNT1);
	}while (timerValue > 30);

	*(ioport unsigned int *)DMACCR2 |= 0x0080;  //  Enable DMA2
	*(ioport unsigned int *)DMACCR3 |= 0x0080;	//  Enable DMA3

	ENA_INT_DMA3;

DMA5ISR_Exit:
	;
	//PROFILE_SUMMED_INTERVAL_END( PROFILE_DMA5ISR );
}

//  C gateway function used to save and restore context
// 	Revised Version with gain control before noise filter.  RH 15-Feb-2007
interrupt void DMA3ISR_C()
{
	volatile int interruptStatus;
	static int maxUS1=0, maxUS2=0;

	//GPIO23_SET;
	wd_cntr = 0;
	PROFILE_SUMMED_INTERVAL_BEGIN( PROFILE_DMA3ISR );
	interruptStatus = *(ioport unsigned int *)DMACSR3;
	if (interruptStatus & 0x0004) { // half frame
		// set up pointers to source buffers
		af1Coeffs.inputBuffer = us1dma;	
		af2Coeffs.inputBuffer = us2dma;	
		maxUS1 = maxInt( maxUS1, ComputeMax( us1dma, SEGMENT_SIZE ));
		maxUS2 = maxInt( maxUS2, ComputeMax( us2dma, SEGMENT_SIZE ));
	}
	if (interruptStatus & 0x0008) { // full frame
		// set up pointers to source buffers
		af1Coeffs.inputBuffer = us1dma+SEGMENT_SIZE;	
		af2Coeffs.inputBuffer = us2dma+SEGMENT_SIZE;	
		maxUS1 = maxInt( maxUS1, ComputeMax( us1dma+SEGMENT_SIZE, SEGMENT_SIZE ));
		maxUS2 = maxInt( maxUS2, ComputeMax( us2dma+SEGMENT_SIZE, SEGMENT_SIZE ));
	}
	if (interruptStatus & 0x0010) { // last frame
		//  Set up the global flag so CPU will read ADC to clear the interrupts
		//  after DMA stops non-finished due to prefetching.
		usDmaLastFrame = 1;

		/* Adjust Gain */
		GAIN_CTRL_AdjustGain( maxUS1, maxUS2 );
		maxUS1 = 0;
		maxUS2 = 0;
	}
	if (interruptStatus & 0x0020) { // end of block
		*(ioport unsigned int *)DMACCR2 &= 0xFF7F;  //  Disable DMA2
		*(ioport unsigned int *)DMACCR3 &= 0xFF7F;  //  Disable DMA3
		DIS_INT_DMA3;
		usDmaLastFrame = 0;  //  clear the global flag
	}			
	if (interruptStatus & 0x000C) { // half frame or full frame
		//  Set up pointers to destination buffers
		af1Coeffs.outputBuffer = (dataArray1Seg + numSegmentsCollected * SEGMENT_SIZE);
		af2Coeffs.outputBuffer = (dataArray2Seg + numSegmentsCollected * SEGMENT_SIZE);

		DMA3IsrAsm();  //  Process the collected data in assembly.

		numSegmentsCollected++;
	}

//DMA3ISR_Exit:
	PROFILE_SUMMED_INTERVAL_END( PROFILE_DMA3ISR );
	//GPIO23_RESET;
}
#endif  //COPY2
