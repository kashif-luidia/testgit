/*----------------------------------------------------------------------
	Module:  colldata.h
	
	Declarations for data collection functions.
		
	Copyright (C) 2000 Electronics For Imaging
----------------------------------------------------------------------*/
                             
#ifndef _COLLDATA_H
#define _COLLDATA_H

#include "comdef.h"
#include "sysdef.h"
#include "error.h"

extern int dataArray1Seg[];
extern int dataArray2Seg[];
extern int us1dma[];
extern int us2dma[];
extern int *irBuffer;
extern int irSpacing;
extern volatile int disableDataCollection;
extern int projectionPenStatus;
extern int previousProjectionPenStatus;
extern int secPulseRisingEdge;

void ConfigDMAforDataCollection();
void BeginDataCollection();
ERROR WaitForNewDataAvailable();
void TerminateDataCollection();

void configADC_IR();
void configADC_US();
void configADC_PowerDown();
int ReadADC();

void DMA5Config_Timer();
void DMA2Config_US1();
void DMA3Config_US2();
void ReadIRDataIntoBuffer(unsigned int *pBuffer, unsigned int numPointsToRead);

#ifdef NEW_US_FILTER
/* 	US Filter */
#define US_FILTER_NBIQ	4					// number of biquads in the US filter
#define US_FILTER_NH	(5*US_FILTER_NBIQ)	// size of coefficient memory
#define US_FILTER_NX	SEGMENT_SIZE		// size of input
typedef Int16 			USWeightVector[US_FILTER_NH];
extern USWeightVector usFilter;
extern Int32 usFilterDelayBuff1[2*US_FILTER_NBIQ];
extern Int32 usFilterDelayBuff2[2*US_FILTER_NBIQ];

UInt16 iircas5_c(			
	Int16   *in, 		// AR0
	Int16   *h, 			// AR1 (loop pointer in AR5)
	Int16   *out, 		// AR2
	Int32   *db, 		// AR3 (loop pointers in AR6 and AR7)
	UInt16  nbiq, 		// T0
	UInt16  nx,			// T1
	UInt16  inGain,		// AR4 ==> T2
	UInt16  outGain		// stack ==> AR6 ==> T3
);
#endif

/*  IR Threshold  */
#define NOMINAL_IR_THRESHOLD 100
#define IR_RMS_FRAC_BITS 17		// Number of fractional bits in irHistory
#define IR_PEAK_FRAC_BITS 3		// Number of fractional bits in peak factor
#define IR_PEAK_FACTOR ((UInt16) (3.0*(1<<IR_PEAK_FRAC_BITS)))	// approximation of 1.414 in Q3.3 format
#define IR_PEAK_OFFSET 25

extern UInt32  	irHistory;		// RMS history in Q23.8 fixed point format
extern UInt16	irRms;			// Integer part of IR history
extern UInt16 	irAverage;		// IR average calculated on power up

#endif

