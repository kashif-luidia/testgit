/*----------------------------------------------------------------------
	Module:  asmfunc.h
	
	C declarations for functions implemented in assembly.
		
	Copyright (C) 2000 Electronics For Imaging
----------------------------------------------------------------------*/
#ifndef __ASMFUNC_H
#define __ASMFUNC_H
#include "structs.h"
#include "error.h"

void SetupStatusPacket(int cmd, int error);	
long SumOfProducts(int len, int *buff1, int *buff2);
int ComputeMax(int *startAddress, int length);
#pragma C54X_CALL(ComputeMax)
void ShiftAndSubtract(int *startAddress, int length, int shift);
#pragma C54X_CALL(ShiftAndSubtract)
void ShiftAndAdd(int *startAddress, int length, int shift);
#pragma C54X_CALL(ShiftAndAdd)
void ScaleDownBy8AndRound(int *startAddress, int length);
#pragma C54X_CALL(ScaleDownBy8AndRound)
long ComputeBlockMismatchAsm(int blockSize, int nBlocks, int *signal1, int *signal2,
		int norm1kS1OverS2, long *aSum, long *bSum);
#pragma C54X_CALL(ComputeBlockMismatchAsm)
int maxInt(int a, int b);
int minInt(int a, int b);
ERROR FindPeakForward(int *start, int length, int *pPeakIndex, int *pPeakValue);
#pragma C54X_CALL(FindPeakForward)
ERROR FindPeakBackward(int *start, int length, int *pPeakIndex, int *pPeakValue);
#pragma C54X_CALL(FindPeakBackward)
ERROR FindPeakForwardAbs(int *start, int length, int *pPeakIndex, int *pPeakValue);
#pragma C54X_CALL(FindPeakForwardAbs)
ERROR FindPeakBackwardAbs(int *start, int length, int *pPeakIndex, int *pPeakValue);
#pragma C54X_CALL(FindPeakBackwardAbs)
// Moved to optfunc.h with desegmentation revision
//int FindThreshold(int *startAddress, int length, int level);
//#pragma C54X_CALL(FindThreshold)

void FinishRestart();
void D12ReadData(int *data, int length);
void D12WriteCommandAndData(int command, int *data, int length);
/*  Variables  */
extern int USB_ResetD12Packet;
extern int sendBadPoints;
extern int lowConfPointErrPacket[];
extern int variablesToZeroStart;
extern int queueHeadIn;
extern int queueTail;
extern int queueHeadValid;
extern int msTimer;
extern int passThroughSendIndex;
extern int passThroughNewDataIndex;
extern int serialNotUsb;
extern int variablesToZeroEnd;
extern int penDataInfoBlock;
extern int penDataTable[];
extern int irSpacing;	
extern int btConnected;
extern int irThreshold;

#endif
