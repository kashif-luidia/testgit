;----------------------------------------------------------------------
;	Module:  asmfunc.asm
;	Assembly functions for eBeam detection
;		
;	Copyright (C) 2000 Electronics For Imaging
;---------------------------------------------------------------------
            
            
	.c54cm_on
	.mmregs     
	.if (COPY2 == 0)
	.global 	_SumOfProducts
	.global		_ShiftAndSubtract
	.global		_ShiftAndAdd
	.global		_FindThreshold
	.global		_ScaleDownBy8AndRound
    .global 	_ComputeBlockMismatchAsm
	.endif
	.global		_FindPeakForward
	.global		_FindPeakBackward
	.global		_FindPeakForwardAbs
	.global		_FindPeakBackwardAbs
	.global		_SetupStatusPacket
	.global 	_statusPacket
    .global		_ComputeBlockMismatchAsm
	.global		_matchingMinTable
	.global 	_minInt
	.global		_ComputeMax
	.global		_maxInt
	.global		dummyISR
	.global		_DelayHalfMicroSecond
	.global		_D12WriteCommandAndData
	.global		_D12ReadData
	.global		EmulatorEntryPoint
	.global		_c_int00		
	.include "Copy1\_asmsyms.asm"
	
	.text	

	.c54cm_off

;void D12ReadData2(int *data, int length)
_D12ReadData:	;  AR0 is data buffer address, T0 is length
		BCC D12Read_end,T0 <= #0			;  length <= 0, quit
		SUB #1,T0,AR1
		MOV AR1,BRC0
		CALL _DelayHalfMicroSecond  ;  this function is often called after D12_Write
									;  delay to ensure enough turn-around time.
		RPTB #(D12Read_end-1)
		MOV *(#D12_DATA_ADDRESS), T0
		AND #0xFF, T0				;  On C5502 8-bit access generates wrote bits[15-8] bits.
									;  just do 16-bit access and mask the bits[15-8]
		MOV T0, *AR0+
		CALL _DelayHalfMicroSecond
D12Read_end:
		RET

;void D12WriteCommandAndData2(int command, int *data, int length)
_D12WriteCommandAndData:  ;  T0 is command, AR0 is data address, T1 is length
		SUB #1,T1,AR1					;  T1 == length.  length-1 -> AR1
		MOV AR1,BRC0					;  length-1 -> BRC0
		CALL _DelayHalfMicroSecond		;  ensure 500ns turn around time on USB
		MOV T0, *(#D12_COMMAND_ADDRESS)	
		CALL _DelayHalfMicroSecond
		BCC D12Write_end, T1 <= #0		
		RPTB #(D12Write_end-1)
		MOV *AR0+, T0
		MOV T0, *(#D12_DATA_ADDRESS)
		CALL _DelayHalfMicroSecond
D12Write_end:
	RET		
		
	.c54cm_on

	.if (COPY2 == 0)
;-------------------------
; Moved FindThreshold() to optfunc.c with desegmentation revision.
;-------------------------
;int FindThreshold(int *startAddress, int length, int level)
;{
;	int *saveAddress = startAddress + 1;
;	length += 1;
;	while(length--)
;		if(abs(*startAddress++) > level)
;			break;
;	return(startAddress - saveAddress);
;}
;  Arguments:
;   AR0 = startAddress
;   T0 = length
;   T1 = level
;   T0 returns index where threshold was found
;	.c54cm_off
;_FindThreshold:
;		RETCC	T0 <= #0			;  Return length as index if length <= 0
;		MOV		T0, AC0 			;  AC0 = count
;		MOV		AC0, mmap(BRC0) 	;  BRC0 = count  (will loop count+1 times)
;		BSET	SXM					;  Do signed math
;		NEG		T1, AC1				;  AC1 = -thresholdLevel
;		MOV		AR0, AC2			;  Save start address in AC2
;		RPTB 	FindThresholdLoopEnd-1
;		MOV 	*AR0+, AC0			;  AC0 = value at current index
;		ABS		AC0					;  AC0 = |value|
;		ADD 	AC1, AC0			;  AC0 = |value| - threshold
;		BCC 	FindThresholdLoopEnd, AC0 > #0	;  If value exceeds threshold, exit loop	
;FindThresholdLoopEnd:
;		MOV		AR0, AC0			;  AC0 = Current address + 1
;		SUB		AC2, AC0			;  AC0 = Current address + 1 - start address
;		SUB		#1, AC0				;  AC0 = Current address - start address = index of threshold
;		MOV		AC0, T0				;  T0 = index (return value)
;		RET


	.c54cm_on

;void ShiftAndAdd(int *startAddress, int length, int shift)
;{
;	int i;
;	for(i=length; i>=shift; --i)
;		startAddress[i] += startAddress[i-shift];
;	return;
;}

; .if 0
_ShiftAndAdd:
;  AR2 = startAddress[i]
;  AR3 = startAddress[i] - shift
;  BRC = remaining length
;  Input:  A = startAddress
;          *(SP+1) = length
;          *(SP+2) = shift
		LD		*SP(1), B				;  B = length
		SUB		#1, B					;  B = length - 1
		ADD	    B, A					;  A = startAddress + length-1
		SUB		*SP(2), B				;  B = length - 1 - shift
		BC		shiftAndAddDone, BLEQ   ;  If length <= 0, forget it
		STLM	B, BRC					;  BRC = length - 1 - shift
		STLM	A, AR2					;  AR2 = startAddress + length-1 (last address to s/a)
		SUB		*SP(2), A				;  A = startAddress + length-1 - shift
		STLM	A, AR3					;  AR3 = AR2 - shift
		RPTB	shiftAndAddDone-1		;  Repeat length times
		LD		*AR2, A					;  A = startAddress[i]
		ADD		*AR3-, A				;  A += startAddress[i-shift]
		STL		A, *AR2-				;  startAddress[i] += startAddress[i-shift];i--
shiftAndAddDone:
		RET		
; .endif

 .if 0
;void ShiftAndSubtract(int *startAddress, int length, int shift)
;{
;	int i;
;	for(i=shift; i < length; ++i)
;		startAddress[i] -= startAddress[i-shift];
;	return;
;}
_ShiftAndSubtract:
;  AR2 = startAddress[i]
;  AR3 = startAddress[i] - shift
;  BRC = remaining length
;  Input:  A = startAddress
;          *(SP+1) = length
;          *(SP+2) = shift
		LD		*SP(1), B					;  B = length
		SUB		#1, B					;  B = length - 1
		SUB		*SP(2), B				;  B = length - 1 - shift
		BC		shiftAndSubDone, BLEQ   ;  If length <= 0, forget it
		STLM	B, BRC					;  BRC = length - 1 - shift
		STLM	A, AR3					;  AR3 = startAddress
		ADD		*SP(2), A				;  A = startAddress + shift
		STLM	A, AR2					;  AR2 = AR3 + shift
		RPTB	shiftAndSubDone-1		;  Repeat length times
		LD		*AR2, A					;  A = startAddress[i]
		SUB		*AR3+, A				;  A -= startAddress[i-shift]
		STL		A, *AR2+				;  startAddress[i] -= startAddress[i-shift];i++
shiftAndSubDone:
		RET		
 .endif
;void ScaleDownBy8AndRound(int *startAddress, int length);
;{
;	int i;
;	for(i=0;i<length;++i)
;		startAddress[i] = (startAddress[i] + 4) >> 3;
;	return;
	.if 0
_ScaleDownBy8AndRound:
		SSBX	SXM						;  Use signed math
		STLM	A, AR2					;  AR2 = startAddress
		LD		*SP(1), A				;  A = length
		SUB		#1, A					;  A = length-1
		STLM	A, BRC					;  BRC = length-1
		NOP
		NOP								;  Let BRC cool down
		RPTB	ScaleDownDone-1
		LD		*AR2, A					;  A = startAddress[i]
		ADD		#4, A					;  A = startAddress[i] + 4
		SFTA	A, -3					;  A = (startAddress[i] + 4) >> 3
		STL		A, *AR2+				;  startAddress[i] = A, ++i
ScaleDownDone:
		RET
	.endif

;***************************************************************
;* FUNCTION DEF: _ComputeBlockMismatchAsm                      *
;***************************************************************
;long ComputeBlockMismatchAsm(int blockSize, int nBlocks, int *signal1, int *signal2, int norm1kS2OverS1,
;	long *aTotal, long *bTotal)	
    ; Requires:    
    ; blockSize>=1
    ; nBlocks>=1
    ; signal values smaller than 8192=2^13
    ; signal amplitude ratio less than 32.
;  This function calcalates minimum mismatches with jitters at -1, 0, and 1, then output values for interpolation.
;  Assuming (jitter, mismatch) fits into y = a*x^2 + b*x + c, then 
;    a = (y_(1)+y_(-1))/2-y_(0)
;    b = (y_(1)-y_(-1))/2
;    c = y_(0)
;  The function puts a, b values of all blocks into matchingMinTable, and sum up a, b, c for all blocks.
;
;C equivalent:
;long ComputeBlockMismatchC(int blockSize, int nBlocks, int *signal1, int *signal2, int norm1kS2OverS1,
;	long *aTotal, long *bTotal)
;{
;	extern double matchingMinTable[];  // matchingMinTable[18] contains a, b for each block
;	int length = blockSize * nBlocks;
;	
;	int i, j;
;	long a1, a2, a3, mina, cTotal, a, b;
;	int tempint;
;	int s1Scaled;
;	int block;
;
;	i = 0;
;	cTotal = 0;
;	block = 0;
;	*aTotal = *bTotal = 0;
;
;	while (i <= length-blockSize) {
;		
;		a1 = 0;
;		a2 = 0;
;		a3 = 0;
;		for(j=i; j<i+blockSize; j++) {
;			s1Scaled = (*(signal1+j)*norm1kS2OverS1)>>10;
;			tempint = *(signal2+j-1) - s1Scaled;
;			a1 += tempint * tempint;
;			tempint = *(signal2+j) - s1Scaled;
;			a2 += tempint * tempint;
;			tempint = *(signal2+j+1) - s1Scaled;
;			a3 += tempint * tempint;
;		}
;
;		a = (a3+a1)/2-a2;					// a = (y_(1)+y_(-1))/2-y_(0)
;		b = (a3-a1)/2;						// b = (y_(1)-y_(-1))/2
;
;		*aTotal += a;						// sum up a
;		*bTotal += b;						// sum up b
;		matchingMinTable[block*2] = a;		// save a, b into abTable
;		matchingMinTable[block*2+1] = b;
;
;		if (a<=0 || abs(b)>2*a) {			// if a<=0 or |b|>2*a, use min at -1, 0, or 1
;			a = 0;			
;			matchingMinTable[block*2] = a;	// a==0 is used as a flag
;			mina = a3<a1 ? a3 : a1;
;			mina = mina<a2 ? mina : a2;
;			cTotal += mina;					// use min of -1, 0, 1 to sum up
;		}
;		else
;			cTotal += a2;
;
;		i += blockSize;
;		block++;
;	}
;
;	return cTotal;
;}
;*************************************************
;  REGISTER USAGE
;  A - Temporary things
;  B - Temporary things
;  AR0 - signal2-1
;  AR1 - signal1
;  AR2 - signal2
;  AR3 - nBlocks-1
;  AR4 - index of abTable
;  AR5 - signal1 block head
;  AR6 - signal2 block head
;  AR7 - signal2+1
;  BRC - remaining samples in the current block
;*************************************************

sumA1		.set	0	; mismatch with signal2 shifted back one int
sumA2		.set	2	; mismatch without shift
sumA3		.set    4	; mismatch with signal2 shifted ahead one int
cTotal	.set	6	; mismatch total for all blocks
aTotal	.set	8
bTotal	.set	10
blockSizeMinusOne	.set	12
localStackSize	.set	14
spDisplacement	.set	localStackSize+3    ; considering 3 PSHM's.
nBlocks		.set	spDisplacement+1
signal1		.set	spDisplacement+2
signal2		.set	spDisplacement+3
norm1kS2OverS1	.set	spDisplacement+4
aTotalAddress	.set	spDisplacement+5
bTotalAddress	.set	spDisplacement+6
        
_ComputeBlockMismatchAsm:

    PSHM AR1		
    PSHM AR6
    PSHM AR7
    FRAME #-localStackSize
    SSBX SXM
	SUB #1, A
	STL A, *SP(blockSizeMinusOne); blockSize-1 saved on stack
    MVDK *SP(nBlocks), AR3		
    MVDK *SP(signal1), AR5	; signal1->AR5
    MVDK *SP(signal2), AR6	; signal2->AR6
    MAR *AR3-			; nBlocks-1->AR3
	LD #_matchingMinTable, A
	STLM A, AR4
    
    XOR A, A
    DST A, *SP(cTotal)
	DST A, *SP(aTotal)
	DST A, *SP(bTotal)
	    
loop_nBlocks:			; start calculation for the block

    ; calculate a1, a2, a3
    XOR A, A
    DST A, *SP(sumA1)		; initilize sumA1, sumA2, sumA3 to 0
    DST A, *SP(sumA2)
    DST A, *SP(sumA3)			
	MVDK *SP(blockSizeMinusOne), *(BRC)	; blockSize-1->BRC
    MVMM AR5, AR1		; signal1->AR1
    MVMM AR6, AR2		; signal2->AR2
    MVMM AR6, AR0
    MAR *AR0-			; signal2-1->AR0
    MVMM AR6, AR7
    MAR *AR7+			; signal2+1->AR7
    LD *SP(norm1kS2OverS1), T	; norm1kS2OverS1->T, make sure T is not changed
    RPTB loop_sum_end-1
    MPY *AR1+, B		; *signal1*norm1kS1OverS2->B
    SFTA B, 6
    SUB *AR0+, 16, B, A		
    SQUR A, A
    DADD *SP(sumA1), A
    DST A, *SP(sumA1)
    SUB *AR2+, 16, B, A			
    SQUR A, A
    DADD *SP(sumA2), A
    DST A, *SP(sumA2)
    SUB *AR7+, 16, B, A			
    SQUR A, A
    DADD *SP(sumA3), A
    DST A, *SP(sumA3)
loop_sum_end:
    
    ;sumA3 already in A
	DADD *SP(sumA1), A, B
	SFTA B, -1
	DSUB *SP(sumA2), B				; B: a = (y_(1)+y_(-1))/2-y_(0)
	DSUB *SP(sumA1), A	
	SFTA A, -1					; A: b = (y_(1)-y_(-1))/2							

	DST B, *AR4
	DADD *SP(aTotal), B				; sum up a
	DST B, *SP(aTotal)
	DADD *SP(bTotal), A, B			; sum up b
	DST B, *SP(bTotal)
	DLD	*AR4+, B
	DST A, *AR4+				; save a, b into abTable

	BC matching_use_min, BLEQ	; if a<=0, use min at -1, 0, or 1
	ABS A
	SUB B, 1, A
	BC matching_use_min, AGT	; if |b|>2*a, use min at -1, 0, or 1
	DLD *SP(sumA2), A				; for a>0 && |b|<=2a, sum up c	
	B matching_block_loop_end

matching_use_min:
	XOR A, A
	DST A, *AR4(-4)				; a==0 is used as a flag.  
	DLD *SP(sumA3), A
	DLD *SP(sumA1), B
	MIN A
	DLD *SP(sumA2), B
	MIN A						; use min of -1, 0, 1 to sum up

matching_block_loop_end:
    DADD *SP(cTotal), A				; add minimum mismatch of this block to the total
    DST A, *SP(cTotal)
	
    MVMM AR1, AR5				;update AR5, AR6 for the next block
    MVMM AR2, AR6
    BANZ loop_nBlocks, *AR3-	; repeat calculation for the next block
	
; out of loop, done with all blocks.  Save aTotal, bTotal, then return.

    MVDK *SP(aTotalAddress), *(AR1)
	DLD	*SP(aTotal), B
	DST	B, *AR1
    MVDK *SP(bTotalAddress), *(AR1)
	DLD	*SP(bTotal), B
	DST	B, *AR1	

;	DLD	cTotal, A				; cTotal already in A	
    FRAME #localStackSize
    POPM AR7
    POPM AR6
    POPM AR1
    RET

	.endif
	
;  ERROR FindPeakForward(int *start, int length, int *pPeakIndex, int *pPeakValue)
;  Searches forward in a buffer for a peak integer value, then returns the index
;  and value of the value.  Returns NOERR if successful, ERROR_NO_PEAK_FOUND if a 
;  peak was not found.
;_FindPeakBackwardAbs:
;	BD		FindPeakAbs
;	STM		#3, AR0		;  Set increment for backwards search
;_FindPeakForwardAbs:
;	STM		#1, AR0		;  Set increment for forward search
;FindPeakAbs:
;	;  Modify core algorithm to not take abs 
;	LD		#0x3200, B	;  B = opcode for ABS A, A
;	STL		B, *((FindPeakMod1+1)/2)
;	ST		#0x3211, *((FindPeakMod2+1)/2) ;  opcode for ABS B, B
;	STL		B, *((FindPeakMod3+1)/2)
;	B		FindPeakCore
;_FindPeakBackward:
;	BD		FindPeakAsm
;	STM		#3, AR0		;  Set increment for backwards search
;_FindPeakForward:
;	STM		#1, AR0		;  Set increment for forward search
;FindPeakAsm:
;	;  Modify core algorithm to not take abs 
;	LD		#0x2020, B	;  B = opcode for NOP
;	STL		B, *((FindPeakMod1+1)/2)
;	STL		B, *((FindPeakMod2+1)/2)	
;	STL		B, *((FindPeakMod3+1)/2)
;FindPeakCore:
;	SUB		#1, A		;  A = start - 1
;	STLM	A, AR2		;  AR2 = start - 1
;	LD		*SP(1), A	;  A = length
;	SUB		#1, A		;  A = length - 1
;	STLM	A, BRC		;  BRC = length - 1
;	STM		#0, AR4		;  AR4 = peakIndex
;	MVDK	*SP(2), *(AR5)   ;  AR5 = pPeakIndex
;	MVDK	*SP(3), *(AR3)   ;  AR3 = pPeakValue
;	RPTB	FindPeakForwardNotFound - 1
;	LD		*AR2+, A	;  A = Value before peak being considered
;FindPeakMod1:
;	NOP
;	NOP
;	NOP
;	LD		*AR2+, B	;  B = Value being considered
;	NOP
;FindPeakMod2:
;	NOP
;	NOP
;	NOP
;	STL		B, *AR3		;  Store in pPeakValue in case this is the peak
;	SUB		B, A
;	BC		FindPeakForwardNextPoint1, AGT	;  If value before was bigger, this isn't a peak
;	LD		*(AR2-AR0), A	;  A = Value after peak being considered, AR2 moves to value before next peak
;FindPeakMod3:
;	NOP
;	NOP
;	NOP
;	SUB		B, A
;	BC		FindPeakForwardNextPoint, AGT	;  If value before was bigger, this isn't a peak
;	;  This is a peak.
;	MVKD	*(AR4), *AR5	;  Update pPeakIndex
;	LD		#DEF_NOERR, A		;  Return noerror
;	RET
;FindPeakForwardNextPoint1:
;	MAR		*AR2-0
;FindPeakForwardNextPoint:
;	MAR		*AR4+		;  Increment index	
;	NOP
;	NOP
;	NOP
;FindPeakForwardNotFound:
;	NOP
;	NOP
;	NOP
;	;  Peak not found
;	LD		#DEF_ERR_NOPEAKFND, A
;	RET	
		
;void SetupStatusPacket(COMMAND cmd, ERROR error)
;{
;	memset(statusPacket, 0, PACKET_LENGTH);
;	statusPacket[0] = 0x10;
;	statusPacket[1] = cmd;
;	statusPacket[2] = (error & 0xff);
;	statusPacket[3] = ((error >> 8) & 0xff);
;	return;
;}
_SetupStatusPacket:
	STM		#(_statusPacket+PACKET_LENGTH-1), AR2
	RPTZ	A, (PACKET_LENGTH-2)
	STL		A, *AR2-
	MOV		T1, AC1		;  B = error
	ST		#0x10, *AR2+
	MOV		T0, *AR2+	;  cmd stored
	AND		#0xff, B, A	;  A = error & 0xff
	STL		A, *AR2+
	STL		B, -8, *AR2
	RET

;int ComputeMax(int *startAddress, int length)
;{
;	int maxValue = 0;
;	while(length--)
;	{
;		maxValue = max(maxValue, abs(*startAddress));
;		startAddress++;
;	}
;	return(maxValue);
;} 14 words used
_ComputeMax:
	STLM	A,AR2					;  AR2 := currentAddress = bufferStart 
	LD		*SP(1),B				;  B = length
	LD		#0, A					;  A := max = 0
	SUB		#1, B, B				;  B = length - 1
	STLM	B, BRC					;  BRC = length - 1
	SSBX	SXM						;  Signed mode
	NOP								;  Wait for BRC to cool off
	RPTB	ComputeMaxLoopEnd-1		;  Loop and calc max
	LD		*AR2+, B				;  Load next value
	ABS		B, B					;  Compute absolute value
	MAX		A						;  A = max(A, B)
ComputeMaxLoopEnd:
	RET

	.c54cm_off

;int maxInt(int a, int b)
;{
;	if(a > b)
;		return(a);
;	return(b);
;}
_maxInt:
	MAX 	T1, T0
	RET
;int minInt(int a, int b)
;{
;	if(a< b)
;		return(a);
;	return(b);
;}
_minInt:
	MIN 	T1, T0
	RET

dummyISR:	B dummyISR	

_DelayHalfMicroSecond:
	RPT #(CPU_CLOCK_FREQUENCY/1000000/2-1)
	NOP
	RET

	.c54cm_off
EmulatorEntryPoint:
	MOV #0, *(#COPY_NUMBER_ADDRESS)
	B _c_int00
	
	.end	
	
