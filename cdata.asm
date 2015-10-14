;  cdata.asm:  Hold table data for C code.
;  If an initialization value is specified for a C variable, it is initialized
;  by cinit, which thus requires space for it.  This is a waste of space
	.c54cm_on
	.global _irThreshold
	.global _penDataTableIndex1
	.global _penDataTableShift1
	.global _penDataTableIndex2
	.global _penDataTableShift2
	.global _usbActivity	
	.if (COPY2 == 0)
	.global _dataArray1Seg
	.global _dataArray2Seg
	.global _enableNotchFilter
	.global	_notchFilterPointer
	.global	_notchFilter1
	.global _disableIrDetection
	.global	_minIrValue, _maxIrValue
	.global _projectionPenStatus
	.global _flipchartMode
	.global _sendBadPoints
	.endif
	.global _variablesToZeroStart
	.global _queueHeadIn
	.global _queueTail
	.global _queueHeadValid
	.global _msTimer
	.global _variablesToZeroEnd
	.global _disableDataCollection	
	.global _lowConfPointErrPacket
	.global _passThroughSendIndex
	.global _passThroughNewDataIndex
	.global _serialNotUsb
	.global _penDataInfoBlock
	.global _penDataTable
	.global _irSpacing
	.global _flashBlueLedCount
	.global _btConnected
	.global _notchState	
	.global _McBSPToInputsTable
	.global _versionBuffer
	.global _zeroWord
	.global _USB_StopClockDuringSuspendPacket
	.global _USBConnectPacket
	.global _USBDisconnectPacket
	.global _USB_EnableEndpointsPacket
	.global _USB_ResetD12Packet
	.global _allFsPacket
	.global _consecutivePreechoSamples
	.include "Copy1\_asmsyms.asm"
	.c54cm_on
	.data


	;  Section of C variables all initialized to zero
_variablesToZeroStart:
_queueHeadIn:
	.space 16
_queueTail:
	.space 16
_queueHeadValid:
	.space 16
_msTimer:
	.space 16
_disableDataCollection:
	.space 16
_passThroughSendIndex:
	.space 16
_passThroughNewDataIndex:
	.space 16
_serialNotUsb:
	.space 16
_usbActivity:
	.space 16
_flipchartMode:
	.space 16
_disableIrDetection:
	.space 16
_flashBlueLedCount:	
	.space 16
_btConnected:
	.space 16
; Queue flag.  
;    1: send all points including bad chain points.  All points are output at once.  The queue is effectively 
;	   disabled.  It relies on client code to make decisions on bad chain points.
;	0: only send good points.  Bad chain points are queued until the decision to discard or output them is made
;	   at a later point.  Since the bad chain decisions are made internally, the client code can be simpler. 
_sendBadPoints:
	.space   16
;  Table for pen data
_penDataTable:
	.space 16*9
_irSpacing:
	.space   16
_consecutivePreechoSamples:
	.space 16	
	
_variablesToZeroEnd:	

;  Version information:  16 (63)
_versionBuffer:
	.word   PACKET_ID_DATAIN
	.word   (VERSION_BASE * 10 + (VERSION_SUB / 10))
	.word   VERSION_SUB - ((VERSION_SUB / 10) * 10)
	.word   BUILD_NUMBER
	.word   0
	;/*  This value will need to have the codeCopyNumber ORed with it  */
	.if (COPY2 == 1)
	.word   (2 << 4)
	.else
	.word   (1 << 4)
	.endif
	.word	BETA_FLAG
	.word   0		; OEM ID 
	.word   PACKET_ID_DATAIN
	.word   0		; OEM Product ID
	.word   0xBEBE, 0xBEBE, 0xBEBE, 0xBEBE, 0xBEBE, 0xBEBE ; version tag value
;  Code can point here when it wants to point to zero.  7 zeros in a row
_zeroWord:
	.word   0, 0, 0, 0, 0, 0, 0
_USB_StopClockDuringSuspendPacket:
	.word 0x98		;			/*  Embedded function mode, softconnect, debug mode.
					;           Close not running, Lazy clock on  */
	.word 0x0b		;  Divide clock by 12
;  Initialize USB connection without connecting to USB bus
_USBDisconnectPacket:
	.word 0x8e		;			/*  Lazy clock off  */
	.word 0x03		;			/*  Clock divide by 4  */

;  Initialize USB connection and connect to USB bus
_USBConnectPacket:
	.word 0x1e		;			/*  Lazy clock off, clockrunning, softconnect, not isochronous  */
	.word 0x03		;			/*  Clock divide by 4  */
_USB_ResetD12Packet:
	.word 0x0c
	.word 0x0b
;  Enable endpoints in D11
_USB_EnableEndpointsPacket:
	.word 0x01
_irThreshold:
	.word   NOMINAL_IR_THRESHOLD
_minIrValue:
	.word   0
_maxIrValue:
	.word   0

;  All 0xff packet
_allFsPacket:
	.word 0xff
	.word 0xff
	.word 0xff
	.word 0xff
	.word 0xff
	.word 0xff
	.word 0xff
	.word 0xff
	
	.sect ".ver"
_version:
	.word VERSION_CHAR_0
	.word VERSION_CHAR_1
	.word VERSION_CHAR_2
	.word VERSION_CHAR_3
	.word VERSION_CHAR_4
	.word VERSION_CHAR_5
	.word VERSION_CHAR_6
	.word VERSION_CHAR_7
	.word VERSION_CHAR_8

	.end
	
	


