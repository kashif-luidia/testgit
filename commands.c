/*---------------------------------------------------------------------------
Module:  commands.c
Purpose:  Handles commands sent from PC to eBeam pods.
Functions:
	ReadMemory() - Send a block of memory back to PC.
	
History:	Feb 4, 2000		Created by Phil Weaver
---------------------------------------------------------------------------*/

/*  Includes  */
#include "error.h"
#include "commands.h"
#include "dspdef.h"
#include "flash.h"
#include "version.h"
#include "usb_func.h"
#include "thresh.h"
#include "colldata.h"
#include "timer.h"
#include "asmfunc.h"
#include "gain_control.h"

#ifdef _GECKO_ENGAGED_
extern UInt16 ledBlinkingInterval;
extern UInt16 ledSettingSelection;
extern UInt16 ledBlinkingCounter;
#endif

/*  Local Prototypes  */
void SendStatusPacket();
void StatusPacketSent();
void CommandReceiveData(int *receiveBuffer, int numBytesToReceive, void (*callbackfunction)());
void FinishCommandSetSerNum();
void FinishFlashProgramming(); 
void SendCollectedData( void );

/*  Global Variables  */
extern int *pCodeCopyNumber;
int receiveBuffer[RECEIVE_BUFFER_LENGTH];
int statusPacket[PACKET_LENGTH];
#ifndef COPY2
extern int sendBadPoints;
#else
/*  Globals for flash programming  */
COMMAND_MODE commandMode;
int dangerModeCommandCount = 0;
unsigned long flashProgLength;
unsigned int flashProgExpectedCheckSum;
unsigned long flashProgOffset;
COMMAND flashProgCommand;
#endif

void HandleNewlyReceivedData()
{
	/*  Stop sending pen data  */
	usbInfo.dontSendPenData = 1;
#ifndef COPY2
	INT_DISABLE;
	TerminateDataCollection();
	INT_ENABLE;
#endif
	/*  10 second timeout  */
	msTimerToTimeoutCommand = msTimer + 10000;
	/*  Must get data 8-bytes at a time  */
	if(usbInfo.receivedDataBuffer[1] != 8)
	{
		ReturnCommStatus(NO_COMMAND, ERROR_INVALID_PACKET);
	}
	/*  Packet is at least the right length.  */
	if(usbInfo.receivedDataBuffer[2] == PACKET_ID_COMMAND)
	{
		/*  This command starts a new command, regardless of our
		    previous state  */
		usbInfo.commandHandlerState = COMMAND_HANDLER_NEW_COMMAND;
		NewCommand(usbInfo.receivedDataBuffer+2);
		return;
	}
	/*  Receive data if that's what we're expecting to do  */
	if(usbInfo.receivedDataBuffer[2] == PACKET_ID_DATAOUT)
	{
#ifdef COPY2
		if(usbInfo.commandHandlerState != COMMAND_HANDLER_RECEIVING_DATA)
		{
			ReturnCommStatus(NO_COMMAND, ERROR_UNEXPECTED_DATA);
			return;
		}
		/*  Copy data into our buffer  */
		memcpy(usbInfo.commandHandlerReceivePtr, usbInfo.receivedDataBuffer + 3, 7*sizeof(int));
		usbInfo.commandHandlerReceivePtr += 7;
		usbInfo.commandHandlerBytesToReceive -= 7;
		/*  If more to receive, just return.  If transfer complete, execute callback  */
		if(usbInfo.commandHandlerBytesToReceive <= 0)
			(*usbInfo.commandHandlerReceiveDataCallback)();
		return;
#else
		/*  Copy 1 doesn't receive data  */
		ReturnCommStatus(NO_COMMAND, ERROR_UNEXPECTED_DATA);
		return;
#endif
	}
	/*  Unknown packet type  */
	ReturnCommStatus(NO_COMMAND, ERROR_INVALID_PACKET);
	return;
}

void NewCommand(BYTE *cmdPacket)
{
	switch(cmdPacket[1])
	{
	case COMMAND_ANDORMEM:
		Command_AndOrMemory(cmdPacket);
		break;
	case COMMAND_GETLASTSTAT:
		Command_GetLastStat();
		break;
	case COMMAND_RESTART:
		Command_Restart(cmdPacket);
		break;
	case COMMAND_GETSERNUM:
		Command_GetSerNum();
		break;
	case COMMAND_GETVERSION:
		Command_GetVersion();
		break;
	case COMMAND_SETPENDATASEND:
		Command_SetPenDataSend(cmdPacket);
		break;
	case COMMAND_WRITETOSERIAL:
		Command_WriteToSerial(cmdPacket);
		break;
	case COMMAND_SETSERIALPASSTHROUGH:
		Command_SetSerialPassthrough(cmdPacket);
		break;
	case COMMAND_GETCOMMSTATE:
		Command_GetCommState();
		break;	
#ifdef _GECKO_ENGAGED_
	case COMMAND_SETLEDMODE:		// For GECKO Engaged
		Command_SetLEDMode(cmdPacket);
		break;
#endif
#ifndef COPY2
	case COMMAND_SETFLIPCHARTMODE:
		Command_SetFlipchartMode(cmdPacket);
		break;
	case COMMAND_GETSTATS :
		Command_GetStats(cmdPacket);
		break;
	case CM_SET_GAINCTRL:
		GAIN_CTRL_SetGainCmd(cmdPacket);
		break;
	case CM_GET_GAINCTRL:
		GAIN_CTRL_GetGainCmd();
		break;
#else

	case COMMAND_GETHEADER2CHECKSUM:
		Command_GetHeader2Checksum();
		break;	

			
	case COMMAND_GETFLASHINFO:
		Command_GetFlashInfo();
		break;
	case COMMAND_MEASURENOISE:
		Command_MeasureNoise();
		break;
	case COMMAND_SETSERNUM:
		Command_SetSerNum();
		break;
	case COMMAND_SETWRITEMODE:
		Command_SetWriteMode(cmdPacket);
		break;
	case COMMAND_WRITEFLASHCODE:
		Command_WriteFlashCode(cmdPacket);
		break;
	case COMMAND_WRITEFLASHHEADER:
		Command_WriteFlashHeader(cmdPacket);
		break;
	case COMMAND_WRITEFLASHSECTOR:
		Command_WriteFlashSector(cmdPacket);
		break;
#endif
	default:
		ReturnCommStatus((COMMAND)cmdPacket[1], ERROR_INVALID_COMMAND);
	}

#ifdef COPY2
	if (dangerModeCommandCount > 0) 
	{
		dangerModeCommandCount--;
		if((dangerModeCommandCount == 0) && (commandMode == FLASH_PROTECTION_DISABLED_MODE))
		{
			commandMode = WRITES_PERMITTED_MODE;
		}
	}
#endif
	return;
}

/*  Return the command status packet
Packet definition
	|  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |
	| 0x10| Cmd | err | err |  0  |  0  |  0  |  0  |
	|     |     | lsb | msb |  0  |  0  |  0  |  0  |  */
	
void ReturnCommStatus(COMMAND cmd, ERROR error)
{
	SetupStatusPacket(cmd, error);
	SendStatusPacket();
}
void SendStatusPacket()
{
	USB_SendCommandDataToHost(statusPacket, PACKET_LENGTH, &StatusPacketSent);
	return;
}
void StatusPacketSent()
{
	usbInfo.commandHandlerState = COMMAND_HANDLER_IDLE;
	usbInfo.dontSendPenData = 0;
#ifndef COPY2
	disableDataCollection = 0;
#endif
	return;
}

/* Command to adjust memory */
/*  This command has never been used, but it is the only way we could 
    externally turn on the IR notch filter, so it stays here.  */
void Command_AndOrMemory(BYTE *cmdPacket)
{
	register int *address;
	
	address = (int *)((cmdPacket[3]<<8) + cmdPacket[2]);
	*address &= (cmdPacket[5]<<8) + cmdPacket[4];
	*address |= (cmdPacket[7]<<8) + cmdPacket[6];
	ReturnCommStatus(COMMAND_ANDORMEM, NOERR);
	return;
}


#ifdef COPY2
/*  Let the host know where we plan to put flash data we send  */

void Command_GetFlashInfo()
{
	static BYTE flashInfo[PACKET_LENGTH];

	unsigned long value;
	int shift = 8;
		

	flashInfo[0] = PACKET_ID_DATAIN;
	/*  Header address  */
	value = (*pCodeCopyNumber == 2) ? 
		flash.headerPosition1+flash.baseAddress : flash.headerPosition2+flash.baseAddress;
	value = value >> shift;
	flashInfo[1] = value & 0xff;
	flashInfo[2] = (value >> 8) & 0xff;
	/*  Code address  */
	value = (*pCodeCopyNumber == 2) ? flash.codePosition1+flash.baseAddress : flash.codePosition2+flash.baseAddress;
	value = value >> shift;
	flashInfo[3] = value & 0xff;
	flashInfo[4] = (value >> 8) & 0xff;
	/*  Code max length  */
	value = (*pCodeCopyNumber == 2) ? flash.codeSizeMax1 : flash.codeSizeMax2;
	value = value >> shift;
	flashInfo[5] = value & 0xff;
	flashInfo[6] = (value >> 8) & 0xff;
	flashInfo[7] = shift;
	/*  Send info back to host  */
	SetupStatusPacket(COMMAND_GETFLASHINFO, NOERR);
	USB_SendCommandDataToHost((int *) flashInfo, PACKET_LENGTH, &SendStatusPacket);
	return;

}


void Command_GetHeader2Checksum(){
	static BYTE flashInfo[16];
	unsigned long value;
		
	memset( flashInfo, 0, 16 );
	
	//packet #1
	flashInfo[0] = PACKET_ID_DATAIN;

	value =  flash.header2Checksum1a;
	flashInfo[1] =value & 0xff;
	flashInfo[2] = (value>>8) & 0xff;

	value = flash.header2Checksum1b;
	flashInfo[3] =value & 0xff;
	flashInfo[4] = (value>>8) & 0xff;

	//packet #2, send only copy 2 checksum
	flashInfo[8] = PACKET_ID_DATAIN;
	value = flash.header2Checksum2a;

	flashInfo[5] =value & 0xff;
	flashInfo[6] = (value>>8) & 0xff;

	value = flash.header2Checksum2b;

	flashInfo[9] =value & 0xff;
	flashInfo[10] = (value>>8) & 0xff;

	//  Send info back to host 
	SetupStatusPacket(COMMAND_GETFLASHINFO, NOERR);
	USB_SendCommandDataToHost((int *) flashInfo, (24), &SendStatusPacket);
	return;



}




#endif

void Command_GetCommState( void )
{

	static BYTE returnPacket[PACKET_LENGTH];
	memset(returnPacket,0,PACKET_LENGTH);
	returnPacket[0] = PACKET_ID_DATAIN;
	returnPacket[1] = btConnected;
	returnPacket[2] = usbActivity;
	returnPacket[3] = serialNotUsb;

	/*  Return data to host  */
	SetupStatusPacket(COMMAND_GETCOMMSTATE, NOERR);
	USB_SendCommandDataToHost((int *) returnPacket, PACKET_LENGTH, &SendStatusPacket);
}

#ifdef _GECKO_ENGAGED_
void Command_SetLEDMode(BYTE *cmdPacket)
{
	ledSettingSelection = ((cmdPacket[2]<<8) + cmdPacket[3]);

	ledBlinkingCounter = ( ledSettingSelection & 0x07FF ); // kick start the blinking counter

	ReturnCommStatus(COMMAND_SETLEDMODE, NOERR);
	return;
}
#endif

/*  Return the serial number to the host  */
int serNumBuff[NUM_SERIAL_NUM_PACKETS*PACKET_LENGTH];
void Command_GetSerNum()
{
	/*  Use dataArray2Seg since data collection is disabled  */
	unsigned long serNumPtr = (flash.baseAddress + flash.serialNumberPosition)/2;
	int *packetPointer = serNumBuff;
	int i, j;
	unsigned long wordBuff;

	memset( packetPointer, 0, NUM_SERIAL_NUM_PACKETS*PACKET_LENGTH );
 	for(i=0; i < NUM_SERIAL_NUM_PACKETS/2;i++)
	{
		*(packetPointer++) = PACKET_ID_DATAIN;
		for(j=0; j < (PACKET_LENGTH/2)-1; j++ ) {
			wordBuff = far_peek(serNumPtr++);
			*(packetPointer++) = (wordBuff & 0xFF00)>>8;
			*(packetPointer++) = wordBuff & 0x00FF;
		}
		wordBuff = far_peek(serNumPtr++);
		*(packetPointer++) = (wordBuff & 0xFF00)>>8;
		*(packetPointer++) = PACKET_ID_DATAIN;
		*(packetPointer++) = wordBuff & 0x00FF;
		for(j=0; j < (PACKET_LENGTH/2)-1; j++ ) {
			wordBuff = far_peek(serNumPtr++);
			*(packetPointer++) = (wordBuff & 0xFF00)>>8;
			*(packetPointer++) = wordBuff & 0x00FF;
		}
	}
	/*  Done accessing flash  */
	SetupStatusPacket(COMMAND_GETSERNUM, NOERR);
	USB_SendCommandDataToHost((int *) serNumBuff, sizeof(serNumBuff), &SendStatusPacket);
	return;
}

/*  Return version info to host  */
void Command_GetVersion()
{
	versionBuffer[5] |= *pCodeCopyNumber;
	SetupStatusPacket(COMMAND_GETVERSION, NOERR);
	USB_SendCommandDataToHost((int *) versionBuffer, VERSION_BUFFER_SIZE, &SendStatusPacket);
	return;
}

#ifndef COPY2
extern int firstPulseMaxAmp, secPulseMaxAmp;	// Maximum amplitude of first & second  pulses
extern struct usChannelInfo sUs1Info, sUs2Info;

#define STAT_PACKET_LENGTH	PACKET_LENGTH * 2
#define BTF	8	//20
#define	PTF 2	//3

#define	STAT_BTF_ID 	0x0B
#define	STAT_DATA_ID 	0x0D

void Command_GetStats(BYTE *cmdPacket)
{
	BYTE StatPacket[STAT_PACKET_LENGTH];
	BYTE length;

	memset( StatPacket, 0, sizeof(StatPacket) );
	StatPacket[0] = PACKET_ID_DATAIN;

	switch(cmdPacket[2]) { 
		case STAT_BTF_ID: 
			StatPacket[1] =	BTF;
			StatPacket[2] = PTF;
			length = PACKET_LENGTH;
			break;
	 	case STAT_DATA_ID: 
			StatPacket[1] =	LSB(firstPulseMaxAmp);
			StatPacket[2] = MSB(firstPulseMaxAmp);
			StatPacket[3] = LSB(secPulseMaxAmp);
			StatPacket[4] = MSB(secPulseMaxAmp);
			StatPacket[5] = LSB(sUs1Info.signalMaximumForSecondPass);
			StatPacket[6] = MSB(sUs1Info.signalMaximumForSecondPass);
			StatPacket[7] = LSB(sUs2Info.signalMaximumForSecondPass);
						
			StatPacket[8]  = PACKET_ID_DATAIN;
			StatPacket[9]  = MSB(sUs2Info.signalMaximumForSecondPass);
			length = STAT_PACKET_LENGTH;
			break;
		default:	
			ReturnCommStatus(COMMAND_GETSTATS, ERROR_INVALID_COMMAND);
			return;
	}

	/*  Return data to host  */
	SetupStatusPacket(COMMAND_GETSTATS, NOERR);
	USB_SendCommandDataToHost((int *) StatPacket, length, &SendStatusPacket);
}
#endif


#ifdef COPY2
/*  Measure the noise present on the ADC channels and report it to the host  */
#define NUM_NOISE_MEASUREMENTS	100
void Command_MeasureNoise()
{
	int negativePeak = 0x7fff;
	int positivePeak = 0x8000;
	int negativePeak1, positivePeak1, negativePeak2, positivePeak2; 
	int adcValue, adcValue1, adcValue2;
	unsigned int IFRval;
	int i;
	static BYTE returnPacket[PACKET_LENGTH];
	
	returnPacket[0] = PACKET_ID_DATAIN;

//	ENA_INT_1;
	
	/*  Measure IR Noise  */
	TimerStop(GPTIMER1_BASE);
	configADC_IR();
	/*  Clear timer interrupt  */
	*(volatile int *)IFR1 = INT_1_BIT;
	ReadADC();  // clear interrupt
	DelayHalfMicroSecond();
	Timers0and1SyncStart();
	for(i=0; i < NUM_NOISE_MEASUREMENTS; ++i)
	{
		/*  Wait for timer to go off again  */
		do
		{
			IFRval = *(volatile unsigned *)IFR1;
		}
		while((IFRval & INT_1_BIT) == 0);		
		INT_DISABLE;
		while ((*(ioport unsigned int *)(GPTIMER1_BASE+GPTCNT1))>30);
		adcValue = ReadADC();
		/*  Clear timer interrupt  */
		*(volatile int *)IFR1 = INT_1_BIT;
		INT_ENABLE;
		negativePeak = minInt(negativePeak, adcValue);
		positivePeak = maxInt(positivePeak, adcValue);
	}
	i = positivePeak - negativePeak;
	returnPacket[1] = i & 0xff;
	returnPacket[2] = (i >> 8) & 0xff;
	
	/*  Measure US Noise on channel 1  */
	TimerStop(GPTIMER1_BASE);
	configADC_US();
	/*  Clear timer interrupt  */
	*(volatile int *)IFR1 = INT_1_BIT;
	ReadADC();  // clear interrupt
	ReadADC();  // clear interrupt
	DelayHalfMicroSecond();
	Timers0and1SyncStart();
	negativePeak1 = 0x7fff;
	positivePeak1 = 0x8000;
	negativePeak2 = 0x7fff;
	positivePeak2 = 0x8000;
	for(i=0; i < NUM_NOISE_MEASUREMENTS; ++i)
	{
		/*  Wait for timer to go off again  */
		do
		{
			IFRval = *(volatile unsigned *)IFR1;
		}
		while((IFRval & INT_1_BIT) == 0);
		INT_DISABLE;
		while ((*(ioport unsigned int *)(GPTIMER1_BASE+GPTCNT1))>30);
		/*  Clear timer interrupt  */
		*(volatile int *)IFR1 = INT_1_BIT;
		adcValue1 = ReadADC();
		adcValue2 = ReadADC();
		INT_ENABLE;
		negativePeak1 = minInt(negativePeak1, adcValue1);
		positivePeak1 = maxInt(positivePeak1, adcValue1);
		negativePeak2 = minInt(negativePeak2, adcValue2);
		positivePeak2 = maxInt(positivePeak2, adcValue2);
	}
	i = positivePeak1 - negativePeak1;
	returnPacket[3] = i & 0xff;
	returnPacket[4] = (i >> 8) & 0xff;
	i = positivePeak2 - negativePeak2;
	returnPacket[5] = i & 0xff;
	returnPacket[6] = (i >> 8) & 0xff;
	returnPacket[7] = 0;
	/*  Return data to host  */
	SetupStatusPacket(COMMAND_MEASURENOISE, NOERR);
	USB_SendCommandDataToHost((int *) returnPacket, PACKET_LENGTH, &SendStatusPacket);
}
#endif


/*  Command to restart the system with a specified code copy  */
void Command_Restart(BYTE *cmdPacket)
{
	int copyNumber = cmdPacket[2];
	/*  Set up the magic memory locations to tell boot loader which copy to check first  */
	if(copyNumber == 2)
	{
		*((unsigned int *)COPY_2_FIRST_MAGIC_ADDRESS_1) = COPY_2_FIRST_MAGIC_VALUE_1;
		*((unsigned int *)COPY_2_FIRST_MAGIC_ADDRESS_2) = COPY_2_FIRST_MAGIC_VALUE_2;
	}
	else
	{
		*((unsigned int *)COPY_2_FIRST_MAGIC_ADDRESS_1) = 0;
	}
	/*  Return good status then reset  */
	SetupStatusPacket(COMMAND_RESTART, NOERR);
	USB_SendCommandDataToHost(statusPacket, PACKET_LENGTH, &FinishRestart);
}

#ifndef COPY2
void Command_SetFlipchartMode(BYTE *cmdPacket)
{
	flipchartMode = cmdPacket[2];
	ReturnCommStatus(COMMAND_SETFLIPCHARTMODE, NOERR);
}
#endif

void Command_SetPenDataSend(BYTE *cmdPacket)
{
#ifndef COPY2
	sendBadPoints = cmdPacket[2]&0x02;
	memcpy(&penDataInfoBlock, &cmdPacket[3], 4);
	ReturnCommStatus(COMMAND_SETPENDATASEND, NOERR);
#endif
	return;
}

#ifdef COPY2
/*  Command to set the serial number  */
void Command_SetSerNum()
{
	/*  Write mode only  */
	if(commandMode == NORMAL_MODE)
	{
		ReturnCommStatus(COMMAND_SETSERNUM, ERROR_PERMISSION_DENIED);
		return;
	}
	/*  Put the serial number in the data buffer.  Since this is copy 2,
	    no data will be collected there.  */
	memset(dataArray2Seg, 0xFFFF, FLASH_SECTOR_LENGTH_OLD_C5402 );
	CommandReceiveData(dataArray2Seg, SERIAL_NUMBER_LENGTH, &FinishCommandSetSerNum);
	return;
}
#endif

#ifdef COPY2
void FinishCommandSetSerNum()
{
	/*  This function is called back when the serial number has been received.  Write it
	    to the appropriate flash sector  */
	program_flash_sector(flash.serialNumberPosition, (unsigned int *)dataArray2Seg);
	ReturnCommStatus(COMMAND_SETSERNUM, NOERR);
}
#endif /*COPY2*/

/*  Enter or exit Bluetooth passthrough mode.  Should not be called if interface is serial.  */
void Command_SetSerialPassthrough(BYTE *cmdPacket)
{
	INT_DISABLE;
	if(cmdPacket[2])
	{
		/*  Enter Bluetooth mode  */
		passThroughSendIndex = passThroughNewDataIndex = 0;
		EnableSerialCommunications();
	}
	else
	{
		/*  Exit Bluetooth mode  */
		DisableSerialCommunications();
	}
	INT_ENABLE;
	ReturnCommStatus(COMMAND_SETSERIALPASSTHROUGH, NOERR);
}

#ifdef COPY2
/*  Command to turn write commands on or off  */
void Command_SetWriteMode(BYTE *cmdPacket)
{
	ERROR err = NOERR;
	/*  Check if password matches  */
	if(!memcmp(cmdPacket+2, "eBeam", 5))
	{
		/*  Request to set writes permitted is OK anytime  */
		if(cmdPacket[7]==1) 
			commandMode = WRITES_PERMITTED_MODE;
		else if(cmdPacket[7]==2) 
		{
			if(commandMode == NORMAL_MODE)
			{
				err = ERROR_PERMISSION_DENIED;
			}
			else
			{
				commandMode = FLASH_PROTECTION_DISABLED_MODE;
				dangerModeCommandCount = 2;  
				/* actual count is 1, since the count will be decremented at the end of
				the function. */
			}
		}
		else 
			commandMode = NORMAL_MODE;
	}
	else 
	{ 
		err = ERROR_INVALID_COMMAND;
		commandMode = NORMAL_MODE;
	}

	ReturnCommStatus(COMMAND_SETWRITEMODE, err);
	return;
}
#endif	

#ifdef COPY2
/*  Command to overwrite a sector in the inactive code copy  */
void Command_WriteFlashCode(BYTE *cmdPacket)
{
	/* 	Don't allow flash upgrades from bluetooth.  The data rate and link reliability don't support it. */
	if (btConnected) {
		ReturnCommStatus(COMMAND_WRITEFLASHCODE, ERROR_LINK_DOES_NOT_SUPPORT);
		return;
	}
		
	/*  Requires write access  */
	if(commandMode == NORMAL_MODE)
	{
		ReturnCommStatus(COMMAND_WRITEFLASHCODE, ERROR_PERMISSION_DENIED);
		return;
	}
	/*  Set up flash programming  */
	flashProgLength = FLASH_SECTOR_LENGTH_OLD_C5402 * cmdPacket[3];
	flashProgExpectedCheckSum = (cmdPacket[5] << 8) + cmdPacket[4];
	flashProgOffset = (unsigned)(cmdPacket[6] << 8) + cmdPacket[2] + ((*pCodeCopyNumber == 2) ?
		flash.codePosition1 : flash.codePosition2);
	flashProgCommand = COMMAND_WRITEFLASHCODE;
	/*  Verify parameters  */
	if((cmdPacket[3] > MAX_FLASH_SECTORS_AT_A_TIME) ||
	   ((flashProgOffset + (flashProgLength)) > ((*pCodeCopyNumber == 2) ?
		flash.codePosition1 + flash.codeSizeMax1 
		: flash.codePosition2 + flash.codeSizeMax2)))
	{
		ReturnCommStatus(COMMAND_WRITEFLASHCODE, ERROR_CMD_PARAMS_INVALID);
		return;
	}
	/*  Receive the data and let the flash programming do the rest  */
	CommandReceiveData(dataArray2Seg, flashProgLength, &FinishFlashProgramming);
	return;	
}
#endif

#ifdef COPY2
/*  Command to overwrite the unused flash header  */
void Command_WriteFlashHeader(BYTE *cmdPacket)
{
	/* 	Don't allow flash upgrades from bluetooth.  The data rate and link reliability don't support it. */
	if (btConnected) {
		ReturnCommStatus(COMMAND_WRITEFLASHHEADER, ERROR_LINK_DOES_NOT_SUPPORT);
		return;
	}
		
	/*  Requires write access  */
	if(commandMode == NORMAL_MODE)
	{
		ReturnCommStatus(COMMAND_WRITEFLASHHEADER, ERROR_PERMISSION_DENIED);
		return;
	}
	/*  Set up flash programming  */
	flashProgLength = FLASH_SECTOR_LENGTH_OLD_C5402;  //  the old 5402 system flash sector size to avoid change in PC upgrader code.
	flashProgExpectedCheckSum = (cmdPacket[3] << 8) + cmdPacket[2];
	flashProgOffset = (*pCodeCopyNumber == 2) ? flash.headerPosition1 : flash.headerPosition2;
	flashProgCommand = COMMAND_WRITEFLASHHEADER;
	/*  Receive the data and let the flash programming do the rest  */
	CommandReceiveData(dataArray2Seg, flashProgLength, &FinishFlashProgramming);
	return;	
}
#endif

#ifdef COPY2
/*  In the new C5502 system, the sector size is 64k.  This function only writes 64 bytes as in old system.
  	Command to program one sector of the flash without range checking  */
void Command_WriteFlashSector(BYTE *cmdPacket)
{
	/* 	Don't allow flash upgrades from bluetooth.  The data rate and link reliability don't support it. */
	if (btConnected) {
		ReturnCommStatus(COMMAND_WRITEFLASHSECTOR, ERROR_LINK_DOES_NOT_SUPPORT);
		return;
	}
		
	/*  Requires total access  */
	if(commandMode != FLASH_PROTECTION_DISABLED_MODE)
	{
		ReturnCommStatus(COMMAND_WRITEFLASHSECTOR, ERROR_PERMISSION_DENIED);
		return;
	}
	/*  Set up flash programming  */
	flashProgLength = FLASH_SECTOR_LENGTH_OLD_C5402;
	flashProgExpectedCheckSum = (cmdPacket[5] << 8) + cmdPacket[4];
	flashProgOffset = ((unsigned long)cmdPacket[3] << 8) + (unsigned long)cmdPacket[2] + ((unsigned long)cmdPacket[6] << 16);
	flashProgCommand = COMMAND_WRITEFLASHSECTOR;
	/*  Receive the data and let the flash programming do the rest  */
	CommandReceiveData(dataArray2Seg, flashProgLength, &FinishFlashProgramming);
	
	return;
}
#endif  /* COPY2  */

/*  Command to write one byte at offset 2 to serial port  */
void Command_WriteToSerial(BYTE *cmdPacket)
{
	static unsigned char writeToSerialBuffer;
	writeToSerialBuffer = cmdPacket[2];
    Uart_Xmit(&writeToSerialBuffer, 1);
    ReturnCommStatus(COMMAND_WRITETOSERIAL, NOERR);
}

#ifdef COPY2
void FinishFlashProgramming() 
{
	unsigned long checksum = 0;
	int i = 0;

	/*  Verify checksum  */
	for(i=0; i < flashProgLength; ++i)
		checksum += dataArray2Seg[i];
		
	if (flashProgExpectedCheckSum != checksum)
	{
		ReturnCommStatus(flashProgCommand, ERROR_INVALID_DATA);
		return;
	}
    /*  Program all sectors  */
    for(i=0; i < flashProgLength; i += FLASH_SECTOR_LENGTH_OLD_C5402)
    {
    	program_flash_sector(flashProgOffset, (unsigned int *)(dataArray2Seg+i));
    	flashProgOffset += FLASH_SECTOR_LENGTH_OLD_C5402;
    }
	/*  Return status  */
	ReturnCommStatus(flashProgCommand, NOERR);
}
#endif /*COPY2*/

#ifdef COPY2
/*  Set up state machine to read data and then call back a specified function  */
void CommandReceiveData(int *receiveBuffer, int numBytesToReceive, void (*callbackfunction)())
{
	/*  This is pretty straightforward.  It is encapsulated for readability.  */
	usbInfo.commandHandlerReceivePtr = receiveBuffer;
	usbInfo.commandHandlerBytesToReceive = numBytesToReceive;
	usbInfo.commandHandlerReceiveDataCallback = callbackfunction;
	usbInfo.commandHandlerState = COMMAND_HANDLER_RECEIVING_DATA;
	return;
}
#endif /*COPY2*/
