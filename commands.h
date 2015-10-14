/*----------------------------------------------------------------------
	Module:  commands.h
	
	Definitions and exports for command.c.
		
	Copyright (C) 2000 Electronics For Imaging
----------------------------------------------------------------------*/
#ifndef _COMMANDS_H
#define _COMMANDS_H
#include "error.h"
#include "sysdef.h"

#define RECEIVE_BUFFER_LENGTH	8
extern int receiveBuffer[];
#ifndef COPY2
extern int flipchartMode;
#endif
/*  Commands  */
void Command_AndOrMemory(BYTE *cmdPacket);
void Command_GetFlashInfo( void );
void Command_GetHeader2Checksum( void );
void Command_GetCommState( void );
/*  Just re-send the last status packet in memory  */
#define Command_GetLastStat() SendStatusPacket()
void Command_GetSerNum( void );    
void Command_GetVersion( void );
void Command_MeasureNoise( void );
void Command_GetStats(BYTE *cmdPacket);
void Command_Restart(BYTE *cmdPacket);
void Command_SetFlipchartMode(BYTE *cmdPacket);
void Command_SetPenDataSend(BYTE *cmdPacket);
void Command_SetSerNum( void );
void Command_SetSerialPassthrough(BYTE *cmdPacket);
void Command_SetWriteMode(BYTE *cmdPacket);
void Command_WriteFlashCode(BYTE *cmdPacket);
void Command_WriteFlashHeader(BYTE *cmdPacket);
void Command_WriteFlashSector(BYTE *cmdPacket);
void Command_WriteToSerial(BYTE *cmdPacket);

void Command_SetLEDMode(BYTE *cmdPacket);

void ReportError(CHANNEL_NUM channel, ERROR errorCode);
void ReportErrorCh1(ERROR errorCode);
void ReportErrorCh2(ERROR errorCode);

typedef int COMMAND_MODE;
enum 
{
	NORMAL_MODE,
	WRITES_PERMITTED_MODE,
	FLASH_PROTECTION_DISABLED_MODE
};
/* 0: conf ratio; 1: Us1Max; 2: Us2Max; 3: Ir1Max; 4: Ir2Max. */
extern int penDataTableIndex1;
extern int penDataTableShift1;
extern int penDataTableIndex2;
extern int penDataTableShift2;

extern COMMAND_MODE commandMode;
#define DISABLE_WRITES					0
#define ENABLE_WRITES					1
#define ENABLE_WRITE_CODE				0xA3B5
#define HEADER_1_ADDRESS				0x400
#define HEADER_2_ADDRESS				0x480
#define COPY_1_END_ADDRESS				(COPY_2_START_ADDRESS-1) 
#define COPY_2_END_ADDRESS				0xFF7F
#define FLASH_START_ADDRESS				0x8000
/*  Maximum # of sectors sent in one programming command  */
#define MAX_FLASH_SECTORS_AT_A_TIME		20
/*  Packet ID  */
typedef int IN_PACKET_ID;
#define PACKET_ID_PEN1 0
#define PACKET_ID_PEN2 1
#define PACKET_ID_PENERR 2
#define PACKET_ID_PENXY 3
#define PACKET_ID_SERIALDATA 7
#define PACKET_ID_PENXY_NEWMKR 8
#define PACKET_ID_CMDSTAT 0x10
#define PACKET_ID_DATAIN 0x11
//#define PACKET_ID_NEW_MARKER_PEN 0x08

typedef enum {PACKET_ID_COMMAND=1, PACKET_ID_DATAOUT=2} OUT_PACKET_ID;
/*  Commands  */
typedef int COMMAND;
#define COMMAND_RESTART				0x00
#define COMMAND_GETLASTSTAT			0x01 
#define COMMAND_GETVERSION			0x02
#define COMMAND_GETSERNUM			0x03 
#define COMMAND_READMEM				0x04
#define COMMAND_SETPENDATASEND		0x05 
#define COMMAND_MEASURENOISE		0x06
#define COMMAND_GETSTATS			0x07
#define COMMAND_GETERRSTATS			0x08
#define COMMAND_GETFLASHINFO		0x09
/*  GetReceiverSpacing for 2-pod system is 0x0a.  */
#define COMMAND_SETFLIPCHARTMODE	0x0b
/*  Passthrough command for Iport 2 is 0x0c  */
#define COMMAND_SETSERIALPASSTHROUGH 0x0d
#define CM_SET_GAINCTRL				0x16	// from SDK
#define CM_GET_GAINCTRL				0x17

#define COMMAND_SETLEDMODE			0x20

#define COMMAND_WRITETOSERIAL		0x0e
#define COMMAND_SETWRITEMODE		0x40
#define COMMAND_SETSERNUM			0x41 
#define COMMAND_WRITEFLASHCODE		0x42
#define COMMAND_WRITEFLASHHEADER	0x43 
#define COMMAND_ANDORMEM			0x81 
#define COMMAND_WRITEFLASHSECTOR	0x82 
#define COMMAND_SETACQSTATE			0x90
#define COMMAND_GETACQSTATE			0x91
#define COMMAND_GETIRDATA			0x95
#define COMMAND_GETUS1DATA			0x96
#define COMMAND_GETUS2DATA			0x97
#define COMMAND_GETHEADER2CHECKSUM  0x98
#define COMMAND_GETCOMMSTATE		0x99
#define NO_COMMAND					0xff


void ReturnCommStatus(COMMAND cmd, ERROR error);
void SendStatusPacket();
void NewCommand(BYTE *commandBuffer);
ERROR programFlash(COMMAND cmd);

/*  Serial number info  */
#define NUM_SERIAL_NUM_PACKETS	4
#define SERIAL_NUMBER_LENGTH	(NUM_SERIAL_NUM_PACKETS * (PACKET_LENGTH-1)) 

#endif

