/*----------------------------------------------------------------------
	Module:  asmsyms.h
	Definition of symbols used by assembly and C code.
		
	Copyright (C) 2000 Electronics For Imaging
----------------------------------------------------------------------*/
#ifndef _ASMSYMS_H
#define _ASMSYMS_H
#include "sysdef.h"
#include "dspdef.h"
#include "structs.h"
#include "error.h"
#include "version.h"
#include "timer.h"
#include "commands.h"
#include "colldata.h"
#include "usb_func.h"

/*  The offsets into this structure must be kept consistant with the C declaration  */
#define OFFSET_INDEX 					0
#define OFFSET_ADDRESS 					1
#define OFFSET_ENERGY 					2
#define OFFSET_ENERGY_MAX 				4
#define SIZE_OF_SEGMENT_DESCRIPT_STRUCT 6

#ifdef MAKE_ASM_SYMBOL_FILE
/*  This is a clever trick I found in a TI technical note.  */
#define ASM_ASG(symbol) asm("\t.asg\t" VAL(symbol) "," #symbol)
#define ASM_SET(symbol) asm(#symbol " .set " VAL(symbol) )
#define VAL(symbol) #symbol


ASM_SET(IR_SCANNING);
ASM_SET(IR_TRACKING);
ASM_SET(IR_PLS12_DIS);
ASM_SET(IR_PLS12_DIS_NEWMKR);
ASM_SET(IR_TICKS_PER_SAMPLE);
ASM_SET(IR_SPACE_BIAS);
ASM_SET(IR_PLS13_DIS);
ASM_SET(IR_PLS13_DIS_NEWMKR);
ASM_SET(IR_START_US_TIME);
//ASM_SET(MAX_IR_SPACING_PROJECTION_1_3)
ASM_SET(IR_PACKET_PERIOD);
ASM_SET(IR_PACKET_PERIOD_HI);
ASM_SET(IR_PACKET_PERIOD_LO);
ASM_SET(IR_START_US_TIME_NEWMKR);
ASM_SET(MAX_IR_SPACING_NEWMKR_1_3);
ASM_SET(IR_PACKET_PERIOD_NEWMKR);
ASM_SET(IR_PACKET_PERIOD_NEWMKR_HI);
ASM_SET(IR_PACKET_PERIOD_NEWMKR_LO);
ASM_SET(IR_WIN_END_OFFSET);
ASM_SET(IR_WIN_END_OFFSET_NEWMKR);
ASM_SET(IR_US_START_OFFSET);
ASM_SET(PEN_UP_INDICATION);
ASM_SET(IRPLL_NEWMKR);
ASM_SET(IR_PLS14_DIS);
ASM_SET(IR_PLS15_DIS);
ASM_SET(IR_PLS16_DIS);
ASM_SET(IR_PLS17_DIS);
ASM_SET(IRPLL_PENTYP);

ASM_SET(NEWMKR_CODE_RED);
ASM_SET(NEWMKR_CODE_GRN);
ASM_SET(NEWMKR_CODE_BLU);
ASM_SET(NEWMKR_CODE_PUP);
ASM_SET(NEWMKR_CODE_BLK);
ASM_SET(NEWMKR_CODE_ERS);
ASM_SET(NEWMKR_CODE_LWB);

ASM_SET(IR_PLS_DIS_ERR_ALLW);


ASM_SET(WAVELEN);
ASM_SET(FIR_LENGTH);
ASM_SET(IR_SPACING_MIN);
ASM_SET(IR_SPACING_MAX);
ASM_SET(IR_SPACING_MAX_UBOARD);
ASM_SET(MAX_PULSE_WIDTH);
ASM_SET(MAX_PULSE_WIDTH_UBOARD);
ASM_SET(MIN_IR_SPACING_PROJECTION_1_2);
ASM_SET(MIN_IR_SPACING_PROJECTION_1_3);
ASM_SET(MAX_IR_SPACING_PROJECTION_1_3);
ASM_SET(NUM_IR_DATA_PULSES);
ASM_SET(IR_DATA_CELL_WIDTH);
ASM_SET(IR_DATA_PULSE_WIDTH);
ASM_SET(SAMPLES_BETWEEN_LAST_CELL_AND_US);
ASM_SET(PROJECTION_PEN_DETECTED);
ASM_SET(NUM_US_SAMPLES);               
ASM_SET(IR_BUFFER_LEN);
ASM_SET(IR_TO_US_SPACING);
ASM_SET(IR_DETECTION_DELAY_MAX);
ASM_SET(SIZE_OF_SEGMENT_DESCRIPT_STRUCT);
ASM_SET(SEGMENT_SIZE);
ASM_SET(NUMBER_OF_SEGMENT_BUFFERS);
ASM_SET(OVERLAY_SEGMENT_ALLOCATION);
ASM_SET(OFFSET_INDEX);
ASM_SET(OFFSET_ADDRESS);
ASM_SET(OFFSET_ENERGY);
ASM_SET(OFFSET_ENERGY_MAX);
ASM_SET(ENABLED);
ASM_SET(DISABLED);
ASM_SET(VERSION_BASE);
ASM_SET(VERSION_SUB);
ASM_SET(BUILD_NUMBER);
ASM_SET(BETA_FLAG);
ASM_SET(PACKET_ID_DATAIN);
ASM_SET(PACKET_ID_PENERR);
ASM_SET(VERSION_CHAR_0);
ASM_SET(VERSION_CHAR_1);
ASM_SET(VERSION_CHAR_2);
ASM_SET(VERSION_CHAR_3);
ASM_SET(VERSION_CHAR_4);
ASM_SET(VERSION_CHAR_5);
ASM_SET(VERSION_CHAR_6);
ASM_SET(VERSION_CHAR_7);
ASM_SET(VERSION_CHAR_8);
#if	( BETA_FLAG != 0 )
ASM_SET(VERSION_CHAR_9);
#endif

//  DMA - 55x
ASM_SET(DMACSR4);
ASM_SET(DMACCR4);
ASM_SET(DMACDAC4);
ASM_SET(DMACEN5);
ASM_SET(DMACCR5);
ASM_SET(IER1);
ASM_SET(DMACCR2);
ASM_SET(DMACCR3);
ASM_SET(DMA3_INT_BIT);
ASM_SET(TOTAL_SEGMENTS_TO_READ);

//  ADC - 55x
ASM_SET(ADC_ADDRESS);
ASM_SET(DC_US);

/* Timers -- 55x*/
ASM_SET(WDTIMER_BASE);
ASM_SET(GPTCNT3);
ASM_SET(GPTCNT4);

/*  Initialization  */
ASM_SET(IVPDaddr);
ASM_SET(IVPD_VALUE);
/*  Constants for overall performance  */
ASM_SET(INLINE_US_FILTER);
/*  Error codes  */
ASM_SET(DEF_NOERR);
ASM_SET(DEF_ERR_NOPEAKFND);
ASM_SET(DEF_ERR_SEG_DISC);
ASM_SET(DEF_ERR_INVLDCMD);
ASM_SET(DEF_ERR_POINTDISCARDED);
/*  Commands  */
ASM_SET(COMMAND_RESTART);
ASM_SET(COMMAND_GETLASTSTAT); 
ASM_SET(COMMAND_GETVERSION);
ASM_SET(COMMAND_GETSERNUM); 
ASM_SET(COMMAND_READMEM);
ASM_SET(COMMAND_SETPENDATASEND); 
ASM_SET(COMMAND_MEASURENOISE);
ASM_SET(COMMAND_GETERRSTATS);
ASM_SET(COMMAND_GETFLASHINFO); 
ASM_SET(COMMAND_SETWRITEMODE);
ASM_SET(COMMAND_SETSERNUM); 
ASM_SET(COMMAND_WRITEFLASHCODE);
ASM_SET(COMMAND_WRITEFLASHHEADER); 
ASM_SET(COMMAND_ANDORMEM); 
ASM_SET(COMMAND_WRITEFLASHSECTOR);
ASM_SET(COMMAND_SETFLIPCHARTMODE);
ASM_SET(COMMAND_WRITETOSERIAL);
ASM_SET(COMMAND_SETSERIALPASSTHROUGH);
ASM_SET(NO_COMMAND);
ASM_SET(PACKET_LENGTH);
ASM_SET(D11_CONTROL_OUT_ENDPOINT_OFFSET);
ASM_SET(D12_COMMAND_ADDRESS);
ASM_SET(D12_DATA_ADDRESS);
ASM_SET(CPU_CLOCK_FREQUENCY);
ASM_SET(GPTIMER1_BASE);
ASM_SET(COPY_NUMBER_ADDRESS);
ASM_SET(NOMINAL_IR_THRESHOLD);
#endif
#endif

