/*----------------------------------------------------------------------
	Module:  utldef.h

	Shared types & macro definitions.  
			
	Copyright (C) 2000 Electronics For Imaging
	Copyright (C) 2004 Luidia Inc.
----------------------------------------------------------------------*/

#ifndef _COMDEF_H
#define _COMDEF_H

#define ENABLED			1
#define DISABLED		0
#define TRUE			1
#define	FALSE			0

#ifdef _DEBUG
	#define ASSERT(_a)	if (!(_a)) { while(1); }
#else
	#define ASSERT(_a)	
#endif
#define LOCK(_b)		ASSERT(!(_b)); _b = 1
#define RELEASE(_b)		ASSERT( (_b)); _b = 0

// I/O space access
#define IOPORT(_p) 		(*(ioport unsigned int *)(_p))

// memory maped registers access
#define MMR(_r)			(*(volatile unsigned int *)(_r))

//  Global interrupt enable/disable
#define INT_DISABLE		_disable_interrupts()
#define INT_ENABLE		_enable_interrupts()
//Save/Restore interrupt registers
#define INT_SAVE(R0, R1) 	R0 = MMR( IER0), R1 = MMR( IER1)
#define INT_MASK(M0, M1)	MMR( IER0) &= M0, MMR( IER1) &= M1
#define INT_RESTORE(R0, M0, R1, M1) MMR( IER0) |= (R0 & ~M0) , MMR( IER1) |= (R1 & ~M1)

/*  Toggle XF line  */
#define XF_BIT			( MMR(ST1_55) & 0x2000 )
#define	SET_XF			asm(" BSET	XF")   
#define CLR_XF			asm(" BCLR	XF")

//max/min value
#define MAX(a,b)		((a)>(b))? (a):(b)
#define MIN(a,b)		((a)<(b))? (a):(b)
#define CYRC_INC(a,p)	a = (++a >= p)? a - p : a
#define CHK_BIT(bf, n)	!!(bf & (1<<n))
#define SET_BIT(bf, n)	(bf |=  (1<<n))
#define CLR_BIT(bf, n)	(bf &= ~(1<<n))

/* Round floating point value to integer */
#define ROUND_TO_INT(v) ((Int16) ((v>0)?(v+0.5):(v-0.5)))

////////////////////////////
//  Type definitions

typedef ioport unsigned int*  ioport_t;
typedef unsigned int 	BOOL;
typedef int 			BYTE;
typedef unsigned short 	UInt16;
typedef unsigned long 	UInt32;
typedef int 			Int16; 	// making this short produces errors in compilation time
typedef long 			Int32;
typedef double			Float32;

#define LSB(a)	(0x00FF&(a))
#define MSB(a)	((a)>>8)

#endif // _COMDEF_H



