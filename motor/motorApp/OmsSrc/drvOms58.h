/*
FILENAME...	drvOms58.h
USAGE...	OMS driver level "include" information that is specific to OMS
		model VME58.

Version:	1.3
Modified By:	sluiter
Last Modified:	2001/12/14 20:53:00
*/

/*
 *      Original Author: Jim Kowalkowski
 *      Current Author: Joe Sullivan
 *      Date: 11/14/94
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the Regents of the University of California,
 *      and the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contracts:
 *      (W-7405-ENG-36) at the Los Alamos National Laboratory,
 *      and (W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *      Initial development by:
 *	      The Controls and Automation Group (AT-8)
 *	      Ground Test Accelerator
 *	      Accelerator Technology Division
 *	      Los Alamos National Laboratory
 *
 *      Co-developed with
 *	      The Controls and Computing Group
 *	      Accelerator Systems Division
 *	      Advanced Photon Source
 *	      Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01  01-18-93	jbk     initialized
 * .02  11-14-94	jps     copy drvOms.c and modify to point to vme58 driver
 *      ...
 * .06  12-07-94	jps	first released version w/interrupt supprt
 * .07	12-20-94	jps	rearrange the device init routines
 * .08  05-03-96	jps     convert 32bit card accesses to 16bit - vme58PCB
 *				version D does not support 32bit accesses.
 * .09  05-09-97	jps     increase maximum card count to 15
 * .10  08-22-01	rls     "int" type specifications for all bit-fields.
 *  
 */

#ifndef INCdrvOms58h
#define INCdrvOms58h 1

#include "drvOmsCom.h"

/*
 * VME58 default profile
 */

#define OMS_NUM_CARDS           15
#define OMS_NUM_CHANNELS        8
#define OMS_NUM_ADDRS           0x4000
#define OMS_BRD_SIZE            0x1000	/* card address boundary */

#define BUFFER_SIZE           256

/* Board control register structures */


/* VME58 DUAL-PORT MEMORY MAP */
typedef struct
{
    unsigned short encPos[2];
    unsigned short cmndPos[2];
    unsigned short cmndVel[2];
    unsigned short accel[2];
    unsigned short maxVel[2];
    unsigned short baseVel[2];
    unsigned short dFltrGain[2];
    unsigned short dFltrPole[2];
    unsigned short dFltrZero[2];
    unsigned short reserved[46];
} MOTOR_DATA_REGS;

/* Definitions for VME58 I/O Registers */

/* Control Register - Offset = 0x0FE1 */
typedef union
{
    unsigned char All;
    struct
    {
	unsigned int intReqEna	:1;	/* Master interrupt request enable */
	unsigned int ioIntEna	:1;	/* I/O bits 0 and 1 interrupt enable */
	unsigned int directInEna:1;	/* Interrupt request to the VME58 ? */
	unsigned int doneIntEna	:1;	/* Done detect interrupt enable */
	unsigned int otIntEna	:1;	/* Overtravel detect interrupt enable */
	unsigned int slipIntEna	:1;	/* Encoder slip detect interrupt enable */
	unsigned int		:1;	/* Unused  */
	unsigned int update	:1;	/* Data area update request */
    } Bits;
} CNTRL_REG;

/* Status Register - Offset = 0x0FE3 */
typedef union
{
    unsigned char All;
    struct
    {
	unsigned int interrupt	:1;	/* Interrupt dectect */
	unsigned int directIn	:1;	/* Direct input interrupt detect */
	unsigned int directOut	:1;	/* Direct ouput interrupt detect */
	unsigned int done	:1;	/* Motion done detect */
	unsigned int overtravel	:1;	/* Overtravel detect */
	unsigned int encoderSlip:1;	/* Encoder slip detect */
	unsigned int cardOK	:1;	/* Powerup initilization complete */
	unsigned int cmndError	:1;	/* Command error dectect */
    } Bits;
} STATUS_REG;

/* I/O Register(0-7) -  Offset = 0x0FE5 */
typedef union
{
    unsigned char All;
    struct
    {
	unsigned int io_7	:1;	/* Bit 7 */
	unsigned int io_6	:1;	/* Bit 6 */
	unsigned int io_5	:1;	/* Bit 5 */
	unsigned int io_4	:1;	/* Bit 4 */
	unsigned int io_3	:1;	/* Bit 3 */
	unsigned int io_2	:1;	/* Bit 2 */
	unsigned int io_1	:1;	/* Bit 1 */
	unsigned int io_0	:1;	/* Bit 0 */
    } Bits;
} IO_LOW_REG;

/* Slip Flag Register - Offset = 0x0FE7 */
typedef union
{
    unsigned char All;
    struct
    {
	unsigned int slip_s	:1;	/* status of S axis */
	unsigned int slip_r	:1;	/* status of R axis */
	unsigned int slip_v	:1;	/* status of V axis */
	unsigned int slip_u	:1;	/* status of U axis */
	unsigned int slip_t	:1;	/* status of T axis */
	unsigned int slip_z	:1;	/* status of Z axis */
	unsigned int slip_y	:1;	/* status of Y axis */
	unsigned int slip_x	:1;	/* status of X axis */
    } Bits;
} SLIP_REG;

/* Done Flag Register - Offset = 0x0FE9 */
typedef union
{
    unsigned char All;
    struct
    {
	unsigned int done_s	:1;	/* status of S axis */
	unsigned int done_r	:1;	/* status of R axis */
	unsigned int done_v	:1;	/* status of V axis */
	unsigned int done_u	:1;	/* status of U axis */
	unsigned int done_t	:1;	/* status of T axis */
	unsigned int done_z	:1;	/* status of Z axis */
	unsigned int done_y	:1;	/* status of Y axis */
	unsigned int done_x	:1;	/* status of X axis */
    } Bits;
} DONE_REG;

/* I/O High Register(8-13) - Offset = 0x0FEB */
typedef union
{
    unsigned char All;
    struct
    {
	unsigned int		:1;	/* Unused */
	unsigned int		:1;	/* Unused */
	unsigned int io_13	:1;	/* Bit 13 */
	unsigned int io_12	:1;	/* Bit 12 */
	unsigned int io_11	:1;	/* Bit 11 */
	unsigned int io_10	:1;	/* Bit 10 */
	unsigned int io_9	:1;	/* Bit 9  */
	unsigned int io_8	:1;	/* Bit 8  */
    } Bits;
} IO_HIGH_REG;


/* Limit Switch Status Register - Offset = 0x0FED */
typedef union
{
    unsigned char All;
    struct
    {
	unsigned int limit_s	:1;	/* status of S axis */
	unsigned int limit_r	:1;	/* status of R axis */
	unsigned int limit_v	:1;	/* status of V axis */
	unsigned int limit_u	:1;	/* status of U axis */
	unsigned int limit_t	:1;	/* status of T axis */
	unsigned int limit_z	:1;	/* status of Z axis */
	unsigned int limit_y	:1;	/* status of Y axis */
	unsigned int limit_x	:1;	/* status of X axis */
    } Bits;
} LIMIT_REG;

/* Home Switch Status Register - Offset = 0x0FEF */
typedef union
{
    unsigned char All;
    struct
    {
	unsigned int home_s	:1;	/* status of S axis */
	unsigned int home_r	:1;	/* status of R axis */
	unsigned int home_v	:1;	/* status of V axis */
	unsigned int home_u	:1;	/* status of U axis */
	unsigned int home_t	:1;	/* status of T axis */
	unsigned int home_z	:1;	/* status of Z axis */
	unsigned int home_y	:1;	/* status of Y axis */
	unsigned int home_x	:1;	/* status of X axis */
    } Bits;
} HOME_REG;

typedef struct
{
    unsigned char unused00;
    unsigned char cntrlReg;	/* Control Register - Read/Write */
    unsigned char unused02;
    unsigned char statusReg;	/* Status Register  - Read */
    unsigned char unused04;
    unsigned char ioLowReg;	/* IO bits 0-7 status register   - Read */
    unsigned char unused06;
    unsigned char slipReg;		/* Encoder slip status register - Read */
    unsigned char unused08;
    unsigned char doneReg;		/* Axis done status register - Read */
    unsigned char unused0A;
    unsigned char ioHighReg;	/* IO bits 8-13 status register - Read */
    unsigned char unused0C;
    unsigned char limitReg;	/* Limit switch  status register - Read */
    unsigned char unused0E;
    unsigned char homeReg;		/* Home switch  status register - Read */
    unsigned char unusedF0;
    unsigned char intVector;	/* Interrupt vector */
} MOTOR_CNTRL_REGS;


/* OMS VME dual port memory map */
struct vmex_motor
{
    unsigned short inPutIndex;
    unsigned short outGetIndex;
    unsigned short inBuffer[BUFFER_SIZE];
    unsigned short reserved0[254];
    MOTOR_DATA_REGS data[OMS_NUM_CHANNELS];
    unsigned short outPutIndex;
    unsigned short inGetIndex;
    unsigned short outBuffer[BUFFER_SIZE];
    unsigned short reserved1[750];
    MOTOR_CNTRL_REGS control;
};

#endif	/* INCdrvOms58h */
