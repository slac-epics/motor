/*
FILENAME: motordrvComCode.h
USAGE... This file contains local variables that are allocated
	in each motor record driver.  The variables are allocated
	in each driver by including this file.

Version:	1.3
Modified By:	sluiter
Last Modified:	2002/10/31 20:46:17
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 08/20/99
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contract
 *      W-31-109-ENG-38 at Argonne National Laboratory.
 *
 *      Beamline Controls & Data Acquisition Group
 *      Experimental Facilities Division
 *      Advanced Photon Source
 *      Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 */

#ifndef	INCmotordrvComCode
#define	INCmotordrvComCode 1

#include <epicsTypes.h>
#include <epicsEvent.h>

/* --- Local data common to each driver. --- */
static int total_cards;
static struct controller **motor_state;
static struct circ_queue **mess_queue;               /* in message queue head */
static struct circ_queue **free_list;
static epicsEvent **queue_lock;
static epicsEvent **freelist_lock;
static epicsEvent **motor_sem;
static bool* initialized;                     /* Driver initialized indicator */

#endif	/* INCmotordrvComCode */
