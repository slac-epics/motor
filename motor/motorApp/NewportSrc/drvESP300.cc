/*
FILENAME...	drvESP300.cc
USAGE...	Motor record driver level support for Newport ESP300.

Version:	1.8
Modified By:	sluiter
Last Modified:	2004/03/16 15:17:18
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 02/19/03
 *	Current Author: Ron Sluiter
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
 * .01 02-19-03 rls copied from drvMM3000
 * .02 05-23-03	rls Converted to R3.14.x.
 * .03 10-28-03 rls initialize "drive_resolution".
 * .04 02-03-04 rls Eliminate erroneous "Motor motion timeout ERROR".
 */


#include <string.h>
#include <epicsThread.h>
#include <drvSup.h>
#include "motor.h"
#include "NewportRegister.h"
#include "drvMMCom.h"
#include "serialIO.h"
#include "epicsExport.h"

#define READ_POSITION   "%.2dTP"
#define STOP_AXIS	"%.2dST"
#define GET_IDENT       "VE?"

#define ESP300_NUM_CARDS	4
#define BUFF_SIZE 100       /* Maximum length of string to/from ESP300 */

/* The ESP300 does not respond for 2 to 5 seconds after hitting a travel limit. */
#define GPIB_TIMEOUT	5000	/* Command timeout in msec. */
#define SERIAL_TIMEOUT	5000	/* Command timeout in msec. */

/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef	DEBUG
	volatile int drvESP300debug = 0;
	#define Debug(l, f, args...) { if(l<=drvESP300debug) printf(f,## args); }
    #else
	#define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif

int ESP300_num_cards = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/*----------------functions-----------------*/
static int recv_mess(int, char *, int);
static RTN_STATUS send_mess(int card, char const *com, char c);
static int set_status(int card, int signal);
static long report(int level);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table ESP300_access =
{
    motor_init,
    motor_send,
    motor_free,
    motor_card_info,
    motor_axis_info,
    &mess_queue,
    &queue_lock,
    &free_list,
    &freelist_lock,
    &motor_sem,
    &motor_state,
    &total_cards,
    &any_motor_in_motion,
    send_mess,
    recv_mess,
    set_status,
    query_done,
    NULL,
    &initialized,
    NULL
};

struct
{
    long number;
#ifdef __cplusplus
    long (*report) (int);
    long (*init) (void);
#else
    DRVSUPFUN report;
    DRVSUPFUN init;
#endif
} drvESP300 = {2, report, init};

epicsExportAddress(drvet, drvESP300);

static struct thread_args targs = {SCAN_RATE, &ESP300_access};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (ESP300_num_cards <=0)
	printf("    No ESP300 controllers configured.\n");
    else
    {
	for (card = 0; card < ESP300_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    ESP300 controller %d connection failed.\n", card);
	    else
	    {
		struct MMcontroller *cntrl;

		cntrl = (struct MMcontroller *) brdptr->DevicePrivate;
		switch (cntrl->port_type)
		{
		case RS232_PORT: 
		    printf("    ESP300 controller %d port type = RS-232, id: %s \n", 
			   card, 
			   brdptr->ident);
		    break;
		case GPIB_PORT:
		    printf("    ESP300 controller %d port type = GPIB, id: %s \n", 
			   card, 
			   brdptr->ident);
		    break;
		default:
		    printf("    ESP300 controller %d port type = Unknown, id: %s \n", 
			   card, 
			   brdptr->ident);
		    break;
		}
	    }
	}
    }
    return(OK);
}


static long init()
{
   /* 
    * We cannot call motor_init() here, because that function can do GPIB I/O,
    * and hence requires that the drvGPIB have already been initialized.
    * That cannot be guaranteed, so we need to call motor_init from device
    * support
    */
    /* Check for setup */
    if (ESP300_num_cards <= 0)
    {
	Debug(1, "init(): ESP300 driver disabled. ESP300Setup() missing from startup script.\n");
    }
    return((long) 0);
}


static void query_done(int card, int axis, struct mess_node *nodeptr)
{
}


/**************************************************************
 * Parse status and position strings for a card and signal
 * set_status()
 ************************************************************/

static int set_status(int card, int signal)
{
    struct MMcontroller *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    /* Message parsing variables */
    char *cptr, *tok_save;
    char inbuff[BUFF_SIZE], outbuff[BUFF_SIZE];
    int rtn_state, charcnt;
    long mstatus;
    double motorData;
    bool power, done, plusdir, ls_active = false;
    msta_field status;

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    sprintf(outbuff, "%.2dMD", signal + 1);
    send_mess(card, outbuff, (char) NULL);
    charcnt = recv_mess(card, inbuff, 1);
    if (charcnt == 3)
    {
	cntrl->status = NORMAL;
	status.Bits.CNTRL_COMM_ERR = 0;
    }
    else
    {
	if (cntrl->status == NORMAL)
	{
	    cntrl->status = RETRY;
	    rtn_state = 0;
	    goto exit;
	}
	else
	{
	    cntrl->status = COMM_ERR;
	    status.Bits.CNTRL_COMM_ERR = 1;
	    status.Bits.RA_PROBLEM     = 1;
	    rtn_state = 1;
	    goto exit;
	}
    }
    
    done = atoi(inbuff) ? true : false;
    status.Bits.RA_DONE = (done == true) ? 1 : 0;

    /* Get motor position. */
    sprintf(outbuff, READ_POSITION, signal + 1);
    send_mess(card, outbuff, (char) NULL);
    charcnt = recv_mess(card, inbuff, 1);
    
    motorData = atof(inbuff) / cntrl->drive_resolution[signal];

    if (motorData == motor_info->position)
    {
	if (nodeptr != 0)	/* Increment counter only if motor is moving. */
	    motor_info->no_motion_count++;
    }
    else
    {
	epicsInt32 newposition;

	newposition = NINT(motorData);
	status.Bits.RA_DIRECTION = (newposition >= motor_info->position) ? 1 : 0;
	motor_info->position = newposition;
	motor_info->no_motion_count = 0;
    }

    plusdir = (status.Bits.RA_DIRECTION) ? true : false;

    /* Get travel limit switch status. */
    sprintf(outbuff, "%.2dPH", signal + 1);
    send_mess(card, outbuff, (char) NULL);
    charcnt = recv_mess(card, inbuff, 1);
    cptr = strchr(inbuff, 'H');
    if (cptr == NULL)
    {
	Debug(2, "set_status(): PH error = %s\n", inbuff);
	rtn_state = 1;
	goto exit;
    }
    mstatus = strtol(inbuff, &cptr, 16);

    /* Set Travel limit switch status bits. */
    if (((mstatus >> signal) & 0x01) == false)
	status.Bits.RA_PLUS_LS = 0;
    else
    {
	status.Bits.RA_PLUS_LS = 1;
	if (plusdir == true)
	    ls_active = true;
    }

    if (((mstatus >> (signal + 8)) & 0x01) == false)
	status.Bits.RA_MINUS_LS = 0;
    else
    {
	status.Bits.RA_MINUS_LS = 1;
	if (plusdir == false)
	    ls_active = true;
    }

    /* Set home switch status. */
    cptr += 2;
    tok_save = strchr(inbuff, 'H');
    mstatus = strtol(cptr, &tok_save, 16);

    status.Bits.RA_HOME = ((mstatus >> signal) & 0x01) ? 1 : 0;

    /* Get motor power on/off status. */
    sprintf(outbuff, "%.2dMO?", signal + 1);
    send_mess(card, outbuff, (char) NULL);
    charcnt = recv_mess(card, inbuff, 1);
    power = atoi(inbuff) ? true : false;

    status.Bits.EA_POSITION = (power == true) ? 1 : 0;

    /* encoder status */
    status.Bits.EA_SLIP		= 0;
    status.Bits.EA_SLIP_STALL	= 0;
    status.Bits.EA_HOME		= 0;
    status.Bits.RA_PROBLEM	= 0;

    /* Parse motor velocity? */
    /* NEEDS WORK */

    motor_info->velocity = 0;

    if (!status.Bits.RA_DIRECTION)
	motor_info->velocity *= -1;

    rtn_state = (!motor_info->no_motion_count || ls_active == true ||
		 status.Bits.RA_DONE | status.Bits.RA_PROBLEM) ? 1 : 0;

    /* Test for post-move string. */
    if ((status.Bits.RA_DONE || ls_active == true) && nodeptr != 0 &&
	nodeptr->postmsgptr != 0)
    {
	strcpy(outbuff, nodeptr->postmsgptr);
	send_mess(card, outbuff, (char) NULL);
	nodeptr->postmsgptr = NULL;
    }

exit:
    motor_info->status.All = status.All;
    return (rtn_state);
}


/*****************************************************/
/* send a message to the ESP300 board		     */
/* send_mess()			                     */
/*****************************************************/
static RTN_STATUS send_mess(int card, char const *com, char inchar)
{
    struct MMcontroller *cntrl;
    char local_buff[BUFF_SIZE];
    int size;

    size = strlen(com);

    if (size > MAX_MSG_SIZE)
    {
	errlogMessage("drvESP300:send_mess(); message size violation.\n");
	return(ERROR);
    }
    else if (size == 0)	/* Normal exit on empty input message. */
	return(OK);
    
    if (!motor_state[card])
    {
	errlogPrintf("drvESP300:send_mess() - invalid card #%d\n", card);
	return(ERROR);
    }

    if (inchar != (char) NULL)
    {
	errlogPrintf("drvESP300:send_mess() - invalid argument = %c\n", inchar);
	return(ERROR);
    }

    /* Make a local copy of the string and add the command line terminator. */
    strcpy(local_buff, com);
    strcat(local_buff, "\r");
    Debug(2, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

    if (cntrl->port_type == GPIB_PORT)
	;
//	gpibIOSend(cntrl->gpibInfo, local_buff, strlen(local_buff), GPIB_TIMEOUT);
    else        
	cntrl->serialInfo->serialIOSend(local_buff, strlen(local_buff), SERIAL_TIMEOUT);
    
    return(OK);
}


/*
 * FUNCTION... recv_mess(int card, char *com, int flag)
 *
 * INPUT ARGUMENTS...
 *	card - controller card # (0,1,...).
 *	*com - caller's response buffer.
 *	flag	| FLUSH  = flush controller's output buffer; set timeout = 0.
 *		| !FLUSH = retrieve response into caller's buffer; set timeout.
 *
 * LOGIC...
 *  Initialize:
 *	- receive timeout to zero
 *	- received string length to zero.
 *
 *  IF controller card does not exist.
 *	ERROR RETURN.
 *  ENDIF
 *
 *  SWITCH on port type.
 *	CASE port type is GPIB.
 *	    BREAK.
 *	CASE port type is RS232.
 *	    IF input "flag" indicates NOT flushing the input buffer.
 *		Set receive timeout nonzero.
 *	    ENDIF
 *	    Call serialIORecv().
 *
 *	    NOTE: The ESP300 sometimes responds to an MS command with an error
 *		message (see ESP300 User's Manual Appendix A).  This is an
 *		unconfirmed ESP300 bug.  Retry read if this is a Hard Travel
 *		limit switch error. This effectively flushes the error message.
 *
 *	    IF input "com" buffer length is > 3 characters, AND, the 1st
 *			character is an "E" (Maybe this an unsolicited error
 *			message response?).
 *	    	Call serialIORecv().
 *	    ENDIF
 *	    BREAK
 *    ENDSWITCH
 *		
 *  NORMAL RETURN.
 */

static int recv_mess(int card, char *com, int flag)
{
    struct MMcontroller *cntrl;
    int timeout = 0;
    int len = 0;

    /* Check that card exists */
    if (!motor_state[card])
	return(ERROR);

    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

    switch (cntrl->port_type)
    {
	case GPIB_PORT:
	    if (flag != FLUSH)
		timeout	= GPIB_TIMEOUT;
//	    len = gpibIORecv(cntrl->gpibInfo, com, BUFF_SIZE, (char *) "\n", timeout);
	    break;
	case RS232_PORT:
	    if (flag != FLUSH)
		timeout	= SERIAL_TIMEOUT;
	    len = cntrl->serialInfo->serialIORecv(com, BUFF_SIZE, (char *) "\n", timeout);
	    if (len > 3 && com[0] == 'E')
	    {
		long error;

		error = strtol(&com[1], NULL, 0);
		if (error >= 35 && error <= 42)
		    len = cntrl->serialInfo->serialIORecv(com, BUFF_SIZE, (char *) "\n", timeout);
	    }
	    break;
    }

    if (len <= 0)
    {
	com[0] = '\0';
	len = 0;
    }
    else
	/* ESP300 responses are always terminated with CR/LF combination (see
	 * ESP300 User' Manual Sec. 3.4 NOTE). Strip both CR&LF from buffer
	 * before returning to caller.
	 */
	com[len-2] = '\0';

    Debug(2, "recv_mess(): message = \"%s\"\n", com);
    return(len);
}


/*****************************************************/
/* Setup system configuration                        */
/* ESP300Setup()                                     */
/*****************************************************/
RTN_STATUS
ESP300Setup(int num_cards,	/* maximum number of controllers in system.  */
	    int num_channels,	/* NOT Used. */
	    int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > ESP300_NUM_CARDS)
	ESP300_num_cards = ESP300_NUM_CARDS;
    else
	ESP300_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

   /* 
    * Allocate space for motor_state structures.  Note this must be done
    * before ESP300Config is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem
    */
    motor_state = (struct controller **) malloc(ESP300_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < ESP300_num_cards; itera++)
	motor_state[itera] = (struct controller *) NULL;

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* ESP300Config()                                    */
/*****************************************************/
RTN_STATUS
ESP300Config(int card,	/* card being configured */
            PortType port_type,	/* GPIB_PORT or RS232_PORT */
	    int location,       /* = link for GPIB or MPF serial server location */
            const char *name)   /* GPIB address or MPF serial server task name */
{
    struct MMcontroller *cntrl;

    if (card < 0 || card >= ESP300_num_cards)
        return (ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct MMcontroller));
    cntrl = (struct MMcontroller *) motor_state[card]->DevicePrivate;

    switch (port_type)
    {
/*
    case GPIB_PORT:
        cntrl->port_type = port_type;
        cntrl->gpib_link = addr1;
        cntrl->gpib_address = addr2;
        break;
*/
    case RS232_PORT:
        cntrl->port_type = port_type;
        cntrl->serial_card = location;
        strcpy(cntrl->serial_task, name);
        break;
    default:
        return (ERROR);
    }
    return(OK);
}



/*****************************************************/
/* initialize all software and hardware		     */
/* This is called from the initialization routine in */
/* device support.                                   */
/* motor_init()			                     */
/*****************************************************/
static int motor_init()
{
    struct controller *brdptr;
    struct MMcontroller *cntrl;
    int card_index, motor_index;
    char buff[BUFF_SIZE];
    int total_axis = 0;
    int status;
    bool success_rtn;

    initialized = true;	/* Indicate that driver is initialized. */
    
    /* Check for setup */
    if (ESP300_num_cards <= 0)
	return(ERROR);

    for (card_index = 0; card_index < ESP300_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;

	brdptr = motor_state[card_index];
	brdptr->cmnd_response = false;
	total_cards = card_index + 1;
	cntrl = (struct MMcontroller *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	success_rtn = false;
	switch (cntrl->port_type)
	{
/*
	    case GPIB_PORT:
		cntrl->gpibInfo = gpibIOInit(cntrl->gpib_link,
					     cntrl->gpib_address);
		if (cntrl->gpibInfo == NULL)
		    success_rtn = true;
		break;
*/
	    case RS232_PORT:
		cntrl->serialInfo = new serialIO(cntrl->serial_card,
					     cntrl->serial_task, &success_rtn);
		break;
	}

	if (success_rtn == true)
	{
	    /* Send a message to the board, see if it exists */
	    /* flush any junk at input port - should not be any data available */
	    do
		recv_mess(card_index, buff, FLUSH);
	    while (strlen(buff) != 0);
    
	    send_mess(card_index, GET_IDENT, (char) NULL);
	    status = recv_mess(card_index, buff, 1);  
	    /* Return value is length of response string */
	}

	if (success_rtn == true && status > 0)
	{
	    brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;
	    strcpy(brdptr->ident, &buff[1]);  /* Skip "\n" */

	    send_mess(card_index, "ZU", (char) NULL);
	    recv_mess(card_index, buff, 1);
	    total_axis = buff[0] >> 4;
	    if (total_axis > 4)
	    {
		Debug(2, "motor_init(): ZU = %s\n", buff);
		total_axis = 4;
	    }

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		sprintf(buff, STOP_AXIS, motor_index + 1);	/* Stop motor */
		send_mess(card_index, buff, (char) NULL);
    		/* Initialize. */
		brdptr->motor_info[motor_index].motor_motion = NULL;
	    }

	    brdptr->total_axis = total_axis;

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];

		/* Set axis resolution. */
		sprintf(buff, "%.2dSU?", motor_index + 1);
		send_mess(card_index, buff, (char) NULL);
		recv_mess(card_index, buff, 1);
		cntrl->drive_resolution[motor_index] = atof(&buff[0]);
		
		motor_info->status.All = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;
		motor_info->encoder_present = YES;
                
		if (motor_info->encoder_present == YES)
		{
		    motor_info->status.Bits.EA_PRESENT = 1;
		    motor_info->pid_present = YES;
		    motor_info->status.Bits.GAIN_SUPPORT = 1;
		}

		set_status(card_index, motor_index);  /* Read status of each motor */
	    }
	}
	else
	    motor_state[card_index] = (struct controller *) NULL;
    }

    any_motor_in_motion = 0;

    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;

    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;

    epicsThreadCreate((char *) "ESP300_motor", 64, 5000, (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}

/*---------------------------------------------------------------------*/
