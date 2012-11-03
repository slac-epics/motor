/* File: drvMDrivePlus.cc
         Motor record driver level support for Intelligent Motion Systems, Inc.
         MDrivePlus series; M17, M23, M34.
   DESIGN LIMITATIONS...
         1 - Like all controllers, the MDrivePlus must be powered-on when EPICS is first booted up.
         2 - The MDrivePlus cannot be power cycled while EPICS is up and running.
             The consequences are permanent communication lose with the MDrivePlus until EPICS is rebooted.
*/

#include <string.h>
#include <epicsThread.h>
#include <drvSup.h>
#include "motor.h"
#include "motorRecord.h"
#include "drvIM483.h"
#include "asynOctetSyncIO.h"
#include "epicsExport.h"

#define MDrivePlus_NUM_CARDS  32
#define MAX_AXES    8
#define BUFF_SIZE  64          /* Maximum length of string to/from MDrivePlus */

/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef  DEBUG
    #define Debug(l, f, args...) {if (l <= drvMDrivePlusdebug) printf(f, ## args);}
    #else
    #define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif
volatile int drvMDrivePlusdebug = 0;
extern "C" {epicsExportAddress(int, drvMDrivePlusdebug);}

/* --- Local data. --- */
int MDrivePlus_num_cards = 0;
static char *MDrivePlus_axis[] = {"1", "2", "3", "4", "5", "6", "7", "8"};

static int   midx = -1;

/* Local data required for every driver; see "motordrvComCode.h" */
#include    "motordrvComCode.h"

/*----------------functions-----------------*/
static int recv_mess(int, char *, int);
static RTN_STATUS send_mess(int, char const *, char *);
static int set_status(int, int);
static long report(int);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table MDrivePlus_access =
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
    send_mess,
    recv_mess,
    set_status,
    query_done,
    NULL,
    initialized,
    MDrivePlus_axis
};

struct
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvMDrivePlus = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvMDrivePlus);}

static struct thread_args targs = {0, &MDrivePlus_access, 0.0};


/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (MDrivePlus_num_cards <=0)
    printf("    No MDrivePlus controllers configured.\n");
    else
    {
    for (card = 0; card < MDrivePlus_num_cards; card++)
    {
        struct controller *brdptr = motor_state[card];

        if (brdptr == NULL)
        printf("    MDrivePlus controller %d connection failed.\n", card);
        else
        {
        struct IM483controller *cntrl;

        cntrl = (struct IM483controller *) brdptr->DevicePrivate;
        printf("    MDrivePlus controller #%d, port=%s, id: %s \n", card,
               cntrl->asyn_port, brdptr->ident);
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
    if (MDrivePlus_num_cards <= 0)
    {
    Debug(1, "init(): MDrivePlus driver disabled. MDrivePlusSetup() missing from startup script.\n");
    }
    return((long) 0);
}


static void query_done(int card, int axis, struct mess_node *nodeptr)
{
}


/******************************************************************************
*
* FUNCTION NAME: set_status
*
* LOGIC:
*   Initialize.
*   Send "Moving Status" query.
*   Read response.
*   IF normal response to query.
*       Set communication status to NORMAL.
*   ELSE
*       IF communication status is NORMAL.
*           Set communication status to RETRY.
*           NORMAL EXIT.
*       ELSE
*           Set communication status error.
*           ERROR EXIT.
*       ENDIF
*   ENDIF
*
*   IF "Moving Status" indicates any motion (i.e. status != 0).
*       Clear "Done Moving" status bit.
*   ELSE
*       Set "Done Moving" status bit.
*   ENDIF
*
******************************************************************************/

epicsShareFunc int set_status(int card, int signal)
{
    struct IM483controller *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    /* Message parsing variables */
    char buff[BUFF_SIZE], fmt_str[32];
    int  rtn_state, nval, scan_rtn;
    int  mvval, stval, puval, ecode, lpval, lmval;
    long motorData;
    unsigned int mchb;
    bool plusdir, ls_active = false;

    msta_field status;

    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;
    input_config *confptr = cntrl->inconfig;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr    = motor_info->motor_motion;
    status.All = motor_info->status.All;

    recv_mess( card, buff, FLUSH );       // jsl - clear initial carriage return

    int retry = 0;

    while(1)
    {
        send_mess( card, "PR \"MV=\",MV,\",P=\",P,\",ST=\",ST", MDrivePlus_axis[signal] );
        rtn_state = recv_mess( card, buff, 1 );

        if ( rtn_state <= 0 )                                     // no response
        {
            recv_mess( card, buff, FLUSH );

            if (++retry > 3)
            {
                Debug(0, "set_status(): MDrivePlus no response for MV, P, ST for 3 tries, ERROR\n");
                cntrl->status = COMM_ERR;
                status.Bits.CNTRL_COMM_ERR = 1;
                status.Bits.RA_PROBLEM     = 1;
                rtn_state = 1;
                goto exit;
            }

            Debug(0, "set_status(): MDrivePlus no response for MV, P, ST retrying ...\n");
            epicsThreadSleep(0.5);
            continue;
        }

        scan_rtn = sscanf(buff, "MV=%d,P=%ld,ST=%d", &mvval, &motorData, &stval);

        if ( (scan_rtn == 3) &&
             (mvval == 1 || mvval == 0) && (stval == 0 || stval == 1) )
        {
            // Parse Position
            if ( motorData == motor_info->position )
            {
                if ( nodeptr != 0 )     /* Increment counter only when moving */
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

            cntrl->status              = NORMAL;

            status.Bits.RA_DONE        = 1 - mvval;
            status.Bits.RA_STALL       = stval;
            status.Bits.CNTRL_COMM_ERR = 0;
            status.Bits.RA_PROBLEM     = 0;
            break;
        }
        else                                                     // bad response
        {
            recv_mess( card, buff, FLUSH );

            if (++retry > 3)
            {
                Debug(0, "set_status(): MDrivePlus bad response for MV, P, ST for 3 tries, ERROR\n");
                cntrl->status              = COMM_ERR;
                status.Bits.CNTRL_COMM_ERR = 1;
                status.Bits.RA_PROBLEM     = 1;
                rtn_state = 1;
                goto exit;
            }

            Debug(0, "set_status(): MDrivePlus bad response for MV, P, ST, retrying ...\n");
            epicsThreadSleep(0.5);
        }
    }

    // No longer query home switch, not useful for MDrive homing sequence - jsl
    status.Bits.RA_HOME = 0;

    /*
     * Check limit switches, power-up flag, MCode heart-beat and error number
     */
    if ( ! status.Bits.RA_DONE ) /* Still moving, don't check Limit switch motor record could think motion done if LS active */
    {
        status.Bits.RA_PLUS_LS  = 0;
        status.Bits.RA_MINUS_LS = 0;
        ls_active               = false;
    }
    else
    {
        retry = 0;

        while(1)
        {
            if (confptr->plusLS == 0 && confptr->minusLS == 0)
            {
                status.Bits.RA_PLUS_LS  = 0;
                status.Bits.RA_MINUS_LS = 0;

                sprintf( buff, "PR \"PU=\",PU,\",HB=\",HB,\",ER=\",ER" );
            }
            else if (confptr->plusLS != 0 && confptr->minusLS != 0)
            {
                sprintf( buff, "PR \"PU=\",PU,\",HB=\",HB,\",ER=\",ER,\",I%d=\",I%d,\",I%d=\",I%d", confptr->plusLS, confptr->plusLS, confptr->minusLS, confptr->minusLS );
            }
            else if (confptr->plusLS != 0 && confptr->minusLS == 0)
            {
                status.Bits.RA_MINUS_LS = 0;

                sprintf( buff, "PR \"PU=\",PU,\",HB=\",HB,\",ER=\",ER,\",I%d=\",I%d", confptr->plusLS, confptr->plusLS);
            }
            else if (confptr->plusLS == 0 && confptr->minusLS != 0)
            {
                status.Bits.RA_PLUS_LS = 0;

                sprintf( buff, "PR \"PU=\",PU,\",HB=\",HB,\",ER=\",ER,\",I%d=\",I%d", confptr->minusLS, confptr->minusLS);
            }

            send_mess( card, buff, MDrivePlus_axis[signal] );
            rtn_state = recv_mess( card, buff, 1 );

            if ( rtn_state <= 0 )                                 // no response
            {
                recv_mess( card, buff, FLUSH );

                if (++retry > 3)
                {
                    Debug(0, "set_status(): MDrivePlus no response for PU, HB, ER etc for 3 tries, ERROR\n");
                    cntrl->status = COMM_ERR;
                    status.Bits.CNTRL_COMM_ERR = 1;
                    status.Bits.RA_PROBLEM     = 1;
                    rtn_state = 1;
                    goto exit;
                }

                Debug(0, "set_status(): MDrivePlus no response for PU, HB, ER etc, retrying ...\n");
                epicsThreadSleep(0.5);
                continue;
            }

            if (confptr->plusLS == 0 && confptr->minusLS == 0)
            {
                nval = 3;
                sprintf( fmt_str, "PU=%%d,HB=%%u,ER=%%d" );
                scan_rtn = sscanf( buff, fmt_str, &puval, &mchb, &ecode );
            }
            else if (confptr->plusLS != 0 && confptr->minusLS != 0)
            {
                nval = 5;
                sprintf( fmt_str, "PU=%%d,HB=%%u,ER=%%d,I%d=%%d,I%d=%%d", confptr->plusLS, confptr->minusLS );
                scan_rtn = sscanf( buff, fmt_str, &puval, &mchb, &ecode, &lpval, &lmval );
            }
            else if (confptr->plusLS != 0 && confptr->minusLS == 0)
            {
                nval = 4;
                sprintf( fmt_str, "PU=%%d,HB=%%u,ER=%%d,I%d=%%d", confptr->plusLS);
                scan_rtn = sscanf( buff, fmt_str, &puval, &mchb, &ecode, &lpval );
            }
            else if (confptr->plusLS == 0 && confptr->minusLS != 0)
            {
                nval = 4;
                sprintf( fmt_str, "PU=%%d,HB=%%u,ER=%%d,I%d=%%d", confptr->minusLS );
                scan_rtn = sscanf( buff, fmt_str, &puval, &mchb, &ecode, &lmval );
            }

            if ( (scan_rtn == nval) && (puval == 0 || puval == 1) &&
                 (confptr->plusLS  == 0 || lpval == 0 || lpval == 1) &&
                 (confptr->minusLS == 0 || lmval == 0 || lmval == 1)    )
            {
                status.Bits.RA_POWERUP = puval;

                /* the MCode heart-beat */
                if ( mchb <= motor_info->MCHB ) status.Bits.MCHB = 1;
                else                            status.Bits.MCHB = 0;
                motor_info->MCHB = mchb;

                /* the error code */
                ecode = ecode & 0xFF;
                if ( ( ecode == 83 ) || ( ecode == 84 ) ) status.Bits.ERRNO = 0;
                else                                      status.Bits.ERRNO = ecode;

                if ( confptr->plusLS  != 0 ) status.Bits.RA_PLUS_LS  = lpval;
                if ( confptr->minusLS != 0 ) status.Bits.RA_MINUS_LS = lmval;

                if ( status.Bits.RA_PLUS_LS  &&   plusdir ) ls_active = true;
                if ( status.Bits.RA_MINUS_LS && ! plusdir ) ls_active = true;

                cntrl->status              = NORMAL;
                status.Bits.CNTRL_COMM_ERR = 0;
                status.Bits.RA_PROBLEM     = 0;

                break;
            }
            else                                                 // bad response
            {
                recv_mess( card, buff, FLUSH );

                if (++retry > 3)
                {
                    Debug(0, "set_status(): MDrivePlus bad response for PU, HB, ER etc for 3 tries, ERROR\n");
                    cntrl->status              = COMM_ERR;
                    status.Bits.CNTRL_COMM_ERR = 1;
                    status.Bits.RA_PROBLEM     = 1;
                    rtn_state = 1;
                    goto exit;
                }

                Debug(0, "set_status(): MDrivePlus bad response for PU, HB, ER etc, retrying ...\n");
                epicsThreadSleep(0.5);
            }
        }
    }

    /* !!! Assume no closed-looped control!!!*/
    status.Bits.EA_POSITION   = 0;

    /* encoder status */
    status.Bits.EA_SLIP_STALL = 0;
    status.Bits.EA_HOME       = 0;

    /* Parse motor velocity? */
    /* NEEDS WORK */

    motor_info->velocity = 0;

    if (!status.Bits.RA_DIRECTION) motor_info->velocity *= -1;

    rtn_state = (!motor_info->no_motion_count || ls_active == true ||
                 status.Bits.RA_DONE | status.Bits.RA_PROBLEM) ? 1 : 0;

    /* Test for post-move string. */
    if ((status.Bits.RA_DONE || ls_active == true) && nodeptr != 0 &&
        nodeptr->postmsgptr != 0)
    {
        strcpy(buff, nodeptr->postmsgptr);
        send_mess(card, buff, MDrivePlus_axis[signal]);
        nodeptr->postmsgptr = NULL;
    }

exit:
    motor_info->status.All = status.All;
    return(rtn_state);
}


/*****************************************************/
/* send a message to the MDrivePlus board                */
/* send_mess()                                       */
/*****************************************************/
epicsShareFunc RTN_STATUS send_mess(int card, char const *com, char *name)
{
    char local_buff[MAX_MSG_SIZE];
    struct IM483controller *cntrl;
    const double timeout = 1.0; // jsl - use local variable instead of COMM_TIMEOUT in drvIM483.h
    int comsize, namesize;
    size_t nwrite;

    comsize = (com == NULL) ? 0 : strlen(com);
    namesize = (name == NULL) ? 0 : strlen(name);

    if ((comsize + namesize) > MAX_MSG_SIZE)
    {
    errlogMessage("drvMDrivePlus.c:send_mess(); message size violation.\n");
    return(ERROR);
    }
    else if (comsize == 0)  /* Normal exit on empty input message. */
    return(OK);

    if (!motor_state[card])
    {
    errlogPrintf("drvMDrivePlus.c:send_mess() - invalid card #%d\n", card);
    return(ERROR);
    }

    /* Make a local copy of the string and add the command line terminator. */
    if (namesize != 0)
    {
    strcpy(local_buff, name);    /* put in axis */
    strcat(local_buff, com);
    }
    else
    strcpy(local_buff, com);

    Debug(2, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;
    pasynOctetSyncIO->write(cntrl->pasynUser, local_buff, strlen(local_buff),
                timeout, &nwrite);

    return(OK);
}


/*****************************************************/
/* receive a message from the MDrivePlus board           */
/* recv_mess()                                       */
/*****************************************************/
epicsShareFunc int recv_mess(int card, char *com, int flag)
{
    struct IM483controller *cntrl;
    const double timeout = 1.0;
    size_t nread = 0;
    asynStatus status = asynError;
    int eomReason;

    /* Check that card exists */
    if (!motor_state[card])
    return(ERROR);

    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;

    if (flag == FLUSH)
    pasynOctetSyncIO->flush(cntrl->pasynUser);
    else
    status = pasynOctetSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE,
                    timeout, &nread, &eomReason);

    if ((status != asynSuccess) || (nread <= 0))
    {
    com[0] = '\0';
    nread = 0;
    }

    Debug(2, "recv_mess(): message = \"%s\"\n", com);
    return(nread);
}


/*****************************************************/
/* Setup system configuration                        */
/* MDrivePlusSetup()                                     */
/*****************************************************/
RTN_STATUS
MDrivePlusSetup(int num_cards)          /* maximum number of chains in system */
{
    int itera;

    if (num_cards < 1 || num_cards > MDrivePlus_NUM_CARDS)
        MDrivePlus_num_cards = MDrivePlus_NUM_CARDS;
    else
        MDrivePlus_num_cards = num_cards;

    /*
     * Allocate space for motor_state structures.  Note this must be done
     * before IM483Config is called, so it cannot be done in motor_init()
     * This means that we must allocate space for a card without knowing
     * if it really exists, which is not a serious problem
     */
    motor_state   = (struct controller **) malloc( MDrivePlus_num_cards *
                                                   sizeof(struct controller *));
    mess_queue    = (struct circ_queue **) malloc( MDrivePlus_num_cards *
                                                   sizeof(struct circ_queue *));
    free_list     = (struct circ_queue **) malloc( MDrivePlus_num_cards *
                                                   sizeof(struct circ_queue *));
    queue_lock    = (epicsEvent        **) malloc( MDrivePlus_num_cards *
                                                   sizeof(epicsEvent *       ));
    freelist_lock = (epicsEvent        **) malloc( MDrivePlus_num_cards *
                                                   sizeof(epicsEvent *       ));
    motor_sem     = (epicsEvent        **) malloc( MDrivePlus_num_cards *
                                                   sizeof(epicsEvent *       ));

    for ( itera = 0; itera < MDrivePlus_num_cards; itera++ )
    {
        motor_state[itera]   = (struct controller *) NULL;
        mess_queue[itera]    = new circ_queue;
        free_list[itera]     = new circ_queue;
        queue_lock[itera]    = new epicsEvent( epicsEventFull  );
        freelist_lock[itera] = new epicsEvent( epicsEventFull  );
        motor_sem[itera]     = new epicsEvent( epicsEventEmpty );
    }

    initialized = (bool *) malloc( MDrivePlus_num_cards * sizeof(bool) );

    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* MDrivePlusConfig()                                    */
/*****************************************************/
RTN_STATUS
MDrivePlusConfig(const char *name,                          /* ASYN port name */
                 int scan_rate)              /* polling rate - 1/60 sec units */
{
    struct IM483controller *cntrl;

    midx++;

    if (midx >= MDrivePlus_num_cards) return(ERROR);

    motor_state[midx] = (struct controller *) malloc(sizeof(struct controller));
    if (scan_rate >= 1 && scan_rate <= 60)
        motor_state[midx]->scan_rate = scan_rate;
    else
        motor_state[midx]->scan_rate = SCAN_RATE;

    motor_state[midx]->DevicePrivate = malloc(sizeof(struct IM483controller));
    cntrl = (struct IM483controller *) motor_state[midx]->DevicePrivate;

    strcpy(cntrl->asyn_port, name);
    // strncpy(cntrl->initCmds, initCmds, 80);
    // printf( "\ninitCmds='%s'\n", cntrl->initCmds );
    // cntrl->init_was_written = 0;
    return(OK);
}


/*****************************************************/
/* initialize all software and hardware              */
/* This is called from the initialization routine in */
/* device support, MDrivePlus_init.                  */
/* motor_init()                                      */
/*****************************************************/
static int motor_init()
{
    struct controller *brdptr;
    struct IM483controller *cntrl;
    int card_index, motor_index;
    char buff[BUFF_SIZE], fmt_str[32];
    int total_axis = 0;
    int status, rtnval;
    asynStatus success_rtn;
    static const char output_terminator[] = "\n";
    static const char input_terminator[]  = "\r\n";

    /* Check for setup */
    if (MDrivePlus_num_cards <= 0)
    return(ERROR);

    for (card_index = 0; card_index < MDrivePlus_num_cards; card_index++)
    {
    if (!motor_state[card_index])
        continue;

    brdptr = motor_state[card_index];
    brdptr->ident[0] = (char) NULL;    /* No controller identification message. */
    brdptr->cmnd_response = false;
    total_cards = card_index + 1;
    cntrl = (struct IM483controller *) brdptr->DevicePrivate;

    /* Initialize communications channel */
    success_rtn = pasynOctetSyncIO->connect(cntrl->asyn_port, 0,
                        &cntrl->pasynUser, NULL);

    if (success_rtn == asynSuccess)
    {
        pasynOctetSyncIO->setOutputEos(cntrl->pasynUser, output_terminator,
                       strlen(output_terminator));
        pasynOctetSyncIO->setInputEos(cntrl->pasynUser, input_terminator,
                      strlen(input_terminator));
        /* Send a message to the board, see if it exists */
        /* flush any junk at input port - should not be any data available */
        pasynOctetSyncIO->flush(cntrl->pasynUser);

        for (total_axis = 0; total_axis < 1 /*MAX_AXES*/; total_axis++)
        {
        int retry = 0;

        /* Try 3 times to connect to controller. */
        do
        {
            send_mess(card_index, "PR VR", MDrivePlus_axis[total_axis]);
            status = recv_mess(card_index, buff, 1);
            retry++;
        } while (status == 0 && retry < 3);

        if (status <= 0)
            break;
        else if (total_axis == 0)
            strcpy(brdptr->ident, buff);
        }
        brdptr->total_axis = total_axis;
        cntrl->inconfig = (input_config *) malloc(
                             sizeof(struct IM483controller) * total_axis);
    }

    if (success_rtn == asynSuccess && total_axis > 0)
    {
        input_config *confptr = cntrl->inconfig;

        brdptr->localaddr = (char *) NULL;
        brdptr->motor_in_motion = 0;

        for (motor_index = 0; motor_index < total_axis; motor_index++)
        {
        int itera;
        struct mess_info *motor_info = &brdptr->motor_info[motor_index];

        motor_info->status.All = 0;
        motor_info->no_motion_count = 0;
        motor_info->encoder_position = 0;
        motor_info->position = 0;
        brdptr->motor_info[motor_index].motor_motion = NULL;
        /* Assume no encoder support. */
        motor_info->encoder_present = NO;

	/* Determine if encoder present based on EE setting */
        /* HW check of the 2 part saftey check */
        /* (HW) EA_PRESENT && (SW) pmr->ueip==1 must be enabled */
        sprintf(buff, "PR \"EE=\",EE");
        send_mess(card_index, buff, MDrivePlus_axis[motor_index]);
        status = recv_mess(card_index, buff, 1);
        if (status > 0)
        {
            status = sscanf( buff, "EE=%d", &rtnval );
            if ( (status == 1) && (rtnval == 0 || rtnval ==1) )
            {
		if (rtnval ==1)
		{
	                motor_info->encoder_present = YES;
        	        motor_info->status.Bits.EA_PRESENT = 1;
		}
            	else
            	{
                	motor_info->encoder_present = NO;
	                motor_info->status.Bits.EA_PRESENT = 0;
        	}
	    }
        }
        else
        {
                errlogPrintf("Error reading Encoder status\n");
                motor_info->encoder_present = NO;
                motor_info->status.Bits.EA_PRESENT = 0;
        }

        /* Determine if encoder present based last character of "ident". 
        if (brdptr->ident[strlen(brdptr->ident) - 1] == 'E')
        {
            motor_info->pid_present = YES;
            motor_info->status.Bits.GAIN_SUPPORT = 1;
            motor_info->encoder_present = YES;
            motor_info->status.Bits.EA_PRESENT = 1;
        }
	*/

        /* Determine input configuration. */
        confptr->homeLS = confptr->minusLS = confptr->plusLS = 0;

        for (itera = 1; itera <= 4; itera++)
        {
            int type, active;

            sprintf(buff, "PR \"S%d=\",S%d", itera, itera);
            send_mess(card_index, buff, MDrivePlus_axis[motor_index]);
            status = recv_mess(card_index, buff, 1);
            if (status == 0)
            {
            errlogPrintf("Error reading I/O configuration.\n");
            break;
            }

            sprintf(fmt_str, "S%d=%%d,%%d", itera);
            status = sscanf(buff, fmt_str, &type, &active);
            switch (type)
            {
            case 0:
            break;
            case 1: // Home switch.
            confptr->homeLS = itera;
            break;
            case 2: // Plus limit switch.
            confptr->plusLS = itera;
            break;
            case 3: // Minus limit switch.
            confptr->minusLS = itera;
            break;
            default:
            errlogPrintf("Invalid I/O type: %d.\n", type);

            }
        }
        // Test for missing configuration.
        if (confptr->minusLS == 0 || confptr->plusLS == 0)
        {
            const char p_label[6] = "Plus", m_label[6] = "Minus";
            errlogPrintf("MDrivePlus chain #%d, motor #%d %s LS not configured.\n",
                 card_index, motor_index,
                 (confptr->minusLS == 0) ? m_label : p_label);
        }

        /* Determine stall detection mode */
        sprintf(buff, "PR \"SM=\",SM");
        send_mess(card_index, buff, MDrivePlus_axis[motor_index]);
        status = recv_mess(card_index, buff, 1);
        if (status > 0)
        {
            status = sscanf( buff, "SM=%d", &rtnval );
            if ( (status == 1) && (rtnval == 0 || rtnval ==1) )
                motor_info->stall_mode = rtnval;
            else
                motor_info->stall_mode = 0;
        }
        else
        {
            errlogPrintf("Error reading stall detection mode\n");
            motor_info->stall_mode = 0;
        }

        // Test for MCode program version.
        sprintf(buff, "PR \"VE=\",VE");
        send_mess(card_index, buff, MDrivePlus_axis[motor_index]);
        status = recv_mess(card_index, buff, 1);
        if (status > 0)
        {
            status = sscanf( buff, "VE=%d", &rtnval );
            if ( status == 1 ) motor_info->mcode_version = rtnval;
            else               motor_info->mcode_version = 0;
        }
        else
        {
            errlogPrintf("Error reading MCode version.\n");
            motor_info->mcode_version = 0;
        }

        set_status(card_index, motor_index);  /* Read status of each motor */
        }
    }
/*  else                       // if cannot find controller, IOC should continue
        motor_state[card_index] = (struct controller *) NULL; */

    mess_queue[card_index]->head = (struct mess_node *) NULL;
    mess_queue[card_index]->tail = (struct mess_node *) NULL;

    free_list[card_index]->head = (struct mess_node *) NULL;
    free_list[card_index]->tail = (struct mess_node *) NULL;

    targs.card = card_index;
    epicsThreadCreate(cntrl->asyn_port, epicsThreadPriorityMedium,
              epicsThreadGetStackSize(epicsThreadStackMedium),
              (EPICSTHREADFUNC) motor_task, (void *) &targs);

    initialized[card_index] = true;    /* Indicate that driver is initialized */
    }

    return(OK);
}

