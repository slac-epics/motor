/* File: drvPerfMaxUSB.cc                    */

/* Device Driver Support routines for perfMaxUSB MoCo dc motor controller. */
/*
 *      Original Author: Kurt Goetze
 *      Date: 11/24/2003
 *
 * Modification Log:
 * -----------------
 * .00  11-24-2003   kag  initialized from drvMCB4B.c
 * .01  02-06-2004   rls  Eliminate erroneous "Motor motion timeout ERROR".
 * .02  02-12-2004   rls  Copied from drvPerfMaxUSB.c. Upgraded from R3.14.x
 * .03  02-17-2004   rls  Removed Debug calls to tickGet().
 * .04  07-12-2004   rls  Converted from MPF to asyn.
 * .05  09-20-2004   rls  support for 32axes/controller.
 * .08  12-16-2004   rls  - asyn R4.0 support.
 *                - make debug variables always available.
 *                - MS Visual C compatibility; make all epicsExportAddress
 *              extern "C" linkage.
 */


#include <string.h>
#include <semaphore.h>
#include <epicsThread.h>
#include <drvSup.h>
#include "motor.h"
#include "drvPerfMaxUSB.h"
#include "epicsExport.h"
#include "usb.h"
#include "ArcusPerformaxDriver.h"

#define DEBUG
#define WAIT 1

#define COMM_TIMEOUT 20  /* Command timeout in milliseconds. */

#define BUFF_SIZE 64       /* Maximum length of string to/from perfMaxUSB */

struct mess_queue
{
    struct mess_node *head;
    struct mess_node *tail;
};

/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef  DEBUG
    #define Debug(l, f, args...) {if (l <= drvPerfMaxUSBDebug) printf(f, ## args);}
    #else
    #define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif
volatile int drvPerfMaxUSBDebug = 0;
extern "C" {epicsExportAddress(int, drvPerfMaxUSBDebug);}

/* Debugging notes:
 *   drvPerfMaxUSBDebug == 0  No debugging information is printed
 *   drvPerfMaxUSBDebug >= 1  Warning information is printed
 *   drvPerfMaxUSBDebug >= 2  Time-stamped messages are printed for each string
 *                       sent to and received from the controller
 *   drvPerfMaxUSBDebug >= 3  Additional debugging messages
 */

volatile int perfMaxUSB_num_cards = 0;
volatile int perfMaxUSB_num_axis = 1;

/* Local data required for every driver; see "motordrvComCode.h" */
#include        "motordrvComCode.h"


/*----------------functions-----------------*/
static int recv_mess(int, char *, int);
static RTN_STATUS send_mess(int, const char *, char *);
static void start_status(int);
static int set_status(int, int);
static long report(int);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

static const char *perfMaxUSBInitCmds[] =
{
   "LSPD=3000" ,
   "HSPD=100000",
   "ACC=300",
   "POL=16",
   "SSPD500",
   "EO=1",
   NULL,
};

char TxBuf[BUFF_SIZE];
char RxBuf[BUFF_SIZE];

/*----------------functions-----------------*/

struct driver_table perfMaxUSB_access =
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
    start_status,
    &initialized,
    NULL
};

struct drvPerfMaxUSB_struct
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvPerfMaxUSB = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvPerfMaxUSB);}

static struct thread_args targs = {SCAN_RATE, &perfMaxUSB_access, 0.0};

sem_t semTx, semRx;

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
  int card;

  if (perfMaxUSB_num_cards <=0)
    printf("    NO perfMaxUSB controllers found\n");
  else
    {
      for (card = 0; card < perfMaxUSB_num_cards; card++)
          if (motor_state[card])
             printf("    perfMaxUSB controller group %d, id: %s \n",
                   card,
                   motor_state[card]->ident);
    }
  return (0);
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
    if (perfMaxUSB_num_cards <= 0)
    {
        Debug(1, "init: *perfMaxUSB driver disabled*\n");
        return (ERROR);
    }
    return ((long) 0);
}

static void query_done(int card, int axis, struct mess_node *nodeptr)
{
}


/*********************************************************
 * Read the status and position of all motors on a card
 * start_status(int card)
 *            if card == -1 then start all cards
 *********************************************************/
static void start_status(int card)
{
    /* The perfMaxUSB cannot query status or positions of all axes with a
     * single command.  This needs to be done on an axis-by-axis basis,
     * so this function does nothing
     */
}


/**************************************************************
 * Query position and status for an axis
 * set_status()
 ************************************************************/

static int set_status(int card, int signal)
{
    register struct mess_info *motor_info;
    struct mess_node *nodeptr;
    int rtn_state = 0;
    long motorData;
    bool ls_active = false;
    msta_field status;

    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    /* Request the moving status of this motor */

    /* Get the motor status (ts) */
    memset(TxBuf, '\0', sizeof(TxBuf));
    sprintf(TxBuf, "MST");
    send_mess(card, TxBuf, 0);
    //printf("RxBuf %c %s\n", RxBuf[0], RxBuf);

    motor_info->encoder_position = 0;
    motor_info->no_motion_count = 0;
    motor_info->velocity = 0;
    status.Bits.RA_MINUS_LS = 0;
    status.Bits.RA_PLUS_LS = 0;
    status.Bits.RA_PROBLEM = 0;

    if (RxBuf[0] == 0x30)
        status.Bits.RA_DONE = 1;
    else
        status.Bits.RA_DONE = 0;

   // if (status.Bits.RA_DONE & 1)
   //     puts("done moving");;

    sprintf(TxBuf, "PX");
    send_mess(card, TxBuf, 0);

    motorData = atoi(RxBuf);
    //printf("RxBuf %s\n", RxBuf);
    //printf("motorData1 %ld\n", motorData);
       
    /* derive direction information */
    if (motorData == motor_info->position)
    {
        if (nodeptr != 0)   /* Increment counter only if motor is moving. */
            motor_info->no_motion_count++;
    }
    else
    {
        status.Bits.RA_DIRECTION = (RxBuf[0] == '-') ? 0 : 1;
        motor_info->position = motorData;
        motor_info->encoder_position = motorData;
        motor_info->no_motion_count = 0;
    }

    rtn_state = (!motor_info->no_motion_count ||
                 ls_active == true ||
                 status.Bits.RA_DONE ||
                 status.Bits.RA_PROBLEM) ? 1 : 0;

    motor_info->status.All = status.All;
    return (rtn_state);
}


/*****************************************************/
/* send a message to the perfMaxUSB board                 */
/* send_mess()                                       */
/*****************************************************/
//static RTN_STATUS send_mess(int card, const char *com, char *name)
RTN_STATUS send_mess(int card, const char *com, char *name)
{
    struct perfMaxUSBController *ctlr;
    RTN_STATUS err = OK;

    if (!strlen(com))
        return(ERROR);

    /* Check that card exists */
    if (!motor_state[card])
    {
        errlogPrintf("send_mess - invalid card #%d\n", card);
        return(ERROR);
    }
 
    ctlr = (struct perfMaxUSBController *) motor_state[card]->DevicePrivate;

    if(!fnPerformaxComSendRecv(ctlr->pmaxUSB.handle, (AR_VOID*) com, BUFF_SIZE, BUFF_SIZE, (AR_VOID*) RxBuf))
        err = ERROR;

    Debug(2, "send_mess: sending message to card %d, message=%s\n", card, com);

    return (err);
}


/*****************************************************/
/* Read a response string from the perfMaxUSB board */
/* recv_mess()                                       */
/*****************************************************/
static int recv_mess(int card, char *com, int flag)
{
    return (BUFF_SIZE);
}



/*****************************************************/
/* Setup system configuration                        */
/* PerfMaxUSBSetup()                                     */
/*****************************************************/
RTN_STATUS
PerfMaxUSBSetup(int num_cards,   /* maximum number of "controllers" in system */
             int num_channels,/* max number of drivers            */
             int scan_rate)   /* polling rate - 1/60 sec units */
{
    int i;

    if (num_cards < 1 || num_cards > PMAXUSB_NUM_CARDS)
        perfMaxUSB_num_cards = PMAXUSB_NUM_CARDS;
    else
        perfMaxUSB_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
        targs.motor_scan_rate = scan_rate;
    else
        targs.motor_scan_rate = SCAN_RATE;

    /*
     * Allocate space for motor_state structures.  Note this must be done
     * before IM483Config is called, so it cannot be done in motor_init()
     * This means that we must allocate space for a card without knowing
     * if it really exists, which is not a serious problem
     */
    motor_state = (struct controller **) malloc(perfMaxUSB_num_cards *
                                                sizeof(struct controller *));

    for (i = 0; i < perfMaxUSB_num_cards; i++)
        motor_state[i] = (struct controller *) NULL;

    return(OK);
}

//libusb_context *PerfMaxUSBContext;

/*****************************************************/
/* Configure a controller                            */
/* PerfMaxUSBConfig()                                    */
/*****************************************************/
RTN_STATUS
PerfMaxUSBConfig(int card,       /* "controller" being configured */
        const char *name)   /* asyn server task name */
{
    struct perfMaxUSBController *ctlr;
    AR_HANDLE handle;
    char rxBuf[BUFF_SIZE];
    char txBuf[BUFF_SIZE];
    int i, cnt;
    RTN_STATUS err = ERROR;

    if (motor_state[card] != NULL)
    {
        printf("Car %d already assigned!!!\n", card);
        return(err);
    }

    if(!fnPerformaxComGetNumDevices((AR_DWORD*) &cnt))
    {
        printf("error in fnPerformaxComGetNumDevices\n");
        return(err);
    }

    if(cnt < 1)
    {
        printf( "No motor found\n");
        return(err);
    }

    printf("There are %d devices\n", cnt);

    if(!fnPerformaxComSetTimeouts(5000,5000))
    {
        printf("Error setting timeouts\n");
        return(err);
    }


    for (i = 0; i < cnt; i++) 
    {
        if(!fnPerformaxComOpen(i, &handle))
        {
            printf( "Error opening device\n");
            continue; 
        }

        if(!fnPerformaxComFlush(handle))
        {
            printf("Error flushing the coms\n");
            fnPerformaxComClose(handle);
            continue;
        }

        strcpy(txBuf, "DN"); //read current
        if(!fnPerformaxComSendRecv(handle, txBuf, 64,64, rxBuf))
        {
            puts("nothing there?");
            fnPerformaxComClose(handle);
	    continue;
        }

        // break if we get a match
        else if(!strcmp(rxBuf, name))
            break;

        else
        {
            printf("The name we got is %s\n", rxBuf);
            usb_reset(handle);
            fnPerformaxComClose(handle);
            continue;
        }
    }

    if (i >= cnt)
        err = ERROR;
    else
    {
        printf("We found %s i = %d\n", name, i);

        motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
        ctlr =  (struct perfMaxUSBController*) malloc(sizeof(struct perfMaxUSBController));
        motor_state[card]->DevicePrivate = (void*) ctlr; 
        strcpy(ctlr->name, rxBuf);

        ctlr->pmaxUSB.found = 1;
        ctlr->pmaxUSB.handle = handle;
        err = OK;
    }
    return(err);
}

RTN_STATUS PerfMaxUSBShow(const char* ctlrName)
{
    int i;
    struct perfMaxUSBController *ctlr;

    for (i = 0; i < perfMaxUSB_num_cards; i++)
    {
        if (motor_state[i] != NULL) 
        {
            if (motor_state[i]->DevicePrivate != NULL)
            {
                ctlr = (struct perfMaxUSBController *)motor_state[i]->DevicePrivate;
                if (ctlr != NULL)
                {
                    printf("found %s\n", ctlr->name);
                }
            }
        }
    }
    return(OK);
}



/*****************************************************/
/* initialize all software and hardware              */
/* This is called from the initialization routine in */
/* device support.                                   */
/* motor_init()                                      */
/*****************************************************/
static int motor_init()
{
    struct controller *brdptr;
    struct perfMaxUSBController *cntrl;
    int card_index, motor_index;
    int total_axis = 0;
    int status = 0;
    int i;
    EPICSTHREADFUNC perfMaxUSBEventThread();

    initialized = true;   /* Indicate that driver is initialized. */

    /* Check for setup */
    if (perfMaxUSB_num_cards <= 0)
    {
        Debug(1, "motor_init: *perfMaxUSB driver disabled*\n");
        Debug(1, "perfMaxUSBSetup() is missing from startup script.\n");
        return (ERROR);
    }

    for (card_index = 0; card_index < perfMaxUSB_num_cards; card_index++)
    {
        if (!motor_state[card_index])
            continue;

        brdptr = motor_state[card_index];
        total_cards = card_index + 1;
        cntrl = (struct perfMaxUSBController *) brdptr->DevicePrivate;

        if ((cntrl != NULL) && (!cntrl->pmaxUSB.found))
            continue;

        /* Initialize communications channel with the magic */
        //status = libusb_control_transfer(cntrl->pmaxUSB.handle, 0x40, 0x02, 0x02, 0, NULL, 0, 500);

        if (status >= 0)
        {
            status = 0;

            /* Each "controller" can have max 16 axes. */
            total_axis = perfMaxUSB_num_axis;
            brdptr->total_axis = total_axis;

        }


        if (!status)
        {
            /* we're good so init everything */
            brdptr->localaddr = (char *) NULL;
            brdptr->motor_in_motion = 0;
            brdptr->cmnd_response = true;
            brdptr->cmnd_response = false;

            start_status(card_index);
            for (motor_index = 0; motor_index < total_axis; motor_index++)
            {
                struct mess_info *motor_info = &brdptr->motor_info[motor_index];
                brdptr->motor_info[motor_index].motor_motion = NULL;
                /* turn off echo */
                for (i = 0; perfMaxUSBInitCmds[i] != NULL; i++)
                {
                    send_mess(card_index, perfMaxUSBInitCmds[i], NULL);
                    recv_mess(card_index, RxBuf, WAIT);
                }

                strcpy(brdptr->ident, "PERFMAXUSB");

                motor_info->status.All = 0;
                motor_info->no_motion_count = 0;
                motor_info->encoder_position = 0;
                motor_info->position = 0;

                motor_info->encoder_present = NO;
                motor_info->status.Bits.EA_PRESENT = 0;
                motor_info->pid_present = NO;
                motor_info->status.Bits.GAIN_SUPPORT = 0;

                set_status(card_index, motor_index);  /* Read status of each motor */
            }

        }
        else
            motor_state[card_index] = (struct controller *) NULL;
    }

    Debug(3, "motor_init: spawning motor task\n");

    any_motor_in_motion = 0;

    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;

    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;

    epicsThreadCreate((char *) "tperfMaxUSB", epicsThreadPriorityMedium,
              epicsThreadGetStackSize(epicsThreadStackMedium),
              (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return (0);
}

