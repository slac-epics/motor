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

//#define	OK	0
//#define	ERR	!OK
#define	ERR	ERROR
#define	FALSE	0
#define	TRUE	!FALSE

#define PMC100_TX_LEN		64
#define	PMC100_RX_LEN		64
#define	PMC100_STATE_ON		'1'
#define	PMC100_STATE_OFF	'0'
#define	PMC100_DIR_POS		TRUE	
#define	PMC100_DIR_NEG		FALSE	

#define	PMC100_STATUS_CONST_SPD	0x01 << 0x00
#define	PMC100_STATUS_ACCEL	0x01 << 0x01
#define	PMC100_STATUS_DECEL	0x01 << 0x02
#define	PMC100_STATUS_UNDEF_B3	0x01 << 0x03
#define	PMC100_STATUS_PLUS_LIM	0x01 << 0x04
#define	PMC100_STATUS_MIN_LIM	0x01 << 0x05
#define	PMC100_STATUS_LATCH	0x01 << 0x06
#define	PMC100_STATUS_UNDEF_B7	0x01 << 0x07

#define	PMC100_SL_STATUS_IDLE	0
#define	PMC100_SL_STATUS_MOV	1
#define	PMC100_SL_STATUS_COR	2
#define	PMC100_SL_STATUS_STOP	3
#define	PMC100_SL_STATUS_JOG	4
#define	PMC100_SL_STATUS_HOME	5	
#define	PMC100_SL_STATUS_RANGE	10	
#define	PMC100_SL_STATUS_TRY	11	
#define	PMC100_SL_STATUS_UNKWN	-1

static int pmc100TxRx(AR_HANDLE, char*, char*);
static AR_HANDLE pmax100EpicsHandleGet(int);
static int pmc100SlStatusGet(int);
static int pmc100PosSet(int, int);
static int pmc100Abort(int);
static int pmc100EncCntGet(int);
static int pmc100SlModeGet(int);
static int pmc100PolSet(int, int);

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
   "LSPD=300" ,
   "HSPD=25000", // was 5000
   "ACC=500", // was 300
   "POL=16",
   "SSPD500",
   "EO=1",
   "SLR=50", // was 10
   "SLT=200",
   "SLR=16",
   "SL=1",
   NULL,
};


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

int set_status(int card, int signal)
{
    register struct mess_info *motor_info;
    struct mess_node *nodeptr;
    int rtn_state = 0;
    long motorData;
    bool ls_active = false;
    msta_field status;
    int slstatus;

    pmc100SlModeGet(card);
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    /* Request the moving status of this motor */

    motor_info->encoder_position = 0;
    motor_info->no_motion_count = 0;
    motor_info->velocity = 0;
    status.Bits.RA_MINUS_LS = 0;
    status.Bits.RA_PLUS_LS = 0;
    status.Bits.RA_PROBLEM = 0;

    /* Get the motor status (ts) */
    slstatus = pmc100SlStatusGet(card);
    if (slstatus == PMC100_SL_STATUS_IDLE)
        status.Bits.RA_DONE = 1;
    else if ((slstatus == PMC100_SL_STATUS_RANGE) ||
             (slstatus == PMC100_SL_STATUS_TRY))
    {
        status.Bits.RA_DONE = 0;
        pmc100Abort(card);
        pmc100PosSet(card, motor_info->position);
    }
    else 
        status.Bits.RA_DONE = 0;

    // get the encoder count
    motorData = pmc100EncCntGet(card); 
       
    /* derive direction information */
    if (motorData == motor_info->position)
    {
        if (nodeptr != 0)   /* Increment counter only if motor is moving. */
            motor_info->no_motion_count++;
    }
    else
    {
        status.Bits.RA_DIRECTION = (motorData <= 0) ? 0 : 1;
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
    char rxBuf[BUFF_SIZE];
    char txBuf[BUFF_SIZE];

    if (!strlen(com))
        return(ERROR);

    /* Check that card exists */
    if (!motor_state[card])
    {
        errlogPrintf("send_mess - invalid card #%d\n", card);
        return(ERROR);
    }
 
    ctlr = (struct perfMaxUSBController *) motor_state[card]->DevicePrivate;

    if(!fnPerformaxComSendRecv(ctlr->pmaxUSB.handle, (AR_VOID*) com, 
                               BUFF_SIZE, BUFF_SIZE, (AR_VOID*) rxBuf))
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
    AR_HANDLE handle = 0;
    char rxBuf[BUFF_SIZE];
    char txBuf[BUFF_SIZE];
    int i, cnt;

    if(!fnPerformaxComGetNumDevices((AR_DWORD*) &cnt))
    {
        printf("error in fnPerformaxComGetNumDevices\n");
        return(ERROR);
    }

    if(cnt < 1)
    {
        printf( "No motor found\n");
        return(ERROR);
    }

    printf("There are %d devices\n", cnt);

    if(!fnPerformaxComSetTimeouts(5000,5000))
    {
        printf("Error setting timeouts\n");
        return(ERROR);
    }

    if (motor_state[card] == NULL)
    {

        for (i = 0; i < cnt; i++) 
        {
            if(!fnPerformaxComOpen(i,&handle))
            {
                printf( "Error opening device\n");
                continue; 
            }

            if(!fnPerformaxComFlush(handle))
            {
                printf("Error flushing the coms\n");
                continue;
            }

            strcpy(txBuf, "DN"); //read current
            if(!fnPerformaxComSendRecv(handle, txBuf, 64,64, rxBuf))
                puts("nothing there?");

            else if(!strcmp(rxBuf, name))
            {
                motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
                ctlr =  (struct perfMaxUSBController*) malloc(sizeof(struct perfMaxUSBController));
                motor_state[card]->DevicePrivate = (void*) ctlr; 
                strcpy(ctlr->name, rxBuf);

                ctlr->pmaxUSB.found = 1;
                ctlr->pmaxUSB.handle = handle;
                break;
            }
            usb_reset(handle);
            fnPerformaxComClose(handle);
        }
    }
    return(OK);
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

RTN_STATUS PerfMaxUSBPolSet(int card, int polarity)
{
   pmc100PolSet(card, polarity);
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
    char txBuf[BUFF_SIZE];
    char rxBuf[BUFF_SIZE];

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
                    strcpy(txBuf, perfMaxUSBInitCmds[i]);
		    pmc100TxRx(pmax100EpicsHandleGet(card_index), txBuf, rxBuf);
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

int pmc100TxRx(AR_HANDLE Handle, char *outBuf, char *inBuf)  
{
   int i;

   /* we do this twice because the first time returns incorrect status */
   for (i = 0; i < 2; i++)
   {
      if(!fnPerformaxComSendRecv(Handle, outBuf, PMC100_TX_LEN, PMC100_RX_LEN, inBuf))
      {
         printf("Could not send\n");
         return(ERR);
      }
   }
   return(OK);
}

int pmc100Abort(int card)
{
   char out[64], in[64];

   strcpy(out, "ABORT");
   if (pmc100TxRx(pmax100EpicsHandleGet(card), out, in))
      return(ERR);
   printf("Abort=%s\n", in); 
   return(OK);
}


int pmc100DrvrPwrSet(int card, int state)
{
   char out[64], in[64];

   sprintf(out, "EO=%s", state?"1":"0");
   pmc100TxRx(pmax100EpicsHandleGet(card), out, in);
   return(OK);
}

int pmc100MtrIdleGet(int card)
{
   char out[64], in[64];

   sprintf(out, "MST");

   pmc100TxRx(pmax100EpicsHandleGet(card), out, in);
   return((in[0] != '?') && ((in[0] ^ (PMC100_STATUS_ACCEL|PMC100_STATUS_DECEL) == 0)?OK:ERR));
}

int pmc100PosSet(int card, int pos)
{
   char out[64], in[64];
//   int i;

      sprintf(out, "X%d", pos);
      pmc100TxRx(pmax100EpicsHandleGet(card), out, in); 
      printf("PosSet=%s\n",in);
#if	0
   do
   {
      sprintf(out, "X%d", pos);
      pmc100TxRx(pmax100EpicsHandleGet(card), out, in); 
      printf("In=%s\n",in);
      //sleep(1);
   } while (in[0] != 'O');
#endif
   return(OK);
}

int pmc100PosGet(int card)
{
   char out[64], in[64];

   do
   {
      sprintf(out, "LTE");
      pmc100TxRx(pmax100EpicsHandleGet(card), out, in);
   } while (in[0] != 'O');
   return(atoi(in));
}
  
int pmc100LatchStatusGet(int card)
{ 
   char out[64], in[64];
   int status;

   memset(in, 0x55, 64);
   sprintf(out, "LTS");
   pmc100TxRx(pmax100EpicsHandleGet(card), out, in);
   status = in[0] << 8|in[1];
   printf("status=%x\n", status);
   return(status);
}

int pmc100SlsModeSet(int card, int state)
{
   char out[64], in[64];

   sprintf(out, "SL=%c", state?PMC100_STATE_ON:PMC100_STATE_OFF);
   if (pmc100TxRx(pmax100EpicsHandleGet(card), out, in))
      return(ERR);
   printf("SlsModeSet=%s\n",in);
   return(OK);
}

int pmc100HomeSet(int card, int direction)
{
   char out[64], in[64];

   strcpy(out, direction?"H+":"H-");
   if (pmc100TxRx(pmax100EpicsHandleGet(card), out, in))
      return(ERR);
   printf("HomeSet=%s\n",in);
   return(OK);
}

int pmc100EncRatioSet(int card, int ratio)
{
   char out[64], in[64];

   sprintf(out, "SLR=%d", ratio); 
   if (pmc100TxRx(pmax100EpicsHandleGet(card), out, in))
      return(ERR);
   printf("EncRatioSet=%s\n",in);
   return(OK);
}

int pmc100EncTolSet(int card, int tolerance)
{
   char out[64], in[64];

   sprintf(out, "SLT=%d", tolerance); 
   if (pmc100TxRx(pmax100EpicsHandleGet(card), out, in))
      return(ERR);
   printf("EncTolSet=%s\n",in);
   return(OK);
}

int pmc100EncCorSet(int card, int correction)
{
   char out[64], in[64];

   sprintf(out, "SLE=%d", correction); 
   if (pmc100TxRx(pmax100EpicsHandleGet(card), out, in))
      return(ERR);
   printf("EncCorSet=%s\n",in);
   return(OK);
}

int pmc100PolSet(int card, int polarity)
{
   char out[64], in[64];

   sprintf(out, "POL=%d", polarity); 
   if (pmc100TxRx(pmax100EpicsHandleGet(card), out, in))
      return(ERR);
   printf("PolaritySet=%s\n",in);
   return(OK);
}

int pmc100EncTrysSet(int card, int attempts)
{
   char out[64], in[64];

   sprintf(out, "SLA=%d", attempts); 
   if (pmc100TxRx(pmax100EpicsHandleGet(card), out, in))
      return(ERR);
   printf("EncTrysSet=%s\n",in);
   return(OK);
}

int pmc100SlStatusGet(int card)
{
   char out[64], in[64];
   int i;

   strcpy(out, "SLS");
   if (pmc100TxRx(pmax100EpicsHandleGet(card), out, in))
      return(ERR);
   printf("SlStatusGet=%s\n",in);

   if ((in[0] == '1') && (in[1] >= '0'))
      i = 10 + in[1] - '0';
   else
      i = in[0] - '0';

   printf("SlStatusGetI=%d\n",i);

   return(i);
}

int pmc100SlModeGet(int card)
{
   char out[64], in[64];
   int cnt;

   strcpy(out, "SL");
   if (pmc100TxRx(pmax100EpicsHandleGet(card), out, in))
      return(ERR);
   printf("SlModeGet=%c\n", in[0]);
   return(OK);
}
int pmc100EncCntGet(int card)
{
   char out[64], in[64];
   int cnt;

   strcpy(out, "EX");
   if (pmc100TxRx(pmax100EpicsHandleGet(card), out, in))
      return(ERR);
   cnt = atoi(in);
   printf("EncCntGet=%d\n", cnt);
   return(cnt);
}

int pmc100EncCntSet(int card, int pos)
{
   char out[64], in[64];

   sprintf(out, "EX=%d", pos);
   if (pmc100TxRx(pmax100EpicsHandleGet(card), out, in))
      return(ERR);
   return(OK);
}

int pmc100PlsCntGet(int card)
{
   char out[64], in[64];
   int cnt;

   strcpy(out, "PX");
   if (pmc100TxRx(pmax100EpicsHandleGet(card), out, in))
      return(ERR);
   cnt = atoi(in);
   printf("PlsCntGet=%d\n",cnt);
   return(cnt);
}

int pmc100PlsCntSet(int card, int pos)
{
   char out[64], in[64];

   sprintf(out, "PX=%d", pos);
   if (pmc100TxRx(pmax100EpicsHandleGet(card), out, in))
      return(ERR);
   return(OK);
}

int pmc100Stop(int card)
{
   char out[64], in[64];

   strcpy(out, "STOP");
   if (pmc100TxRx(pmax100EpicsHandleGet(card), out, in))
      return(ERR);
   printf("Stop=%s\n", in); 
   return(OK);
}

static AR_HANDLE pmax100EpicsHandleGet(int card)
{
    struct perfMaxUSBController *ctlr;

    /* Check that card exists */
    if (!motor_state[card])
    {
        errlogPrintf("send_mess - invalid card #%d\n", card);
        return(NULL);
    }
 
    ctlr = (struct perfMaxUSBController *) motor_state[card]->DevicePrivate;
    return(ctlr->pmaxUSB.handle);
}












   


