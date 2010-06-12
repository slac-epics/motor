/* File: devPerfMaxUSB.cc                    */

/* Device Support Routines for perfMaxUSB MoCo dc motor controller. */
/*
 *      Original Author: Kurt Goetze
 *      Date: 11-24-2003
 *
 * Modification Log:
 * -----------------
 * .00  11-24-2003   kag  initialized from drvMCB4B.c
 * .01  02-06-2004   rls  Eliminate erroneous "Motor motion timeout ERROR".
 * .02  02-12-2004   rls  copied from devPerfMaxUSB.c; ported to R3.14.x
 */


#define VERSION 2.00

#include <string.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvPerfMaxUSB.h"
#include "epicsExport.h"
#define DEBUG
/*----------------debugging-----------------*/
#ifdef __GNUG__
    #ifdef	DEBUG
	volatile int devPerfMaxUSBDebug = 0;
	#define Debug(l, f, args...) {if (l <= devPerfMaxUSBDebug) printf(f, ## args);}
    #else
	#define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif

/* Debugging levels: 
 *      devPerfMaxUSBDebug >= 3  Print new part of command and command string so far
 *                          at the end of perfMaxUSB_build_trans
 */

extern struct driver_table perfMaxUSB_access;

/* ----------------Create the dsets for devPerfMaxUSB----------------- */
static struct driver_table *drvtabptr;
static long perfMaxUSB_init(void *);
static long perfMaxUSB_init_record(void *);
static long perfMaxUSB_start_trans(struct motorRecord *);
static RTN_STATUS perfMaxUSB_build_trans(motor_cmnd, double *, struct motorRecord *);
static RTN_STATUS perfMaxUSB_end_trans(struct motorRecord *);

struct motor_dset devPerfMaxUSB =
{
    {8, NULL, (DEVSUPFUN) perfMaxUSB_init, (DEVSUPFUN) perfMaxUSB_init_record, NULL},
    motor_update_values,
    perfMaxUSB_start_trans,
    perfMaxUSB_build_trans,
    perfMaxUSB_end_trans
};

extern "C" {epicsExportAddress(dset,devPerfMaxUSB);}

/* --------------------------- program data --------------------- */
/* This table is used to define the command types */

static msg_types perfMaxUSB_table[] = {
    MOTION,     /* MOVE_ABS */
    MOTION,     /* MOVE_REL */
    MOTION,     /* HOME_FOR */
    MOTION,     /* HOME_REV */
    IMMEDIATE,  /* LOAD_POS */
    IMMEDIATE,  /* SET_VEL_BASE */
    IMMEDIATE,  /* SET_VELOCITY */
    IMMEDIATE,  /* SET_ACCEL */
    IMMEDIATE,  /* GO */
    IMMEDIATE,  /* SET_ENC_RATIO */
    INFO,       /* GET_INFO */
    MOVE_TERM,  /* STOP_AXIS */
    VELOCITY,   /* JOG */
    IMMEDIATE,  /* SET_PGAIN */
    IMMEDIATE,  /* SET_IGAIN */
    IMMEDIATE,  /* SET_DGAIN */
    IMMEDIATE,  /* ENABLE_TORQUE */
    IMMEDIATE,  /* DISABL_TORQUE */
    IMMEDIATE,  /* PRIMITIVE */
    IMMEDIATE,  /* SET_HIGH_LIMIT */
    IMMEDIATE   /* SET_LOW_LIMIT */
};


static struct board_stat **perfMaxUSB_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for perfMaxUSB DC motor */
static long perfMaxUSB_init(void *arg)
{
    long rtnval = 0;
    int after = (arg == 0) ? 0 : 1;

    Debug(5, "perfMaxUSB_init: entry\n");
    if (after == 0)
    {
	drvtabptr = &perfMaxUSB_access;
	(drvtabptr->init)();
    }
#if	1
    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &perfMaxUSB_cards);
#endif
    Debug(5, "perfMaxUSB_init: exit\n");
    return(rtnval);
}


/* initialize a record instance */
static long perfMaxUSB_init_record(void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    long rtnval = 0;

    Debug(5, "perfMaxUSB_init_record: entry\n");
#if	1
    rtnval = motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, 
                                   drvtabptr, perfMaxUSB_cards);
#endif
    Debug(5, "perfMaxUSB_init_record: exit\n");
    return(rtnval);
}


/* start building a transaction */
static long perfMaxUSB_start_trans(struct motorRecord *mr)
{
    return(OK);
}


/* end building a transaction */
static RTN_STATUS perfMaxUSB_end_trans(struct motorRecord *mr)
{
    return(OK);
}

/* add a part to the transaction */
static RTN_STATUS perfMaxUSB_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct perfMaxUSBController *cntrl;
    char buff[30];
    int axis, card;
    RTN_STATUS rtnval;
    double dval = 0.0;   /* placeholder for double values passed from motor record */
    long ival;     /* placeholder for ints passed from motor record */

    rtnval = OK;
    buff[0] = '\0';
    //dval = parms[0];
    //ival = NINT(parms[0]);
    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
    ival = (parms == NULL) ? 0 : NINT(parms[0]);

    rtnval = (RTN_STATUS) motor_start_trans_com(mr, perfMaxUSB_cards);
    Debug(5, "perfMaxUSB_build_trans: entry, motor_start_trans_com=%d\n", rtnval);

    motor_call = &(trans->motor_call);
    motor_call->type = perfMaxUSB_table[command];
    card = motor_call->card;    /* card is the group of drivers per unique serial port */
    axis = motor_call->signal;  /* axis is perfMaxUSB address, up to 16 (0-15) per serial port */
                                /* Note: Each perfMaxUSB driver drives _1_ motor */
    brdptr = (*trans->tabptr->card_array)[card];
    Debug(5, "perfMaxUSB_build_trans: axis=%d, command=%d\n", axis, command);
    if (brdptr == NULL)
        return(rtnval = ERROR);

    cntrl = (struct perfMaxUSBController *) brdptr->DevicePrivate;


    if (trans->state != BUILD_STATE)
        return(rtnval = ERROR);

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
    {
        strcpy(motor_call->message, mr->init);
        rtnval = motor_end_trans_com(mr, drvtabptr);
        rtnval = (RTN_STATUS) motor_start_trans_com(mr, perfMaxUSB_cards);
        motor_call->type = perfMaxUSB_table[command];
    }

    switch (command)
    {
        case MOVE_ABS:
        case MOVE_REL:
        case HOME_FOR:
        case HOME_REV:
        case JOG:
            if (strlen(mr->prem) != 0)
            {
                strcpy(motor_call->message, mr->prem);
                rtnval = (RTN_STATUS) motor_start_trans_com(mr, perfMaxUSB_cards);
                motor_call->type = perfMaxUSB_table[command];
            }
            if (strlen(mr->post) != 0)
                motor_call->postmsgptr = (char *) &mr->post;
            break;

        default:
            break;
    }

    switch (command)
    {
    case MOVE_REL:
        trans->state = IDLE_STATE;
        break;
    case MOVE_ABS:
        sprintf(motor_call->message, "X%ld", ival);
        break;
    case HOME_FOR:
        sprintf(motor_call->message, "H+");
        break;
    case HOME_REV:
        sprintf(motor_call->message, "H-");
        break;
    case LOAD_POS:       
        trans->state = IDLE_STATE;
        break;
    case SET_VEL_BASE:
	break;
        if (ival < 0) ival = 0;
        if (ival > 10000) ival = 10000;
            sprintf(motor_call->message, "LSPD=%ld", ival);
        break;          /* perfMaxUSB does not use base velocity */
    case SET_VELOCITY:
	break;
        printf("velocity %d\n", ival);
        if (ival < 0) ival = 0;
        if (ival > 10000) ival = 10000;
            sprintf(motor_call->message, "HSPD=%ld", ival);
        break;
    case SET_ACCEL:
        /* dval is acceleration in steps/sec/sec */
        if (ival < 0) ival = 0;
        if (ival > 10000) ival = 10000;
        //sprintf(motor_call->message, "ACC=%ld", ival);
        break;
    case GO:
    /*
         * The perfMaxUSB starts moving immediately on move commands, GO command
         * does nothing
         */
        trans->state = IDLE_STATE;
        break;
    case SET_ENC_RATIO:
    /*
         * The perfMaxUSB does not have the concept of encoder ratio, ignore this
         * command
         */
        trans->state = IDLE_STATE;
        break;
    case GET_INFO:
        /* These commands are not actually done by sending a message, but
           rather they will indirectly cause the driver to read the status
           of all motors */
        break;
    case STOP_AXIS:   /* (decelerate to a) stop */
        sprintf(motor_call->message, "STOP");
        break;
    case JOG:
    /* 
          * perfMaxUSB does not have a jog command.  Simulate with move absolute
          * to the appropriate software limit.  The record will prevent JOG motion 
          * beyond its soft limits
          */
	if (dval > 0.0)
	    sprintf(motor_call->message, "J+");
	else
            sprintf(motor_call->message, "J-");
	break;

    case SET_PGAIN:
    case SET_IGAIN:
    case SET_DGAIN:
        trans->state = IDLE_STATE;
        break;

    case ENABLE_TORQUE:
        trans->state = IDLE_STATE;
        break;

    case DISABL_TORQUE:
        trans->state = IDLE_STATE;
        break;

    case SET_HIGH_LIMIT:
    case SET_LOW_LIMIT:
        trans->state = IDLE_STATE;
        break;

    default:
        rtnval = ERROR;
    }

    rtnval = motor_end_trans_com(mr, drvtabptr);
    Debug(5, "perfMaxUSB_send_msg: motor_end_trans_com status=%d, exit\n", rtnval); 
    return (rtnval);
}
