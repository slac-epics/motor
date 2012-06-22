/* FIle: devMDrivePlus.cc
         Motor record device level support for Intelligent Motion Systems, Inc. MDrivePlus series of controllers.
 */

#include <string.h>

#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvIM483.h"
#include "epicsExport.h"

#define STATIC static

extern struct driver_table MDrivePlus_access;

/* ----------------Create the dsets for devMDrivePlus----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long MDrivePlus_init(void *);
STATIC long MDrivePlus_init_record(void *);
STATIC long MDrivePlus_start_trans(struct motorRecord *);
STATIC RTN_STATUS MDrivePlus_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC RTN_STATUS MDrivePlus_end_trans(struct motorRecord *);

struct motor_dset devMDrivePlus =
{
    // struct dset from base, devSup.
    // 8: number - Total number of routines (function pointers).
    // NULL: report - No 'print report' routine provided.
    // MDrivePlus_init: init - 'init support layer'.
    // MDrivePlus_init_record: init_record - 'init device for particular record'
    // NULL: get_ioint_info - No 'get io interrupt information' routine provided.
    {8, NULL, (DEVSUPFUN) MDrivePlus_init, (DEVSUPFUN) MDrivePlus_init_record, NULL},
    // The rest are from motor_dset in motor.h
    motor_update_values,
    MDrivePlus_start_trans,
    MDrivePlus_build_trans,
    MDrivePlus_end_trans
};

extern "C" {epicsExportAddress(dset,devMDrivePlus);}

/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static msg_types MDrivePlus_table[] = {
    MOTION,     /* MOVE_ABS */
    MOTION,     /* MOVE_REL */
    MOTION,     /* HOME_FOR */
    MOTION,     /* HOME_REV */
    IMMEDIATE,    /* LOAD_POS */
    IMMEDIATE,    /* SET_VEL_BASE */
    IMMEDIATE,    /* SET_VELOCITY */
    IMMEDIATE,    /* SET_ACCEL */
    IMMEDIATE,    /* GO */
    IMMEDIATE,    /* SET_ENC_RATIO */
    INFO,    /* GET_INFO */
    MOVE_TERM,    /* STOP_AXIS */
    VELOCITY,    /* JOG */
    IMMEDIATE,    /* SET_PGAIN */
    IMMEDIATE,    /* SET_IGAIN */
    IMMEDIATE,    /* SET_DGAIN */
    IMMEDIATE,    /* ENABLE_TORQUE */
    IMMEDIATE,    /* DISABL_TORQUE */
    IMMEDIATE,    /* PRIMITIVE */
    IMMEDIATE,    /* SET_HIGH_LIMIT */
    IMMEDIATE,    /* SET_LOW_LIMIT */
    VELOCITY,   /* JOG_VELOCITY */
    IMMEDIATE,    /* SET_RESOLUTION */
    IMMEDIATE     /* LOAD_MCODE */
};


static struct board_stat **MDrivePlus_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for MDrivePlus stepper motor */
STATIC long MDrivePlus_init(void *arg)
{
    long rtnval;
    int after = (arg == 0) ? 0 : 1;

    printf( "** devMDrivePlus is @ 0x%p.\n", &devMDrivePlus );

    if (after == 0)
    {
    drvtabptr = &MDrivePlus_access;
    // Call the driver's initialization routine.
    (drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &MDrivePlus_cards);
    return(rtnval);
}


// extern "C" {
// extern int init_motor( struct motorRecord* arg );
// }

/* initialize a record instance */
STATIC long MDrivePlus_init_record(void *arg)
{
  long ret;
    struct motorRecord *mr = (struct motorRecord *) arg;
    // init_motor( mr );
    ret = motor_init_record_com(mr,
				*drvtabptr->cardcnt_ptr,
				drvtabptr,
				MDrivePlus_cards);
    return( ret );
}


/* start building a transaction */
STATIC long MDrivePlus_start_trans(struct motorRecord *mr)
{
    return(OK);
}


/* end building a transaction */
STATIC RTN_STATUS MDrivePlus_end_trans(struct motorRecord *mr)
{
    return(OK);
}


/* add a part to the transaction */
STATIC RTN_STATUS MDrivePlus_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    register struct mess_info *motor_info;
    struct controller *brdptr;
    struct IM483controller *cntrl;
    char buff[110];
    int axis, card, intval=0;
    unsigned int size;
    RTN_STATUS rtnval;
    bool send;
    msta_field msta;

    send = true;        /* Default to send motor command. */
    rtnval = OK;
    buff[0] = '\0';

    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
    if ( command != LOAD_MCODE ) intval = (parms == NULL) ? 0 : NINT(parms[0]);

    msta.All = mr->msta;

    motor_start_trans_com(mr, MDrivePlus_cards);
    
    motor_call = &(trans->motor_call);
    card = motor_call->card;
    axis = motor_call->signal + 1;
    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
    return(rtnval = ERROR);

    cntrl = (struct IM483controller *) brdptr->DevicePrivate;
    motor_info = &(brdptr->motor_info[motor_call->signal]);
    
    if (MDrivePlus_table[command] > motor_call->type)
    motor_call->type = MDrivePlus_table[command];

    if (trans->state != BUILD_STATE)
    return(rtnval = ERROR);

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
    {
    // strcat(motor_call->message, " ");
    // strcat(motor_call->message, mr->init);
    // printf( "Card #%d: Setting init: '%s'.\n", card, mr->init );
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
        strcat(motor_call->message, mr->prem);
        strcat(motor_call->message, " ");
        }
        if (strlen(mr->post) != 0)
        motor_call->postmsgptr = (char *) &mr->post;
        break;
        
    default:
        break;
    }


    switch (command)
    {
    case MOVE_ABS:
        motor_info->p_time  = time(NULL);
        motor_info->RA_DONE = 0;

        sprintf(buff, "MA %d", intval);
        break;
    
    case MOVE_REL:
        sprintf(buff, "MR %d", intval);
        break;
    
    case HOME_FOR:
        sprintf(buff, " F1000 0");
        break;

    case HOME_REV:
        sprintf(buff, " F1000 1");
        break;
    
    case LOAD_POS:
        sprintf(buff, "P=%d", intval);
        if (msta.Bits.EA_PRESENT == 1)
        {
        /* Finish 1st message; MDrivePlus can only handle one msg. */
                strcpy(motor_call->message, buff);
                rtnval = motor_end_trans_com(mr, drvtabptr);
        rtnval = (RTN_STATUS) motor_start_trans_com(mr, MDrivePlus_cards);

        intval = NINT(mr->dval / mr->eres);
        sprintf(buff, "C2=%d", intval);
        motor_call->type = MDrivePlus_table[command];
        }
        break;
    
    case SET_VEL_BASE:
        sprintf(buff, "VI %d", intval);
        break;
    
    case SET_VELOCITY:
        sprintf(buff, "VM %d", intval);
        break;
    
    case SET_ACCEL:
        sprintf(buff, "A=%d", intval);
        strcpy(motor_call->message, buff);
        rtnval = motor_end_trans_com(mr, drvtabptr);
        rtnval = (RTN_STATUS) motor_start_trans_com(mr, MDrivePlus_cards);
        sprintf(buff, "D=A");
        motor_call->type = MDrivePlus_table[command];
        break;
    
    case GO:
        /* The MDrivePlus starts moving immediately on move commands, GO command
         * does nothing. */
        send = false;
        break;
    
    case PRIMITIVE:
    case GET_INFO:
        /* These commands are not actually done by sending a message, but
           rather they will indirectly cause the driver to read the status
           of all motors */
        break;
    
    case STOP_AXIS:
        sprintf(buff, "SL 0");
        break;
    
    case JOG_VELOCITY:
    case JOG:
        sprintf(buff, "SL=%d", intval);
        break;
    
    case SET_PGAIN:
    case SET_IGAIN:
    case SET_DGAIN:
        send = false;
        break;
    
    case ENABLE_TORQUE:
        sprintf(buff, "DE=1");
        break;
    
    case DISABL_TORQUE:
        sprintf(buff, "DE=0");
        break;
    
    case SET_HIGH_LIMIT:
    case SET_LOW_LIMIT:
    case SET_ENC_RATIO:
        trans->state = IDLE_STATE;    /* No command sent to the controller. */
        send = false;
        break;
    
    case LOAD_MCODE:
        sprintf(buff, "%s", (char *)parms);
        break;
    
    default:
        send = false;
        rtnval = ERROR;
    }

    if ( send == true )
    {
        if ( (strlen(motor_call->message) + strlen(buff)) > MAX_MSG_SIZE )
        {
            errlogMessage( "MDrivePlus_build_trans(): buffer overflow.\n" );
            return( ERROR );
        }

        strcat( motor_call->message, buff );
        motor_end_trans_com(mr, drvtabptr);
    }

    return(rtnval);
}
