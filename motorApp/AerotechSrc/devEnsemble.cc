/*
 * FILENAME... devEnsemble.cc
 * USAGE... Motor record device level support for Aerotech Ensemble.
 *
 * Version:        1.4.2.3
 * Modified By:    sluiter
 * Last Modified:  2009/06/22 17:53:29
 */

/*
 *      Original Author: Chad Weimer
 *      Date: 04/04/08
 *	Current Author: Chad Weimer
 *
 *	Experimental Physics and Industrial Control System (EPICS)
 *
 *	Copyright 1991, the Regents of the University of California,
 *	and the University of Chicago Board of Governors.
 *
 *	This software was produced under  U.S. Government contracts:
 *	(W-7405-ENG-36) at the Los Alamos National Laboratory,
 *	and (W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *	Initial development by:
 *		The Controls and Automation Group (AT-8)
 *		Ground Test Accelerator
 *		Accelerator Technology Division
 *		Los Alamos National Laboratory
 *
 *	Co-developed with
 *		The Controls and Computing Group
 *		Accelerator Systems Division
 *		Advanced Photon Source
 *		Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01  04-01-08 caw  initialized from devMM4000.cc (Newport)
 * .02  11-20-08 rls  - support homing in both directions.
 *                    - disable LOAD_POS until ABORT command fixed.
 *                    - SET_VELOCITY sets both default speed and home feedrate.
 * .03  01-22-09 rls  - enable LOAD_POS; ABORT command fixed with Ensemble
 *                      firmware 2.5.2.
 * .04  05-01-09 rls  - Fix for jog velocity not adjusted by
 *                      cntrl->drive_resolution.
 */


#include <string.h>
#include <math.h>
#include "motorRecord.h"
#include "motor.h"
#include "motordevCom.h"
#include "drvEnsemble.h"
#include "epicsExport.h"

extern struct driver_table Ensemble_access;

/* ----------------Create the dsets for devEnsemble----------------- */
static struct driver_table *drvtabptr;
static long Ensemble_init (void *);
static long Ensemble_init_record (void *);
static long Ensemble_start_trans (struct motorRecord *);
static RTN_STATUS Ensemble_build_trans (motor_cmnd, double *,
                                        struct motorRecord *);
static RTN_STATUS Ensemble_end_trans (struct motorRecord *);

struct motor_dset devEnsemble =
{
    {8, NULL, (DEVSUPFUN) Ensemble_init, (DEVSUPFUN) Ensemble_init_record, NULL},
    motor_update_values,
    Ensemble_start_trans,
    Ensemble_build_trans,
    Ensemble_end_trans
};

extern "C" {epicsExportAddress (dset, devEnsemble);}
/* --------------------------- program data --------------------- */

/*
 * This table is used to define the command types
 * WARNING! this must match "motor_cmnd" in motor.h
 */

static msg_types Ensemble_table[] = {
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
    IMMEDIATE,  /* SET_LOW_LIMIT */
    VELOCITY,   /* JOG_VELOCITY */
    IMMEDIATE   /* SET_RESOLUTION */
};


static struct board_stat **Ensemble_cards;

/* --------------------------- program data --------------------- */


// initialize device support for Ensemble
static long Ensemble_init (void *arg)
{
    long rtnval;
    int after = (arg == 0) ? 0 : 1;

    if (after == 0)
    {
        drvtabptr = &Ensemble_access;
        (drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr,
                            drvtabptr, &Ensemble_cards);
    return(rtnval);
}


// initialize a record instance 
static long Ensemble_init_record (void *arg)
{
    struct motorRecord *mr = (struct motorRecord *) arg;
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr,
                                 drvtabptr, Ensemble_cards));
}


// start building a transaction 
static long Ensemble_start_trans (struct motorRecord *mr)
{
    return(OK);
    //return (motor_start_trans_com(mr, Ensemble_cards));
}


// end building a transaction 
static RTN_STATUS Ensemble_end_trans (struct motorRecord * mr)
{
    return(OK);
    //return (motor_end_trans_com(mr, drvtabptr));
}


// add a part to the transaction
static RTN_STATUS Ensemble_build_trans (motor_cmnd command, double *parms,
                                        struct motorRecord * mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct Ensemblecontroller *cntrl;
    char buff[BUFF_SIZE], temp[BUFF_SIZE];
    int axis, card, maxdigits;
    unsigned int size;
    double dval, cntrl_units;
    RTN_STATUS rtnval;
    bool send = true;

    rtnval = OK;
    buff[0] = '\0';

    // Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL).
    dval = (parms == NULL) ? 0.0 : *parms;

    motor_start_trans_com(mr, Ensemble_cards);

    motor_call = &(trans->motor_call);
    card = motor_call->card;
    axis = motor_call->signal;
    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
        return(rtnval = ERROR);

    cntrl = (struct Ensemblecontroller *) brdptr->DevicePrivate;
    cntrl_units = dval * cntrl->drive_resolution[axis];
    maxdigits = cntrl->res_decpts[axis];

    if (Ensemble_table[command] > motor_call->type)
    {
        motor_call->type = Ensemble_table[command];
    }

    if (trans->state != BUILD_STATE)
    {
        return(rtnval = ERROR);
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
            sprintf(temp, "%s%c", mr->prem, ASCII_EOS_CHAR);
            strcat(motor_call->message, temp);
        }
        if (strlen(mr->post) != 0)
        {
            motor_call->postmsgptr = (char *) &mr->post;
        }
        break;

    default:
        break;
    }

    switch (command)
    {
    case MOVE_ABS:
        sprintf(buff, "MOVEABS @%d %.*f",
                axis, maxdigits, cntrl_units);
        break;
    case MOVE_REL:
        sprintf(buff, "MOVEINC @%d %.*f",
                axis, maxdigits, cntrl_units);
        break;

    case HOME_FOR:
    case HOME_REV:
        {
            epicsUInt32 hparam = cntrl->home_dparam[axis];
            if (command == HOME_FOR)
                hparam |= 0x00000001;
            else
                hparam &= 0xFFFFFFFE;
            cntrl->home_dparam[axis] = hparam;

            sprintf(buff, "SETPARM @%d, 106, %d", axis, hparam);
            strcpy(motor_call->message, buff);
            rtnval = motor_end_trans_com(mr, drvtabptr);

            rtnval = (RTN_STATUS) motor_start_trans_com(mr, Ensemble_cards);
            sprintf(buff, "HOME @%d", axis);
            motor_call->type = Ensemble_table[command];
        }
        break;

    case LOAD_POS:
        sprintf(buff, "SETPOSCMD @%d, %.*f", axis, maxdigits, cntrl_units);
        break;

    case SET_VEL_BASE:
        send = false;
        break;                  // Ensemble does not use base velocity

    case SET_VELOCITY:
        sprintf(buff, "SETPARM @%d, 102, %.*f", //DefaultSpeed
                axis, maxdigits, cntrl_units);
        strcpy(motor_call->message, buff);
        rtnval = motor_end_trans_com(mr, drvtabptr);

        rtnval = (RTN_STATUS) motor_start_trans_com(mr, Ensemble_cards);
        sprintf(buff, "SETPARM @%d, 107, %.*f", //HomeFeedRate
                axis, maxdigits, cntrl_units);
        motor_call->type = Ensemble_table[command];
        break;

    case SET_ACCEL:
        sprintf(buff, "SETPARM @%d, 103, %.*f", //DefaultRampRate
                axis, maxdigits, cntrl_units);
        break;

    case GO:
        /*
         * The Ensemble starts moving immediately on move commands, GO command
         * does nothing
         */
        send = false;
        break;

    case SET_ENC_RATIO:
        //sprintf(buff, "SETPARM @%d, 3, %.*f", //PosScaleFactor
        //	axis, maxdigits, cntrl_units);
        send = false;
        break;

    case GET_INFO:
        /*
         * These commands are not actually done by sending a message, but
         * rather they will indirectly cause the driver to read the status
         * of all motors 
         */
        break;

    case STOP_AXIS:
        sprintf(buff, "ABORT @%d", axis);
        break;

    case JOG_VELOCITY:
    case JOG:
        sprintf(buff, "FREERUN @%d %.*f", axis, maxdigits, cntrl_units);
        break;

    case SET_PGAIN:
        sprintf(buff, "SETPARM @%d, 8, %.*f", //GainKp
                axis, maxdigits, dval);
        break;

    case SET_IGAIN:
        sprintf(buff, "SETPARM @%d, 7, %.*f", //GainKi
                axis, maxdigits, dval);
        break;

    case SET_DGAIN:
        sprintf(buff, "SETPARM @%d, 12, %.*f", //GainKpi
                axis, maxdigits, dval); // which gain??
        break;

    case ENABLE_TORQUE:
        sprintf(buff, "ENABLE @%d", axis);
        break;

    case DISABL_TORQUE:
        sprintf(buff, "DISABLE @%d", axis);
        break;

    case PRIMITIVE:
        if (mr->init != NULL && strlen(mr->init) != 0)
        {
            strcat(motor_call->message, mr->init);
        }
        break;

    case SET_HIGH_LIMIT:
        sprintf(buff, "SETPARM(@%d, 48, %.*f)", axis, maxdigits, cntrl_units); //ThresholdSoftCW
        //motor_info = &(*trans->tabptr->card_array)[card]->motor_info[axis];
        //trans->state = IDLE_STATE;	// No command sent to the controller.

        //if(cntrl_units > motor_info->high_limit)
        //{
        //	mr->dhlm = motor_info->high_limit;
        //	rtnval = ERROR;
        //}
        //send = false;
        break;

    case SET_LOW_LIMIT:
        sprintf(buff, "SETPARM(@%d, 47, %.*f)", axis, maxdigits, cntrl_units); //ThresholdSoftCCW
        //motor_info = &(*trans->tabptr->card_array)[card]->motor_info[axis];
        //trans->state = IDLE_STATE;	// No command sent to the controller.

        //if(cntrl_units < motor_info->low_limit)
        //{
        //	mr->dllm = motor_info->low_limit;
        //	rtnval = ERROR;
        //}
        //send = false;
        break;

    case SET_RESOLUTION:
        sprintf(buff, "SETPARM @%d, 3, %.*f", //PosScaleFactor
                axis, maxdigits, 1 / cntrl_units);
        break;

    default:
        send = false;
        rtnval = ERROR;
    }

    if (send == true)
    {
        size = strlen (buff);
        if (size > sizeof(buff) || (strlen(motor_call->message) + size) > MAX_MSG_SIZE)
        {
            errlogMessage("Ensemble_build_trans(): buffer overflow.\n");
        }
        else
        {
            strcat(motor_call->message, buff);
            motor_end_trans_com(mr, drvtabptr);
        }
    }

    return(rtnval);
}
