#include <stddef.h>
#include "epicsThread.h"
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "drvXPSAsyn.h"
#include "paramLib.h"

#include "epicsFindSymbol.h"
#include "epicsTime.h"
#include "epicsThread.h"
#include "epicsEvent.h"
#include "epicsString.h"
#include "epicsMutex.h"
#include "ellLib.h"
#include "asynDriver.h"

#include "drvSup.h"
#include "epicsExport.h"
#define DEFINE_MOTOR_PROTOTYPES 1
#include "motor_interface.h"
#include "xps_c8_driver.h"

motorAxisDrvSET_t motorXPS = 
  {
    20,
    motorAxisReport,            /**< Standard EPICS driver report function (optional) */
    motorAxisInit,              /**< Standard EPICS dirver initialisation function (optional) */
    motorAxisSetLog,            /**< Defines an external logging function (optional) */
    motorAxisSetLogParam,       /**< Defines an external logging function user parameter (optional) */
    motorAxisOpen,              /**< Driver open function */
    motorAxisClose,             /**< Driver close function */
    motorAxisSetCallback,       /**< Provides a callback function the driver can call when the status updates */
    motorAxisPrimitive,         /**< Passes a controller dependedent string */
    motorAxisSetDouble,         /**< Pointer to function to set a double value */
    motorAxisSetInteger,        /**< Pointer to function to set an integer value */
    motorAxisGetDouble,         /**< Pointer to function to get a double value */
    motorAxisGetInteger,        /**< Pointer to function to get an integer value */
    motorAxisHome,              /**< Pointer to function to execute a more to reference or home */
    motorAxisMove,              /**< Pointer to function to execute a position move */
    motorAxisVelocityMove,      /**< Pointer to function to execute a velocity mode move */
    motorAxisStop               /**< Pointer to function to stop motion */
  };

epicsExportAddress(drvet, motorXPS);

typedef enum { none, positionMove, velocityMove, homeReverseMove, homeForwardsMove } moveType;

/* typedef struct motorAxis * AXIS_ID; */

typedef struct {
    epicsMutexId XPSC8Lock;
    asynUser *pasynUser;   /* For everything except moving */
    int numAxes;
    char firmwareVersion[100];
    double movingPollPeriod;
    double idlePollPeriod;
    epicsEventId pollEventId;
    AXIS_HDL pAxis;  /* array of axes */
} XPSController;

typedef struct motorAxisHandle
{
    XPSController *pController;
    int moveSocket;
    int pollSocket;
    PARAMS params;
    double currentPosition;
    double setpointPosition;
    double targetPosition;
    double velocity;
    double accel;
    double minJerkTime; /* for the SGamma function */
    double maxJerkTime;
    double jogVelocity;
    double jogAccel;
    double highLimit;
    double lowLimit;
    double stepSize;
    char *ip;
    char *positionerName;  /* read in using NameConfig*/
    char *groupName;
    int axisStatus;
    int positionerError;
    int moving;         /* 1 = moving! */
    int card;
    int axis;
    void *logParam;
    epicsMutexId mutexId;
} motorAxis;

typedef struct
{
  AXIS_HDL pFirst;
  epicsThreadId motorThread;
  motorAxisLogFunc print;
  epicsTimeStamp now;
} motorXPS_t;

static int motorXPSLogMsg(void * param, const motorAxisLogMask_t logMask, const char *pFormat, ...);
#define PRINT   (drv.print)
#define FLOW    motorAxisTraceFlow
#define ERROR   motorAxisTraceError

#define XPS_MAX_AXES 8
#define XPSC8_END_OF_RUN_MINUS  0x00000100
#define XPSC8_END_OF_RUN_PLUS   0x00000200

#define TCP_TIMEOUT 0.5
static motorXPS_t drv={ NULL, NULL, motorXPSLogMsg, { 0, 0 } };
static int numXPSControllers;
/* Pointer to array of controller strutures */
static XPSController *pXPSController=NULL;

#define MAX(a,b) ((a)>(b)? (a): (b))
#define MIN(a,b) ((a)<(b)? (a): (b))

static char* getXPSError(AXIS_HDL pAxis, int status, char *buffer);

static void motorAxisReportAxis(AXIS_HDL pAxis, int level)
{
    if (level > 0)
    {
        printf("Axis %d name:%s\n", pAxis->axis, pAxis->positionerName);
        printf("   pollSocket:%d, moveSocket:%d\n", pAxis->pollSocket, pAxis->moveSocket);
        printf("   axisStatus:%d\n", pAxis->axisStatus);
        printf("   stepSize:%g\n", pAxis->stepSize);
    }
}

static void motorAxisReport(int level)
{
    int i, j;

    for(i=0; i<numXPSControllers; i++) {
        printf("Controller %d firmware version: %s\n", i, pXPSController[i].firmwareVersion);
        for(j=0; j<pXPSController[i].numAxes; j++) {
           motorAxisReportAxis(&pXPSController[i].pAxis[j], level);
        }
    }   
}


static int motorAxisInit(void)
{
  return MOTOR_AXIS_OK;
}

static int motorAxisSetLog(motorAxisLogFunc logFunc)
{
  if (logFunc == NULL) drv.print=motorXPSLogMsg;
  else drv.print = logFunc;

  return MOTOR_AXIS_OK;
}

static int motorAxisSetLogParam(AXIS_HDL pAxis, void * param)
{
  if (pAxis == NULL) return MOTOR_AXIS_ERROR;
  else
    {
      pAxis->logParam = param;
    }
  return MOTOR_AXIS_OK;
}

static AXIS_HDL motorAxisOpen(int card, int axis, char * param)
{
  AXIS_HDL pAxis;

  if (card > numXPSControllers) return(NULL);
  if (axis > pXPSController[card].numAxes) return(NULL);
  pAxis = &pXPSController[card].pAxis[axis];
  pAxis->params = motorParam->create(0, MOTOR_AXIS_NUM_PARAMS);
  return pAxis;
}

static int motorAxisClose(AXIS_HDL pAxis)
{
  return MOTOR_AXIS_OK;
}

static int motorAxisGetInteger(AXIS_HDL pAxis, motorAxisParam_t function, int * value)
{
  if (pAxis == NULL) return MOTOR_AXIS_ERROR;
  else
    {
      return motorParam->getInteger(pAxis->params, (paramIndex) function, value);
    }
}

static int motorAxisGetDouble(AXIS_HDL pAxis, motorAxisParam_t function, double * value)
{
  if (pAxis == NULL) return MOTOR_AXIS_ERROR;
  else
    {
      return motorParam->getDouble(pAxis->params, (paramIndex) function, value);
    }
}

static int motorAxisSetCallback(AXIS_HDL pAxis, motorAxisCallbackFunc callback, void * param)
{
  if (pAxis == NULL) return MOTOR_AXIS_ERROR;
  else
    {
      return motorParam->setCallback(pAxis->params, callback, param);
    }
}

static int motorAxisPrimitive(AXIS_HDL pAxis, int length, char * string)
{
  return MOTOR_AXIS_OK;
}

static int motorAxisSetDouble(AXIS_HDL pAxis, motorAxisParam_t function, double value)
{
    int ret_status = MOTOR_AXIS_ERROR;
    int status;
    double deviceValue;

    if (pAxis == NULL) return MOTOR_AXIS_ERROR;
    else
    {
        switch (function)
        {
        case motorAxisPosition:
        {
            PRINT(pAxis->logParam, ERROR, "motorAxisSetDouble: XPS does not support setting position\n");
            break;
        }
        case motorAxisEncoderRatio:
        {
            PRINT(pAxis->logParam, ERROR, "motorAxisSetDouble: XPS does not support setting encoder ratio\n");
            break;
        }
        case motorAxisResolution:
        {
            pAxis->stepSize = value;
            PRINT(pAxis->logParam, FLOW, "Set card %d, axis %d stepSize to %f\n", pAxis->card, pAxis->axis, value);
            break;
        }
        case motorAxisLowLimit:
        {
            deviceValue = value*pAxis->stepSize;
            status = PositionerUserTravelLimitsSet(pAxis->pollSocket,
                                                   pAxis->positionerName,
                                                   deviceValue, pAxis->highLimit);
            if (status != 0) {
                PRINT(pAxis->logParam, ERROR, "motorAxisSetDouble: error performing PositionerUserTravelLimitsSet "
                      "for low limit=%f, status=%d\n", deviceValue, status);
            } else { 
                pAxis->lowLimit = deviceValue;
                PRINT(pAxis->logParam, FLOW, "Set card %d, axis %d low limit to %f\n", pAxis->card, pAxis->axis, deviceValue);
                ret_status = MOTOR_AXIS_OK;
            }
            break;
        }
        case motorAxisHighLimit:
        {
            deviceValue = value*pAxis->stepSize;
            status = PositionerUserTravelLimitsSet(pAxis->pollSocket,
                                                   pAxis->positionerName,
                                                   pAxis->lowLimit, deviceValue);
            if (status != 0) {
                PRINT(pAxis->logParam, ERROR, "motorAxisSetDouble: error performing PositionerUserTravelLimitsSet "
                      "for high limit=%f, status=%d\n", deviceValue, status);
            } else { 
                pAxis->highLimit = deviceValue;
                PRINT(pAxis->logParam, FLOW, "Set card %d, axis %d high limit to %f\n", pAxis->card, pAxis->axis, deviceValue);
                ret_status = MOTOR_AXIS_OK;
            }
            break;
        }
        case motorAxisPGain:
        {
            PRINT(pAxis->logParam, ERROR, "XPS does not support setting proportional gain\n");
            break;
        }
        case motorAxisIGain:
        {
            PRINT(pAxis->logParam, ERROR, "XPS does not support setting integral gain\n");
            break;
        }
        case motorAxisDGain:
        {
            PRINT(pAxis->logParam, ERROR, "XPS does not support setting derivative gain\n");
            break;
        }
        case motorAxisClosedLoop:
        {
            PRINT(pAxis->logParam, ERROR, "XPS does not support changing closed loop or torque\n");
            break;
        }
        default:
            PRINT(pAxis->logParam, ERROR, "motorAxisSetDouble: unknown function %d\n", function);
            break;
        }
    }
    if (ret_status != MOTOR_AXIS_ERROR) status = motorParam->setDouble(pAxis->params, function, value);
    return ret_status;
}

static int motorAxisSetInteger(AXIS_HDL pAxis, motorAxisParam_t function, int value)
{
    int ret_status = MOTOR_AXIS_ERROR;
    int status;

    if (pAxis == NULL) return MOTOR_AXIS_ERROR;

    switch (function) {
    case motorAxisClosedLoop:
        if (value) {
            status = GroupMotionEnable(pAxis->pollSocket, pAxis->groupName);
            if (status) {
                PRINT(pAxis->logParam, ERROR, "motorAxisSetInteger: error calling GroupMotionEnable status=%d\n",
                      status);
            } else {
                PRINT(pAxis->logParam, FLOW, "motorAxisSetInteger set card %d, axis %d closed loop enable\n",
                      pAxis->card, pAxis->axis);
            }
            ret_status = MOTOR_AXIS_OK;
        } else {
            status = GroupMotionDisable(pAxis->pollSocket, pAxis->groupName);
            if (status) {
                PRINT(pAxis->logParam, ERROR, "motorAxisSetInteger: error calling GroupMotionDisable status=%d\n",
                      status);
            } else {
                PRINT(pAxis->logParam, FLOW, "motorAxisSetInteger set card %d, axis %d closed loop disable\n",
                      pAxis->card, pAxis->axis);
            }
            ret_status = MOTOR_AXIS_OK;
        }
        break;
    default:
        PRINT(pAxis->logParam, ERROR, "motorAxisSetInteger: unknown function %d\n", function);
        break;
    }
    if (ret_status != MOTOR_AXIS_ERROR) status = motorParam->setInteger(pAxis->params, function, value);
    return ret_status;
}


static int motorAxisMove(AXIS_HDL pAxis, double position, int relative, 
                          double min_velocity, double max_velocity, double acceleration)
{
    int status;
    char errorString[100];
    double deviceUnits;

    if (pAxis == NULL) return MOTOR_AXIS_ERROR;

    /* Look at the last poll value of the positioner status.  If it is disabled, then enable it */
    if (pAxis->axisStatus >= 20 && pAxis->axisStatus <= 36) {
        status = GroupMotionEnable(pAxis->pollSocket, pAxis->groupName);
        if (status) {
            PRINT(pAxis->logParam, ERROR, "motorAxisMove: error performing GroupMotionEnable %d\n",status);
            /* Error -27 is caused when the motor record changes dir i.e. when it aborts a move!*/
            return MOTOR_AXIS_ERROR;
        }
    }
    status = PositionerSGammaParametersSet(pAxis->pollSocket,
                                           pAxis->positionerName, 
                                           max_velocity*pAxis->stepSize,
                                           acceleration*pAxis->stepSize,
                                           pAxis->minJerkTime,
                                           pAxis->maxJerkTime);
    if (status != 0) {
        ErrorStringGet(pAxis->pollSocket, status, errorString);
        PRINT(pAxis->logParam, ERROR, " Error performing PositionerSGammaParametersSet %d: %s\n",
              status, errorString);
        return MOTOR_AXIS_ERROR;
    }

    deviceUnits = position * pAxis->stepSize;
    if (relative) {
        status = GroupMoveRelative(pAxis->moveSocket,
                                   pAxis->positionerName,
                                   1,
                                   &deviceUnits); 
        if (status != 0 && status != -27) {
            PRINT(pAxis->logParam, ERROR, " Error performing GroupMoveRelative %d\n",status);
            /* Error -27 is caused when the motor record changes dir i.e. when it aborts a move!*/
            return MOTOR_AXIS_ERROR;
        }
    } else {
        status = GroupMoveAbsolute(pAxis->moveSocket,
                                   pAxis->positionerName,
                                   1,
                                   &deviceUnits); 
        if (status != 0 && status != -27) {
            PRINT(pAxis->logParam, ERROR, " Error performing GroupMoveAbsolute %d\n",status);
            /* Error -27 is caused when the motor record changes dir i.e. when it aborts a move!*/
            return MOTOR_AXIS_ERROR;
        }
    }
    /* Send a signal to the poller task which will make it do a poll, and switch to the moving poll rate */
    epicsEventSignal(pAxis->pController->pollEventId);

    PRINT(pAxis->logParam, FLOW, "Set card %d, axis %d move to %f, min vel=%f, max_vel=%f, accel=%f\n",
          pAxis->card, pAxis->axis, position, min_velocity, max_velocity, acceleration);
    return MOTOR_AXIS_OK;
}

static int motorAxisHome(AXIS_HDL pAxis, double min_velocity, double max_velocity, double acceleration, int forwards)
{
    int status;
    int groupStatus;
    char errorBuffer[100];

    if (pAxis == NULL) return MOTOR_AXIS_ERROR;

    status = GroupStatusGet(pAxis->pollSocket, pAxis->groupName, &groupStatus);
    /* The XPS won't allow a home command if the group is in the Ready state
     * If the group is Ready, then make it not Ready  */
    if (groupStatus >= 10 && groupStatus <= 18) {
        status = GroupKill(pAxis->pollSocket, pAxis->groupName);
        if (status) {
            PRINT(pAxis->logParam, ERROR, "motorAxisHome: error calling GroupKill error=%s\n",
                  getXPSError(pAxis, status, errorBuffer));
            return MOTOR_AXIS_ERROR;
        }
    }
    status = GroupStatusGet(pAxis->pollSocket, pAxis->groupName, &groupStatus);
    /* If axis not initialized, then initialize it */
    if (groupStatus >= 0 && groupStatus <= 9) {
        status = GroupInitialize(pAxis->pollSocket, pAxis->groupName);
        if (status) {
            PRINT(pAxis->logParam, ERROR, "motorAxisHome: error calling GroupInitialize error=%s\n",
                  getXPSError(pAxis, status, errorBuffer));
            return MOTOR_AXIS_ERROR;
        }
    }
    status = GroupHomeSearch(pAxis->moveSocket, pAxis->groupName);
    if (status) {
        PRINT(pAxis->logParam, ERROR, "motorAxisHome: error calling GroupInitialize error=%s\n",
              getXPSError(pAxis, status, errorBuffer));
        return MOTOR_AXIS_ERROR;
    }

    /* Send a signal to the poller task which will make it do a poll, and switch to the moving poll rate */
    epicsEventSignal(pAxis->pController->pollEventId);
    PRINT(pAxis->logParam, FLOW, "motorAxisHome: set card %d, axis %d to home\n",
          pAxis->card, pAxis->axis);
    return MOTOR_AXIS_OK;
}


static int motorAxisVelocityMove(AXIS_HDL pAxis, double min_velocity, double velocity, double acceleration)
{
    int status = MOTOR_AXIS_ERROR;
    double deviceVelocity, deviceAcceleration;

    if (pAxis == NULL) return(status);
    status = GroupJogModeEnable(pAxis->pollSocket, pAxis->groupName);
    if (status) {
        PRINT(pAxis->logParam, ERROR, "motorAxisVelocityMove: error calling GroupJogModeEnable=%d\n",
              status);
        return MOTOR_AXIS_ERROR;
    }
    deviceVelocity = velocity * pAxis->stepSize;
    deviceAcceleration = acceleration * pAxis->stepSize;
    status = GroupJogParametersSet(pAxis->moveSocket, pAxis->positionerName, 1, &deviceVelocity, &deviceAcceleration);
    if (status) {
        PRINT(pAxis->logParam, ERROR, "motorAxisVelocityMove: error calling GroupJogParametersSet=%d\n",
              status);
        return MOTOR_AXIS_ERROR;
    }
    /* Send a signal to the poller task which will make it do a poll, and switch to the moving poll rate */
    epicsEventSignal(pAxis->pController->pollEventId);
    PRINT(pAxis->logParam, FLOW, "motorAxisVelocityMove card %d, axis %d move velocity=%f, accel=%f\n",
          pAxis->card, pAxis->axis, velocity, acceleration);
    return status;
}

static int motorAxisProfileMove(AXIS_HDL pAxis, int npoints, double positions[], double times[], int relative, int trigger)
{
  return MOTOR_AXIS_ERROR;
}

static int motorAxisTriggerProfile(AXIS_HDL pAxis)
{
  return MOTOR_AXIS_ERROR;
}

static int motorAxisStop(AXIS_HDL pAxis, double acceleration)
{
    int status;
    double deviceVelocity=0.;
    double deviceAcceleration;

    if (pAxis == NULL) return MOTOR_AXIS_ERROR;
    
    /* We need to read the status, because a jog is stopped differently from a move */ 

    status = GroupStatusGet(pAxis->pollSocket, pAxis->groupName, &pAxis->axisStatus);
    if (status != 0) {
        PRINT(pAxis->logParam, ERROR, " Error performing GroupStatusGet status=%d%\n",\
              status);
        return MOTOR_AXIS_ERROR;
    }
    if (pAxis->axisStatus == 47) {
        deviceAcceleration = acceleration * pAxis->stepSize;
        status = GroupJogParametersSet(pAxis->moveSocket, pAxis->positionerName, 1, &deviceVelocity, &deviceAcceleration);
        if (status != 0) {
            PRINT(pAxis->logParam, ERROR, " Error performing GroupJogParametersSet status=%d\n",\
                  status);
            return MOTOR_AXIS_ERROR;
        }
    }
    
    if (pAxis->axisStatus == 44) {
        status = GroupMoveAbort(pAxis->pollSocket, pAxis->groupName);
        if (status != 0) {
            PRINT(pAxis->logParam, ERROR, " Error performing GroupMoveAbort axis=%s status=%d\n",\
                  pAxis->positionerName, status);
            return MOTOR_AXIS_ERROR;
        }
    }

    PRINT(pAxis->logParam, FLOW, "Set card %d, axis %d to stop with accel=%f\n",
          pAxis->card, pAxis->axis, acceleration);
    return MOTOR_AXIS_OK;
}


static void XPSPoller(XPSController *pController)
{
    /* This is the task that polls the XPS */
    double timeout;
    AXIS_HDL pAxis;
    int status;
    int i;
    int axisDone;
    int anyMoving;
    int forcedFastPolls=0;
    double actualVelocity, theoryVelocity, acceleration;

    timeout = pController->idlePollPeriod;
    epicsEventSignal(pController->pollEventId);  /* Force on poll at startup */

    while(1) {
        if (timeout != 0.) status = epicsEventWaitWithTimeout(pController->pollEventId, timeout);
        else               status = epicsEventWait(pController->pollEventId);
        if (status == epicsEventWaitOK) {
            /* We got an event, rather than a timeout.  This is because other software
             * knows that an axis should have changed state (started moving, etc.).
             * Force a minimum number of fast polls, because the controller status
             * might not have changed the first few polls
             */
            forcedFastPolls = 10;
         }
        
        anyMoving = 0;
        for (i=0; i<pController->numAxes; i++) {
            pAxis = &pController->pAxis[i];
            if (!pAxis->mutexId) break;
            epicsMutexLock(pAxis->mutexId);
            status = GroupStatusGet(pAxis->pollSocket, 
                                    pAxis->groupName, 
                                    &pAxis->axisStatus);
            if (status != 0) {
                PRINT(pAxis->logParam, ERROR, "XPSPoller: error calling GroupStatusGet, status=%d\n");
                motorParam->setInteger(pAxis->params, motorAxisCommError, 1);
                continue;
            } else {
                motorParam->setInteger(pAxis->params, motorAxisCommError, 0);
                /* Set done flag by default */
                axisDone = 1;
                if (pAxis->axisStatus >= 10 && pAxis->axisStatus <= 18) {
                    /* These states mean ready from move/home/jog etc */
                }
                if (pAxis->axisStatus >= 43 && pAxis->axisStatus <= 48) {
                    /* These states mean it is moving/homeing/jogging etc*/
                    axisDone = 0;
                    anyMoving = 1;
                    if (pAxis->axisStatus == 47) {
                        /* We are jogging.  When the velocity gets back to 0 disable jogging */
                        status = GroupJogParametersGet(pAxis->pollSocket, pAxis->positionerName, 1, &theoryVelocity, &acceleration);
                        status = GroupJogCurrentGet(pAxis->pollSocket, pAxis->positionerName, 1, &actualVelocity, &acceleration);
                        if (status != 0) {
                            PRINT(pAxis->logParam, ERROR, "XPSPoller: error calling GroupJogCurrentGet, status=%d\n");
                        } else {
                            if (actualVelocity == 0. && theoryVelocity == 0.) {
                                status = GroupJogModeDisable(pAxis->pollSocket, pAxis->groupName);
                                if (status != 0) {
                                    PRINT(pAxis->logParam, ERROR, "XPSPoller: error calling GroupJogModeDisable, status=%d\n", status);
                                    /* In this mode must do a group kill? */
                                    status = GroupKill(pAxis->pollSocket, pAxis->groupName);
                                    PRINT(pAxis->logParam, ERROR, "XPSPoller: called GroupKill!\n");
                                }
                            } 
                        }
                    }
                }
                /* Set the axis done parameter */
                motorParam->setInteger(pAxis->params, motorAxisDone, axisDone);
                if (pAxis->axisStatus == 11) {
                    motorParam->setInteger(pAxis->params, motorAxisHomeSignal, 1);
                } else {
                    motorParam->setInteger(pAxis->params, motorAxisHomeSignal, 0);
                }
                if ((pAxis->axisStatus >= 0 && pAxis->axisStatus <= 9) || 
                    (pAxis->axisStatus >= 20 && pAxis->axisStatus <= 42)) {
                    /* Not initialized, homed or disabled */
                     PRINT(pAxis->logParam, FLOW, "axis %d in bad state %d\n",
                           pAxis->axis, pAxis->axisStatus);
                    /* motorParam->setInteger(pAxis->params, motorAxisHighHardLimit, 1);
                     * motorParam->setInteger(pAxis->params, motorAxisLowHardLimit,  1);
                     */
                }
            }

            status = GroupPositionCurrentGet(pAxis->pollSocket,
                                             pAxis->positionerName,
                                             1,
                                             &pAxis->currentPosition);
            if (status != 0) {
                PRINT(pAxis->logParam, ERROR, "XPSPoller: error calling GroupPositionCurrentGet, status=%d\n");
                motorParam->setInteger(pAxis->params, motorAxisCommError, 1);
            } else {
                motorParam->setInteger(pAxis->params, motorAxisCommError, 0);
                motorParam->setDouble(pAxis->params, motorAxisPosition,    (pAxis->currentPosition/pAxis->stepSize));
                motorParam->setDouble(pAxis->params, motorAxisEncoderPosn, (pAxis->currentPosition/pAxis->stepSize));
            }

            status = PositionerErrorGet(pAxis->pollSocket,
                                        pAxis->positionerName,
                                        &pAxis->positionerError);
            if (status != 0) {
                PRINT(pAxis->logParam, ERROR, "XPSPoller: error calling PositionerErrorGet, status=%d\n");
                motorParam->setInteger(pAxis->params, motorAxisCommError, 1);
            } else {
                motorParam->setInteger(pAxis->params, motorAxisCommError, 0);
                /* These are hard limits */
                if (pAxis->positionerError & XPSC8_END_OF_RUN_PLUS) {
                    motorParam->setInteger(pAxis->params, motorAxisHighHardLimit, 1);
                } else {
                    motorParam->setInteger(pAxis->params, motorAxisHighHardLimit, 0);
                }
                if (pAxis->positionerError & XPSC8_END_OF_RUN_MINUS) {
                    motorParam->setInteger(pAxis->params, motorAxisLowHardLimit, 1);
                } else {
                    motorParam->setInteger(pAxis->params, motorAxisLowHardLimit, 0);
                }
            }

            /* We would like a way to query the actual velocity, but this is not possible.  If we could we could
             * set the direction, and Moving flags
             *     motorParam->setInteger(pAxis->params, motorAxisDirection,     (pAxis->currentVelocity >  0));
             *     motorParam->setInteger(pAxis->params, motorAxisMoving,        (pAxis->nextpoint.axis[0].v != 0));
             */

            motorParam->callCallback(pAxis->params);

            epicsMutexUnlock(pAxis->mutexId);

            if (forcedFastPolls > 0) {
                timeout = pController->movingPollPeriod;
                forcedFastPolls--;
            } else if (anyMoving) {
                timeout = pController->movingPollPeriod;
            } else {
                timeout = pController->idlePollPeriod;
            }

        } /* Next axis */

    } /* End while */

}

static char *getXPSError(AXIS_HDL pAxis, int status, char *buffer)
{
    status = ErrorStringGet(pAxis->pollSocket, status, buffer);
    return buffer;
}

static int motorXPSLogMsg(void * param, const motorAxisLogMask_t mask, const char *pFormat, ...)
{

    va_list     pvar;
    int         nchar;

    va_start(pvar, pFormat);
    nchar = vfprintf(stdout,pFormat,pvar);
    va_end (pvar);
    printf("\n");
    return(nchar);
}


int XPSSetup(int num_controllers)   /* number of XPS controllers in system.  */
{

    if (num_controllers < 1) {
        printf("XPSSetup, num_controllers must be > 0\n");
        return MOTOR_AXIS_ERROR;
    }
    numXPSControllers = num_controllers;
    pXPSController = (XPSController *)calloc(numXPSControllers, sizeof(XPSController)); 
    return MOTOR_AXIS_OK;
}


int XPSConfig(int card,           /* Controller number */
              const char *ip,     /* XPS IP address or IP name */
              int port,           /* IP port number that XPS is listening on */
              int numAxes,        /* Number of axes this controller supports */
              int movingPollPeriod, /* Time to poll (msec) when an axis is in motion */
              int idlePollPeriod)   /* Time to poll (msec) when an axis is idle. 0 for no polling */

{
    AXIS_HDL pAxis;
    int axis;
    int pollSocket;
    XPSController *pController;
    char threadName[20];

    if (numXPSControllers < 1) {
        printf("XPSConfig: no XPS controllers allocated, call XPSSetup first\n");
        return MOTOR_AXIS_ERROR;
    }
    if ((card < 0) || (card >= numXPSControllers)) {
        printf("XPSConfig: card must in range 0 to %d\n", numXPSControllers-1);
        return MOTOR_AXIS_ERROR;
    }
    if ((numAxes < 1) || (numAxes > XPS_MAX_AXES)) {
        printf("XPSConfig: numAxes must in range 1 to %d\n", XPS_MAX_AXES);
        return MOTOR_AXIS_ERROR;
    }

    pController = &pXPSController[card];
    pController->pAxis = (AXIS_HDL) calloc(numAxes, sizeof(motorAxis));
    pController->numAxes = numAxes;
    pController->movingPollPeriod = movingPollPeriod/1000.;
    pController->idlePollPeriod = idlePollPeriod/1000.;

    pollSocket = TCP_ConnectToServer((char *)ip, port, TCP_TIMEOUT);

    if (pollSocket < 0) {
        printf("XPSConfig: error calling TCP_ConnectToServer for pollSocket\n");
        return MOTOR_AXIS_ERROR;
    }

    for (axis=0; axis<numAxes; axis++) {
        pAxis = &pController->pAxis[axis];
        pAxis->pController = pController;
        pAxis->card = card;
        pAxis->axis = axis;
        pAxis->pollSocket = pollSocket;
        pAxis->ip = epicsStrDup(ip);
        pAxis->moveSocket = TCP_ConnectToServer((char *)ip, port, TCP_TIMEOUT);
        if (pAxis->moveSocket < 0) {
            printf("XPSConfig: error calling TCP_ConnectToServer for move socket\n");
            return MOTOR_AXIS_ERROR;
        }
        /* Set the poll rate on the moveSocket to a negative number, which means that SendAndReceive should do only a write, no read */
        TCP_SetTimeout(pAxis->moveSocket, -0.1);
        printf("XPSConfig: pollSocket=%d, moveSocket=%d, ip=%s, port=%d,"
              " axis=%d controller=%d\n",
              pAxis->pollSocket, pAxis->moveSocket, ip, port, axis, card);
    }

    FirmwareVersionGet(pollSocket, pController->firmwareVersion);
    
    pController->pollEventId = epicsEventMustCreate(epicsEventEmpty);

    /* Create the poller thread for this controller */
    epicsSnprintf(threadName, sizeof(threadName), "XPS:%d", card);
    epicsThreadCreate(threadName,
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC) XPSPoller, (void *) pController);

    return MOTOR_AXIS_OK;

}


int XPSConfigAxis(int card,                   /* specify which controller 0-up*/
                  int axis,                   /* axis number 0-7 */
                  const char *positionerName, /* groupName.positionerName e.g. Diffractometer.Phi */
                  int stepsPerUnit)           /* steps per user unit */
{
    XPSController *pController;
    AXIS_HDL pAxis;
    char *index;
    int status;

    if (numXPSControllers < 1) {
        printf("XPSConfigAxis: no XPS controllers allocated, call XPSSetup first\n");
        return MOTOR_AXIS_ERROR;
    }
    if ((card < 0) || (card >= numXPSControllers)) {
        printf("XPSConfigAxis: card must in range 0 to %d\n", numXPSControllers-1);
        return MOTOR_AXIS_ERROR;
    }
    pController = &pXPSController[card];
    if ((axis < 0) || (axis >= pController->numAxes)) {
        printf("XPSConfigAxis: axis must in range 0 to %d\n", pController->numAxes-1);
        return MOTOR_AXIS_ERROR;
    }
    pAxis = &pController->pAxis[axis];
    index = strchr(positionerName, '.');
    if (index == NULL) {
        printf("XPSConfigAxis: positionerName must be of form group.positioner\n");
        return MOTOR_AXIS_ERROR;
    }
    pAxis->positionerName = epicsStrDup(positionerName);
    pAxis->groupName = epicsStrDup(positionerName);
    index = strchr(pAxis->groupName, '.');
    if (index != NULL) *index = '\0';  /* Terminate group name at place of '.' */

    pAxis->stepSize = 1./stepsPerUnit;
    /* Read some information from the controller for this axis */
    status = PositionerSGammaParametersGet(pAxis->pollSocket,
                                           pAxis->positionerName,
                                           &pAxis->velocity,
                                           &pAxis->accel,
                                           &pAxis->minJerkTime,
                                           &pAxis->maxJerkTime);
    pAxis->mutexId = epicsMutexMustCreate();

    /* Send a signal to the poller task which will make it do a poll, 
     * updating values for this axis to use the new resolution (stepSize) */
    epicsEventSignal(pAxis->pController->pollEventId);
    
    return MOTOR_AXIS_OK;
}

