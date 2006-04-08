#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "drvMotorSim.h"
#include "paramLib.h"

#include "epicsFindSymbol.h"
#include "epicsTime.h"
#include "epicsThread.h"
#include "epicsMutex.h"
#include "ellLib.h"

#include "drvSup.h"
#include "epicsExport.h"
#define DEFINE_MOTOR_PROTOTYPES 1
#include "motor_interface.h"

#include "route.h"

motorAxisDrvSET_t motorSim = 
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

epicsExportAddress(drvet, motorSim);

typedef enum { none, positionMove, velocityMove, homeReverseMove, homeForwardsMove } moveType;

/* typedef struct motorAxis * AXIS_ID; */

typedef struct motorAxisHandle
{
  AXIS_HDL pNext;
  AXIS_HDL pPrev;
  int card;
  int axis;
  ROUTE_ID route;
  PARAMS params;
  route_reroute_t reroute;
  route_demand_t endpoint;
  route_demand_t nextpoint;
  double hiHardLimit;
  double lowHardLimit;
  double enc_offset;
  double home;
  int homing;
  void * logParam;
  epicsTimeStamp tLast;
  epicsMutexId axisMutex;
} motorAxis;

typedef struct
{
  AXIS_HDL pFirst;
  epicsThreadId motorThread;
  motorAxisLogFunc print;
  epicsTimeStamp now;
} motorSim_t;

static int motorSimLogMsg( void * param, const motorAxisLogMask_t logMask, const char *pFormat, ...);
#define PRINT   (drv.print)
#define FLOW    motorAxisTraceFlow
#define ERROR   motorAxisTraceError

static motorSim_t drv={ NULL, NULL, motorSimLogMsg, { 0, 0 } };

#define MAX(a,b) ((a)>(b)? (a): (b))
#define MIN(a,b) ((a)<(b)? (a): (b))

static void motorAxisReportAxis( AXIS_HDL pAxis, int level )
{
    printf( "Found driver for motorSim card %d, axis %d", pAxis->card, pAxis->axis );

    if (level > 0)
    {
        double lowSoftLimit=0.0;
        double hiSoftLimit=0.0;

        printf( "Current position = %f, velocity = %f at current time: %f", 
               pAxis->nextpoint.axis[0].p, 
               pAxis->nextpoint.axis[0].v,
               pAxis->nextpoint.T );
        printf( "Destination posn = %f, velocity = %f at desination time:  %f",
               pAxis->endpoint.axis[0].p, 
               pAxis->endpoint.axis[0].v,
               pAxis->endpoint.T );

        printf( "Hard limits: %f, %f", pAxis->lowHardLimit, pAxis->hiHardLimit );
        motorParam->getDouble( pAxis->params, motorAxisHighLimit, &hiSoftLimit );
        motorParam->getDouble( pAxis->params, motorAxisLowLimit, &lowSoftLimit );
        printf( "Soft limits: %f, %f", lowSoftLimit, hiSoftLimit );

        if (pAxis->homing) printf( "Currently homing axis" );

        motorParam->dump( pAxis->params );
    }
}

static void motorAxisReport( int level )
{
    AXIS_HDL pAxis;

    for ( pAxis=drv.pFirst; pAxis != NULL; pAxis = pAxis->pNext ) motorAxisReportAxis( pAxis, level );
}


static int motorAxisInit( void )
{
  return MOTOR_AXIS_OK;
}

static int motorAxisSetLog( motorAxisLogFunc logFunc )
{
  if (logFunc == NULL) drv.print=motorSimLogMsg;
  else drv.print = logFunc;

  return MOTOR_AXIS_OK;
}

static int motorAxisSetLogParam( AXIS_HDL pAxis, void * param )
{
  if (pAxis == NULL) return MOTOR_AXIS_ERROR;
  else
    {
      pAxis->logParam = param;
    }
  return MOTOR_AXIS_OK;
}

static AXIS_HDL motorAxisOpen( int card, int axis, char * param )
{
  AXIS_HDL pAxis;

  for ( pAxis=drv.pFirst;
	pAxis != NULL && (card != pAxis->card || axis != pAxis->axis ); 
	pAxis = pAxis->pNext ) {}

  return pAxis;
}

static int motorAxisClose( AXIS_HDL pAxis )
{
  return MOTOR_AXIS_OK;
}

static int motorAxisGetInteger( AXIS_HDL pAxis, motorAxisParam_t function, int * value )
{
  if (pAxis == NULL) return MOTOR_AXIS_ERROR;
  else
    {
      return motorParam->getInteger( pAxis->params, (paramIndex) function, value );
    }
}

static int motorAxisGetDouble( AXIS_HDL pAxis, motorAxisParam_t function, double * value )
{
  if (pAxis == NULL) return MOTOR_AXIS_ERROR;
  else
    {
      return motorParam->getDouble( pAxis->params, (paramIndex) function, value );
    }
}

static int motorAxisSetCallback( AXIS_HDL pAxis, motorAxisCallbackFunc callback, void * param )
{
  if (pAxis == NULL) return MOTOR_AXIS_ERROR;
  else
    {
      return motorParam->setCallback( pAxis->params, callback, param );
    }
}

static int motorAxisPrimitive( AXIS_HDL pAxis, int length, char * string )
{
  return MOTOR_AXIS_OK;
}

static int motorAxisSetDouble( AXIS_HDL pAxis, motorAxisParam_t function, double value )
{
    int status = MOTOR_AXIS_OK;

    if (pAxis == NULL) return MOTOR_AXIS_ERROR;
    else
    {
        switch (function)
        {
        case motorAxisPosition:
        {
            pAxis->enc_offset = value - pAxis->nextpoint.axis[0].p;
            PRINT( pAxis->logParam, FLOW, "Set card %d, axis %d to position %f", pAxis->card, pAxis->axis, value );
            break;
        }
        case motorAxisResolution:
        {
            PRINT( pAxis->logParam, FLOW, "Set card %d, axis %d resolution to %f", pAxis->card, pAxis->axis, value );
            break;
        }
        case motorAxisEncoderRatio:
        {
            PRINT( pAxis->logParam, FLOW, "Set card %d, axis %d to enc. ratio to %f", pAxis->card, pAxis->axis, value );
            break;
        }
        case motorAxisLowLimit:
        {
            PRINT( pAxis->logParam, FLOW, "Set card %d, axis %d low limit to %f", pAxis->card, pAxis->axis, value );
            break;
        }
        case motorAxisHighLimit:
        {
            PRINT( pAxis->logParam, FLOW, "Set card %d, axis %d high limit to %f", pAxis->card, pAxis->axis, value );
            break;
        }
        case motorAxisPGain:
        {
            PRINT( pAxis->logParam, FLOW, "Set card %d, axis %d pgain to %f", pAxis->card, pAxis->axis, value );
            break;
        }
        case motorAxisIGain:
        {
            PRINT( pAxis->logParam, FLOW, "Set card %d, axis %d igain to %f", pAxis->card, pAxis->axis, value );
            break;
        }
        case motorAxisDGain:
        {
            PRINT( pAxis->logParam, FLOW, "Set card %d, axis %d dgain to %f", pAxis->card, pAxis->axis, value );
            break;
        }
        case motorAxisClosedLoop:
        {
            PRINT( pAxis->logParam, FLOW, "Set card %d, axis %d closed loop to %s", pAxis->card, pAxis->axis, (value!=0?"ON":"OFF") );
            break;
        }
        default:
            status = MOTOR_AXIS_ERROR;
            break;
        }

        if (status != MOTOR_AXIS_ERROR ) status = motorParam->setDouble( pAxis->params, function, value );
    }
  return status;
}

static int motorAxisSetInteger( AXIS_HDL pAxis, motorAxisParam_t function, int value )
{
    int status = MOTOR_AXIS_OK;

    if (pAxis == NULL) return MOTOR_AXIS_ERROR;
    else
    {
        switch (function)
        {
        case motorAxisPosition:
        {
            pAxis->enc_offset = (double) value - pAxis->nextpoint.axis[0].p;
            PRINT( pAxis->logParam, FLOW, "Set card %d, axis %d to position %d", pAxis->card, pAxis->axis, value );
            break;
        }
        case motorAxisLowLimit:
        {
            PRINT( pAxis->logParam, FLOW, "Set card %d, axis %d low limit to %d", pAxis->card, pAxis->axis, value );
            break;
        }
        case motorAxisHighLimit:
        {
            PRINT( pAxis->logParam, FLOW, "Set card %d, axis %d high limit to %d", pAxis->card, pAxis->axis, value );
            break;
        }
        case motorAxisClosedLoop:
        {
            PRINT( pAxis->logParam, FLOW, "Set card %d, axis %d closed loop to %s", pAxis->card, pAxis->axis, (value?"ON":"OFF") );
            break;
        }
        default:
            status = MOTOR_AXIS_ERROR;
            break;
        }

        if (status != MOTOR_AXIS_ERROR ) status = motorParam->setInteger( pAxis->params, function, value );
    }
  return status;
}


static int motorAxisMove( AXIS_HDL pAxis, double position, int relative, double min_velocity, double max_velocity, double acceleration )
{
  if (pAxis == NULL) return MOTOR_AXIS_ERROR;
  else
    {
      if (relative) position += pAxis->endpoint.axis[0].p + pAxis->enc_offset;

      /* Check to see if in hard limits */
      if ((pAxis->nextpoint.axis[0].p >= pAxis->hiHardLimit  &&  position > pAxis->nextpoint.axis[0].p) ||
          (pAxis->nextpoint.axis[0].p <= pAxis->lowHardLimit &&  position < pAxis->nextpoint.axis[0].p)  ) return MOTOR_AXIS_ERROR;
      else if (epicsMutexLock( pAxis->axisMutex ) == epicsMutexLockOK)
	{
	  route_pars_t pars;

	  pAxis->endpoint.axis[0].p = position - pAxis->enc_offset;
	  pAxis->endpoint.axis[0].v = 0.0;
	  routeGetParams( pAxis->route, &pars );
	  if (max_velocity != 0) pars.axis[0].Vmax = fabs(max_velocity);
	  if (acceleration != 0) pars.axis[0].Amax = fabs(acceleration);
	  routeSetParams( pAxis->route, &pars );
          motorParam->setInteger( pAxis->params, motorAxisDone, 0 );
          motorParam->setInteger( pAxis->params, motorAxisMoving, 1 );
	  epicsMutexUnlock( pAxis->axisMutex );

	  PRINT( pAxis->logParam, FLOW, "Set card %d, axis %d move to %f, min vel=%f, max_vel=%f, accel=%f",
		 pAxis->card, pAxis->axis, position, min_velocity, max_velocity, acceleration );
	}
    }
  return MOTOR_AXIS_OK;
}

static int motorAxisVelocity( AXIS_HDL pAxis, double velocity, double acceleration )
{
  route_pars_t pars;
  double deltaV = velocity - pAxis->nextpoint.axis[0].v;				

  /* Check to see if in hard limits */
  if ((pAxis->nextpoint.axis[0].p > pAxis->hiHardLimit && velocity > 0) ||
      (pAxis->nextpoint.axis[0].p < pAxis->lowHardLimit && velocity < 0)  ) return MOTOR_AXIS_ERROR;
  else
  {
      double time;

      routeGetParams( pAxis->route, &pars );
      if (acceleration != 0) pars.axis[0].Amax = fabs(acceleration);
      routeSetParams( pAxis->route, &pars );

      time = fabs( deltaV / pars.axis[0].Amax );

      pAxis->endpoint.axis[0].v = velocity;
      pAxis->endpoint.axis[0].p = ( pAxis->nextpoint.axis[0].p +
				    time * ( pAxis->nextpoint.axis[0].v + 0.5 * deltaV ));
      motorParam->setInteger( pAxis->params, motorAxisDone, 0 );
      motorParam->setInteger( pAxis->params, motorAxisMoving, 1 );
      pAxis->reroute = ROUTE_NEW_ROUTE;
  }
  return MOTOR_AXIS_OK;
}

static int motorAxisHome( AXIS_HDL pAxis, double min_velocity, double max_velocity, double acceleration, int forwards )
{
  int status = MOTOR_AXIS_ERROR;

  if (pAxis == NULL) status = MOTOR_AXIS_ERROR;
  else
    {
      status = motorAxisVelocity( pAxis, (forwards? max_velocity: -max_velocity), acceleration );
      pAxis->homing = 1;

      PRINT( pAxis->logParam, FLOW, "Set card %d, axis %d to home %s, min vel=%f, max_vel=%f, accel=%f",
	     pAxis->card, pAxis->axis, (forwards?"FORWARDS":"REVERSE"), min_velocity, max_velocity, acceleration );
    }
  return status;
}


static int motorAxisVelocityMove(  AXIS_HDL pAxis, double min_velocity, double velocity, double acceleration )
{
  int status = MOTOR_AXIS_ERROR;

  if (pAxis == NULL) status = MOTOR_AXIS_ERROR;
  else
    {
      if (epicsMutexLock( pAxis->axisMutex ) == epicsMutexLockOK)
	{
	  status = motorAxisVelocity( pAxis, velocity, acceleration );
	  epicsMutexUnlock( pAxis->axisMutex );
	  PRINT( pAxis->logParam, FLOW, "Set card %d, axis %d move with velocity of %f, accel=%f",
		 pAxis->card, pAxis->axis, velocity, acceleration );
        }
    }
  return status;
}

static int motorAxisProfileMove( AXIS_HDL pAxis, int npoints, double positions[], double times[], int relative, int trigger )
{
  return MOTOR_AXIS_ERROR;
}

static int motorAxisTriggerProfile( AXIS_HDL pAxis )
{
  return MOTOR_AXIS_ERROR;
}

static int motorAxisStop( AXIS_HDL pAxis, double acceleration )
{
  if (pAxis == NULL) return MOTOR_AXIS_ERROR;
  else
    {
      motorAxisVelocity( pAxis, 0.0, acceleration );

      PRINT( pAxis->logParam, FLOW, "Set card %d, axis %d to stop with accel=%f",
	     pAxis->card, pAxis->axis, acceleration );
    }
  return MOTOR_AXIS_OK;
}

/**\defgroup motorSimTask Routines to implement the motor axis simulation task
@{
*/

/** Process one iteration of an axis

    This routine takes a single axis and propogates its motion forward a given amount
    of time.

    \param pAxis  [in]   Pointer to axis information.
    \param delta  [in]   Time in seconds to propogate motion forwards.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

static void motorSimProcess( AXIS_HDL pAxis, double delta )
{
  double lastpos = pAxis->nextpoint.axis[0].p;

  pAxis->nextpoint.T += delta;
  routeFind( pAxis->route, pAxis->reroute, &(pAxis->endpoint), &(pAxis->nextpoint) );
 /*  if (pAxis->reroute == ROUTE_NEW_ROUTE) routePrint( pAxis->route, pAxis->reroute, &(pAxis->endpoint), &(pAxis->nextpoint), stdout ); */
  pAxis->reroute = ROUTE_CALC_ROUTE;

  /* No, do a limits check */
  if (pAxis->homing && 
      ((lastpos - pAxis->home) * (pAxis->nextpoint.axis[0].p - pAxis->home)) <= 0)
    {
      /* Homing and have crossed the home sensor - return to home */
      pAxis->homing = 0;
      pAxis->reroute = ROUTE_NEW_ROUTE;
      pAxis->endpoint.axis[0].p = pAxis->home;
      pAxis->endpoint.axis[0].v = 0.0;   
    }
  if ( pAxis->nextpoint.axis[0].p > pAxis->hiHardLimit && pAxis->nextpoint.axis[0].v > 0 )
    {
      if (pAxis->homing) motorAxisVelocity( pAxis, -pAxis->endpoint.axis[0].v, 0.0 );
      else
	{
	  pAxis->reroute = ROUTE_NEW_ROUTE;
	  pAxis->endpoint.axis[0].p = pAxis->hiHardLimit;
	  pAxis->endpoint.axis[0].v = 0.0;
	}
    }
  else if (pAxis->nextpoint.axis[0].p < pAxis->lowHardLimit && pAxis->nextpoint.axis[0].v < 0)
    {
      if (pAxis->homing) motorAxisVelocity( pAxis, -pAxis->endpoint.axis[0].v, 0.0 );
      else
	{
	  pAxis->reroute = ROUTE_NEW_ROUTE;
	  pAxis->endpoint.axis[0].p = pAxis->lowHardLimit;
	  pAxis->endpoint.axis[0].v = 0.0;
	}
    }

  motorParam->setDouble(  pAxis->params, motorAxisPosition,      (pAxis->nextpoint.axis[0].p+pAxis->enc_offset) );
  motorParam->setDouble(  pAxis->params, motorAxisEncoderPosn,   (pAxis->nextpoint.axis[0].p+pAxis->enc_offset) );
  motorParam->setInteger( pAxis->params, motorAxisDirection,     (pAxis->nextpoint.axis[0].v >  0) );
  motorParam->setInteger( pAxis->params, motorAxisDone,          (pAxis->nextpoint.axis[0].v ==  0) );
  motorParam->setInteger( pAxis->params, motorAxisHighHardLimit, (pAxis->nextpoint.axis[0].p >= pAxis->hiHardLimit) );
  motorParam->setInteger( pAxis->params, motorAxisHomeSignal,    (pAxis->nextpoint.axis[0].p == pAxis->home) );
  motorParam->setInteger( pAxis->params, motorAxisMoving,        (pAxis->nextpoint.axis[0].v != 0) );
  motorParam->setInteger( pAxis->params, motorAxisLowHardLimit,  (pAxis->nextpoint.axis[0].p <= pAxis->lowHardLimit) );
}

#define DELTA 0.1
static void motorSimTask( motorSim_t * pDrv )
{
  epicsTimeStamp now;
  double delta;
  AXIS_HDL pAxis;

  while ( 1 )
    {
      /* Get a new timestamp */
      epicsTimeGetCurrent( &now );
      delta = epicsTimeDiffInSeconds( &now, &(pDrv->now) );
      pDrv->now = now;

      if ( delta > (DELTA/4.0) && delta <= (4.0*DELTA) )
	{
	  /* A reasonable time has elapsed, it's not a time step in the clock */

	  for (pAxis = pDrv->pFirst; pAxis != NULL; pAxis = pAxis->pNext )
	    {
	      if (epicsMutexLock( pAxis->axisMutex ) == epicsMutexLockOK)
		{
		  motorSimProcess( pAxis, delta );
                  motorParam->callCallback( pAxis->params );
		  epicsMutexUnlock( pAxis->axisMutex );
		}
	    }
	}
      epicsThreadSleep( DELTA );
    }
}

static int motorSimCreateAxis( motorSim_t * pDrv, int card, int axis, double lowLimit, double hiLimit, double home )
{
  AXIS_HDL pAxis;
  AXIS_HDL * ppLast = &(pDrv->pFirst);

  for ( pAxis = pDrv->pFirst;
	pAxis != NULL &&
	  ! ((pAxis->card == card) && (pAxis->axis == axis)); 
	pAxis = pAxis->pNext )
    {
      ppLast = &(pAxis->pNext);
    }

  if ( pAxis == NULL)
    {
      pAxis = (AXIS_HDL) calloc( 1, sizeof(motorAxis) );
      if (pAxis != NULL)
	{
	  route_pars_t pars;

	  pars.numRoutedAxes = 1;
	  pars.routedAxisList[0] = 1;
	  pars.Tsync = 0.0;
	  pars.Tcoast = 0.0;
	  pars.axis[0].Amax = 1.0;
	  pars.axis[0].Vmax = 1.0;

	  pAxis->endpoint.T = 0;
	  pAxis->endpoint.axis[0].p = 0;
	  pAxis->endpoint.axis[0].v = 0;

	  if ((pAxis->route = routeNew( &(pAxis->endpoint), &pars )) != NULL &&
	      (pAxis->params = motorParam->create( 0, MOTOR_AXIS_NUM_PARAMS )) != NULL &&
	      (pAxis->axisMutex = epicsMutexCreate( )) != NULL )
	    {
	      pAxis->card = card;
	      pAxis->axis = axis;
	      pAxis->hiHardLimit = hiLimit;
	      pAxis->lowHardLimit = lowLimit;
	      pAxis->home = home;
	      *ppLast = pAxis;
	      PRINT( pAxis->logParam, FLOW, "Created motor for card %d, signal %d OK", card, axis );
	    }
	  else
	    {
	      if (pAxis->route != NULL) routeDelete( pAxis->route );
	      if (pAxis->params != NULL) motorParam->destroy( pAxis->params );
	      if (pAxis->axisMutex != NULL) epicsMutexDestroy( pAxis->axisMutex );
	      free ( pAxis );
	      pAxis = NULL;
	    }
	}
      else
	{
	  free ( pAxis );
	  pAxis = NULL;
	}
    }
  else
    {
      PRINT( pAxis->logParam, ERROR, "Motor for card %d, signal %d already exists", card, axis );
      return MOTOR_AXIS_ERROR;
    }

  if (pAxis == NULL)
    {
      PRINT( pAxis->logParam, ERROR, "Cannot create motor for card %d, signal %d", card, axis );
      return MOTOR_AXIS_ERROR;
    }
    
  return MOTOR_AXIS_OK;
}


void motorSimCreate( int card, int axis, double lowLimit, double hiLimit, double home, int nCards, int nAxes )
{
  int i;
  int j;

  if (nCards < 1) nCards = 1;
  if (nAxes < 1 ) nAxes = 1;

  PRINT( NULL, FLOW,
	 "Creating motor simulator: card: %d, axis: %d, hi: %f, low %f, home: %f, ncards: %d, naxis: %d",
	 card, axis, hiLimit, lowLimit, home, nCards, nAxes );

  if (drv.motorThread==NULL)
    {
      drv.motorThread = epicsThreadCreate( "motorSimThread", 
					   epicsThreadPriorityLow,
					   epicsThreadGetStackSize(epicsThreadStackMedium),
					   (EPICSTHREADFUNC) motorSimTask, (void *) &drv );

      if (drv.motorThread == NULL)
	{
	  PRINT( NULL, ERROR, "Cannot start motor simulation thread" );
	  return;
	}
    }

  for ( i = card; i < card+nCards; i++ )
    {
      for (j = axis; j < axis+nAxes; j++ )
	{
	  motorSimCreateAxis( &drv, i, j, lowLimit, hiLimit, home );
	}
    }
}

static int motorSimLogMsg( void * param, const motorAxisLogMask_t mask, const char *pFormat, ...)
{

    va_list	pvar;
    int		nchar;

    va_start(pvar, pFormat);
    nchar = vfprintf(stdout,pFormat,pvar);
    va_end (pvar);
    printf("\n");
    return(nchar);
}
