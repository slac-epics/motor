/* Motor driver support for smarAct MCS2 Controller   */

#include <iocsh.h>

#include <asynOctetSyncIO.h>
#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <smarActMCS2MotorDriver.h>
#include <errlog.h>

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <exception>

#include <math.h>

#include <epicsString.h>
#include <epicsExport.h>

static char dummy[10];

/* Static configuration parameters (compile-time constants) */
#undef   DEBUG
#undef   DEBUGPOLL

#define CMD_LEN 100
#define REP_LEN 100
#define DEFLT_TIMEOUT 2.0

#define HOLD_FOREVER    -1
#define HOLD_NEVER       0
#define MM2PM(x)         ((x)*1000000000.)
#define PM2MM(x)         ((x)/1000000000.)
#define PM2NM(x)         ((x)/1000.)
#define NM2PM(x)         ((x)*1000.)
#define FAR_AWAY         1e12 /*pm*/

/* The asyn motor driver apparently can't cope with exceptions */
#undef  ASYN_CANDO_EXCEPTIONS
/* Define this if exceptions should be thrown and it is OK to abort the application */
#undef  DO_THROW_EXCEPTIONS

#if defined(ASYN_CANDO_EXCEPTIONS) || defined(DO_THROW_EXCEPTIONS)
#define THROW_(e) throw e
#else
#define THROW_(e) epicsPrintf("%s\n",e.what());
#endif

#define SmarActMCS2StatusMoving             0x0001
#define SmarActMCS2StatusClosedLoopActive   0x0002
#define Holding(v) (!((v) & SmarActMCS2StatusMoving) && ((v) & SmarActMCS2StatusClosedLoopActive))
#define SmarActMCS2StatusCalibrating        0x0004
#define SmarActMCS2StatusReferencing        0x0008
#define SmarActMCS2StatusMoveDelayed        0x0010
#define SmarActMCS2StatusSensorPresent      0x0020
#define SmarActMCS2StatusCalibrated         0x0040
#define SmarActMCS2StatusReferenced         0x0080
#define SmarActMCS2StatusEndStopReached     0x0100
#define SmarActMCS2StatusSoftLimitReached   0x0200
#define SmarActMCS2StatusFollowLimitReached 0x0400
#define SmarActMCS2StatusMoveFailed         0x0800
#define SmarActMCS2StatusStreaming          0x1000
#define SmarActMCS2StatusOverTemp           0x4000
#define SmarActMCS2StatusRefMark            0x8000

SmarActMCS2Exception::SmarActMCS2Exception(SmarActMCS2ExceptionType t, const char *fmt, ...)
	: t_(t)
{
    va_list ap;
    if ( fmt ) {
        va_start(ap, fmt);
        epicsVsnprintf(str_, sizeof(str_), fmt, ap);
        va_end(ap);
    } else {
        str_[0] = 0;
    }
};

SmarActMCS2Exception::SmarActMCS2Exception(SmarActMCS2ExceptionType t, const char *fmt, va_list ap)
		: t_(t)
{
    epicsVsnprintf(str_, sizeof(str_), fmt, ap);
}

SmarActMCS2Controller::SmarActMCS2Controller(const char *portName, const char *IOPortName, 
                                             int numAxes, double movingPollPeriod, double idlePollPeriod)
	: asynMotorController(portName, numAxes,
	                      0, // parameters
	                      0, // interface mask
	                      0, // interrupt mask
	                      ASYN_CANBLOCK | ASYN_MULTIDEVICE,
	                      1, // autoconnect
	                      0,0) // default priority and stack size
	, asynUserMot_p_(0)
{
    asynStatus       status;
    char             junk[100];
    size_t           got_junk;
    int              eomReason;

    pAxes_ = (SmarActMCS2Axis **)(asynMotorController::pAxes_);
    status = pasynOctetSyncIO->connect(IOPortName, 0, &asynUserMot_p_, NULL);
    if ( status ) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "SmarActMCS2Controller:SmarActMCS2Controller: cannot connect to MCS2 controller\n");
        THROW_(SmarActMCS2Exception(MCS2ConnectionError, "SmarActMCS2Controller: unable to connect serial channel"));
    }

    // slurp away any initial telnet negotiation; there is no guarantee that
    // the other end will not send some telnet chars in the future. The terminal
    // server should really be configured to 'raw' mode!
    pasynOctetSyncIO->read(asynUserMot_p_, junk, sizeof(junk), 2.0, &got_junk, &eomReason);
    if ( got_junk ) {
        epicsPrintf("SmarActMCS2Controller(%s): WARNING - detected unexpected characters on link (%s); make sure you have a RAW (not TELNET) connection\n", portName, IOPortName);
    }
    pasynOctetSyncIO->setInputEos ( asynUserMot_p_, "\r\n", 1 );
    pasynOctetSyncIO->setOutputEos( asynUserMot_p_, "\r\n", 1 );

    // FIXME the 'forcedFastPolls' may need to be set if the 'sleep/wakeup' feature
    //       of the sensor/readback is used.
#ifdef DEBUG
    printf("startPoller, moving = %f, idle = %f\n", movingPollPeriod, idlePollPeriod);
#endif
    startPoller( movingPollPeriod, idlePollPeriod, 0 );
}

asynStatus
SmarActMCS2Controller::sendCmd(size_t *got_p, char *rep, int len, double timeout, const char *fmt, va_list ap)
{
    char       buf[CMD_LEN];
    size_t     nwrite;
    int        eomReason;
    asynStatus status;

    epicsVsnprintf(buf, sizeof(buf), fmt, ap);

    if (len != -1) {
        status = pasynOctetSyncIO->writeRead( asynUserMot_p_, buf, strlen(buf), rep, len, timeout, &nwrite, got_p, &eomReason);
#ifdef DEBUG
        printf("sendCmd(%s) --> %s\n", buf, rep);
#endif
    } else {
        status = pasynOctetSyncIO->write( asynUserMot_p_, buf, strlen(buf), timeout, &nwrite );
#ifdef DEBUG
        printf("sendCmd(%s) --> status %d\n", buf, status);
#endif
    }

    //asynPrint(c_p_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "sendCmd()=%s", buf);

    return status;
}

asynStatus
SmarActMCS2Controller::sendCmd(size_t *got_p, char *rep, int len, double timeout, const char *fmt, ...)
{
    va_list    ap;
    asynStatus status;
    va_start(ap, fmt);
    status = sendCmd(got_p, rep, len, timeout, fmt, ap);
    va_end(ap);
    return status;
}


asynStatus SmarActMCS2Controller::sendCmd(size_t *got_p, char *rep, int len, const char *fmt, ...)
{
    va_list    ap;
    asynStatus status;
    va_start(ap, fmt);
    status = sendCmd(got_p, rep, len, DEFLT_TIMEOUT, fmt, ap);
    va_end(ap);
    return status;
}

asynStatus SmarActMCS2Controller::sendCmd(char *rep, int len, const char *fmt, ...)
{
    va_list    ap;
    asynStatus status;
    size_t     got;
    va_start(ap, fmt);
    status = sendCmd(&got, rep, len, DEFLT_TIMEOUT, fmt, ap);
    va_end(ap);
    return status;
}

/* Obtain value of the 'motorClosedLoop_' parameter (which
 * maps to the record's CNEN field)
 */
int
SmarActMCS2Axis::getClosedLoop()
{
    int val;
    c_p_->getIntegerParam(axisNo_, c_p_->motorClosedLoop_, &val);
    return val;
}

SmarActMCS2Axis::SmarActMCS2Axis(class SmarActMCS2Controller *cnt_p, int axis, int channel)
	: asynMotorAxis(cnt_p, axis), c_p_(cnt_p)
{
    int val;
    channel_ = channel;

    asynPrint(c_p_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "SmarActMCS2Axis::SmarActMCS2Axis -- creating axis %u\n", axis);

    comStatus_ = getVal("VEL",&vel_);
#ifdef DEBUG
    printf("VEL %u returned %lld\n", axis, vel_);
#endif
    if ( comStatus_ )
        goto bail;
    if ( (comStatus_ = getVal("ACC", &accl_)) )
        goto bail;
#ifdef DEBUG
    printf("ACC %u returned %lld\n", axis, accl_);
#endif
    if ( (comStatus_ = getVal("STAT", &val)) )
        goto bail;

    setIntegerParam(c_p_->motorStatusPowerOn_, Holding(val));
    if ( Holding(val) ) {
        // still holding? This means that - in a previous life - the
        // axis was configured for 'infinite holding'. Inherit this
        // (until the next 'move' command that is).
        ///
        holdTime_ = HOLD_FOREVER;
    } else {
        // initial value from 'closed-loop' property
        holdTime_ = getClosedLoop() ? HOLD_FOREVER : 0;
    }

    // Determine if stage has a sensor.
    if (val & SmarActMCS2StatusSensorPresent) {
        if ( asynSuccess == getVal("POS:CURR", &val) ) {
            setIntegerParam(c_p_->motorStatusHasEncoder_, 1);
            setIntegerParam(c_p_->motorStatusGainSupport_, 1);
        }
    }

 bail:
    clearErrors();
    setIntegerParam(c_p_->motorStatusProblem_, comStatus_ ? 1 : 0 );
    setIntegerParam(c_p_->motorStatusCommsError_, comStatus_ ? 1 : 0 );

    callParamCallbacks();

    if ( comStatus_ ) {
        THROW_(SmarActMCS2Exception(MCS2CommunicationError, "SmarActMCS2Axis::SmarActMCS2Axis -- channel %u ASYN error %i", axis, comStatus_));
    }

}

/* Read a parameter from the MCS2 (nothing to do with asyn's parameter
 * library).
 *
 * parm_cmd: MCS2 command (w/o ':' char) to read parameter
 * val_p:    where to store the value returned by the MCS2
 * 
 * RETURNS:  asynError if an error occurred, asynSuccess otherwise.
 */
asynStatus
SmarActMCS2Axis::getVal(const char *parm_cmd, int *val_p)
{
    char       rep[REP_LEN];
    asynStatus st;
    int rc;

    //asynPrint(c_p_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "getVal() cmd=:CHAN%u:%s?", this->channel_, parm_cmd);

    st = c_p_->sendCmd(rep, sizeof(rep), ":CHAN%u:%s?", this->channel_, parm_cmd);
    if ( st )
        return st;
    rc = sscanf(rep, "%d", val_p);
#ifdef DEBUG
    if (rc == 1)
        printf("getValI(%d, %s) --> %d\n", this->channel_, parm_cmd, *val_p);
    else
        printf("getValI(%d, %s) --> ERROR!\n", this->channel_, parm_cmd);
#endif
    return (rc != 1) ? asynError: asynSuccess;
}

asynStatus
SmarActMCS2Axis::getVal(const char *parm_cmd, long long *val_p)
{
    char       rep[REP_LEN];
    asynStatus st;
    int        rc;

    //asynPrint(c_p_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "getVal() cmd=:CHAN%u:%s?", this->channel_, parm_cmd);

    st = c_p_->sendCmd(rep, sizeof(rep), ":CHAN%u:%s?", this->channel_, parm_cmd);
    if ( st )
        return st;
    rc = sscanf(rep, "%lld", val_p);
#ifdef DEBUG
    if (rc == 1)
        printf("getValL(%d, %s) --> %lld\n", this->channel_, parm_cmd, *val_p);
    else
        printf("getValL(%d, %s) --> ERROR!\n", this->channel_, parm_cmd);
#endif
    return (rc != 1) ? asynError: asynSuccess;
}

char *
SmarActMCS2Axis::getError()
{
    static char rep[REP_LEN];
    char       *s, *t;

    comStatus_ = c_p_->sendCmd(rep, sizeof(rep), ":SYST:ERR?");
    if (comStatus_ != asynSuccess)
        return (char *)"Communication Failure";
    if (rep[0] == '0')
        return NULL;
    // Response is NNN,"ERRORMSG".  We just want to return ERRORMSG without the quotes.
    for (s = rep; *s != ','; s++);
    s += 2;
    for (t = rep; *s != '"';)
        *t++ = *s++;
    *t = 0;
#ifdef DEBUG
    printf("getError --> %s\n", rep);
#endif
    return rep;
}

void
SmarActMCS2Axis::clearErrors()
{
    static char rep[REP_LEN];
    asynStatus  st;
    // Read errors until we don't see any more.
    do {
        st = c_p_->sendCmd(rep, sizeof(rep), ":SYST:ERR?");
    } while ((asynSuccess == st) && (rep[0] != '0'));
}

asynStatus
SmarActMCS2Axis::poll(bool *moving_p)
{
    long long val;

    if ( (comStatus_ = getVal("STAT", &val)) )
        goto bail;

    *moving_p = (val & SmarActMCS2StatusMoving) ? true : false;
    setIntegerParam(c_p_->motorStatusDone_, !*moving_p );

    /* Check if the sensor 'knows' absolute position and
     * update the MSTA 'HOMED' bit.
     */
    setIntegerParam(c_p_->motorStatusHomed_, (val & SmarActMCS2StatusReferenced) ? 1 : 0 );

#ifdef DEBUGPOLL
    printf(" status %lld\n", val);
#endif
    if (!(val & SmarActMCS2StatusSensorPresent))   /* Can't ask for position if there isn't a sensor! */
        goto bail;

    if ( (comStatus_ = getVal("POS:CURR", &val)) )
        goto bail;

    setDoubleParam(c_p_->motorEncoderPosition_, PM2NM((double)val));
    setDoubleParam(c_p_->motorPosition_, PM2NM((double)val));
#ifdef DEBUGPOLL
    printf("POLL (position %.6lf)\n", PM2MM((double)val));
#endif

 bail:
    clearErrors();
    setIntegerParam(c_p_->motorStatusProblem_,    comStatus_ ? 1 : 0 );
    setIntegerParam(c_p_->motorStatusCommsError_, comStatus_ ? 1 : 0 );
    callParamCallbacks();

    return comStatus_;
}

asynStatus
SmarActMCS2Axis::setSpeed(double velocity)
{
    long long  vel;
    asynStatus status;

    vel = (long long)fabs(velocity);
#ifdef DEBUG
    printf("setSpeed %d %lf --> %lld\n", channel_, velocity, vel);
#endif
    if (vel !=  vel_) {
        /* change speed */
        if ( (asynSuccess == (status = c_p_->sendCmd(dummy, -1, ":CHAN%u:VEL %lld", channel_, vel))) &&
             !getError() ) {
            vel_ = vel;
        }
        return status;
    }
    return asynSuccess;
}

asynStatus
SmarActMCS2Axis::setAccel(double accel)
{
    long long  accl;
    asynStatus status;

    accl = (long long)fabs(accel);
#ifdef DEBUG
    printf("setAccel %d %lf --> %lld\n", channel_, accel, accl);
#endif
    if (accl !=  accl_) {
        /* change speed */
        if ( (asynSuccess == (status = c_p_->sendCmd(dummy, -1, ":CHAN%u:ACC %lld", channel_, accl))) &&
             !getError() ) {
            accl_ = accl;
        }
        return status;
    }
    return asynSuccess;
}

asynStatus  
SmarActMCS2Axis::setHoldTime(int holdTime)
{
    asynStatus status;

    if (holdTime_ != holdTime) {
        /* change holdtime */
        if ( (asynSuccess == (status = c_p_->sendCmd(dummy, -1, ":CHAN%u:HOLD %d", channel_, holdTime))) && !getError() ) {
            holdTime_ = holdTime;
        }
        setIntegerParam(c_p_->motorStatusPowerOn_, holdTime == HOLD_FOREVER);
        return status;
    }
    return asynSuccess;
}

asynStatus  
SmarActMCS2Axis::move(double position, int relative, double min_vel, double max_vel, double accel)
{
    enum SmarActMCS2MoveMode movemode;
#ifdef DEBUG
    printf("Move to %f (speed %f - %f, accel %f)\n", position, min_vel, max_vel, accel);
#endif

    if ( (comStatus_ = setSpeed(NM2PM(max_vel))) )
        goto bail;
    if ( (comStatus_ = setAccel(NM2PM(accel))) )
        goto bail;

    /* cache 'closed-loop' setting until next move */
    if ( (comStatus_ = setHoldTime(getClosedLoop() ? HOLD_FOREVER : 0)) )
        goto bail;

    movemode = (relative ? MCS2MM_REL : MCS2MM_ABS);

    if ( (comStatus_ = c_p_->sendCmd(dummy, -1, "CHAN%u:MMOD %d", channel_, (int)movemode)) )
        goto bail;

    comStatus_ = c_p_->sendCmd(dummy, -1, ":MOVE%u %.0lf", channel_, NM2PM(position));

 bail:
    clearErrors();
    if ( comStatus_ ) {
        setIntegerParam(c_p_->motorStatusProblem_, 1);
        setIntegerParam(c_p_->motorStatusCommsError_, 1);
        callParamCallbacks();
    }
    return comStatus_;
}

asynStatus
SmarActMCS2Axis::home(double min_vel, double max_vel, double accel, int forwards)
{
#ifdef DEBUG
    printf("Home %u\n", forwards);
#endif

    if ( (comStatus_ = setSpeed(max_vel)) )
        goto bail;

    /* cache 'closed-loop' setting until next move */
    if ( (comStatus_ = setHoldTime(getClosedLoop() ? HOLD_FOREVER : 0)) )
        goto bail;

    /* Set the "safe direction", aka the direction the home goes towards. */
    if ( (comStatus_ = c_p_->sendCmd(dummy, -1, "CHAN%u:SDIR %u", channel_, forwards ? 0 : 1)) )
        goto bail;

    /* Set the reference options.  Direction is bit0, whee! */
    if ( (comStatus_ = c_p_->sendCmd(dummy, -1, "CHAN%u:REF:OPT %u", channel_, forwards ? 0 : 1)) )
        goto bail;

    comStatus_ = c_p_->sendCmd(dummy, -1, ":REF%u", channel_);

 bail:
    clearErrors();
    if ( comStatus_ ) {
        setIntegerParam(c_p_->motorStatusProblem_, 1);
        setIntegerParam(c_p_->motorStatusCommsError_, 1);
        callParamCallbacks();
    }
    return comStatus_;
}

asynStatus
SmarActMCS2Axis::stop(double acceleration)
{
#ifdef DEBUG
    printf("Stop\n");
#endif
    if ( (asynSuccess == (comStatus_ = c_p_->sendCmd(dummy, -1, ":STOP%u", channel_))) && !getError() )
        return asynSuccess;

    if ( comStatus_ ) {
        clearErrors();
        setIntegerParam(c_p_->motorStatusProblem_, 1);
        setIntegerParam(c_p_->motorStatusCommsError_, 1);
        callParamCallbacks();
    }
    return comStatus_;
}

asynStatus
SmarActMCS2Axis::setPosition(double position)
{
    if ( (asynSuccess == (comStatus_ = c_p_->sendCmd(dummy, -1, ":CHAN%u:POS:CURR %lld",
                                               channel_, (long long) position))) && !getError())
        return asynSuccess;

    if ( comStatus_ ) {
        clearErrors();
        setIntegerParam(c_p_->motorStatusProblem_,    1);
        setIntegerParam(c_p_->motorStatusCommsError_, 1);
        callParamCallbacks();
    }
    return comStatus_;
}

asynStatus
SmarActMCS2Axis::moveVelocity(double min_vel, double max_vel, double accel)
{
    double tgt_pos = FAR_AWAY;

    /* No MCS2 command we an use directly. Just use a 'relative move' to
     * very far target.
     */

#ifdef DEBUG
    printf("moveVelocity (%f - %f)\n", min_vel, max_vel);
#endif

    if ( 0 == max_vel ) {
        /* Here we are in a dilemma. If we set the MCS2 speed to zero
         * then it will move at unlimited speed which is so fast that
         * 'JOG' makes no sense.
         * Just 'STOP' the motion - hope that works...
         */
        setIntegerParam(c_p_->motorStop_, 1);
        callParamCallbacks();
        return asynSuccess;
    }

    if ( max_vel < 0 ) {
        tgt_pos = -tgt_pos; 
    }

    if ( (comStatus_ = setSpeed(max_vel)) )
        goto bail;

    if ( (comStatus_ = c_p_->sendCmd(dummy, -1, "CHAN%u:MMOD %u", channel_, MCS2MM_REL)) )
        goto bail;

    comStatus_ = c_p_->sendCmd(dummy, -1, ":MOVE%u %.0lf", channel_, tgt_pos);

 bail:
    clearErrors();
    if ( comStatus_ ) {
        setIntegerParam(c_p_->motorStatusProblem_, 1);
        setIntegerParam(c_p_->motorStatusCommsError_, 1);
        callParamCallbacks();
    }
    return comStatus_;
}

/* iocsh wrapping and registration business (stolen from ACRMotorDriver.cpp) */
static const iocshArg cc_a0 = {"Port name [string]",               iocshArgString};
static const iocshArg cc_a1 = {"I/O port name [string]",           iocshArgString};
static const iocshArg cc_a2 = {"Number of axes [int]",             iocshArgInt};
static const iocshArg cc_a3 = {"Moving poll period (s) [double]",  iocshArgDouble};
static const iocshArg cc_a4 = {"Idle poll period (s) [double]",    iocshArgDouble};

static const iocshArg * const cc_as[] = {&cc_a0, &cc_a1, &cc_a2, &cc_a3, &cc_a4};

static const iocshFuncDef cc_def = {"smarActMCS2CreateController", sizeof(cc_as)/sizeof(cc_as[0]), cc_as};

extern "C" void *
smarActMCS2CreateController(
	const char *motorPortName,
	const char *ioPortName,
	int         numAxes,
	double      movingPollPeriod,
	double      idlePollPeriod)
{
    void *rval = 0;
    // the asyn stuff doesn't seem to be prepared for exceptions. I get segfaults
    // if constructing a controller (or axis) incurs an exception even if its
    // caught (IMHO asyn should behave as if the controller/axis never existed...)
#ifdef ASYN_CANDO_EXCEPTIONS
    try {
#endif
        rval = new SmarActMCS2Controller(motorPortName, ioPortName, numAxes, movingPollPeriod, idlePollPeriod);
#ifdef ASYN_CANDO_EXCEPTIONS
    } catch (SmarActMCS2Exception &e) {
        epicsPrintf("smarActMCS2CreateController failed (exception caught):\n%s\n", e.what());
        rval = 0;
    }
#endif

    return rval;
}

static void cc_fn(const iocshArgBuf *args)
{
    smarActMCS2CreateController(args[0].sval,
                                args[1].sval,
                                args[2].ival,
                                args[3].dval,
                                args[4].dval);
}


static const iocshArg ca_a0 = {"Controller Port name [string]",    iocshArgString};
static const iocshArg ca_a1 = {"Axis number [int]",                iocshArgInt};
static const iocshArg ca_a2 = {"Channel [int]",                    iocshArgInt};

static const iocshArg * const ca_as[] = {&ca_a0, &ca_a1, &ca_a2};

/* iocsh wrapping and registration business (stolen from ACRMotorDriver.cpp) */
/* smarActMCS2CreateAxis called to create each axis of the smarActMCS2 controller*/
static const iocshFuncDef ca_def = {"smarActMCS2CreateAxis", 3, ca_as};

extern "C" void *
smarActMCS2CreateAxis(
	const char *controllerPortName,
	int        axisNumber,
	int        channel)
{
    void *rval = 0;

    SmarActMCS2Controller *pC;
    SmarActMCS2Axis *pAxis;
    asynMotorAxis *pAsynAxis;

    // the asyn stuff doesn't seem to be prepared for exceptions. I get segfaults
    // if constructing a controller (or axis) incurs an exception even if its
    // caught (IMHO asyn should behave as if the controller/axis never existed...)
#ifdef ASYN_CANDO_EXCEPTIONS
    try {
#endif
        //		rval = new SmarActMCS2Axis(, axisNumber, channel);
        pC = (SmarActMCS2Controller*) findAsynPortDriver(controllerPortName);
        if (!pC) {
            printf("smarActMCS2CreateAxis: Error, port %s not found\n", controllerPortName);
            rval = 0;
            return rval;
        }
        // check if axis number already exists
        pAsynAxis = pC->getAxis(axisNumber);
        if (pAsynAxis != NULL) { // axis already exists
            epicsPrintf("SmarActMCS2CreateAxis failed:axis %u already exists\n", axisNumber);
#ifdef ASYN_CANDO_EXCEPTIONS
            THROW_(SmarActMCS2Exception(MCS2CommunicationError, "axis %u already exists", axisNumber));
#endif
            rval = 0;
            return rval;
        }
        pC->lock();
        pAxis = new SmarActMCS2Axis(pC, axisNumber, channel);
        pAxis = NULL;
        pC->unlock();

#ifdef ASYN_CANDO_EXCEPTIONS
    } catch (SmarActMCS2Exception &e) {
        epicsPrintf("SmarActMCS2Axis failed (exception caught):\n%s\n", e.what());
        rval = 0;
    }
#endif

    return rval;
}

static void ca_fn(const iocshArgBuf *args)
{
    smarActMCS2CreateAxis(args[0].sval,
                          args[1].ival,
                          args[2].ival);
}

static void smarActMCS2MotorRegister(void)
{
    iocshRegister(&cc_def, cc_fn);  // smarActMCS2CreateController
    iocshRegister(&ca_def, ca_fn);  // smarActMCS2CreateAxis
}

extern "C" {
    epicsExportRegistrar(smarActMCS2MotorRegister);
}
