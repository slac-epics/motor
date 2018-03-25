/*
FILENAME... Pico8742Driver.cpp
USAGE...    Motor driver support for the Newport Pico8742 controller.

Based on the SMC100 Model 3 device driver written by:
K. Goetze
Mark Rivers
March 1, 2012

M. D'Ewart 2014-10-14  Initial version

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>
#include <asynCommonSyncIO.h>

#include "Pico8742Driver.h"
#include <epicsExport.h>

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

#define NO_MOTOR 0
#define UNKNOWN_MOTOR 1
#define TINY_MOTOR 2
#define STANDARD_MOTOR 3

#define TINY_VMAX 1750
#define STANDARD_VMAX 2000

#define OPEN_LOOP 0
#define CLOSED_LOOP 1

#define WRITE_DELAY 0.01

static const char *driverName = "Pico8742Driver";

/** Creates a new Pico8742Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] Pico8742PortName     The name of the drvAsynSerialPort that was created previously to connect to the Pico8742 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
Pico8742Controller::Pico8742Controller(const char *portName, const char *Pico8742PortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod, int ctrlNo)
  :  asynMotorController(portName, numAxes, NUM_Pico8742_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0),  // Default priority and stack size
			ctrlNo_(ctrlNo)
{
  int axis;
  asynStatus status;
  Pico8742Axis *pAxis;
  static const char *functionName = "Pico8742Controller::Pico8742Controller";

  createParam(Pico8742FirmwareString,                 asynParamOctet,   &Pico8742FirmwareString_);
  createParam(Pico8742MotorCheckString,               asynParamInt32,   &Pico8742MotorCheck_);
  createParam(Pico8742MotorTypeString,                asynParamInt32,   &Pico8742MotorType_);
  createParam(Pico8742SoftResetString,                asynParamInt32,   &Pico8742SoftReset_);
  
  /* Connect to Pico8742 controller */
  status = pasynOctetSyncIO->connect(Pico8742PortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to Pico8742 controller\n",
      functionName);
  }
  /* Connect with asynCommon */
  status = pasynCommonSyncIO->connect(Pico8742PortName, 0, &pasynUserCommonController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
      "%s: cannot connect to Pico8742 controller\n",
      functionName);
  }


  /* get version is it 8742 or 8743 */ 
  /* 8742 Version 1.9 11/01/12 */
  /* 8743-CL Version 1.9 11/01/12 */
  sprintf(outString_, "VE?");
  status = writeReadController();

  closedLoop_ = (strncmp(inString_, "8743", 4) == 0) ? CLOSED_LOOP:OPEN_LOOP;
  if(closedLoop_ == CLOSED_LOOP){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "ERMAGERD CLERSED LERP\n");
  } else {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "ERRPEN LERP\n");
  }
  
  /* get identification string - model, fw, version */
  sprintf(outString_, "*IDN?");
  status = writeReadController();
  setStringParam(Pico8742FirmwareString_, inString_);
  
  for (axis=0; axis<numAxes; axis++) {
    pAxis = new Pico8742Axis(this, axis);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new Pico8742Controller object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] Pico8742PortName       The name of the drvAsynIPPPort that was created previously to connect to the Pico8742 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  * \param[in] eguPerStep        The stage resolution
  */
extern "C" int Pico8742CreateController(const char *portName, const char *Pico8742PortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod, int ctrlNo)
{
   
  new Pico8742Controller(portName, Pico8742PortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000., ctrlNo);
  
  return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void Pico8742Controller::report(FILE *fp, int level)
{
  fprintf(fp, "Pico8742 motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an Pico8742Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
Pico8742Axis* Pico8742Controller::getAxis(asynUser *pasynUser)
{
  return static_cast<Pico8742Axis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an Pico8742Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] No Axis index number. */
Pico8742Axis* Pico8742Controller::getAxis(int axisNo)
{
  return static_cast<Pico8742Axis*>(asynMotorController::getAxis(axisNo));
}

asynStatus Pico8742Controller::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  Pico8742Axis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeInt32";

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setIntegerParam(pAxis->axisNo_, function, value);

  if (function == Pico8742MotorCheck_)
  {
    this->motorCheck();
  }
  else if (function == Pico8742SoftReset_)
  {
    this->softReset();
  }
  else
  {
    /* Call base class method */
    status = asynMotorController::writeInt32(pasynUser, value);
  }

  /* Do callbacks so higher layers see any changes */
  callParamCallbacks(pAxis->axisNo_);
  if (status)
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
        "%s:%s: error, status=%d function=%d, value=%d\n",
        driverName, functionName, status, function, value);
  else
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
        "%s:%s: function=%d, value=%d\n",
        driverName, functionName, function, value);
  return status;
}

asynStatus Pico8742Controller::writeReadController()
{
  asynStatus status = asynSuccess;
  char temp[20];
  char resp[20];
  int ctrlNo;
  const char *functionName = "Pico8742::writeReadController()";
  sprintf(temp, "%1d>%s", ctrlNo_, outString_);
  sprintf(outString_, temp);
  //pasynManager->lockPort(pasynUserController_);
  //pasynOctectSyncIO->lockPort(pasynUserController_);
#ifdef WRITE_DELAY
  epicsThreadSleep(WRITE_DELAY);
#endif
  status = asynMotorController::writeReadController();
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s: error, status=%d\n",
        driverName, functionName, status);
/* device getting into weird state with daisy chain, need to reconnect port */
    if (status == asynTimeout) {
         /* If we have an I/O error or are disconnected then disconnect device and reconnect */
            status = pasynCommonSyncIO->disconnectDevice(this->pasynUserCommonController_);
            if (status == asynSuccess) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                          "%s::Pico8742 disconnect device OK\n",
                          driverName);
            } else {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s::Pico8742 disconnect error=%s\n",
                          driverName, this->pasynUserSelf->errorMessage);
            }
         /* device should auto reconnect */ 
/*
            status = pasynCommonSyncIO->connectDevice(this->pasynUserCommonController_);
            if (status == asynSuccess) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                          "%s::Pico8742 connect device OK\n",
                          driverName);
            } else {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s::Pico8742 connect device error=%s\n",
                          driverName, this->pasynUserSelf->errorMessage);
            }
*/
    }
  }
  else {
    //pasynOctectSyncIO->unlockPort(pasynUserController_);
    //pasynManager->lockPort(pasynUserController_);
    sscanf(inString_, "%1d>%s", &ctrlNo, resp );
    if( ctrlNo != ctrlNo_) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Communication Error: Ctrl %d gets Ctrl %d response\n", driverName, functionName, ctrlNo_, ctrlNo);
    }
    sprintf(inString_, resp);
    
  }
  return status;
}

asynStatus Pico8742Controller::writeController()
{
  asynStatus status = asynSuccess;
  char temp[20];
  sprintf(temp, "%1d>%s", ctrlNo_, outString_);
  sprintf(outString_, temp);
  //pasynManager->lockPort(pasynUserController_);
#ifdef WRITE_DELAY
  epicsThreadSleep(WRITE_DELAY);
#endif
  status = asynMotorController::writeController();
  return status;
}
/** Scans for motors connect to controller, sets the motor type.  Saves motor 
  * type in non-volatile memory.
  *
  */

asynStatus Pico8742Controller::motorCheck()
{
  asynStatus status = asynSuccess;
  Pico8742Axis *pAxis; 

  /* scan for motors connected to controller */
  sprintf(outString_, "MC");
  status = writeController();
  /* save motor parameters */
  sprintf(outString_, "SM");
  status = writeController();
  /* update axis settings */
  for (int i=0; i<numAxes_; i++) {
        pAxis=getAxis(i);
        if (!pAxis) continue;
        pAxis->setMotorType();
  }
 
  return status;
}

/** Performs soft reset on controller 
  *
  */

asynStatus Pico8742Controller::softReset()
{
  asynStatus status = asynSuccess;

  /* soft reset */
  sprintf(outString_, "RS");
  status = writeController();

  return status;
}


// These are the Pico8742Axis methods

/** Creates a new Pico8742Axis object.
  * \param[in] pC Pointer to the Pico8742Controller to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
Pico8742Axis::Pico8742Axis(Pico8742Controller *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{ 

  asynStatus status;
  status = this->setMotorType();
}

/** This command is used to query the motor type of an axis.
  * It simply reports the present motor type setting in memory - it does
  * not perform a check to determin if the setting is still valid.
  * 
  */
asynStatus Pico8742Axis::setMotorType()
{
  asynStatus status = asynSuccess;
  /* get pico motor type, if small limit velocity */
  sprintf(pC_->outString_, "%1dQM?", axisNo_ + 1);
  status = pC_->writeReadController();
  picoType_ = (atoi(pC_->inString_));
  setIntegerParam(pC_->Pico8742MotorType_, picoType_);
  callParamCallbacks();
  return status;
}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void Pico8742Axis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n",
            axisNo_ + 1);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus Pico8742Axis::sendAccelAndVelocity(double acceleration, double velocity) 
{
  asynStatus status;
  // static const char *functionName = "Pico8742::sendAccelAndVelocity";
  
  /* limit to maximum velocity, if standard motor set to STANDARD_VMAX */
  /* otherwise it is TINY_MOTOR or UNKNOWN_MOTOR, limit to TINY_VMAX */
  if(picoType_ == STANDARD_MOTOR) {
      velocity = velocity > STANDARD_VMAX ? STANDARD_VMAX:velocity; 
  }
  else {
      velocity = velocity > TINY_VMAX ? TINY_VMAX:velocity; 
  }
  // Send the velocity in egus
  sprintf(pC_->outString_, "%1dVA%f", axisNo_ + 1, (velocity));
  status = pC_->writeController();

  // Send the acceleration in egus/sec/sec
  sprintf(pC_->outString_, "%1dAC%f", axisNo_ + 1, (acceleration));
  status = pC_->writeController();
  return status;
}


asynStatus Pico8742Axis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "Pico8742Axis::move";

  status = sendAccelAndVelocity(acceleration, maxVelocity);
  
  if (relative) {
    sprintf(pC_->outString_, "%1dPR%f", axisNo_ + 1, (position));
  } else {
    sprintf(pC_->outString_, "%1dPA%f", axisNo_ + 1, (position));
  }
  status = pC_->writeController();
  return status;
}

asynStatus Pico8742Axis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status = asynSuccess;
  static const char *functionName = "Pico8742Axis::home";

  if (pC_->closedLoop_ == CLOSED_LOOP) {
    sprintf(pC_->outString_, "%1dOR", axisNo_ + 1);
    status = pC_->writeController();
  }
  // set Home search velocity
  //sprintf(pC_->outString_, "%1dOH%f", axisNo_ + 1, maxVelocity);
  //status = pC_->writeController();

  //sprintf(pC_->outString_, "%1dOR", axisNo_ + 1);
  
  //status = pC_->writeController();
  return status;
}

// Jog
asynStatus Pico8742Axis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  double high_limit;
  double low_limit;
  asynStatus comStatus;
  static const char *functionName = "Pico8742Axis::moveVelocity";

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s: minVelocity=%f, maxVelocity=%f, acceleration=%f\n",
    functionName, minVelocity, maxVelocity, acceleration);
    
  comStatus = sendAccelAndVelocity(acceleration, maxVelocity);
  if (comStatus) goto skip;

  if (maxVelocity > 0.) {
    /* This is a positive move in Pico8742 coordinates  */
    sprintf(pC_->outString_, "%1dMV+", axisNo_ + 1);
  } else {
      /* This is a negative move in Pico8742 coordinates */
      sprintf(pC_->outString_, "%1dMV-", axisNo_ + 1);
  }
  comStatus = pC_->writeController();
  if (comStatus) goto skip;
  
  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;

}

asynStatus Pico8742Axis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "Pico8742Axis::stop";

  sprintf(pC_->outString_, "%1dST", axisNo_ + 1);
  status = pC_->writeController();
  sprintf(pC_->outString_, "AB");
  status = pC_->writeController();
  return status;
}

asynStatus Pico8742Axis::setPosition(double position)
{
  asynStatus status;
  //static const char *functionName = "Pico8742Axis::setPosition";
  // set the current position counter, absolute move will be realtive to this
  sprintf(pC_->outString_, "%1dDH%f", axisNo_ + 1, position);
  status = pC_->writeController();
  return status;
}

asynStatus Pico8742Axis::setClosedLoop(bool closedLoop)
{
  asynStatus status = asynSuccess;
  //static const char *functionName = "Pico8742Axis::setClosedLoop";
  if (pC_->closedLoop_ == CLOSED_LOOP) {
    if (closedLoop) {
      sprintf(pC_->outString_, "%1dMM1", axisNo_ + 1);
      status = pC_->writeController();
    }
    else {
      sprintf(pC_->outString_, "%1dMM0", axisNo_ + 1);
      status = pC_->writeController();
    }
  }
  // ? not sure yet
  //status = pC_->writeReadController();
  return status;
}

/** Polls the axis.
  * This function reads motor position, limit status, home status, and moving status
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus Pico8742Axis::poll(bool *moving)
{ 
  static const char *functionName = "poll()";
  
  int done, posenable;
  int hardwareStatus;
  //int driveOn;
  double position;
  asynStatus comStatus;

  // Read the current motor position
  sprintf(pC_->outString_, "%1dTP?", axisNo_ + 1);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is in steps
  position = (atof(pC_->inString_));
  setDoubleParam(pC_->motorPosition_, position);

  // Read the motion done status
  sprintf(pC_->outString_, "%1dMD?", axisNo_ + 1);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is 0 for motion in progress, 1 for motion not in progress
  // May need to add logic for moving while homing
  done = atoi(pC_->inString_);
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false:true;

  /* if closed loop controller we have extra status bits */
  if(pC_->closedLoop_ == CLOSED_LOOP) {
    setIntegerParam(pC_->motorStatusGainSupport_, 1);
    sprintf(pC_->outString_, "PH?");
    comStatus = pC_->writeReadController();
    if (comStatus) goto skip;
    // The response string is in steps
    hardwareStatus = (atof(pC_->inString_));
    setIntegerParam(pC_->motorStatusLowLimit_, (hardwareStatus >> (3*axisNo_))&0x1);
    setIntegerParam(pC_->motorStatusLowLimit_, (hardwareStatus >> (3*axisNo_))&0x2);
    //setIntegerParam(pC_->motorStatusAtHome_, (hardwareStatus >> (3*axisNo_))&0x4);
    
    // Position maintenance readback
    sprintf(pC_->outString_, "%1dMM?", axisNo_ + 1);
    comStatus = pC_->writeReadController();
    if (comStatus) goto skip;
    posenable = (atoi(pC_->inString_));
    setIntegerParam(pC_->motorStatusPowerOn_, posenable);
  }
  // Read the drive power on status
  //sprintf(pC_->outString_, "#%02dE", axisNo_ + 1);
  //comStatus = pC_->writeReadController();
  //if (comStatus) goto skip;
  //driveOn = (pC_->inString_[5] == '1') ? 1:0;
  //setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
  //setIntegerParam(pC_->motorStatusProblem_, 0);

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  if(comStatus)
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Communication Error\n", driverName, functionName);
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg Pico8742CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg Pico8742CreateControllerArg1 = {"Pico8742 port name", iocshArgString};
static const iocshArg Pico8742CreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg Pico8742CreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg Pico8742CreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg Pico8742CreateControllerArg5 = {"Controller Number", iocshArgInt};
static const iocshArg * const Pico8742CreateControllerArgs[] = {&Pico8742CreateControllerArg0,
                                                             &Pico8742CreateControllerArg1,
                                                             &Pico8742CreateControllerArg2,
                                                             &Pico8742CreateControllerArg3,
                                                             &Pico8742CreateControllerArg4,
                                                             &Pico8742CreateControllerArg5};
static const iocshFuncDef Pico8742CreateControllerDef = {"Pico8742CreateController", 6, Pico8742CreateControllerArgs};
static void Pico8742CreateContollerCallFunc(const iocshArgBuf *args)
{
  Pico8742CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}

static void Pico8742Register(void)
{
  iocshRegister(&Pico8742CreateControllerDef, Pico8742CreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(Pico8742Register);
}
