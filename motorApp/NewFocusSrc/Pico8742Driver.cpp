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

static const char *driverName = "Pico8742Driver";

/** Creates a new Pico8742Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] Pico8742PortName     The name of the drvAsynSerialPort that was created previously to connect to the Pico8742 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
Pico8742Controller::Pico8742Controller(const char *portName, const char *Pico8742PortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_Pico8742_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  Pico8742Axis *pAxis;
  static const char *functionName = "Pico8742Controller::Pico8742Controller";

  createParam(Pico8742FirmwareString,                 asynParamOctet,   &Pico8742FirmwareString_);
  
  /* Connect to Pico8742 controller */
  status = pasynOctetSyncIO->connect(Pico8742PortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to Pico8742 controller\n",
      functionName);
  }
  /* get FW version is it 8742 or 8743 */ 
  /* 8742 Version 1.9 11/01/12 */
  /* 8743-CL Version 1.9 11/01/12 */
  sprintf(outString_, "VE?");
  status = writeReadController();
  setStringParam(Pico8742FirmwareString_, inString_);

  closedLoop_ = (strncmp(inString_, "8743", 4) == 0) ? CLOSED_LOOP:OPEN_LOOP;
  
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
                                   int movingPollPeriod, int idlePollPeriod)
{
   
  new Pico8742Controller(portName, Pico8742PortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  
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
/* get pico motor type, if small limit velocity */
  sprintf(pC_->outString_, "%1dMC", axisNo_ + 1);
  status = pC_->writeReadController();
  sprintf(pC_->outString_, "%1dQM?", axisNo_ + 1);
  status = pC_->writeReadController();
  picoType_ = (atoi(pC_->inString_));
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
  // static const char *functionName = "Pico8742Axis::home";

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
  if (picoType_ == CLOSED_LOOP) {
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
  
  int done;
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
static const iocshArg * const Pico8742CreateControllerArgs[] = {&Pico8742CreateControllerArg0,
                                                             &Pico8742CreateControllerArg1,
                                                             &Pico8742CreateControllerArg2,
                                                             &Pico8742CreateControllerArg3,
                                                             &Pico8742CreateControllerArg4};
static const iocshFuncDef Pico8742CreateControllerDef = {"Pico8742CreateController", 5, Pico8742CreateControllerArgs};
static void Pico8742CreateContollerCallFunc(const iocshArgBuf *args)
{
  Pico8742CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void Pico8742Register(void)
{
  iocshRegister(&Pico8742CreateControllerDef, Pico8742CreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(Pico8742Register);
}
