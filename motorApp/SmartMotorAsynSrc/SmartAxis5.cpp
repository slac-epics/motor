/*
FILENAME... SmartMotorDriver.cpp
USAGE...    Motor driver support for the Amimatics SmartMotor series of integrated motor/controllers.

Mark Rivers
March 4, 2011

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "SmartController.h"
#include <epicsExport.h>

#define DEBUG

/** System 16-bit status word CLASS 5 SmartMotor */
#define DRIVE_READY     0x1
#define MOFF            0x2
#define MOVING          0x4
#define VOLT_ERROR      0x8
#define OVERCURRENT     0x10
/* TEMP_MINOR - excessive temperature real time */
#define TEMP_MINOR      0x20
#define POS_ERROR       0x40
#define VEL_ERROR       0x80
/* TEMP_MAJOR - real-time temperature limit */
#define TEMP_MAJOR      0x100
#define DERIV_ERROR     0x200
#define POS_LIM_EN      0x400
#define NEG_LIM_EN      0x800
#define HIST_POS_LIM    0x1000
#define HIST_NEG_LIM    0x2000
#define POS_LIM         0x4000
#define NEG_LIM         0x8000

static const char *driverName = "SmartAxis5Driver";


// These are the SmartAxis methods

/** Creates a new SmartAxis object.
  * \param[in] pC Pointer to the SmartController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
SmartAxis5::SmartAxis5(SmartController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  asynStatus status;
  axisNum_ = Smart_addr[axisNo];

  sprintf(pC->outString_, "%cRSAMP", axisNum_);
  status = pC->writeReadController();
  if (status) {
        sampleRate_ =8000.0; // assume default sample rate
        asynPrint(pC->pasynUserSelf, ASYN_TRACE_ERROR,
        "%s: Axis %d sample rate not reported",driverName, axisNo);
        setIntegerParam(pC->motorStatusProblem_, 1);
        goto skip;
  } 
  sampleRate_ = atof(pC->inString_);
  /* velocity units are 65536/sample rate */
  velConst_ = 65536.0/sampleRate_;
  /* acclerations units are 65536/(sample*sample)  */
  aclConst_ = 65536.0/(sampleRate_*sampleRate_);
#ifdef DEBUG
  printf("Sample rate = %f\n", sampleRate_);
  printf("Velocity constant = %f\n", velConst_);
  printf("Accleration constant = %f\n", aclConst_);
#endif
  setClosedLoop(true);

  skip:
// assume servo motor with encoder
  setIntegerParam(pC->motorStatusGainSupport_, 1);
  setIntegerParam(pC->motorStatusHasEncoder_, 1);

  callParamCallbacks();
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorController::report()
  */
void SmartAxis5::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n"
            "    encoder position=%f\n"
            "    commanded position=%f\n"
            "    status=0x%x\n", 
            axisNo_, 
            encoderPosition_, 
            commandedPosition_,
            currentStatus_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}


asynStatus SmartAxis5::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "moveAxis";
  
  status = resetFlags();
  if (status) goto skip;
  sprintf(pC_->outString_, "%cMP", axisNum_); // Set position mode
  status = pC_->writeController();
  if (status) goto skip;
  status = setAcceleration(acceleration);
  if (status) goto skip;
  status = setVelocity(minVelocity, maxVelocity);
  if (status) goto skip;
  if (relative) {
    sprintf(pC_->outString_, "%cPRT=%ld", axisNum_, (long)round(position)); // Set target relative position
    status = pC_->writeController();
    if (status) goto skip;
  } else {
    sprintf(pC_->outString_, "%cPT=%ld", axisNum_, (long)round(position)); // Set target absolute position
    printf("PT=%ld\n", (long)round(position)); // Set target absolute position
    status = pC_->writeController();
    if (status) goto skip;
  }
  status = go();
  skip:
  setIntegerParam(pC_->motorStatusProblem_, status ? 1:0);
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1:0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis5::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status;
  // static const char *functionName = "homeAxis";
  status = setPosition(0);
  status = setAcceleration(acceleration);
  status = setVelocity(minVelocity, maxVelocity);
  return status;
}

asynStatus SmartAxis5::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "moveVelocityAxis";
  status = resetFlags();
  if(status) goto skip;
  sprintf(pC_->outString_, "%cMV", axisNum_); // Velocity mode
  status = pC_->writeController();
  if(status) goto skip;
  status = setAcceleration(acceleration);
  if(status) goto skip;
  status = setVelocity(minVelocity, maxVelocity);
  if(status) goto skip;
  status = go();
  
  skip:
  setIntegerParam(pC_->motorStatusProblem_, status ? 1:0);
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1:0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis5::stop(double acceleration )
{
#ifdef DEBUG
  printf("STOP\n");
#endif
  asynStatus status;
  //static const char *functionName = "stopAxis";

  sprintf(pC_->outString_,"%cS", axisNum_); // stop without deceleration
//  sprintf(pC_->outString_,"%cADT=%f", axisNum_, acceleration); //set target decleration
//  sprintf(pC_->outString_,"%cX", axisNum_); // decelerate to stop
  status = pC_->writeController();
  setPosition(encoderPosition_);
  setEncoderPosition(encoderPosition_);
  return status;
}

asynStatus SmartAxis5::setPosition(double position)
{
#ifdef DEBUG
  printf("SET POSITION:  ");
  printf("O=%ld\n",(long)round(position));
#endif
  asynStatus status;
  sprintf(pC_->outString_, "%cO=%ld", axisNum_, (long)round(position));
  status = pC_->writeController();
  return status;
}

asynStatus SmartAxis5::setEncoderPosition(double position)
{
  return setPosition(position);
}

asynStatus SmartAxis5::setClosedLoop(bool closedLoop)
{
  asynStatus status;

  if(closedLoop){
    /* set motor to ON, servo at current position */
    sprintf(pC_->outString_, "%cRPT", axisNum_);
    status = pC_->writeReadController();
    if (status) goto skip;
    commandedPosition_ = atof(pC_->inString_);
    status = move(commandedPosition_, 0, 0, 0, 0);
  }
  else{
    sprintf(pC_->outString_,"%cOFF", axisNum_);  //enable software limits
    status = pC_->writeController();
  }

skip:
  return status;
}


asynStatus SmartAxis5::setPGain(double pGain)
{
  asynStatus status;
  int pval;
  pGain = pGain > 1 ? 1:pGain; // expecting value between 0 and 1 from motor record
  pGain = pGain < 0 ? 0:pGain;
  pval = (int) (pGain * 65535);
#ifdef DEBUG
  printf("Set P Gain:  \n");
  printf("%cKP=%d", axisNum_, pval);
#endif
  sprintf(pC_->outString_, "%cKP=%d", axisNum_, pval);
  status = pC_->writeController();
  return status;
}

asynStatus SmartAxis5::setIGain(double iGain)
{
  asynStatus status;
  int ival;
  iGain = iGain > 1 ? 1:iGain;
  iGain = iGain < 0 ? 0:iGain;
  ival = (int) (iGain * 65535);
#ifdef DEBUG
  printf("Set I Gain:  \n");
  printf("%cKI=%d", axisNum_, ival);
#endif
  sprintf(pC_->outString_, "%cKI=%d", axisNum_, ival);
  status = pC_->writeController();
  return status;
}

asynStatus SmartAxis5::setDGain(double dGain)
{
  asynStatus status;
  int dval;
  dGain = dGain > 1 ? 1:dGain;
  dGain = dGain < 0 ? 0:dGain;
  dval = (int) (dGain * 65535);
#ifdef DEBUG
  printf("Set D Gain:  \n");
  printf("%cKD=%d", axisNum_, dval);
#endif
  sprintf(pC_->outString_, "%cKD=%d", axisNum_, dval);
  status = pC_->writeController();
  return status;
}

asynStatus SmartAxis5::setHighLimit(double highLimit)
{
#ifdef DEBUG
  printf("High limit %ld\n",(long)round(highLimit));
#endif
  asynStatus status;
  sprintf(pC_->outString_,"%cSLE", axisNum_);  //enable software limits
  status = pC_->writeController();
  if(status)	goto skip;
  sprintf(pC_->outString_,"%cSLM(0)", axisNum_); //software limit doesn't cause fault, allows move off
  status = pC_->writeController();
  if(status) goto skip;
  sprintf(pC_->outString_,"%cSLP=%ld", axisNum_, (long)round(highLimit));
  status = pC_->writeController();  
  skip:
  return status;
}

asynStatus SmartAxis5::setLowLimit(double lowLimit)
{
#ifdef DEBUG
  printf("Low limit %ld\n",(long)round(lowLimit));
#endif
  asynStatus status;
  sprintf(pC_->outString_,"%cSLE", axisNum_);  //enable software limits
  status = pC_->writeController();
  if(status) goto skip;
  sprintf(pC_->outString_,"%cSLM(0)", axisNum_); //software limit doesn't cause fault, allows move off
  status = pC_->writeController();
  if(status) goto skip;
  sprintf(pC_->outString_,"%cSLN=%ld", axisNum_, (long)round(lowLimit));
  status = pC_->writeController();  
  skip:
  return status;
}

asynStatus SmartAxis5::setVelocity(double minVelocity, double maxVelocity)
{
  asynStatus status;
  /* velocity units are 65536/sample rate */
  sprintf(pC_->outString_, "%cVT=%ld", axisNum_, (long)round(maxVelocity*velConst_)); // Set target velocity
#ifdef DEBUG  
  printf("Set velocity:  VT=%ld\n", (long)round(maxVelocity*aclConst_));
#endif
  status = pC_->writeController();
  return status;
}

asynStatus SmartAxis5::setAcceleration(double acceleration)
{
  asynStatus status;
  /* acclerations units are 65536/(sample*sample) */
  sprintf(pC_->outString_, "%cADT=%ld", axisNum_, (long)round(acceleration*aclConst_)); // Set target acceleration
#ifdef DEBUG
  printf("Set acceleration:  ADT=%ld\n", (long)round(acceleration*aclConst_));
#endif
  status = pC_->writeController();
  return status;
}

asynStatus SmartAxis5::go()
{
#ifdef DEBUG
  printf("GO\n");
#endif
  asynStatus status;
  sprintf(pC_->outString_, "%cG", axisNum_); // GO
  status = pC_->writeController();
  return status;
}

asynStatus SmartAxis5::resetFlags()
{
#ifdef DEBUG
  printf("Reset flags\n");
#endif
  asynStatus status;
  sprintf(pC_->outString_,"%cZS", axisNum_); //
  status = pC_->writeController();
  return status;
}



/** Polls the axis.
  * This function reads the controller position, encoder position, the limit status, the moving status, 
  * and the drive power-on status.  It does not current detect following error, etc. but this could be
  * added.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus SmartAxis5::poll(bool *moving)
{ 
  int done;
  int driveOn;
  int limit;
  int followingError;
  int motorError;
  asynStatus status;
  // Read the current encoder position
  sprintf(pC_->outString_, "%cRPA", axisNum_); // Report actual position
  status = pC_->writeReadController();
  if (status) goto skip;
  encoderPosition_ = atof(pC_->inString_);
  setDoubleParam(pC_->motorEncoderPosition_,encoderPosition_);
  sprintf(pC_->outString_, "%cRPC", axisNum_);  // Report commanded position
  status = pC_->writeReadController();
  if (status) goto skip;
  commandedPosition_ = atof(pC_->inString_);
  setDoubleParam(pC_->motorPosition_, commandedPosition_);

  // Read the drive 16-bit status word
  sprintf(pC_->outString_, "%cRW(0)", axisNum_); // Report 16-bit status word
  status = pC_->writeReadController();
  if (status) goto skip;
  currentStatus_ = atoi(pC_->inString_);
  done = (currentStatus_ & MOVING)?0:1; // Trajectory in progress
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false:true;
  limit = (currentStatus_ & POS_LIM)?1:0; // Real time positive limit active
  setIntegerParam(pC_->motorStatusHighLimit_, limit);
  limit = (currentStatus_ & NEG_LIM)?1:0; // Real time negative limit switch
  setIntegerParam(pC_->motorStatusLowLimit_, limit);
//  limit = (currentStatus_ & 0x300)?1:0;
//  setIntegerParam(pC_->motorStatusAtHome_, limit);
  driveOn = (currentStatus_ & MOFF) ? 0:1; //Motor OFF bit
  setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
  followingError = (currentStatus_ & POS_ERROR) ? 1:0; //Check for following error
  setIntegerParam(pC_->motorStatusFollowingError_,followingError);
  motorError = (currentStatus_ & (VOLT_ERROR | OVERCURRENT | TEMP_MINOR | TEMP_MAJOR | VEL_ERROR | DERIV_ERROR)) ? 1:0; 

  skip:
  setIntegerParam(pC_->motorStatusProblem_, (status != asynSuccess) ? 1:motorError);
  callParamCallbacks();
  return status;
}
