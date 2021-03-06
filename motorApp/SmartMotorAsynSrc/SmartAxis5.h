/*
FILENAME...   SmartMotorDriver.h
USAGE...      Motor driver support for the Parker Smart series of controllers,
including the Aries.

Mark Rivers
March 28, 2011

*/
#ifndef SMART_AXIS5
#define SMART_AXIS5

#include "SmartAxisBase.h"
#include "SmartCommands.h"

class SmartAxis5 : public SmartAxisBase {
 public:
  /* These are the methods we override from the base class */
  SmartAxis5(class SmartController *pC, int axis);
  asynStatus move(double position, int relative, double min_velocity,
                  double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity,
                          double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration,
                  int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus pollStatus(bool *moving);
  asynStatus pollPosition();
  asynStatus pollSmartParameters();
  asynStatus setPosition(double position);
  asynStatus setEncoderPosition(double position);
  asynStatus setClosedLoop(bool closedLoop);
  asynStatus setPGain(double pGain);
  asynStatus setIGain(double iGain);
  asynStatus setDGain(double dGain);
  asynStatus setHighLimit(double highLimit);
  asynStatus setLowLimit(double lowLimit);
  asynStatus storePosition(double position);
  asynStatus setControllerMemory(char *location, epicsInt32 value);
  asynStatus processDeferredMoves();

 protected:
  int canAddr_;
  int pollPeriodDivisor_;
  int pollCounter_;

  asynStatus setAcceleration(double accleration);
  asynStatus setVelocity(double minVelocity, double maxVelocity);
  asynStatus go();
  asynStatus resetFlags();
  asynStatus handleError(int error);
};

#endif
