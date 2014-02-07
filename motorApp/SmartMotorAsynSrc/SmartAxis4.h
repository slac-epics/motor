/*
FILENAME...   SmartMotorDriver.h
USAGE...      Motor driver support for the Parker Smart series of controllers, including the Aries.

Mark Rivers
March 28, 2011

*/
#ifndef SMART_AXIS
#define SMART_AXIS

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "SmartController.h"



class SmartAxis4 : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  SmartAxis4(class SmartController *pC, int axis);
  void report(FILE *fp, int level);
  virtual asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  virtual asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  virtual asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  virtual asynStatus stop(double acceleration);
  virtual asynStatus poll(bool *moving);
  virtual asynStatus setPosition(double position);
  virtual asynStatus setEncoderPosition(double position);
  virtual asynStatus setClosedLoop(bool closedLoop);
  virtual asynStatus setPGain(double pGain);
  virtual asynStatus setIGain(double iGain);
  virtual asynStatus setDGain(double dGain);


protected:
  SmartController *pC_;       /** Pointer to the asynMotorController */
  double encoderPosition_;    /** Cached copy of the encoder position */ 
  double commandedPosition_;  /** Cached copy of the commanded position */ 
  int currentStatus_;         /** Cached copy of the current status word */ 
  double sampleRate_;         /** Sample rate for smart motor */
  double velConst_;           /** velocity commands are scaled by this variable */
  double aclConst_;           /** acceleration commands are scaled by this variable */
  int axisNum_;               /** Used to decide which address to send command to */ 

  virtual asynStatus setAcceleration(double accleration);
  virtual asynStatus setVelocity(double minVelocity, double maxVelocity);
  virtual asynStatus go();
  virtual asynStatus resetFlags();
  virtual asynStatus forcePIDUpdate();
  virtual asynStatus getPID();
friend class SmartController;
};

#endif
