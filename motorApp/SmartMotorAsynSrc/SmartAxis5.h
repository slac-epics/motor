/*
FILENAME...   SmartMotorDriver.h
USAGE...      Motor driver support for the Parker Smart series of controllers, including the Aries.

Mark Rivers
March 28, 2011

*/
#ifndef SMART_AXIS5
#define SMART_AXIS5

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "SmartController.h"



   
class SmartAxis5 : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  SmartAxis5(class SmartController *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setEncoderPosition(double position);
  asynStatus setClosedLoop(bool closedLoop);
  asynStatus setPGain(double pGain);
  asynStatus setIGain(double iGain);
  asynStatus setDGain(double dGain);
  asynStatus setHighLimit(double highLimit);
  asynStatus setLowLimit(double lowLimit);


protected:
  SmartController *pC_;       /** Pointer to the asynMotorController  */
  double encoderPosition_;    /** Cached copy of the encoder position */ 
  double commandedPosition_;  /** Cached copy of the commanded position */ 
  int currentStatus_;         /** Cached copy of the current status word */ 
  double sampleRate_;         /** Sample rate for smart motor */
  double velConst_;           /** velocity commands are scaled by this variable */
  double aclConst_;           /** acceleration commands are scaled by this variable */
  int axisNum_;               /** Used to decide which address to send command to */  

  asynStatus setAcceleration(double accleration);
  asynStatus setVelocity(double minVelocity, double maxVelocity);
  asynStatus go();
  asynStatus resetFlags();
friend class SmartController;
};

#endif
