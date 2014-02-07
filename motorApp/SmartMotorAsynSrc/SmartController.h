/*
FILENAME...   SmartMotorDriver.h
USAGE...      Motor driver support for the Parker Smart series of controllers, including the Aries.

Mark Rivers
March 28, 2011

*/
#ifndef SMART_CONTROLLER_H
#define SMART_CONTROLLER_H
#define DEBUG
#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "SmartAxis4.h"
#include "SmartAxis5.h"
#include <epicsString.h>

inline double round(double num)
{
  return (num >0.0)?(num + 0.5):(num - 0.5);
}

static int Smart_addr[] = {129, 130, 131, 132, 133, 134, 135, 136, 137, 138,
                           139, 140, 141, 142, 143, 144, 145, 146, 147, 148,
                           149, 150, 151, 152, 153, 154, 155, 156, 157, 158,
                           159, 160};

/** drvInfo strings for extra parameters that the Smart controller supports */
class SmartController : public asynMotorController {
public:
  SmartController(const char *portName, const char *SmartPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);
  /* These are the methods that we override from asynMotorDriver */
  void report(FILE *fp, int level);
//  SmartAxis* getAxis(asynUser *pasynUser);
//  SmartAxis* getAxis(int axisNo);
  asynMotorAxis* getAxis(asynUser *pasynUser);
  asynMotorAxis* getAxis(int axisNo);
  asynStatus writeReadController();
  asynStatus writeController();
  
protected:
  char inputEos[5];
  char outputEos[5];
  char echoEos[5];
  asynStatus getFW(int &fwMajor, int axisNum);
  
  
friend class SmartAxis4;
friend class SmartAxis5;
};
#endif
