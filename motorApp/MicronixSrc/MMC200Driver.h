/*
FILENAME...   MMC200Driver.h
USAGE...      Motor driver support for the Micronix MMC-100 and MMC-200 controllers.

Kevin Peterson
July 10, 2013

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "dbAccess.h"

#define MAX_MMC200_AXES 99

// No controller-specific parameters yet
#define NUM_MMC200_PARAMS 0  

class MMC200Axis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  MMC200Axis(class MMC200Controller *pC, int axis, const char *fbkPVPrefix);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setClosedLoop(bool closedLoop);

private:
  MMC200Controller *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  int axisIndex_;    /* Numbered from 1 */
  char versionStr_[256];  /* Version string */
  int model_;         /* Model number (200 or 100) */
  int rez_;          /* Units = picometers per full step */
  int microSteps_;   /* Units = microsteps per full step */
  unsigned int desiredFbk_; /* Desired feedback mode from fbkPV_, can be 0, 2, 3 for MMC200 */
  unsigned int lastFbkModeSet_;  /* Last requested feedback mode setting */
  double resolution_;   /* Units = mm per microstep */
  double maxVelocity_;  /* Units = mm per second */
  const char *fbkPV_;  /* PV providing user control of axis feedback mode, constructed from prefix */
  asynStatus sendAccelAndVelocity(double accel, double velocity);
  dbAddr *fbkPVAddr_;
  
friend class MMC200Controller;
};

class MMC200Controller : public asynMotorController {
public:
  MMC200Controller(const char *portName, const char *MMC200PortName, int numAxes, double movingPollPeriod, double idlePollPeriod, int ignoreLimits, const char *fbkPVPrefix);

  void report(FILE *fp, int level);
  MMC200Axis* getAxis(asynUser *pasynUser);
  MMC200Axis* getAxis(int axisNo);

protected:
	MMC200Axis **pAxes_;
 
private:
  int ignoreLimits_;  /* 1 = ignore limits */

friend class MMC200Axis;
};
