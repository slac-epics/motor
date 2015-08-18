/*
FILENAME...   Pico8742Driver.h
USAGE...      Motor driver support for the Newport Pico8742 controller.

Based on the SMC100 Model 3 device driver written by:
K. Goetze
Mark Rivers
March 1, 2012

M. D'Ewart 2014-10-14  Initial version

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_Pico8742_AXES 4
#define MAX_Pico8743_AXES 2

#define Pico8742FirmwareString               "FIRMWARE_VERSION"
#define Pico8742MotorCheckString             "MOTOR_CHECK"
#define Pico8742MotorTypeString              "MOTOR_TYPE"
#define Pico8742SoftResetString              "SOFT_RESET"


class Pico8742Axis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  Pico8742Axis(class Pico8742Controller *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setClosedLoop(bool closedLoop);

  asynStatus setMotorType();
private:
  Pico8742Controller *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  int picoType_;                   /** Pico type, returned from QM command */
  asynStatus sendAccelAndVelocity(double accel, double velocity);
  
friend class Pico8742Controller;
};

class Pico8742Controller : public asynMotorController {
public:
  Pico8742Controller(const char *portName, const char *Pico8742PortName, int numAxes, double movingPollPeriod, double idlePollPeriod, int ctrlNo);

  void report(FILE *fp, int level);
  Pico8742Axis* getAxis(asynUser *pasynUser);
  Pico8742Axis* getAxis(int axisNo);

  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeReadController();
  asynStatus writeController();

protected:
  int closedLoop_;
  int ctrlNo_;

  #define FIRST_Pico8742_PARAM Pico8742FirmwareString_
  int Pico8742FirmwareString_;
  int Pico8742MotorCheck_;
  int Pico8742MotorType_;
  int Pico8742SoftReset_;
  #define LAST_Pico8742_PARAM Pico8742SoftReset_
  asynUser *pasynUserCommonController_;  
  asynStatus motorCheck();
  asynStatus softReset();

friend class Pico8742Axis;
};
#define NUM_Pico8742_PARAMS (&LAST_Pico8742_PARAM - &FIRST_Pico8742_PARAM + 1)
