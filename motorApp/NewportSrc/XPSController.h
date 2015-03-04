/*
FILENAME...     XPSMotorDriver.cpp
USAGE...        Newport XPS EPICS asyn motor device driver
*/

#ifndef XPSController_H
#define XPSController_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "XPSAxis.h"

#define XPS_MAX_AXES 8
#define XPS_POLL_TIMEOUT 2.0
#define XPS_MOVE_TIMEOUT 100000.0 // "Forever"
#define XPS_MIN_PROFILE_ACCEL_TIME 0.25

// drvInfo strings for extra parameters that the XPS controller supports
#define XPSMinJerkString                "XPS_MIN_JERK"
#define XPSMaxJerkString                "XPS_MAX_JERK"
#define XPSProfileMaxVelocityString     "XPS_PROFILE_MAX_VELOCITY"
#define XPSProfileMaxAccelerationString "XPS_PROFILE_MAX_ACCELERATION"
#define XPSProfileMinPositionString     "XPS_PROFILE_MIN_POSITION"
#define XPSProfileMaxPositionString     "XPS_PROFILE_MAX_POSITION"
#define XPSProfileGroupNameString       "XPS_PROFILE_GROUP_NAME"
#define XPSTrajectoryFileString         "XPS_TRAJECTORY_FILE"
#define XPSStatusString                 "XPS_STATUS"
#define updateAxisInfoString            "UPDATE_AXIS_INFO"
#define XPSVbasString                   "XPS_VBAS"
#define XPSVeloString                   "XPS_VELO"
#define XPSVmaxString                   "XPS_VMAX"
#define XPSAcclString                   "XPS_ACCL"
#define XPSMresString                   "XPS_MRES"
#define XPSHlmString                    "XPS_HLM"
#define XPSLlmString                    "XPS_LLM"
#define XPSUnitsString                  "XPS_EGU"
#define XPSSStatusString                "XPS_SSTATUS"
#define XPSStageStringString            "XPS_STAGE"
#define XPSDriverString                 "XPS_DRIVER"
#define XPSConnectedString              "XPS_CONNECTED"
#define XPSControllerStatusString       "XPS_CONTROLLER_STATUS"
#define XPSFirmwareString               "FIRMWARE_VERSION"

class XPSController : public asynMotorController {

  public:
  XPSController(const char *portName, const char *IPAddress, int IPPort,
                int numAxes, double movingPollPeriod, double idlePollPeriod,
                int enableSetPosition, double setPositionSettlingTime);

  /* These are the methods that we override from asynMotorDriver */
  void report(FILE *fp, int level);
  XPSAxis* getAxis(asynUser *pasynUser);
  XPSAxis* getAxis(int axisNo);
  asynStatus poll();
  asynStatus setDeferredMoves(bool deferMoves);
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

  /* These are the functions for profile moves */
  asynStatus initializeProfile(size_t maxPoints, const char* ftpUsername, const char* ftpPassword);
  asynStatus buildProfile();
  asynStatus executeProfile();
  asynStatus abortProfile();
  asynStatus readbackProfile();

  /* These are the methods that are new to this class */
  void profileThread();
  asynStatus runProfile();
  asynStatus waitMotors();

  /* Deferred moves functions.*/
  asynStatus processDeferredMovesInGroup(char * groupName);

  /*Disable automatic enable of axes.*/
  asynStatus disableAutoEnable();
  /* Function to disable the MSTA problem bit in the event of an XPS Disable state 20 */
  asynStatus noDisableError();

  protected:
  XPSAxis **pAxes_;       /**< Array of pointers to axis objects */

  #define FIRST_XPS_PARAM XPSMinJerk_
  int XPSMinJerk_;
  int XPSMaxJerk_;
  int XPSProfileMaxVelocity_;
  int XPSProfileMaxAcceleration_;
  int XPSProfileMinPosition_;
  int XPSProfileMaxPosition_;
  int XPSProfileGroupName_;
  int XPSTrajectoryFile_;
  int XPSStatus_;
  int updateAxisInfo_;
  int XPSvbas_;
  int XPSvelo_;
  int XPSvmax_;
  int XPSaccl_;
  int XPSmres_;
  int XPShlm_;
  int XPSllm_;
  int XPSUnitsString_;
  int XPSSStatusString_;
  int XPSStageString_;
  int XPSDriverString_;
  int XPSConnected_;
  int XPSControllerStatus_;
  int XPSFirmwareString_;
  #define LAST_XPS_PARAM XPSFirmwareString_

  private:
  asynStatus connectController();
  asynStatus disconnectController();
  
  bool enableSetPosition_;          /**< Enable/disable setting the position from EPICS */ 
  double setPositionSettlingTime_;  /**< The settling (sleep) time used when setting position. */
  char *IPAddress_;
  int IPPort_;
  char *ftpUsername_;
  char *ftpPassword_;
  int pollSocket_;
  int moveSocket_;
  char firmwareVersion_[100];
  bool movesDeferred_;
  epicsEventId profileExecuteEvent_;
  int autoEnable_;
  int noDisableError_;
  int controllerStatus_;
  int connected_;
  
  friend class XPSAxis;
};
#define NUM_XPS_PARAMS (&LAST_XPS_PARAM - &FIRST_XPS_PARAM + 1)
#endif /* XPSController_H */

