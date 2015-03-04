//!  Description : This is the "model 3" asyn motor driver for IMS MDrivePlus.
//!                Based on HytecMotorDriver.h.
//! @ImsMDrivePlus.h

#ifndef ImsMDrivePlusMotorController_H
#define ImsMDrivePlusMotorController_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "ImsMDrivePlusMotorAxis.h"

////////////////////////////////////
//  ImsMDrivePlusMotorController class
//! derived from asynMotorController class
////////////////////////////////////
class ImsMDrivePlusMotorController : public asynMotorController
{
public:
	/////////////////////////////////////////
	// Override asynMotorController functions
	/////////////////////////////////////////
	ImsMDrivePlusMotorController(const char *motorPortName, const char *IOPortName, const char *deviceName, double movingPollPeriod, double idlePollPeriod, int homeSwitchIndex);
	ImsMDrivePlusMotorAxis* getAxis(asynUser *pasynUser);
	ImsMDrivePlusMotorAxis* getAxis(int axisNo);
	//void report(FILE *fp, int level);
	asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

	/////////////////////////////////////////
	// IMS specific functions
	/////////////////////////////////////////
	asynStatus writeReadController(const char *output, char *input, size_t maxChars, size_t *nread, double timeout);
	asynStatus writeController(const char *output, double timeout);

	

protected:
	 ImsMDrivePlusMotorAxis **pAxes_;       // Array of pointers to axis objects

	///////////////////////////////////////////
	// IMS MDrivePlus controller function codes
	///////////////////////////////////////////

	//! extra parameters that the ImsMDrivePlus controller supports
	int ImsMDrivePlusLoadMCode_;    //! Load MCode string, NOT SUPPORTED YET
	int ImsMDrivePlusClearMCode_;   //! Clear program buffer, NOT SUPPORTED YET
	int ImsMDrivePlusSaveToNVM_;    //! Store current user variables and flags to nonvolatile ram
	int ImsMDrivePlusErrorCode_;       //! IMS Error Code
	int ImsMDrivePlusErrorString_; //! IMS Error String
#define FIRST_IMS_PARAM ImsMDrivePlusLoadMCode_
#define LAST_IMS_PARAM ImsMDrivePlusErrorString_
#define NUM_IMS_PARAMS (&LAST_IMS_PARAM - &FIRST_IMS_PARAM + 1)

private:
// drvInfo strings for extra parameters that the ImsMDrivePlus controller supports
#define ImsMDrivePlusLoadMCodeControlString	"IMS_LOADMCODE"    // NOT SUPPORTED YET
#define ImsMDrivePlusClearMCodeControlString	"IMS_CLEARMCODE"   // NOT SUPPORTED YET
#define ImsMDrivePlusSaveToNVMControlString	"IMS_SAVETONVM"
#define ImsMDrivePlusErrorCodeString	        "IMS_ERROR"
#define ImsMDrivePlusErrorStringString	        "IMS_ERROR_STRING"

	asynUser *pAsynUserIMS;
	char motorName[MAX_NAME_LEN];
	char deviceName[MAX_NAME_LEN];
	int homeSwitchInput;
	int posLimitSwitchInput;
	int negLimitSwitchInput;

	// added for defining Home Switch or Index
	int homeSwIn;

	void initController(const char *devName, double movingPollPeriod, double idlePollPeriod, int homeSwitchIndex);
	int readHomeAndLimitConfig();  // read home, positive limit, and neg limit switch configuration from controller (S1-S4 settings)

	friend class ImsMDrivePlusMotorAxis;
};
//! iocsh function to create controller object
//! NOTE: drvAsynIPPortConfigure() must be called first
//extern "C" int ImsMDrivePlusCreateController(const char *motorPortName, const char *IOPortName, char *devName, double movingPollPeriod, double idlePollPeriod);

#endif // ImsMDrivePlusMotorController_H

