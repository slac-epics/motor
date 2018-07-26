#ifndef SMARACT_MCS2_MOTOR_DRIVER_H
#define SMARACT_MCS2_MOTOR_DRIVER_H

/* Motor driver support for smarAct MCS2 Controller   */

#ifdef __cplusplus

#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <stdarg.h>
#include <exception>

enum SmarActMCS2ExceptionType {
	MCS2UnknownError,
	MCS2ConnectionError,
	MCS2CommunicationError,
};

enum SmarActMCS2MoveMode {
    MCS2MM_ABS     =0,
    MCS2MM_REL     =1,
    MCS2MM_SCAN_ABS=2,
    MCS2MM_SCAN_REL=3,
    MCS2MM_STEP    =4,
};

class SmarActMCS2Exception : public std::exception {
public:
	SmarActMCS2Exception(SmarActMCS2ExceptionType t, const char *fmt, ...);
	SmarActMCS2Exception(SmarActMCS2ExceptionType t)
		: t_(t)
		{ str_[0] = 0; }
	SmarActMCS2Exception()
		: t_(MCS2UnknownError)
		{ str_[0] = 0; }
	SmarActMCS2Exception(SmarActMCS2ExceptionType t, const char *fmt, va_list ap);
	SmarActMCS2ExceptionType getType()
		const { return t_; }
	virtual const char *what()
		const throw() {return str_ ;}
protected:
	char str_[100];	
	SmarActMCS2ExceptionType t_;
};

class SmarActMCS2Controller;

class SmarActMCS2Axis : public asynMotorAxis
{
public:
	SmarActMCS2Axis(class SmarActMCS2Controller *cnt_p, int axis, int channel);
	asynStatus  poll(bool *moving_p);
	asynStatus  move(double position, int relative, double min_vel, double max_vel, double accel);
	asynStatus  home(double min_vel, double max_vel, double accel, int forwards);
	asynStatus  stop(double acceleration);
	asynStatus  setPosition(double position);
	asynStatus  moveVelocity(double min_vel, double max_vel, double accel);

	virtual asynStatus getVal(const char *parm, long long *val_p);
	virtual asynStatus getVal(const char *parm, int *val_p);
	char              *getError();  // Return NULL for no error, static string otherwise!
        void               clearErrors();
	virtual int        getClosedLoop();
	long long          getVel() const { return vel_; }

protected:
	asynStatus  setSpeed(double velocity);
	asynStatus  setAccel(double accel);
	asynStatus  setHoldTime(int holdTime);

private:
	SmarActMCS2Controller *c_p_;  // pointer to asynMotorController for this axis
	asynStatus             comStatus_;
	long long              vel_;
	long long              accl_;
	int                    holdTime_;
	int                    channel_;
friend class SmarActMCS2Controller;
};

class SmarActMCS2Controller : public asynMotorController
{
public:
	SmarActMCS2Controller(const char *portName, const char *IOPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);
	virtual asynStatus sendCmd(size_t *got_p, char *rep, int len, double timeout, const char *fmt, va_list ap);
	virtual asynStatus sendCmd(size_t *got_p, char *rep, int len, double timeout, const char *fmt, ...);
	virtual asynStatus sendCmd(size_t *got_p, char *rep, int len, const char *fmt, ...);
	virtual asynStatus sendCmd(char *rep, int len, const char *fmt, ...);

protected:
	SmarActMCS2Axis **pAxes_;

private:
	asynUser *asynUserMot_p_;
friend class SmarActMCS2Axis;
};

#endif // _cplusplus
#endif // SMARACT_MCS2_MOTOR_DRIVER_H
