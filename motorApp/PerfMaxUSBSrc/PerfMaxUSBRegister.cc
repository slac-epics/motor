/*
FILENAME... PerfMaxUSBRegister.cc
USAGE...    Register PerfMaxUSB MoCo dc motor controller device driver shell commands.

Version:	1.2
Modified By:	sluiter
Last Modified:	2004/07/16 19:22:58
*/

/*****************************************************************
                          COPYRIGHT NOTIFICATION
*****************************************************************

(C)  COPYRIGHT 1993 UNIVERSITY OF CHICAGO

This software was developed under a United States Government license
described on the COPYRIGHT_UniversityOfChicago file included as part
of this distribution.
**********************************************************************/

#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <iocsh.h>
#include "motor.h"
#include "drvPerfMaxUSB.h"
#include "epicsExport.h"

extern "C"
{

// PerfMaxUSBSetup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Max. motor count", iocshArgInt};
static const iocshArg setupArg2 = {"Polling rate", iocshArgInt};
// PerfMaxUSBConfig arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};
// PerfMaxUSBShow arguments
static const iocshArg showArg0 = {"PerfMaxUSB ctlr name", iocshArgString};
// PerfMaxUSBPolSet arguments
static const iocshArg polSetArg0 = {"Card being set", iocshArgInt};
static const iocshArg polSetArg1 = {"Polarity word being set", iocshArgInt};
//////
static const iocshArg * const PerfMaxUSBSetupArgs[3]  = {&setupArg0, &setupArg1,
						     &setupArg2};
static const iocshArg * const PerfMaxUSBConfigArgs[2] = {&configArg0, &configArg1};

static const iocshArg * const PerfMaxUSBShowArgs[1] = {&showArg0};
static const iocshArg * const PerfMaxUSBPolSetArgs[2]  = {&polSetArg0, &polSetArg1};

static const iocshFuncDef setupPerfMaxUSB = {"PerfMaxUSBSetup",  3, PerfMaxUSBSetupArgs};
static const iocshFuncDef configPerfMaxUSB = {"PerfMaxUSBConfig", 2, PerfMaxUSBConfigArgs};
static const iocshFuncDef showPerfMaxUSB = {"PerfMaxUSBShow", 1, PerfMaxUSBShowArgs};
static const iocshFuncDef polSetPerfMaxUSB = {"PerfMaxUSBPolSet", 2, PerfMaxUSBPolSetArgs};

static void setupPerfMaxUSBCallFunc(const iocshArgBuf *args)
{
    PerfMaxUSBSetup(args[0].ival, args[1].ival, args[2].ival);
}

static void configPerfMaxUSBCallFunc(const iocshArgBuf *args)
{
    PerfMaxUSBConfig(args[0].ival, args[1].sval);
}

static void showPerfMaxUSBCallFunc(const iocshArgBuf *args)
{
    PerfMaxUSBShow(args[0].sval);
}

static void polSetPerfMaxUSBCallFunc(const iocshArgBuf *args)
{
    PerfMaxUSBPolSet(args[0].ival, args[1].ival);
}

static void PerfMaxUSBRegister(void)
{
    iocshRegister(&showPerfMaxUSB, showPerfMaxUSBCallFunc);
    iocshRegister(&setupPerfMaxUSB, setupPerfMaxUSBCallFunc);
    iocshRegister(&configPerfMaxUSB, configPerfMaxUSBCallFunc);
    iocshRegister(&polSetPerfMaxUSB, polSetPerfMaxUSBCallFunc);
}

epicsExportRegistrar(PerfMaxUSBRegister);

} // extern "C"
