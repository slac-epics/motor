/*
FILENAME...	ImsRegister.cc
USAGE...	Register IMS motor device driver shell commands.

Version:	1.3
Modified By:	sluiter
Last Modified:	2004/07/16 19:10:15
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
#include "drvIM483.h"
#include "epicsExport.h"

extern "C"
{

// Ims Setup arguments
static const iocshArg setupArg0  = {"Max. controller count", iocshArgInt};
// Ims Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"Asyn port name",        iocshArgString};
static const iocshArg configArg2 = {"Polling rate",          iocshArgInt   };

static const iocshArg * const IM483SetupArgs [1] = {&setupArg0};
static const iocshArg * const IM483ConfigArgs[3] = {&configArg0, &configArg1, &configArg2};

static const iocshFuncDef setupMDrivePlus  = {"MDrivePlusSetup",  1, IM483SetupArgs };
static const iocshFuncDef configMDrivePlus = {"MDrivePlusConfig", 3, IM483ConfigArgs};

static void setupMDrivePlusCallFunc(const iocshArgBuf *args)
{
    MDrivePlusSetup (args[0].ival);
}

static void configMDrivePlusCallFunc(const iocshArgBuf *args)
{
    MDrivePlusConfig(args[0].ival, args[1].sval, args[2].ival);
}

static void IMSmotorRegister(void)
{
    iocshRegister(&setupMDrivePlus,  setupMDrivePlusCallFunc );
    iocshRegister(&configMDrivePlus, configMDrivePlusCallFunc);
}

epicsExportRegistrar(IMSmotorRegister);

} // extern "C"

