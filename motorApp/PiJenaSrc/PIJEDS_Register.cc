/*
FILENAME...	PIJEDS_Register.cc
USAGE...	Register piezosystem jena motor device driver shell commands.

Version:	1.1
Modified By:	sullivan
Last Modified:	2007/06/14 15:56:59
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
#include "drvPIJEDS.h"
#include "epicsExport.h"

extern "C"
{

// Pi Setup arguments
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"Polling rate", iocshArgInt};
// Pi Config arguments
static const iocshArg configArg0 = {"Card being configured", iocshArgInt};
static const iocshArg configArg1 = {"asyn port name", iocshArgString};
static const iocshArg configArg2 = {"asyn address (GPIB)", iocshArgInt};

static const iocshArg * const PIJEDSSetupArgs[2]  = {&setupArg0, &setupArg1};
static const iocshArg * const PIJEDSConfigArgs[3] = {&configArg0, &configArg1,
						     &configArg2};

static const iocshFuncDef  setupPIJEDS = {"PIJEDSSetup",  2, PIJEDSSetupArgs};
static const iocshFuncDef configPIJEDS = {"PIJEDSConfig", 3, PIJEDSConfigArgs};

static void setupPIJEDSCallFunc(const iocshArgBuf *args)
{
    PIJEDSSetup(args[0].ival, args[1].ival);
}


static void configPIJEDSCallFunc(const iocshArgBuf *args)
{
    PIJEDSConfig(args[0].ival, args[1].sval, args[2].ival);
}


static void PIJEDSmotorRegister(void)
{
    iocshRegister(&setupPIJEDS,  setupPIJEDSCallFunc);
    iocshRegister(&configPIJEDS, configPIJEDSCallFunc);
}

epicsExportRegistrar(PIJEDSmotorRegister);

} // extern "C"
