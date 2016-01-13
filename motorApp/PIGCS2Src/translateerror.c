/*
FILENAME...     translateerror.c

*************************************************************************
* Copyright (c) 2011-2013 Physik Instrumente (PI) GmbH & Co. KG
* This file is distributed subject to the EPICS Open License Agreement
* found in the file LICENSE that is included with this distribution.
*************************************************************************

Version:        $Revision: 1$
Modified By:    $Author: Steffen Rau$
Last Modified:  $Date: 23.09.2013 14:19:03$
HeadURL:        $URL$

Original Author: Steffen Rau 
*/

#include <stdio.h>
#include <string.h>

#include "picontrollererrors.h"

#ifndef BOOL
#define BOOL int
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/**
 * translate error code to c-string
 */
BOOL TranslatePIError(const int error, char* szBuffer, const int maxlen)
{
    switch (error)
    {
/*************************************************
 **
 ** Dll Errors - DLL errors occured in GCS DLL
 */
        case(PI_UNKNOWN_AXIS_IDENTIFIER): /* -1001 */
        {
            if (strlen("Unknown axis identifier")<maxlen)
            {
                sprintf(szBuffer, "Unknown axis identifier");
                return TRUE;
            }
            break;
        }
        case(PI_NR_NAV_OUT_OF_RANGE): /* -1002 */
        {
            if (strlen("Number for NAV out of range--must be in [1,10000]")<maxlen)
            {
                sprintf(szBuffer, "Number for NAV out of range--must be in [1,10000]");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_SGA): /* -1003 */
        {
            if (strlen("Invalid value for SGA--must be one of {1, 10, 100, 1000}")<maxlen)
            {
                sprintf(szBuffer, "Invalid value for SGA--must be one of {1, 10, 100, 1000}");
                return TRUE;
            }
            break;
        }
        case(PI_UNEXPECTED_RESPONSE): /* -1004 */
        {
            if (strlen("Controller sent unexpected response")<maxlen)
            {
                sprintf(szBuffer, "Controller sent unexpected response");
                return TRUE;
            }
            break;
        }
        case(PI_NO_MANUAL_PAD): /* -1005 */
        {
            if (strlen("No manual control pad installed, calls to SMA and related commands are not allowed")<maxlen)
            {
                sprintf(szBuffer, "No manual control pad installed, calls to SMA and related commands are not allowed");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_MANUAL_PAD_KNOB): /* -1006 */
        {
            if (strlen("Invalid number for manual control pad knob")<maxlen)
            {
                sprintf(szBuffer, "Invalid number for manual control pad knob");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_MANUAL_PAD_AXIS): /* -1007 */
        {
            if (strlen("Axis not currently controlled by a manual control pad")<maxlen)
            {
                sprintf(szBuffer, "Axis not currently controlled by a manual control pad");
                return TRUE;
            }
            break;
        }
        case(PI_CONTROLLER_BUSY): /* -1008 */
        {
            if (strlen("Controller is busy with some lengthy operation (e.g. reference move, fast scan algorithm)")<maxlen)
            {
                sprintf(szBuffer, "Controller is busy with some lengthy operation (e.g. reference move, fast scan algorithm)");
                return TRUE;
            }
            break;
        }
        case(PI_THREAD_ERROR): /* -1009 */
        {
            if (strlen("Internal error--could not start thread")<maxlen)
            {
                sprintf(szBuffer, "Internal error--could not start thread");
                return TRUE;
            }
            break;
        }
        case(PI_IN_MACRO_MODE): /* -1010 */
        {
            if (strlen("Controller is (already) in macro mode--command not valid in macro mode")<maxlen)
            {
                sprintf(szBuffer, "Controller is (already) in macro mode--command not valid in macro mode");
                return TRUE;
            }
            break;
        }
        case(PI_NOT_IN_MACRO_MODE): /* -1011 */
        {
            if (strlen("Controller not in macro mode--command not valid unless macro mode active")<maxlen)
            {
                sprintf(szBuffer, "Controller not in macro mode--command not valid unless macro mode active");
                return TRUE;
            }
            break;
        }
        case(PI_MACRO_FILE_ERROR): /* -1012 */
        {
            if (strlen("Could not open file to write or read macro")<maxlen)
            {
                sprintf(szBuffer, "Could not open file to write or read macro");
                return TRUE;
            }
            break;
        }
        case(PI_NO_MACRO_OR_EMPTY): /* -1013 */
        {
            if (strlen("No macro with given name on controller, or macro is empty")<maxlen)
            {
                sprintf(szBuffer, "No macro with given name on controller, or macro is empty");
                return TRUE;
            }
            break;
        }
        case(PI_MACRO_EDITOR_ERROR): /* -1014 */
        {
            if (strlen("Internal error in macro editor")<maxlen)
            {
                sprintf(szBuffer, "Internal error in macro editor");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_ARGUMENT): /* -1015 */
        {
            if (strlen("One or more arguments given to function is invalid (empty string, index out of range, ...)")<maxlen)
            {
                sprintf(szBuffer, "One or more arguments given to function is invalid (empty string, index out of range, ...)");
                return TRUE;
            }
            break;
        }
        case(PI_AXIS_ALREADY_EXISTS): /* -1016 */
        {
            if (strlen("Axis identifier is already in use by a connected stage")<maxlen)
            {
                sprintf(szBuffer, "Axis identifier is already in use by a connected stage");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_AXIS_IDENTIFIER): /* -1017 */
        {
            if (strlen("Invalid axis identifier")<maxlen)
            {
                sprintf(szBuffer, "Invalid axis identifier");
                return TRUE;
            }
            break;
        }
        case(PI_COM_ARRAY_ERROR): /* -1018 */
        {
            if (strlen("Could not access array data in COM server")<maxlen)
            {
                sprintf(szBuffer, "Could not access array data in COM server");
                return TRUE;
            }
            break;
        }
        case(PI_COM_ARRAY_RANGE_ERROR): /* -1019 */
        {
            if (strlen("Range of array does not fit the number of parameters")<maxlen)
            {
                sprintf(szBuffer, "Range of array does not fit the number of parameters");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_SPA_CMD_ID): /* -1020 */
        {
            if (strlen("Invalid parameter ID given to SPA or SPA?")<maxlen)
            {
                sprintf(szBuffer, "Invalid parameter ID given to SPA or SPA?");
                return TRUE;
            }
            break;
        }
        case(PI_NR_AVG_OUT_OF_RANGE): /* -1021 */
        {
            if (strlen("Number for AVG out of range--must be >0")<maxlen)
            {
                sprintf(szBuffer, "Number for AVG out of range--must be >0");
                return TRUE;
            }
            break;
        }
        case(PI_WAV_SAMPLES_OUT_OF_RANGE): /* -1022 */
        {
            if (strlen("Incorrect number of samples given to WAV")<maxlen)
            {
                sprintf(szBuffer, "Incorrect number of samples given to WAV");
                return TRUE;
            }
            break;
        }
        case(PI_WAV_FAILED): /* -1023 */
        {
            if (strlen("Generation of wave failed")<maxlen)
            {
                sprintf(szBuffer, "Generation of wave failed");
                return TRUE;
            }
            break;
        }
        case(PI_MOTION_ERROR): /* -1024 */
        {
            if (strlen("Motion error: position error too large, servo is switched off automatically")<maxlen)
            {
                sprintf(szBuffer, "Motion error: position error too large, servo is switched off automatically");
                return TRUE;
            }
            break;
        }
        case(PI_RUNNING_MACRO): /* -1025 */
        {
            if (strlen("Controller is (already) running a macro")<maxlen)
            {
                sprintf(szBuffer, "Controller is (already) running a macro");
                return TRUE;
            }
            break;
        }
        case(PI_PZT_CONFIG_FAILED): /* -1026 */
        {
            if (strlen("Configuration of PZT stage or amplifier failed")<maxlen)
            {
                sprintf(szBuffer, "Configuration of PZT stage or amplifier failed");
                return TRUE;
            }
            break;
        }
        case(PI_PZT_CONFIG_INVALID_PARAMS): /* -1027 */
        {
            if (strlen("Current settings are not valid for desired configuration")<maxlen)
            {
                sprintf(szBuffer, "Current settings are not valid for desired configuration");
                return TRUE;
            }
            break;
        }
        case(PI_UNKNOWN_CHANNEL_IDENTIFIER): /* -1028 */
        {
            if (strlen("Unknown channel identifier")<maxlen)
            {
                sprintf(szBuffer, "Unknown channel identifier");
                return TRUE;
            }
            break;
        }
        case(PI_WAVE_PARAM_FILE_ERROR): /* -1029 */
        {
            if (strlen("Error while reading/writing wave generator parameter file")<maxlen)
            {
                sprintf(szBuffer, "Error while reading/writing wave generator parameter file");
                return TRUE;
            }
            break;
        }
        case(PI_UNKNOWN_WAVE_SET): /* -1030 */
        {
            if (strlen("Could not find description of wave form. Maybe WG.INI is missing?")<maxlen)
            {
                sprintf(szBuffer, "Could not find description of wave form. Maybe WG.INI is missing?");
                return TRUE;
            }
            break;
        }
        case(PI_WAVE_EDITOR_FUNC_NOT_LOADED): /* -1031 */
        {
            if (strlen("The WGWaveEditor DLL function was not found at startup")<maxlen)
            {
                sprintf(szBuffer, "The WGWaveEditor DLL function was not found at startup");
                return TRUE;
            }
            break;
        }
        case(PI_USER_CANCELLED): /* -1032 */
        {
            if (strlen("The user cancelled a dialog")<maxlen)
            {
                sprintf(szBuffer, "The user cancelled a dialog");
                return TRUE;
            }
            break;
        }
        case(PI_C844_ERROR): /* -1033 */
        {
            if (strlen("Error from C-844 Controller")<maxlen)
            {
                sprintf(szBuffer, "Error from C-844 Controller");
                return TRUE;
            }
            break;
        }
        case(PI_DLL_NOT_LOADED): /* -1034 */
        {
            if (strlen("DLL necessary to call function not loaded, or function not found in DLL")<maxlen)
            {
                sprintf(szBuffer, "DLL necessary to call function not loaded, or function not found in DLL");
                return TRUE;
            }
            break;
        }
        case(PI_PARAMETER_FILE_PROTECTED): /* -1035 */
        {
            if (strlen("The open parameter file is protected and cannot be edited")<maxlen)
            {
                sprintf(szBuffer, "The open parameter file is protected and cannot be edited");
                return TRUE;
            }
            break;
        }
        case(PI_NO_PARAMETER_FILE_OPENED): /* -1036 */
        {
            if (strlen("There is no parameter file open")<maxlen)
            {
                sprintf(szBuffer, "There is no parameter file open");
                return TRUE;
            }
            break;
        }
        case(PI_STAGE_DOES_NOT_EXIST): /* -1037 */
        {
            if (strlen("Selected stage does not exist")<maxlen)
            {
                sprintf(szBuffer, "Selected stage does not exist");
                return TRUE;
            }
            break;
        }
        case(PI_PARAMETER_FILE_ALREADY_OPENED): /* -1038 */
        {
            if (strlen("There is already a parameter file open. Close it before opening a new file")<maxlen)
            {
                sprintf(szBuffer, "There is already a parameter file open. Close it before opening a new file");
                return TRUE;
            }
            break;
        }
        case(PI_PARAMETER_FILE_OPEN_ERROR): /* -1039 */
        {
            if (strlen("Could not open parameter file")<maxlen)
            {
                sprintf(szBuffer, "Could not open parameter file");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_CONTROLLER_VERSION): /* -1040 */
        {
            if (strlen("The version of the connected controller is invalid")<maxlen)
            {
                sprintf(szBuffer, "The version of the connected controller is invalid");
                return TRUE;
            }
            break;
        }
        case(PI_PARAM_SET_ERROR): /* -1041 */
        {
            if (strlen("Parameter could not be set with SPA--parameter not defined for this controller!")<maxlen)
            {
                sprintf(szBuffer, "Parameter could not be set with SPA--parameter not defined for this controller!");
                return TRUE;
            }
            break;
        }
        case(PI_NUMBER_OF_POSSIBLE_WAVES_EXCEEDED): /* -1042 */
        {
            if (strlen("The maximum number of wave definitions has been exceeded")<maxlen)
            {
                sprintf(szBuffer, "The maximum number of wave definitions has been exceeded");
                return TRUE;
            }
            break;
        }
        case(PI_NUMBER_OF_POSSIBLE_GENERATORS_EXCEEDED): /* -1043 */
        {
            if (strlen("The maximum number of wave generators has been exceeded")<maxlen)
            {
                sprintf(szBuffer, "The maximum number of wave generators has been exceeded");
                return TRUE;
            }
            break;
        }
        case(PI_NO_WAVE_FOR_AXIS_DEFINED): /* -1044 */
        {
            if (strlen("No wave defined for specified axis")<maxlen)
            {
                sprintf(szBuffer, "No wave defined for specified axis");
                return TRUE;
            }
            break;
        }
        case(PI_CANT_STOP_OR_START_WAV): /* -1045 */
        {
            if (strlen("Wave output to axis already stopped/started")<maxlen)
            {
                sprintf(szBuffer, "Wave output to axis already stopped/started");
                return TRUE;
            }
            break;
        }
        case(PI_REFERENCE_ERROR): /* -1046 */
        {
            if (strlen("Not all axes could be referenced")<maxlen)
            {
                sprintf(szBuffer, "Not all axes could be referenced");
                return TRUE;
            }
            break;
        }
        case(PI_REQUIRED_WAVE_NOT_FOUND): /* -1047 */
        {
            if (strlen("Could not find parameter set required by frequency relation")<maxlen)
            {
                sprintf(szBuffer, "Could not find parameter set required by frequency relation");
                return TRUE;
            }
            break;
        }
        case(PI_INVALID_SPP_CMD_ID): /* -1048 */
        {
            if (strlen("Command ID given to SPP or SPP? is not valid")<maxlen)
            {
                sprintf(szBuffer, "Command ID given to SPP or SPP? is not valid");
                return TRUE;
            }
            break;
        }
        case(PI_STAGE_NAME_ISNT_UNIQUE): /* -1049 */
        {
            if (strlen("A st