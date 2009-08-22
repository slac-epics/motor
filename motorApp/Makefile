TOP = ..
include $(TOP)/configure/CONFIG

# MotorSrc is required for all motor record configurations.  It MUST appear
# 1st in this list of directories.

# "#!" marks lines that can be uncommented.

DIRS += MotorSrc

# Select individual device/driver modules by removing the "#!" comment.
#!DIRS += DeltaTauSrc
#!DIRS += OmsSrc
#!DIRS += SoftMotorSrc
#!DIRS += MotorSimSrc

# All the following modules require ASYN.
ifdef ASYN
#!DIRS += NewportSrc
DIRS += ImsSrc
#!DIRS += AcsSrc
#!DIRS += MclennanSrc
#!DIRS += PiSrc
#!DIRS += MicroMoSrc
#!DIRS += MicosSrc
#!DIRS += FaulhaberSrc
#!DIRS += PC6KSrc
#!DIRS += NewFocusSrc
#!DIRS += AcsTech80Src
#!DIRS += OrielSrc
#!DIRS += ThorLabsSrc
DIRS += SmartMotorSrc
#!DIRS += PiJenaSrc
#!DIRS += KohzuSrc
endif

# Install the edl files
#DIRS += opi

ifdef MX
DIRS += MXmotorSrc
endif

# Make driver modules depend on MotorSrc
DeltaTauSrc_DEPEND_DIRS		= MotorSrc
OmsSrc_DEPEND_DIRS			= MotorSrc
SoftMotorSrc_DEPEND_DIRS	= MotorSrc
MotorSimSrc_DEPEND_DIRS		= MotorSrc
NewportSrc_DEPEND_DIRS		= MotorSrc
ImsSrc_DEPEND_DIRS			= MotorSrc
AcsSrc_DEPEND_DIRS			= MotorSrc
MclennanSrc_DEPEND_DIRS		= MotorSrc
PiSrc_DEPEND_DIRS			= MotorSrc
MicroMoSrc_DEPEND_DIRS		= MotorSrc
MicosSrc_DEPEND_DIRS		= MotorSrc
FaulhaberSrc_DEPEND_DIRS	= MotorSrc
PC6KSrc_DEPEND_DIRS			= MotorSrc
NewFocusSrc_DEPEND_DIRS		= MotorSrc
AcsTech80Src_DEPEND_DIRS	= MotorSrc
OrielSrc_DEPEND_DIRS		= MotorSrc
ThorLabsSrc_DEPEND_DIRS		= MotorSrc
SmartMotorSrc_DEPEND_DIRS	= MotorSrc
PiJenaSrc_DEPEND_DIRS		= MotorSrc
KohzuSrc_DEPEND_DIRS		= MotorSrc
MXmotorSrc_DEPEND_DIRS		= MotorSrc

DIRS += Db
include $(TOP)/configure/RULES_DIRS
