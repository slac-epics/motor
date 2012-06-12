# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG

CROSS_COMPILER_TARGET_ARCHS=linux-x86

# Both the following line, and a line in the *.dbd file,
# must be uncommented to use diagnostic debugging messages.
#!USR_CXXFLAGS += -DDEBUG
USR_CXXFLAGS += `pkg-config --cflags libusb`
#USR_LDFLAGS += `pkg-config --libs libusb-1.0`
USR_LDFLAGS += `pkg-config --libs libusb`
#USR_LDFLAGS += -L /usr/local/lib -l ArcusPerformaxDriver

DBD += devPerfMaxUSB.dbd

LIBRARY_IOC = PerfMaxUSB

# Intelligent Motion Systems driver support.
SRCS += PerfMaxUSBRegister.cc
SRCS += devPerfMaxUSB.cc drvPerfMaxUSB.cc

#PerfMaxUSB_LIBS += motor asyn
PerfMaxUSB_LIBS += motor 
PerfMaxUSB_LIBS += $(EPICS_BASE_IOC_LIBS)
PerfMaxUSB_LIBS += ArcusPerformaxDriver

include $(TOP)/configure/RULES

