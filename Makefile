#Makefile at top of application tree
TOP = ..
include $(TOP)/configure/CONFIG
ifneq ($(EPICS_HOST_ARCH),solaris-sparc-gnu)
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard motor*))
endif
include $(TOP)/configure/RULES_TOP
