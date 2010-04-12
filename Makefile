#Makefile at top of application tree
TOP = ..
include $(TOP)/configure/CONFIG
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard motor*))
include $(TOP)/configure/RULES_TOP
