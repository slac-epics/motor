TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install) dbd files
# <name>.dbd will be created from <name>Include.dbd
#DBD += motor.dbd

#----------------------------------------------------
# Create and install (or just install)
# databases, templates, substitutions like this
DB += motor.db
DB += basic_motor.db
DB += basic_asyn_motor.db
DB += IMS_extra.db
DB += XPSAuxAi.db
DB += XPSAuxAo.db
DB += XPSAuxBi.db
DB += XPSAuxBo.db
DB += XPSAuxLi.db
DB += XPSAuxLo.db
DB += XPS_extra.db
DB += XPS_extra_axis.db
DB += XPS_extra_controller.db
DB += OmsAsyn_extra.db
DB += OmsAsyn_aux.db
DB += OmsAsyn_gpio.db
DB += ACRAux.template
DB += ACRAuxRead.template
DB += ACRAuxLi.template
DB += ACRAuxBi.template
DB += ACRAuxBo.template
DB += ACRAuxBoRBV.template
DB += TransPos.db
DB += motorUtil.db
DB += asyn_motor.db
DB += profileMoveController.template
DB += profileMoveAxis.template
DB += profileMoveControllerXPS.template
DB += profileMoveAxisXPS.template
DB += smart_extra.db
DB += pico8742Controller.db
DB += pico8742Axis.db

#----------------------------------------------------
# Declare template files which do not show up in DB
#USES_TEMPLATE += motor.template

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
