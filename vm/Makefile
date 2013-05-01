# Target specific macros
TARGET = TASK_CYCLE

NXT_OSEK_ROOT = /cygwin/nxtOSEK

# nxtway_gs_balancer library designed for NXTway-GS two wheeled self-balancing robot
USER_INC_PATH = $(NXTOSEK_ROOT)/ecrobot/nxtway_gs_balancer
USER_LIB = nxtway_gs_balancer

# using NXT standard tires (not Motorcycle tires)
#USER_DEF = NXT_STD_TIRE

# User applicaion source
TARGET_SOURCES = \
	task_cycle.c \
	statics/balancer_param.c \
	statics/StateMachine.c

# OSEK OIL file
TOPPERS_OSEK_OIL_SOURCE = ./task_cycle.oil

# Don't modify below part
O_PATH ?= build
include /cygdrive/c/cygwin/nxtOSEK/ecrobot/ecrobot.mak
