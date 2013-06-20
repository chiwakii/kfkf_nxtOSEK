# Target specific macros
TARGET = KFKF_MODEL

#NXT_OSEK_ROOT = /cygwin/nxtOSEK
NXT_OSEK_ROOT = /cygdrive/c/cygwin/nxtOSEK

# nxtway_gs_balancer library designed for NXTway-GS two wheeled self-balancing robot
USER_INC_PATH = $(NXT_OSEK_ROOT)/ecrobot/nxtway_gs_balancer
USER_LIB = nxtway_gs_balancer

# using NXT standard tires (not Motorcycle tires)
#USER_DEF = NXT_STD_TIRE


# User applicaion source
TARGET_SOURCES := \
	Main.c \
	balancer_param.c \
	kfkf/kfkfModel.c
	

# OSEK OIL file
TOPPERS_OSEK_OIL_SOURCE = Task.oil

# Don't modify below part
O_PATH ?= build
include /cygdrive/c/cygwin/nxtOSEK/ecrobot/ecrobot.mak


