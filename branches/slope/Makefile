# Target specific macros
TARGET = SLOPE

NXT_OSEK_ROOT = ../../../../nxtOSEK

# nxtway_gs_balancer library designed for NXTway-GS two wheeled self-balancing robot
USER_INC_PATH = $(NXTOSEK_ROOT)/ecrobot/nxtway_gs_balancer
USER_LIB = nxtway_gs_balancer

# using NXT standard tires (not Motorcycle tires)
#USER_DEF = NXT_STD_TIRE

# User applicaion source
TARGET_SOURCES = \
	slope.c \
	balancer_param.c

# OSEK OIL file
TOPPERS_OSEK_OIL_SOURCE = ./slope.oil

# Don't modify below part
O_PATH ?= build
include ../../../../nxtOSEK/ecrobot/ecrobot.mak
