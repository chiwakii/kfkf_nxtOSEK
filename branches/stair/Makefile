# nxtOSEKルートディレクトリ
NXT_OSEK_ROOT = /cygwin/nxtOSEK

# ターゲット実行形式ファイル名
TARGET = balance_linetrace

# インクルードパス
USER_INC_PATH= $(NXTOSEK_ROOT)/ecrobot/nxtway_gs_balancer

# ライブラリ
USER_LIB = nxtway_gs_balancer

# Cソースファイル
TARGET_SOURCES = \
	balance_linetrace.c \
	balancer_param.c

# TOPPERS/ATK(OSEK)設定ファイル
TOPPERS_OSEK_OIL_SOURCE =./balance_linetrace.oil

# 下記のマクロは変更しないでください
O_PATH ?= build

include ../../../ecrobot/ecrobot.mak
