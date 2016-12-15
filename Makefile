# Target specific macros
TARGET = StaticDynamic
TARGET_SOURCES = \
	src.c
TOPPERS_OSEK_OIL_SOURCE = ./src.oil

# Don't modify below part
O_PATH ?= build
include C:/nxtOSEK/ecrobot/ecrobot.mak
