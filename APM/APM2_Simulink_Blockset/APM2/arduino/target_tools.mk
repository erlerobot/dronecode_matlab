#
# Copyright 2011 The MathWorks, Inc.
#
include $(START_DIR)/slprj/arduino_prefs.mk

LOCAL_AVR_TOOLS_PATH = $(ARDUINO_ROOT)/hardware/tools/avr/bin/

LOCAL_OPT = -Os -DARDUINO=100 -DUSE_FAST_SERIAL
LOCAL_CDEFS = -DF_CPU=$(F_CPU)
LOCAL_CXXDEFS = -DF_CPU=$(F_CPU)
LOCAL_CSTANDARD = -std=gnu99
LOCAL_CWARN = -Wall -Wstrict-prototypes
LOCAL_MCU_OPT = -mmcu=$(MCU)
CDEBUG = -gstabs

# Compiler command and options
CC                  = $(LOCAL_AVR_TOOLS_PATH)/avr-gcc
CFLAGS = $(LOCAL_MCU_OPT) -I. $(LOCAL_CDEFS) 
CFLAGS += $(LOCAL_OPT) $(LOCAL_CWARN) $(LOCAL_CSTANDARD)

# Specify the output extension from compiler
CCOUTPUTFLAG        = -o
OBJ_EXT             = .o

CXX                 = $(LOCAL_AVR_TOOLS_PATH)/avr-g++
CXXFLAGS            = $(LOCAL_MCU_OPT) -I. $(LOCAL_CXXDEFS) $(LOCAL_OPT) 

# Linker command and options
LD                  = $(LOCAL_AVR_TOOLS_PATH)/avr-gcc
LDFLAGS = $(LOCAL_MCU_OPT) $(LOCAL_CDEFS) $(LOCAL_OPT) $(LOCAL_CWARN) $(LOCAL_CSTANDARD) -lm -Wl,-Map,mapFile.map,--cref

# Specify extension from linker
LDOUTPUTFLAG        = -o
PROGRAM_FILE_EXT    = .elf

# Archiver command and options
AR                  = $(LOCAL_AVR_TOOLS_PATH)/avr-ar
ARFLAGS             = rcs

# Binary file format converter command and options
OBJCOPY             = $(LOCAL_AVR_TOOLS_PATH)/avr-objcopy
OBJCOPYFLAGS        = -O ihex -R .eeprom
BINARY_FILE_EXT     = .hex

# Specify extension for final product at end of build
EXE_FILE_EXT = $(BINARY_FILE_EXT)

TARGET_INC_DIR      = $(ARDUINO_ROOT)/hardware/arduino/cores/arduino
TARGET_INCS         = -I$(TARGET_INC_DIR)
TARGET_SRC_DIR      = $(ARDUINO_ROOT)/hardware/arduino/cores/arduino
LOCAL_ARDUINO_SRCS  = $(notdir $(wildcard $(TARGET_SRC_DIR)/*.cpp)) $(notdir $(wildcard $(TARGET_SRC_DIR)/*.c))
TARGET_SRCS         = $(filter-out main.cpp,$(LOCAL_ARDUINO_SRCS))

