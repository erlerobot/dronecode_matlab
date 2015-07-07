# Copyright 2009-2010 The MathWorks, Inc.

AVRDUDE_PROGRAMMER = stk500v1
UPLOAD = 1


#------------------------ Arduino Tools Path ---------------------------------------
ARDUINO = $(INSTALL_DIR)/hardware/arduino/cores/arduino
AVR_TOOLS_PATH = $(INSTALL_DIR)/hardware/tools/avr/bin

FORMAT = ihex

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
DEBUG = stabs


#------------------------------ Includes -------------------------------------

# Place -I options here


INCLUDES = 

# Place -I options here
CINCS = $(INCLUDES) -I$(ARDUINO)
CXXINCS = $(INCLUDES) -I$(ARDUINO)

#-------------------------------- Flags --------------------------------------

OPT = s

# Place -D or -U options here
CDEFS = -DF_CPU=$(F_CPU)
CXXDEFS = -DF_CPU=$(F_CPU)

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu99
CDEBUG = -g$(DEBUG)
CWARN = -Wall -Wstrict-prototypes
CTUNING = -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums

CFLAGS = $(CDEBUG) $(CDEFS) $(CINCS) -O$(OPT) $(CWARN) $(CSTANDARD) $(CEXTRA)
CXXFLAGS = $(CDEFS) $(CINCS) -O$(OPT)
LDFLAGS = -lm

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS)
ALL_CXXFLAGS = -mmcu=$(MCU) -I. $(CXXFLAGS)
ALL_ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS)

#-------------------------------- Arduino Program --------------------------------------
# Define target file to be above generated code directory
PRODUCT = rtiostream_serial_test.hex

# Programming support using avrdude. Settings and variables.
AVRDUDE_PORT = $(PORT)
AVRDUDE_WRITE_FLASH = -U flash:w:$(PRODUCT)
AVRDUDE_FLAGS = -V -F -C $(INSTALL_DIR)/hardware/tools/avr/etc/avrdude.conf \
-p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER) \
-b $(UPLOAD_RATE)

# Program settings
CC = $(AVR_TOOLS_PATH)/avr-gcc
CXX = $(AVR_TOOLS_PATH)/avr-g++
OBJCOPY = $(AVR_TOOLS_PATH)/avr-objcopy
OBJDUMP = $(AVR_TOOLS_PATH)/avr-objdump
AR  = $(AVR_TOOLS_PATH)/avr-ar
SIZE = $(AVR_TOOLS_PATH)/avr-size
NM = $(AVR_TOOLS_PATH)/avr-nm
AVRDUDE = $(AVR_TOOLS_PATH)/avrdude
REMOVE = rm 
MV = mv

#-------------- Source Files, Object Files and Dependency Files -----------


# Define Source files
SRC = rtiostream_serial_test.c wiring.c wiring_digital.c pins_arduino.c
CXXSRC = rtiostream_serial.cpp HardwareSerial.cpp Print.cpp WString.cpp
# Define path to search for source files

# Define all object files.
OBJ = $(SRC:.c=.o) $(CXXSRC:.cpp=.o)

#-------------- Default target -----------
TARGETS = $(PRODUCT)
ifeq ($(UPLOAD),1)
    TARGETS += upload
endif

all: $(TARGETS)

#-------------------------- Support for building modules ----------------------
$(PRODUCT) : $(MODEL).elf
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@
	@echo ### Created $(PRODUCT)

$(MODEL).elf: core.a
	$(CC) $(ALL_CFLAGS) $^ -o $@ $(LDFLAGS)

core.a: $(OBJ)
	echo "OBJ=$(OBJ)"
	$(AR) rcs core.a $(OBJ)

%.o: %.cpp
	echo "CXXSRC=$(CXXSRC)"
	$(CXX) -c $(ALL_CXXFLAGS) $< -o $@ 

%.o: %.c
	echo "SRC=$(SRC)"
	$(CC) -c $(ALL_CFLAGS) $< -o $@ 

%.o: $(ARDUINO)/%.cpp
	echo "CXXSRC=$(CXXSRC)"
	$(CXX) -c $(ALL_CXXFLAGS) $< -o $@ 

%.o: $(ARDUINO)/%.c
	echo "SRC=$(SRC)"
	$(CC) -c $(ALL_CFLAGS) $< -o $@ 

#-------------------------- Program the device ----------------- 
upload: $(PRODUCT)
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)
	@echo ### Uploaded $(PRODUCT)

#-------------------------- clean project ----------------------
clean:
	$(REMOVE) $(MODEL).elf core.a $(PRODUCT) *.o

.PHONY:	clean upload

# EOF: serial_simple.mk




