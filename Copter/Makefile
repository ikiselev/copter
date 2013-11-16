#
# PATHS
#
ARDUINO_VERSION = 105
ARDUINO_PATHER   = /Users/$(USER)/Dropbox/Arduino
FRAMEWORK_PATH   = $(ARDUINO_PATHER)/Framework

ARDUINO_CORE_PATH    := $(FRAMEWORK_PATH)/cores/arduino
ARDUINO_LIB_PATH     := $(FRAMEWORK_PATH)/Libraries
BOARDS_TXT       := $(FRAMEWORK_PATH)/boards.txt

USER_LIB_PATH  = $(wildcard $(ARDUINO_PATHER)/?ibraries)

TOOLCHAIN_BIN_DIRECTORY = /usr/local/CrossPack-AVR/bin

#
# CONSOLE OVERRIDE
#
BOARD_TAG ?= nano328
BOARD_TAG := $(strip $(BOARD_TAG))
SKETCH_EXTENSION = ino

# - ispload
# - raw_upload
UPLOAD_TYPE = raw_upload

#arduino - usb-upload with bootloader
PROGRAMMER = arduino
ARDUINO_PORT = /dev/tty.usbserial*

ISP_PROG = usbasp


#
# VARIABLES
#
TARGET := sketch
OBJDIR = ../build/Makefile


# Function PARSE_BOARD data retrieval from boards.txt
# result = $(call PARSE_BOARD 'boardname','parameter')
PARSE_BOARD = $(shell if [ -f $(BOARDS_TXT) ]; then grep ^$(1).$(2)= $(BOARDS_TXT) | cut -d = -f 2; fi; )


########################################################################
# boards.txt parsing


VARIANT      = $(call PARSE_BOARD,$(BOARD_TAG),build.variant)
VARIANT_PATH = $(FRAMEWORK_PATH)/variants/$(VARIANT)

MCU   = $(call PARSE_BOARD,$(BOARD_TAG),build.mcu)
F_CPU = $(call PARSE_BOARD,$(BOARD_TAG),build.f_cpu)


AVRDUDE_UPLOAD_PROTOCOL = $(call PARSE_BOARD,$(BOARD_TAG),upload.protocol)
AVRDUDE_ARD_BAUDRATE = $(call PARSE_BOARD,$(BOARD_TAG),upload.speed)

ISP_LOCK_FUSE_PRE = $(call PARSE_BOARD,$(BOARD_TAG),bootloader.unlock_bits)
ISP_HIGH_FUSE = $(call PARSE_BOARD,$(BOARD_TAG),bootloader.high_fuses)
ISP_LOW_FUSE = $(call PARSE_BOARD,$(BOARD_TAG),bootloader.low_fuses)
ISP_EXT_FUSE = $(call PARSE_BOARD,$(BOARD_TAG),bootloader.extended_fuses)
ISP_LOCK_FUSE_POST = $(call PARSE_BOARD,$(BOARD_TAG),bootloader.lock_bits)

BOOTLOADER_PATH = $(call PARSE_BOARD,$(BOARD_TAG),bootloader.path)
BOOTLOADER_FILE = $(call PARSE_BOARD,$(BOARD_TAG),bootloader.file)

HEX_MAXIMUM_SIZE  = $(call PARSE_BOARD,$(BOARD_TAG),upload.maximum_size)




OBJDIR = ../build/build-$(BOARD_TAG)


########################################################################
# Reset

ifndef RESET_CMD
       RESET_CMD = $(ARDMK_PATH)/ard-reset-arduino \
          $(ARD_RESET_OPTS) $(call get_monitor_port)
endif

########################################################################
# Local sources

LOCAL_C_SRCS    ?= $(wildcard *.c)
LOCAL_CPP_SRCS  ?= $(wildcard *.cpp)
LOCAL_CC_SRCS   ?= $(wildcard *.cc)
LOCAL_PDE_SRCS  ?= $(wildcard *.pde)
LOCAL_INO_SRCS  ?= $(wildcard *.ino)

LOCAL_SRCS      = $(LOCAL_C_SRCS)   $(LOCAL_CPP_SRCS) \
                $(LOCAL_CC_SRCS)   $(LOCAL_PDE_SRCS) \
                $(LOCAL_INO_SRCS)
LOCAL_OBJ_FILES = $(LOCAL_C_SRCS:.c=.o)   $(LOCAL_CPP_SRCS:.cpp=.o) \
                $(LOCAL_CC_SRCS:.cc=.o)   $(LOCAL_PDE_SRCS:.pde=.o) \
                $(LOCAL_INO_SRCS:.ino=.o)
LOCAL_OBJS      = $(patsubst %,$(OBJDIR)/%,$(LOCAL_OBJ_FILES))

ifeq ($(words $(LOCAL_SRCS)), 0)
    $(error Atleast one source file (*.ino, *.pde, *.cpp, *c, *cc, *.S) is needed)
endif


# Ideally, this should just check if there are more than one file
ifneq ($(words $(LOCAL_PDE_SRCS) $(LOCAL_INO_SRCS)), 1)
    ifeq ($(words $(LOCAL_PDE_SRCS) $(LOCAL_INO_SRCS)), 0)
        $(call show_config_info,No .pde or .ino files found. If you are compiling .c or .cpp files then you need to explicitly include Arduino header files)
    else
        #TODO: Support more than one file. https://github.com/sudar/Arduino-Makefile/issues/49
        $(error Need exactly one .pde or .ino file. This makefile doesn't support multiple .ino/.pde files yet)
    endif
endif


# core sources

CORE_C_SRCS     = $(wildcard $(ARDUINO_CORE_PATH)/*.c)
CORE_CPP_SRCS   = $(wildcard $(ARDUINO_CORE_PATH)/*.cpp)

CORE_OBJ_FILES  = $(CORE_C_SRCS:.c=.o) $(CORE_CPP_SRCS:.cpp=.o)
CORE_OBJS       = $(patsubst $(ARDUINO_CORE_PATH)/%, $(OBJDIR)/%,$(CORE_OBJ_FILES))



########################################################################
# Determine ARDUINO_LIBS automatically

ifndef ARDUINO_LIBS
    # automatically determine included libraries
    ARDUINO_LIBS += $(filter $(notdir $(wildcard $(ARDUINO_LIB_PATH)/*)), \
        $(shell sed -ne "s/^ *\# *include *[<\"]\(.*\)\.h[>\"]/\1/p" $(LOCAL_SRCS)))
    ARDUINO_LIBS += $(filter $(notdir $(wildcard $(USER_LIB_PATH)/*)), \
        $(shell sed -ne "s/^ *\# *include *[<\"]\(.*\)\.h[>\"]/\1/p" $(LOCAL_SRCS)))
endif

########################################################################

ifeq ($(strip $(NO_CORE)),)
    ifndef MONITOR_BAUDRATE
        ifeq ($(words $(LOCAL_PDE_SRCS) $(LOCAL_INO_SRCS)), 1)
            SPEED = $(shell egrep -h 'Serial.begin\([0-9]+\)' $(LOCAL_PDE_SRCS) $(LOCAL_INO_SRCS) | sed -e 's/[^0-9]//g'| head -n1)
            MONITOR_BAUDRATE = $(findstring $(SPEED),300 1200 2400 4800 9600 14400 19200 28800 38400 57600 115200)
        endif

        ifeq ($(MONITOR_BAUDRATE),)
            MONITOR_BAUDRATE = 9600
            $(call show_config_variable,MONITOR_BAUDRATE,[ASSUMED])
        else
            $(call show_config_variable,MONITOR_BAUDRATE,[DETECTED], (in sketch))
        endif
    else
        $(call show_config_variable,MONITOR_BAUDRATE, [USER])
    endif

    ifndef MONITOR_CMD
        MONITOR_CMD = screen
    endif
endif

########################################################################
# Include file to use for old .pde files

ARDUINO_HEADER=Arduino.h

########################################################################
# Rules for making stuff

# The name of the main targets
TARGET_HEX = $(OBJDIR)/$(TARGET).hex
TARGET_ELF = $(OBJDIR)/$(TARGET).elf
TARGET_EEP = $(OBJDIR)/$(TARGET).eep
TARGETS    = $(OBJDIR)/$(TARGET).*
CORE_LIB   = $(OBJDIR)/libcore.a





# Names of executables
CC      = $(TOOLCHAIN_BIN_DIRECTORY)/avr-gcc
CXX     = $(TOOLCHAIN_BIN_DIRECTORY)/avr-g++
OBJCOPY = $(TOOLCHAIN_BIN_DIRECTORY)/avr-objcopy
OBJDUMP = $(TOOLCHAIN_BIN_DIRECTORY)/avr-objdump
AR      = $(TOOLCHAIN_BIN_DIRECTORY)/avr-ar
SIZE    = $(TOOLCHAIN_BIN_DIRECTORY)/avr-size
NM      = $(TOOLCHAIN_BIN_DIRECTORY)/avr-nm

REMOVE  = rm -rf
MV      = mv -f
CAT     = cat
ECHO    = echo
MKDIR   = mkdir -p

# General arguments
USER_LIBS_ROOT = $(wildcard $(patsubst %,$(USER_LIB_PATH)/%,$(ARDUINO_LIBS)))


USER_LIBS      = $(realpath $(sort $(dir $(foreach dir,$(USER_LIBS_ROOT),$(wildcard $(dir)/*.h $(dir)/*/*.h $(dir)/*/*/*.h )))))
USER_LIB_NAMES = $(patsubst $(USER_LIB_PATH)/%,%,$(USER_LIBS_ROOT))


# Let user libraries override system ones.
SYS_LIBS       = $(wildcard $(patsubst %,$(ARDUINO_LIB_PATH)/%,$(filter-out $(USER_LIB_NAMES),$(ARDUINO_LIBS))))
SYS_LIB_NAMES  = $(patsubst $(ARDUINO_LIB_PATH)/%,%,$(SYS_LIBS))

# Error here if any are missing.
LIBS_NOT_FOUND = $(filter-out $(USER_LIB_NAMES) $(SYS_LIB_NAMES),$(ARDUINO_LIBS))
ifneq (,$(strip $(LIBS_NOT_FOUND)))
    $(error The following libraries specified in ARDUINO_LIBS could not be found (searched USER_LIB_PATH and ARDUINO_LIB_PATH): $(LIBS_NOT_FOUND))
endif

SYS_LIBS           := $(wildcard $(SYS_LIBS) $(addsuffix /utility,$(SYS_LIBS)))
USER_LIBS          := $(wildcard $(USER_LIBS) $(addsuffix /utility,$(USER_LIBS)))
SYS_INCLUDES        = $(patsubst %,-I%,$(SYS_LIBS))
USER_INCLUDES       = $(patsubst %,-I%,$(USER_LIBS))
LIB_C_SRCS          = $(wildcard $(patsubst %,%/*.c,$(SYS_LIBS)))
LIB_CPP_SRCS        = $(wildcard $(patsubst %,%/*.cpp,$(SYS_LIBS)))
USER_LIB_CPP_SRCS   = $(wildcard $(patsubst %,%/*.cpp,$(USER_LIBS)))
USER_LIB_C_SRCS     = $(wildcard $(patsubst %,%/*.c,$(USER_LIBS)))
LIB_OBJS            = $(patsubst $(ARDUINO_LIB_PATH)/%.c,$(OBJDIR)/libs/%.o,$(LIB_C_SRCS)) \
                      $(patsubst $(ARDUINO_LIB_PATH)/%.cpp,$(OBJDIR)/libs/%.o,$(LIB_CPP_SRCS))
USER_LIB_OBJS       = $(patsubst $(USER_LIB_PATH)/%.cpp,$(OBJDIR)/libs/%.o,$(USER_LIB_CPP_SRCS)) \
                      $(patsubst $(USER_LIB_PATH)/%.c,$(OBJDIR)/libs/%.o,$(USER_LIB_C_SRCS))

# Dependency files
DEPS                = $(LOCAL_OBJS:.o=.d) $(LIB_OBJS:.o=.d) $(USER_LIB_OBJS:.o=.d) $(CORE_OBJS:.o=.d)

# Optimization level for the compiler.
# You can get the list of options at http://www.nongnu.org/avr-libc/user-manual/using_tools.html#gcc_optO
# Also read http://www.nongnu.org/avr-libc/user-manual/FAQ.html#faq_optflags
OPTIMIZATION_LEVEL=s

MCU_FLAG_NAME = mmcu

# Using += instead of =, so that CPPFLAGS can be set per sketch level
CPPFLAGS      += -$(MCU_FLAG_NAME)=$(MCU) -DF_CPU=$(F_CPU) -DARDUINO=$(ARDUINO_VERSION) \
        -I. -I$(ARDUINO_CORE_PATH) -I$(VARIANT_PATH) \
        $(SYS_INCLUDES) $(USER_INCLUDES) -w -Wall -ffunction-sections -fdata-sections


ifdef DEBUG
CPPFLAGS += -O0 -g
else
CPPFLAGS += -O$(OPTIMIZATION_LEVEL)
endif


ifndef CFLAGS_STD
    CFLAGS_STD        = -std=gnu99
endif

CFLAGS        += $(EXTRA_FLAGS) $(EXTRA_CFLAGS)
CXXFLAGS      += -fno-exceptions $(EXTRA_FLAGS) $(EXTRA_CXXFLAGS)
LDFLAGS       += -$(MCU_FLAG_NAME)=$(MCU) -Wl,--gc-sections -O$(OPTIMIZATION_LEVEL) $(EXTRA_FLAGS) $(EXTRA_CXXFLAGS) $(EXTRA_LDFLAGS)
SIZEFLAGS     ?= --mcu=$(MCU) -C

# for backwards compatibility, grab ARDUINO_PORT if the user has it set
MONITOR_PORT ?= $(ARDUINO_PORT)

# Returns the Arduino port (first wildcard expansion) if it exists, otherwise it errors.
get_monitor_port = $(if $(wildcard $(MONITOR_PORT)),$(firstword $(wildcard $(MONITOR_PORT))),$(error Arduino port $(MONITOR_PORT) not found!))

# Returns the ISP port (first wildcard expansion) if it exists, otherwise it errors.
get_isp_port = $(if $(wildcard $(ISP_PORT)),$(firstword $(wildcard $(ISP_PORT))),$(error ISP port $(ISP_PORT) not found!))

# Command for avr_size: do $(call avr_size,elffile,hexfile)
ifneq (,$(findstring AVR,$(shell $(SIZE) --help)))
    # We have a patched version of binutils that mentions AVR - pass the MCU
    # and the elf to get nice output.
    avr_size = $(SIZE) $(SIZEFLAGS) --format=avr $(1)
    $(call show_config_info,Size utility: AVR-aware for enhanced output,[AUTODETECTED])
else
    # We have a plain-old binutils version - just give it the hex.
    avr_size = $(SIZE) $(2)
    $(call show_config_info,Size utility: Basic (not AVR-aware),[AUTODETECTED])
endif


# Implicit rules for building everything (needed to get everything in
# the right directory)
#
# Rather than mess around with VPATH there are quasi-duplicate rules
# here for building e.g. a system C++ file and a local C++
# file. Besides making things simpler now, this would also make it
# easy to change the build options in future

# library sources
$(OBJDIR)/libs/%.o: $(ARDUINO_LIB_PATH)/%.c
		@$(MKDIR) $(dir $@)
		$(CC) -MMD -c $(CPPFLAGS) $(CFLAGS) $< -o $@

$(OBJDIR)/libs/%.o: $(ARDUINO_LIB_PATH)/%.cpp
		@$(MKDIR) $(dir $@)
		$(CC) -MMD -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

$(OBJDIR)/libs/%.o: $(USER_LIB_PATH)/%.cpp
		@$(MKDIR) $(dir $@)
		$(CC) -MMD -c $(CPPFLAGS) $(CFLAGS) $< -o $@

$(OBJDIR)/libs/%.o: $(USER_LIB_PATH)/%.c
		@$(MKDIR) $(dir $@)
		$(CC) -MMD -c $(CPPFLAGS) $(CFLAGS) $< -o $@

ifdef COMMON_DEPS
	COMMON_DEPS := $(COMMON_DEPS) Makefile
else
	COMMON_DEPS := Makefile
endif

# normal local sources
$(OBJDIR)/%.o: %.c $(COMMON_DEPS) | $(OBJDIR)
		@$(MKDIR) $(dir $@)
		$(CC) -MMD -c $(CPPFLAGS) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o: %.cc $(COMMON_DEPS) | $(OBJDIR)
		@$(MKDIR) $(dir $@)
		$(CXX) -MMD -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

$(OBJDIR)/%.o: %.cpp $(COMMON_DEPS) | $(OBJDIR)
		@$(MKDIR) $(dir $@)
		$(CXX) -MMD -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

# the pde -> o file
$(OBJDIR)/%.o: %.pde $(COMMON_DEPS) | $(OBJDIR)
		@$(MKDIR) $(dir $@)
		$(CXX) -x c++ -include $(ARDUINO_HEADER) -MMD -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

# the ino -> o file
$(OBJDIR)/%.o: %.ino $(COMMON_DEPS) | $(OBJDIR)
		@$(MKDIR) $(dir $@)
		$(CXX) -x c++ -include Arduino.h -MMD -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

# generated assembly
$(OBJDIR)/%.s: %.pde $(COMMON_DEPS) | $(OBJDIR)
		@$(MKDIR) $(dir $@)
		$(CXX) -x c++ -include $(ARDUINO_HEADER) -MMD -S -fverbose-asm $(CPPFLAGS) $(CXXFLAGS) $< -o $@

$(OBJDIR)/%.s: %.ino $(COMMON_DEPS) | $(OBJDIR)
		@$(MKDIR) $(dir $@)
		$(CXX) -x c++ -include Arduino.h -MMD -S -fverbose-asm $(CPPFLAGS) $(CXXFLAGS) $< -o $@


# core files
$(OBJDIR)/%.o: $(ARDUINO_CORE_PATH)/%.c $(COMMON_DEPS) | $(OBJDIR)
		@$(MKDIR) $(dir $@)
		$(CC) -MMD -c $(CPPFLAGS) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o: $(ARDUINO_CORE_PATH)/%.cpp $(COMMON_DEPS) | $(OBJDIR)
		@$(MKDIR) $(dir $@)
		$(CXX) -MMD -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

# various object conversions
$(OBJDIR)/%.hex: $(OBJDIR)/%.elf $(COMMON_DEPS)
		@$(MKDIR) $(dir $@)
		$(OBJCOPY) -O ihex -R .eeprom $< $@
		@$(ECHO)
		$(call avr_size,$<,$@)
ifneq ($(strip $(HEX_MAXIMUM_SIZE)),)
		@if [ `$(SIZE) $@ | awk 'FNR == 2 {print $$2}'` -le $(HEX_MAXIMUM_SIZE) ]; then touch $@.sizeok; fi
else
		@$(ECHO) Maximum flash memory of $(BOARD_TAG) is not specified. Make sure the size of $@ is less than $(BOARD_TAG)\'s flash memory
		@touch $@.sizeok
endif

$(OBJDIR)/%.eep: $(OBJDIR)/%.elf $(COMMON_DEPS)
		@$(MKDIR) $(dir $@)
		-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
				--change-section-lma .eeprom=0 -O ihex $< $@

$(OBJDIR)/%.lss: $(OBJDIR)/%.elf $(COMMON_DEPS)
		@$(MKDIR) $(dir $@)
		$(OBJDUMP) -h --source --demangle --wide $< > $@

$(OBJDIR)/%.sym: $(OBJDIR)/%.elf $(COMMON_DEPS)
		@$(MKDIR) $(dir $@)
		$(NM) --size-sort --demangle --reverse-sort --line-numbers $< > $@

########################################################################
# Avrdude

AVRDUDE = $(TOOLCHAIN_BIN_DIRECTORY)/avrdude

# avrdude options
# http://www.nongnu.org/avrdude/user-manual/avrdude_4.html

AVRDUDE_OPTS = -V


AVRDUDE_COM_OPTS = $(AVRDUDE_OPTS) -p $(MCU)

AVRDUDE_ARD_OPTS = -c $(PROGRAMMER) -b $(AVRDUDE_ARD_BAUDRATE) -P $(call get_monitor_port)



AVRDUDE_ISP_BAUDRATE = $(AVRDUDE_ARD_BAUDRATE)

# Fuse settings copied from Arduino IDE.
# https://github.com/arduino/Arduino/blob/master/app/src/processing/app/debug/AvrdudeUploader.java#L254

# Pre fuse settings
ifndef AVRDUDE_ISP_FUSES_PRE
    ifneq ($(strip $(ISP_LOCK_FUSE_PRE)),)
        AVRDUDE_ISP_FUSES_PRE += -U lock:w:$(ISP_LOCK_FUSE_PRE):m
    endif

    ifneq ($(strip $(ISP_EXT_FUSE)),)
        AVRDUDE_ISP_FUSES_PRE += -U efuse:w:$(ISP_EXT_FUSE):m
    endif

    ifneq ($(strip $(ISP_HIGH_FUSE)),)
        AVRDUDE_ISP_FUSES_PRE += -U hfuse:w:$(ISP_HIGH_FUSE):m
    endif

    ifneq ($(strip $(ISP_LOW_FUSE)),)
        AVRDUDE_ISP_FUSES_PRE += -U lfuse:w:$(ISP_LOW_FUSE):m
    endif
endif

# Bootloader file settings
# TODO: Handle relative bootloader file path as well
ifndef AVRDUDE_ISP_BURN_BOOTLOADER
    ifneq ($(strip $(BOOTLOADER_PATH)),)
        ifneq ($(strip $(BOOTLOADER_FILE)),)
            AVRDUDE_ISP_BURN_BOOTLOADER += -U flash:w:$(BOOTLOADER_PATH)/$(BOOTLOADER_FILE):i
        endif
    endif
endif

# Post fuse settings
ifndef AVRDUDE_ISP_FUSES_POST
    ifneq ($(strip $(ISP_LOCK_FUSE_POST)),)
        AVRDUDE_ISP_FUSES_POST += -U lock:w:$(ISP_LOCK_FUSE_POST):m
    endif
endif

AVRDUDE_ISP_OPTS = -c $(ISP_PROG) -b $(AVRDUDE_ISP_BAUDRATE)

ifneq ($(strip $(ISP_PROG)),usbasp)
    AVRDUDE_ISP_OPTS += -P $(call get_isp_port)
endif

ifndef ISP_EEPROM
    ISP_EEPROM  = 0
endif

AVRDUDE_UPLOAD_HEX = -U flash:w:$(TARGET_HEX):i
AVRDUDE_UPLOAD_EEP = -U eeprom:w:$(TARGET_EEP):i
AVRDUDE_ISPLOAD_OPTS = $(AVRDUDE_UPLOAD_HEX)

ifneq ($(ISP_EEPROM), 0)
    AVRDUDE_ISPLOAD_OPTS += $(AVRDUDE_UPLOAD_EEP)
endif

########################################################################
# Explicit targets start here

all:                 $(TARGET_EEP) $(TARGET_HEX) $(UPLOAD_TYPE)

# Rule to create $(OBJDIR) automatically. All rules with recipes that
# create a file within it, but do not already depend on a file within it
# should depend on this rule. They should use a "order-only
# prerequisite" (e.g., put "| $(OBJDIR)" at the end of the prerequisite
# list) to prevent remaking the target when any file in the directory
# changes.
$(OBJDIR):
				$(MKDIR) $(OBJDIR)

$(TARGET_ELF):		 $(LOCAL_OBJS) $(CORE_LIB) $(OTHER_OBJS)
				$(CC) $(LDFLAGS) -o $@ $(LOCAL_OBJS) $(CORE_LIB) $(OTHER_OBJS) -lc -lm

$(CORE_LIB):		$(CORE_OBJS) $(LIB_OBJS) $(USER_LIB_OBJS)
				$(AR) rcs $@ $(CORE_OBJS) $(LIB_OBJS) $(USER_LIB_OBJS)


# Use submake so we can guarantee the reset happens
# before the upload, even with make -j
upload:				$(TARGET_HEX) verify_size
				$(MAKE) reset
				$(MAKE) do_upload

raw_upload:		$(TARGET_HEX) verify_size
				$(MAKE) do_upload

do_upload:
				$(AVRDUDE) $(AVRDUDE_COM_OPTS) $(AVRDUDE_ARD_OPTS) \
						$(AVRDUDE_UPLOAD_HEX)

do_eeprom:		$(TARGET_EEP) $(TARGET_HEX)
				$(AVRDUDE) $(AVRDUDE_COM_OPTS) $(AVRDUDE_ARD_OPTS) \
						$(AVRDUDE_UPLOAD_EEP)

eeprom:				$(TARGET_HEX) verify_size
				$(MAKE) reset
				$(MAKE) do_eeprom

raw_eeprom:		$(TARGET_HEX) verify_size
				$(MAKE) do_eeprom

reset:
				$(call arduino_output,Resetting Arduino...)
				$(RESET_CMD)

# stty on MacOS likes -F, but on Debian it likes -f redirecting
# stdin/out appears to work but generates a spurious error on MacOS at
# least. Perhaps it would be better to just do it in perl ?
reset_stty:
				for STTYF in 'stty -F' 'stty --file' 'stty -f' 'stty <' ; \
				  do $$STTYF /dev/tty >/dev/null 2>&1 && break ; \
				done ; \
				$$STTYF $(call get_monitor_port)  hupcl ; \
				(sleep 0.1 2>/dev/null || sleep 1) ; \
				$$STTYF $(call get_monitor_port) -hupcl

ispload:		$(TARGET_EEP) $(TARGET_HEX) verify_size
				$(AVRDUDE) $(AVRDUDE_COM_OPTS) $(AVRDUDE_ISP_OPTS) \
						$(AVRDUDE_ISPLOAD_OPTS)

burn_bootloader:
ifneq ($(strip $(AVRDUDE_ISP_FUSES_PRE)),)
				$(AVRDUDE) $(AVRDUDE_COM_OPTS) $(AVRDUDE_ISP_OPTS) -e $(AVRDUDE_ISP_FUSES_PRE)
endif
ifneq ($(strip $(AVRDUDE_ISP_BURN_BOOTLOADER)),)
				$(AVRDUDE) $(AVRDUDE_COM_OPTS) $(AVRDUDE_ISP_OPTS) $(AVRDUDE_ISP_BURN_BOOTLOADER)
endif
ifneq ($(strip $(AVRDUDE_ISP_FUSES_POST)),)
				$(AVRDUDE) $(AVRDUDE_COM_OPTS) $(AVRDUDE_ISP_OPTS) $(AVRDUDE_ISP_FUSES_POST)
endif

clean:
				$(REMOVE) $(LOCAL_OBJS) $(CORE_OBJS) $(LIB_OBJS) $(CORE_LIB) $(TARGETS) $(DEPS) $(USER_LIB_OBJS) ${OBJDIR}

size:		$(TARGET_HEX)
				$(call avr_size,$(TARGET_ELF),$(TARGET_HEX))

show_boards:
				@cat $(BOARDS_TXT) | grep -E "^[[:alnum:]]" | cut -d . -f 1 | uniq

monitor:
				$(MONITOR_CMD) $(call get_monitor_port) $(MONITOR_BAUDRATE)

disasm: $(OBJDIR)/$(TARGET).lss
				@$(ECHO) The compiled ELF file has been disassembled to $(OBJDIR)/$(TARGET).lss

symbol_sizes: $(OBJDIR)/$(TARGET).sym
				@$(ECHO) A symbol listing sorted by their size have been dumped to $(OBJDIR)/$(TARGET).sym

verify_size:
ifeq ($(strip $(HEX_MAXIMUM_SIZE)),)
		@$(ECHO)
		@$(ECHO) Maximum flash memory of $(BOARD_TAG) is not specified. Make sure the size of $(TARGET_HEX) is less than $(BOARD_TAG)\'s flash memory
		@$(ECHO)
endif
		@if [ ! -f $(TARGET_HEX).sizeok ]; then echo >&2 "\nThe size of the compiled binary file is greater than the $(BOARD_TAG)'s flash memory. \
See http://www.arduino.cc/en/Guide/Troubleshooting#size for tips on reducing it."; false; fi

generate_assembly: $(OBJDIR)/$(TARGET).s
				@$(ECHO) Compiler-generated assembly for the main input source has been dumped to $(OBJDIR)/$(TARGET).s

generated_assembly: generate_assembly
				@$(ECHO) "generated_assembly" target is deprecated. Use "generate_assembly" target instead

help:
				@$(ECHO) "\nAvailable targets:\n\
  make				  - no upload\n\
  make upload		   - upload\n\
  make clean			-  remove all our dependencies\n\
  make depends		  - update dependencies\n\
  make reset			- reset the Arduino by tickling DTR on the serial port\n\
  make raw_upload	   - upload without first resetting\n\
  make show_boards	  - list all the boards defined in boards.txt\n\
  make monitor		  - connect to the Arduino's serial port\n\
  make size			 - show the size of the compiled output (relative to\n\
						  resources, if you have a patched avr-size)\n\
  make disasm		   - generate a .lss file in build-cli that contains\n\
						  disassembly of the compiled file interspersed\n\
						  with your original source code.\n\
  make verify_size	  - Verify that the size of the final file is less than\n\
						  the capacity of the micro controller.\n\
  make eeprom		   - upload the eep file\n\
  make raw_eeprom	   - upload the eep file without first resetting\n\
  make burn_bootloader  - Burn bootloader and/or fuses\n\
  make help			 - show this help\n\
"
		@$(ECHO) "Please refer to $(ARDMK_FILE) for more details."

.PHONY: all upload raw_upload raw_eeprom reset reset_stty ispload \
		clean depends size show_boards monitor disasm symbol_sizes generated_assembly \
		generate_assembly verify_size burn_bootloader help

# added - in the beginning, so that we don't get an error if the file is not present
-include $(DEPS)