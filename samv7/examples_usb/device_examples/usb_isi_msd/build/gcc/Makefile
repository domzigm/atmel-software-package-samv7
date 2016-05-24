# ----------------------------------------------------------------------------
#         SAM Software Package License
# ----------------------------------------------------------------------------
# Copyright (c) 2014, Atmel Corporation
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice,
# this list of conditions and the disclaimer below.
#
# Atmel's name may not be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ----------------------------------------------------------------------------

#   Makefile for compiling the USB Still Image Camera Example project

#-------------------------------------------------------------------------------
#        User-modifiable options
#-------------------------------------------------------------------------------

# Chip & board used for compilation
# (can be overriden by adding CHIP=chip and BOARD=board to the command-line)
CHIP  = SAMV71Q21
BOARD = BOARD_SAMV71_XULT

# Defines which are the available memory targets for the samv71 board.
MEMORIES = flash

# Trace level used for compilation
# (can be overriden by adding TRACE_LEVEL=#number to the command-line)
# TRACE_LEVEL_DEBUG      5
# TRACE_LEVEL_INFO       4
# TRACE_LEVEL_WARNING    3
# TRACE_LEVEL_ERROR      2
# TRACE_LEVEL_FATAL      1
# TRACE_LEVEL_NO_TRACE   0
TRACE_LEVEL = 2

# Optimization level, put in comment for debugging
OPTIMIZATION = -O0

# Output file basename
OUTPUT = usb_isi_msd

# Output directories
BIN = bin
OBJ = obj

#-------------------------------------------------------------------------------
#		Tools
#-------------------------------------------------------------------------------

# Tool suffix when cross-compiling
CROSS_COMPILE = arm-none-eabi-

# Libraries
LIBRARIES = ../../../../../libraries
# Chip library directory
CHIP_LIB = $(LIBRARIES)/libchip
# Board library directory
BOARD_LIB = $(LIBRARIES)/libboard

# USB library directory
USB_LIB = $(LIBRARIES)/libusb

# storage library directory
STORAGE_LIB = $(LIBRARIES)/libstoragemedia

# FATFS library directory
FAT_LIB = $(LIBRARIES)/libfatfs

# JEPG library directory
JPEG_LIB = $(LIBRARIES)/libjpeg


LIBS = -Wl,--start-group -lgcc -lc -Wl,--end-group

# Compilation tools
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
SIZE = $(CROSS_COMPILE)size
STRIP = $(CROSS_COMPILE)strip
OBJCOPY = $(CROSS_COMPILE)objcopy
GDB = $(CROSS_COMPILE)gdb
NM = $(CROSS_COMPILE)nm

# Flags
INCLUDES  = -I$(LIBRARIES)
INCLUDES += -I$(CHIP_LIB)
INCLUDES += -I$(CHIP_LIB)/include
INCLUDES += -I$(CHIP_LIB)/include/samv71
INCLUDES += -I$(BOARD_LIB)
INCLUDES += -I$(BOARD_LIB)/include
INCLUDES += -I$(CHIP_LIB)/include/cmsis/CMSIS/Include
INCLUDES += -I$(USB_LIB)
INCLUDES += -I$(USB_LIB)/include
INCLUDES += -I$(STORAGE_LIB)
INCLUDES += -I$(STORAGE_LIB)/include
INCLUDES += -I$(STORAGE_LIB)/include/sdmmc
INCLUDES += -I$(FAT_LIB)
INCLUDES += -I$(FAT_LIB)/inc
INCLUDES += -I$(FAT_LIB)/src
INCLUDES += -I$(JPEG_LIB)
INCLUDES += -I$(JPEG_LIB)/include

#INCLUDES += -I$(USB_LIB)/include
INCLUDES += -I$(LIBRARIES)
INCLUDES += -I../../

CFLAGS += -Wall -Wchar-subscripts -Wcomment -Wformat=2 -Wimplicit-int
CFLAGS += -Werror-implicit-function-declaration -Wmain -Wparentheses
CFLAGS += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs
CFLAGS += -Wuninitialized -Wunknown-pragmas -Wfloat-equal
CFLAGS += -Wshadow -Wpointer-arith -Wbad-function-cast -Wwrite-strings
CFLAGS += -Waggregate-return -Wstrict-prototypes
CFLAGS += -Wmissing-prototypes -Wmissing-declarations
CFLAGS += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CFLAGS += -Wredundant-decls -Wnested-externs -Wlong-long
CFLAGS += -Wunreachable-code

# To reduce application size use only integer printf function.
CFLAGS += -Dprintf=iprintf

CFLAGS += -march=armv7-m -mtune=cortex-m7 -mthumb -ffunction-sections
CFLAGS += -g $(OPTIMIZATION) $(INCLUDES) -DBOARD_SAMV71_XULT -D__$(CHIP)__ -DTRACE_LEVEL=$(TRACE_LEVEL) -DSD_SINGLE_LINE_MODE
ASFLAGS = -march=armv7-m -mtune=cortex-m7  -mthumb -Wall -g $(OPTIMIZATION) $(INCLUDES) -D__$(CHIP)__ -D__ASSEMBLY__
LDFLAGS= -march=armv7-m -mtune=cortex-m7 -mthumb  -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all -Wl,--warn-common
LD_OPTIONAL= -L./ -Wl,--print-gc-sections -Wl,--stats

#-------------------------------------------------------------------------------
#		Files
#-------------------------------------------------------------------------------

# Directories where source files can be found
VPATH =.
VPATH += ../..
VPATH += $(CHIP_LIB)/source
VPATH += $(BOARD_LIB)/source
VPATH += $(USB_LIB)/common/core
VPATH += $(USB_LIB)/device/massstorage
VPATH += $(USB_LIB)/device/core
VPATH += $(STORAGE_LIB)/source
VPATH += $(STORAGE_LIB)/source/sdmmc
VPATH += $(FAT_LIB)/src
VPATH += $(FAT_LIB)/src/option
VPATH += $(JPEG_LIB)
VPATH += $(JPEG_LIB)/source
VPATH += $(BOARD_LIB)/resources_v71
VPATH += $(BOARD_LIB)/resources_v71/gcc

# Objects built from C source files
C_OBJECTS += board_lowlevel.o
C_OBJECTS += board_memories.o
C_OBJECTS += dbg_console.o
C_OBJECTS += syscalls.o
C_OBJECTS += led.o
C_OBJECTS += system_samv71.o
C_OBJECTS += trace.o
C_OBJECTS += exceptions.o
C_OBJECTS += mpu.o
C_OBJECTS += pio.o
C_OBJECTS += pio_it.o
C_OBJECTS += pmc.o
C_OBJECTS += startup_samv71.o
C_OBJECTS += supc.o
C_OBJECTS += tc.o
C_OBJECTS += timetick.o
C_OBJECTS += uart.o
C_OBJECTS += xdma_hardware_interface.o
C_OBJECTS += xdmac.o
C_OBJECTS += xdmad.o
C_OBJECTS += wdt.o
C_OBJECTS += image_sensor_inf.o
C_OBJECTS += ov2640_config.o
C_OBJECTS += ov2643_config.o
C_OBJECTS += ov5640_config.o
C_OBJECTS += ov7740_config.o
C_OBJECTS += ov9740_config.o
C_OBJECTS += ov7670_config.o
C_OBJECTS += bmp.o
C_OBJECTS += isi.o
C_OBJECTS += uart_dma.o
C_OBJECTS += usart.o
C_OBJECTS += usart_dma.o
C_OBJECTS += twi.o
C_OBJECTS += twid.o
C_OBJECTS += mcid_dma.o
C_OBJECTS += hsmci.o



# source files of libusb
C_OBJECTS += USBD_HAL.o
C_OBJECTS += USBDescriptors.o
C_OBJECTS += USBRequests.o
C_OBJECTS += MSDDStateMachine.o
C_OBJECTS += MSDFunction.o
C_OBJECTS += MSDIOFifo.o
C_OBJECTS += MSDLun.o
C_OBJECTS += SBCMethods.o
C_OBJECTS += MSDDriver.o
C_OBJECTS += USBD.o
C_OBJECTS += USBDCallbacks.o
C_OBJECTS += USBDDriver.o
C_OBJECTS += USBDDriverCallbacks.o

# source files of libfatfs
C_OBJECTS += diskio_samv7x.o
C_OBJECTS += fattime_rtc.o
C_OBJECTS += ff.o
C_OBJECTS += lowlevel.o
C_OBJECTS += ccsbcs.o

# source files of libmemory
C_OBJECTS += sdmmc.o
C_OBJECTS += Media.o
C_OBJECTS += MEDRamDisk.o
C_OBJECTS += MEDSdcard.o

# source files of libjpeg
C_OBJECTS += jaricom.o
C_OBJECTS += jerror.o
C_OBJECTS += jmemmgr.o
C_OBJECTS += jmemnobs.o
C_OBJECTS += jutils.o
C_OBJECTS += jcarith.o
C_OBJECTS += jccoefct.o
C_OBJECTS += jccolor.o
C_OBJECTS += jcdctmgr.o
C_OBJECTS += jchuff.o
C_OBJECTS += jcinit.o
C_OBJECTS += jcmainct.o
C_OBJECTS += jcmarker.o
C_OBJECTS += jcmaster.o
C_OBJECTS += jcprepct.o
C_OBJECTS += jcsample.o
C_OBJECTS += jdatadst_mem.o
C_OBJECTS += jfdctflt.o
C_OBJECTS += jfdctfst.o
C_OBJECTS += jfdctint.o
C_OBJECTS += jdarith.o
C_OBJECTS += jdatasrc_mem.o
C_OBJECTS += jdcoefct.o
C_OBJECTS += jdcolor.o
C_OBJECTS += jddctmgr.o
C_OBJECTS += jdhuff.o
C_OBJECTS += jdinput.o
C_OBJECTS += jdmainct.o
C_OBJECTS += jdmarker.o
C_OBJECTS += jdmaster.o
C_OBJECTS += jdmerge.o
C_OBJECTS += jdpostct.o
C_OBJECTS += jdsample.o
C_OBJECTS += jidctflt.o
C_OBJECTS += jidctfst.o
C_OBJECTS += jidctint.o
C_OBJECTS += jquant1.o
C_OBJECTS += jquant2.o
C_OBJECTS += jcapimin.o
C_OBJECTS += jcapistd.o
C_OBJECTS += jcomapi.o
C_OBJECTS += jcparam.o
C_OBJECTS += jctrans.o
C_OBJECTS += jdapimin.o
C_OBJECTS += jdapistd.o
C_OBJECTS += jdtrans.o
C_OBJECTS += libjpeg.o

C_OBJECTS += main.o
C_OBJECTS += isi_capture.o
C_OBJECTS += USBDDriverDescriptors.o

# Append OBJ and BIN directories to output filename
OUTPUT := $(BIN)/$(OUTPUT)

#-------------------------------------------------------------------------------
#		Rules
#-------------------------------------------------------------------------------

all: $(BIN) $(OBJ) $(MEMORIES)

$(BIN) $(OBJ):
	mkdir $@

define RULES
C_OBJECTS_$(1) = $(addprefix $(OBJ)/$(1)_, $(C_OBJECTS))
ASM_OBJECTS_$(1) = $(addprefix $(OBJ)/$(1)_, $(ASM_OBJECTS))

$(1): $$(ASM_OBJECTS_$(1)) $$(C_OBJECTS_$(1))
	$(CC) $(LDFLAGS) $(LD_OPTIONAL) -T"samv71q21_$$@.ld" -Wl,-Map,$(OUTPUT)-$$@.map -o $(OUTPUT)-$$@.elf $$^ $(LIBS)
	$(NM) $(OUTPUT)-$$@.elf >$(OUTPUT)-$$@.elf.txt
	$(OBJCOPY) -O binary $(OUTPUT)-$$@.elf $(OUTPUT)-$$@.bin
	$(SIZE) $$^ $(OUTPUT)-$$@.elf

$$(C_OBJECTS_$(1)): $(OBJ)/$(1)_%.o: %.c Makefile $(OBJ) $(BIN)
	$(CC) $(CFLAGS) -D$(1) -c -o $$@ $$<

$$(ASM_OBJECTS_$(1)): $(OBJ)/$(1)_%.o: %.S Makefile $(OBJ) $(BIN)
	$(CC) $(ASFLAGS) -D$(1) -c -o $$@ $$<

debug_$(1): $(1)
	$(GDB) -x "$(BOARD_LIB)/resources_v71/gcc/samv71-$(1).gdb" -ex "reset" -readnow -se $(OUTPUT)-$(1).elf
endef

$(foreach MEMORY, $(MEMORIES), $(eval $(call RULES,$(MEMORY))))

clean:
	-rm -fR $(OBJ) $(BIN)
