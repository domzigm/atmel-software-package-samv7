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

#   Makefile for compiling the GMAC uIP Hello World Example project

#-------------------------------------------------------------------------------
#        User-modifiable options
#-------------------------------------------------------------------------------

# Chip & board used for compilation
# (can be overriden by adding CHIP=chip and BOARD=board to the command-line)
CHIP  = SAMV71Q21
BOARD = BOARD_SAMV71_XULT

# Defines which are the available memory targets for the samv71 board.
MEMORIES = sram flash

# Trace level used for compilation
# (can be overriden by adding TRACE_LEVEL=#number to the command-line)
# TRACE_LEVEL_DEBUG      5
# TRACE_LEVEL_INFO       4
# TRACE_LEVEL_WARNING    3
# TRACE_LEVEL_ERROR      2
# TRACE_LEVEL_FATAL      1
# TRACE_LEVEL_NO_TRACE   0
TRACE_LEVEL = 4

# Optimization level, put in comment for debugging
OPTIMIZATION = -O0

# Output file basename
OUTPUT = gmac_uip_helloworld

# Output directories
BIN = bin
OBJ = obj

#-------------------------------------------------------------------------------
#		Tools
#-------------------------------------------------------------------------------

# Tool suffix when cross-compiling
CROSS_COMPILE = arm-none-eabi-

# Libraries
LIBRARIES = ../../../../libraries
# Chip library directory
CHIP_LIB = $(LIBRARIES)/libchip
# Board library directory
BOARD_LIB = $(LIBRARIES)/libboard

# UIP library directory
UIP_LIB = $(LIBRARIES)/libuip


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
INCLUDES += -I$(UIP_LIB)/source/samv7-specific
INCLUDES += -I$(UIP_LIB)/source/uip_1.0
INCLUDES += -I$(UIP_LIB)/source/uip_1.0/apps/dhcpc
INCLUDES += -I$(UIP_LIB)/source/uip_1.0/lib
INCLUDES += -I$(UIP_LIB)/source/uip_1.0/uip
INCLUDES += -I$(UIP_LIB)/source/uip_1.0/apps/hello-world

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
CFLAGS += -g $(OPTIMIZATION) $(INCLUDES) -DBOARD_SAMV71_XULT -D__$(CHIP)__ -DTRACE_LEVEL=$(TRACE_LEVEL) -DMPU_HAS_NOCACHE_REGION
ASFLAGS = -march=armv7-m -mtune=cortex-m7  -mthumb -Wall -g $(OPTIMIZATION) $(INCLUDES) -D__$(CHIP)__ -D__ASSEMBLY__
LDFLAGS= -march=armv7-m -mtune=cortex-m7 -mthumb  -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all -Wl,--warn-common
LD_OPTIONAL= -L$(BOARD_LIB)/resources_v71/nocache_region/gcc -Wl,--print-gc-sections -Wl,--stats

#-------------------------------------------------------------------------------
#		Files
#-------------------------------------------------------------------------------

# Directories where source files can be found
VPATH =.
VPATH += ../..
VPATH += $(CHIP_LIB)/source
VPATH += $(BOARD_LIB)/source
VPATH += $(UIP_LIB)/source/samv7-specific
VPATH += $(UIP_LIB)/source/uip_1.0/uip
VPATH += $(UIP_LIB)/source/uip_1.0/apps/hello-world
VPATH += $(UIP_LIB)/source/uip_1.0/lib
VPATH += $(UIP_LIB)/source/uip_1.0/uip
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
C_OBJECTS += gmacb_phy.o
C_OBJECTS += gmac.o
C_OBJECTS += gmacd.o
C_OBJECTS += twi.o
C_OBJECTS += twid.o



# source files of libuip
C_OBJECTS += hello-world.o
C_OBJECTS += memb.o
C_OBJECTS += clock-arch.o
C_OBJECTS += gmac_tapdev.o
C_OBJECTS += psock.o
C_OBJECTS += timer.o
C_OBJECTS += uip-fw.o
C_OBJECTS += uip.o
C_OBJECTS += uip_arp.o
C_OBJECTS += uiplib.o

C_OBJECTS += main.o

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
	$(CC) $(LDFLAGS) $(LD_OPTIONAL) -T"$(BOARD_LIB)/resources_v71/nocache_region/gcc/samv71q21_$$@.ld" -Wl,-Map,$(OUTPUT)-$$@.map -o $(OUTPUT)-$$@.elf $$^ $(LIBS)
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
