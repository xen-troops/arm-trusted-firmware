#
# Copyright (c) 2015-2017, Renesas Electronics Corporation. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
#

###################################################
# makefile
###################################################

#output file name
FILE_NAME_SA0   = bootparam_sa0
FILE_NAME_SA6   = cert_header_sa6
FILE_NAME_SA6_EMMC = cert_header_sa6_emmc

OUTPUT_FILE_SA0 = $(FILE_NAME_SA0).elf
OUTPUT_FILE_SA6 = $(FILE_NAME_SA6).elf
OUTPUT_FILE_SA6_EMMC = $(FILE_NAME_SA6_EMMC).elf

#object file name
OBJ_FILE_SA0 =	sa0.o
OBJ_FILE_SA6 =	sa6.o
OBJ_FILE_SA6_EMMC =	sa6_emmc.o

# source file name
SOURCE_FILE_SA6 =	sa6.c

#linker script name 
MEMORY_DEF_SA0 = sa0.ld.S
MEMORY_DEF_SA6 = sa6.ld.S

###################################################
# Convenience function for adding build definitions
# $(eval $(call add_define,FOO)) will have:
# -DFOO if $(FOO) is empty; -DFOO=$(FOO) otherwise
define add_define
DEFINES			+=	-D$(1)$(if $(value $(1)),=$(value $(1)),)
endef

# Process RCAR_SA6_TYPE flag
ifndef RCAR_SA6_TYPE
RCAR_SA6_TYPE := 0
else
ifeq (${RCAR_SA6_TYPE},0)
RCAR_SA6_TYPE := 0
else
RCAR_SA6_TYPE := 1
endif
endif
$(eval $(call add_define,RCAR_SA6_TYPE))

###################################################

#c compiler
CC = $(CROSS_COMPILE)gcc
CFLAGS += ${DEFINES}
CFLAGS += -I../../include/lib/stdlib

# Explicitly set RCAR_SA6_TYPE flag
CFLAGS_SA6_EMMC = `echo "${CFLAGS}" | sed -e 's/-DRCAR_SA6_TYPE=0/-DRCAR_SA6_TYPE=1/'`

#Linker
LD = $(CROSS_COMPILE)ld

#objcopy
objcopy = $(CROSS_COMPILE)objcopy

#clean
CL = rm -f

###################################################
.SUFFIXES : .s .c .o

###################################################
# command

.PHONY: all
all: $(OUTPUT_FILE_SA0) $(OUTPUT_FILE_SA6) $(OUTPUT_FILE_SA6_EMMC)
###################################################
# Linker
###################################################
$(OUTPUT_FILE_SA0) : $(MEMORY_DEF_SA0) $(OBJ_FILE_SA0)
	$(LD) $(OBJ_FILE_SA0)		 	\
	-T $(MEMORY_DEF_SA0)			\
	-o $(OUTPUT_FILE_SA0)			\
	-Map $(FILE_NAME_SA0).map 		\

	$(objcopy) -O srec --adjust-vma=0xE6320000 --srec-forceS3  $(OUTPUT_FILE_SA0) $(FILE_NAME_SA0).srec
	$(objcopy) -O binary --adjust-vma=0xE6320000 --srec-forceS3  $(OUTPUT_FILE_SA0) $(FILE_NAME_SA0).bin

$(OUTPUT_FILE_SA6) : $(MEMORY_DEF_SA6) $(OBJ_FILE_SA6)
	$(LD) $(OBJ_FILE_SA6)		 	\
	-T $(MEMORY_DEF_SA6)			\
	-o $(OUTPUT_FILE_SA6)			\
	-Map $(FILE_NAME_SA6).map 		\

	$(objcopy) -O srec --adjust-vma=0xE6320000 --srec-forceS3  $(OUTPUT_FILE_SA6) $(FILE_NAME_SA6).srec
	$(objcopy) -O binary --adjust-vma=0xE6320000 --srec-forceS3  $(OUTPUT_FILE_SA6) $(FILE_NAME_SA6).bin

$(OUTPUT_FILE_SA6_EMMC) : $(MEMORY_DEF_SA6) $(OBJ_FILE_SA6_EMMC)
	$(LD) $(OBJ_FILE_SA6_EMMC)	 	\
	-T $(MEMORY_DEF_SA6)			\
	-o $(OUTPUT_FILE_SA6_EMMC)		\
	-Map $(FILE_NAME_SA6_EMMC).map 	\

	$(objcopy) -O srec --adjust-vma=0xE6320000 --srec-forceS3  $(OUTPUT_FILE_SA6_EMMC) $(FILE_NAME_SA6_EMMC).srec
	$(objcopy) -O binary --adjust-vma=0xE6320000 --srec-forceS3  $(OUTPUT_FILE_SA6_EMMC) $(FILE_NAME_SA6_EMMC).bin

###################################################
# Compile
###################################################

$(OBJ_FILE_SA6_EMMC):$(SOURCE_FILE_SA6)
	${CC} -c ${CFLAGS_SA6_EMMC} ${CPPFLAGS} $< -o $@

%.o:../%.c
	$(CC) -c -I $< -o $@

.PHONY: clean
clean:
	$(CL)  *.bin *.map *.srec *.elf *.o
