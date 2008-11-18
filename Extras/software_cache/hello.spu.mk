#   SCE CONFIDENTIAL                                      
#   PLAYSTATION(R)3 Programmer Tool Runtime Library 085.007
#   Copyright (C) 2005 Sony Computer Entertainment Inc.   
#   All Rights Reserved.                                  
#

CELL_MK_DIR ?= $(CELL_SDK)/samples/mk


include $(CELL_MK_DIR)/sdk.makedef.mk 

SPU_INCDIRS += -Icache/include
SPU_SRCS     = hello.spu.c
SPU_TARGET   = hello.spu.elf
SPU_OPTIMIZE_LV=-O0

include $(CELL_MK_DIR)/sdk.target.mk


