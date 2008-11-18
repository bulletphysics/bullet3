#   SCE CONFIDENTIAL                                      
#   PLAYSTATION(R)3 Programmer Tool Runtime Library 085.007
#   Copyright (C) 2005 Sony Computer Entertainment Inc.   
#   All Rights Reserved.                                  
#

CELL_MK_DIR ?= $(CELL_SDK)/samples/mk

include $(CELL_MK_DIR)/sdk.makedef.mk

PPU_SRCS     = spu_thr_printf.ppu.c spu_printf_server.ppu.c
PPU_TARGET   = spu_thr_printf.ppu.elf

include $(CELL_MK_DIR)/sdk.target.mk


