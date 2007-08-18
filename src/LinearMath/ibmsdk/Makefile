#### Source code Dirs
VPATH =	../

ROOT = ../../..

#### Library
LIBRARY_ppu = bulletmath.a

#### Compiler flags
CPPFLAGS        = 		\
-I$(ROOT)/src			\
-I$(SDKINC)

#### Optimization level flags
#CC_OPT_LEVEL =  $(CC_OPT_LEVEL_DEBUG)
CC_OPT_LEVEL =  -O3

##### Objects to be archived in lib

OBJS = 					\
btAlignedAllocator.o			\
btGeometryUtil.o			\
btQuickprof.o

#### Install directories
INSTALL_DIR	=  $(ROOT)/lib/ibmsdk
INSTALL_FILES	= $(LIBRARY_ppu)
CELL_TOP ?= $(ROOT)
include $(CELL_TOP)/make.footer
