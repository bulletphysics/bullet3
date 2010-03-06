#### Source code Dirs
VPATH =	../

ROOT = ../../..
HOST = /usr/include

#### Library
LIBRARY_ppu = bulletopenglsupport.a

#### Compiler flags
CPPFLAGS        = -DUSE_LIBSPE2		\
-I$(ROOT)/Extras/ConvexHull		\
-I$(ROOT)/src/			\
-I$(SDKINC)			\
-I$(HOST)

#### Optimization level flags
#CC_OPT_LEVEL =  $(CC_OPT_LEVEL_DEBUG)
CC_OPT_LEVEL =  -O3

##### Objects to be archived in lib
OBJS = 				\
BMF_Api.o			\
BMF_BitmapFont.o		\
BMF_font_helv10.o		\
DemoApplication.o		\
GLDebugDrawer.o			\
GL_ShapeDrawer.o		\
GL_Simplex1to4.o		\
GlutStuff.o			\
RenderTexture.o	

#### Install directories
INSTALL_DIR	= $(ROOT)/lib/ibmsdk/
INSTALL_FILES	= $(LIBRARY)

IBM_CELLSDK_VERSION := $(shell if [ -d /opt/cell ]; then echo "3.0"; fi)

ifeq ("$(IBM_CELLSDK_VERSION)","3.0")
        CELL_TOP ?= /opt/cell/sdk
        include $(CELL_TOP)/buildutils/make.footer
else
        CELL_TOP ?= /opt/ibm/cell-sdk/prototype
        include $(CELL_TOP)/make.footer
endif
