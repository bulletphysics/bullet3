# --------------------------------------------------------------- 
# PLEASE DO NOT MODIFY THIS SECTION                               
# This prolog section is automatically generated.                 
#                                                                 
# (C) Copyright 2001,2006,                                        
# International Business Machines Corporation,                    
#                                                                 
# All Rights Reserved.                                            
# --------------------------------------------------------------- 
# --------------------------------------------------------------  
# PROLOG END TAG zYx                                              

########################################################################
#                       Source Code
########################################################################
VPATH		= ../
########################################################################
#                       Target
########################################################################
ROOT 		= ../../..
HOSTINC		= /usr/include
PROGRAM_ppu	:= CcdPhysicsDemo

#######################################################################
#			Objs
######################################################################
OBJS		= CcdPhysicsDemo.o main.o


########################################################################
#                       Libraries, Include paths,  Defines
########################################################################
INCLUDE		=	-DUSE_LIBSPE2				\
			-I$(CELL_SYSROOT)/usr/include	\
			-I../			\
			-I$(ROOT)/src		\
			-I$(ROOT)/Demos/OpenGL	\
			-I$(HOSTINC)
SYS_LIBS	:= -lspe2  -lglut -lGLU -lGL -lm -lXext -lXmu -lX11

IMPORTS		= 	$(ROOT)/lib/ibmsdk/bulletmultithreaded.a		\
			$(ROOT)/lib/ibmsdk/bulletopenglsupport.a		\
			$(ROOT)/lib/ibmsdk/bulletdynamics.a 	\
			$(ROOT)/lib/ibmsdk/bulletconvexhull.a 	\
			$(ROOT)/lib/ibmsdk/bulletcollision.a	\
			$(ROOT)/lib/ibmsdk/bulletmath.a	
	
#######################################################################
# 			Install files/dirs
#######################################################################
INSTALL_DIR     = $(ROOT)/ibmsdk
INSTALL_FILES   = $(PROGRAM_ppu)
########################################################################
#                       make.footer
########################################################################

IBM_CELLSDK_VERSION := $(shell if [ -d /opt/cell ]; then echo "3.0"; fi)

ifeq ("$(IBM_CELLSDK_VERSION)","3.0")
        CELL_TOP ?= /opt/cell/sdk
#	CELL_SYSROOT = $(CELL_TOP);
        include $(CELL_TOP)/buildutils/make.footer
else
        CELL_TOP ?= /opt/ibm/cell-sdk/prototype
	CELL_SYSROOT = $(CELL_TOP)/sysroot
        include $(CELL_TOP)/make.footer
endif

$(PPMS): 
	cp textures/$@ .

cleanlocal: 
	rm -f $(PPMS)
