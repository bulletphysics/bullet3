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
PROGRAM_ppu	:= BasicDemo2

#######################################################################
#			Objs
######################################################################
OBJS		= BasicDemo2.o

########################################################################
#                       Libraries, Include paths,  Defines
########################################################################
INCLUDE		=	-DUSE_LIBSPE2		\
			-I../			\
			-I$(ROOT)/src		\
			-I$(ROOT)/Demos/OpenGL	\
			-I$(HOSTINC)
SYS_LIBS	:= -lspe2  -lm 

IMPORTS		= 	$(ROOT)/lib/ibmsdk/bulletmultithreaded.a \
			$(ROOT)/lib/ibmsdk/bulletdynamics.a 	\
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
        include $(CELL_TOP)/buildutils/make.footer
else
        CELL_TOP ?= /opt/ibm/cell-sdk/prototype
        include $(CELL_TOP)/make.footer
endif


$(PPMS): 
	cp textures/$@ .

cleanlocal: 
	rm -f $(PPMS)
