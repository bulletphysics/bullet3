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

CELL_TOP ?= /opt/ibm/cell-sdk/prototype

########################################################################
#                       Libraries, Include paths,  Defines
########################################################################
INCLUDE		=	-DUSE_LIBSPE2				\
			-I$(CELL_TOP)/sysroot/usr/include	\
			-I../			\
			-I$(ROOT)/src		\
			-I$(ROOT)/Demos/OpenGL	\
			-I$(HOSTINC)
SYS_LIBS	:= -lspe2  -lglut -lGLU -lGL -lm -lXext -lXmu -lX11

IMPORTS		= 	$(ROOT)/lib/ibmsdk/bulletmultithreaded.a		\
			$(ROOT)/lib/ibmsdk/bulletopenglsupport.a		\
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


include $(CELL_TOP)/make.footer

$(PPMS): 
	cp textures/$@ .

cleanlocal: 
	rm -f $(PPMS)
