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
PROGRAM_ppu	:= CollisionDemo

#######################################################################
#			Objs
######################################################################
OBJS		= CollisionDemo.o

########################################################################
#                       Libraries, Include paths,  Defines
########################################################################
INCLUDE		=				\
			-I../			\
			-I$(ROOT)/src		\
			-I$(ROOT)/Demos/OpenGL	\
			-I$(HOSTINC)
SYS_LIBS	:= -lglut -lGLU -lGL -lm -lXext -lXmu -lX11

IMPORTS		= 	$(ROOT)/lib/ibmsdk/bulletopenglsupport.a			\
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

ifdef CELL_TOP
	include $(CELL_TOP)/make.footer
else
	include ../../../make.footer
endif

$(PPMS): 
	cp textures/$@ .

cleanlocal: 
	rm -f $(PPMS)
