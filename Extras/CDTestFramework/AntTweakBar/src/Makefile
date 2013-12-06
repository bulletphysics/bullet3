####### Compiler, tools and options

#---- LINUX
SO_EXT		= .so
SO_VERSION	= 1

#---- Release
CXXCFG   	= -O3
LFLAGS		= 
OUT_DIR		= ../lib
#---- Debug
#CXXCFG   	= -g -D_DEBUG
#LFLAGS		= 
#OUT_DIR	= ../lib/debug


CXX      	= gcc
CXXFLAGS 	= $(CXXCFG) -Wall -fPIC -fno-strict-aliasing -D_UNIX -D__PLACEMENT_NEW_INLINE
INCPATH  	= -I../include -I/usr/local/include -I/usr/X11R6/include -I/usr/include
LINK     	= gcc
#LIBS     	= -L/usr/X11R6/lib -L. -lglfw -lGL -lGLU -lX11 -lXxf86vm -lXext -lpthread -lm
#LIBS     	= -L/usr/X11R6/lib -lGL -lX11 -lXxf86vm -lXext -lpthread -lm
LIBS 		=
AR       	= ar cqs
RANLIB   	=
TAR      	= tar -cf
GZIP     	= gzip -9f
COPY     	= cp -f
COPY_FILE	= $(COPY) -p
COPY_DIR 	= $(COPY) -pR
DEL_FILE 	= rm -f
SYMLINK  	= ln -sf
DEL_DIR  	= rmdir
MOVE     	= mv
NO_STDERR	= 2> /dev/null


####### Files


# name of the application:
TARGET 		= AntTweakBar

# source files without extension:
SRC_FILES	= TwColors.cpp TwFonts.cpp TwOpenGL.cpp TwOpenGLCore.cpp TwBar.cpp TwMgr.cpp TwPrecomp.cpp LoadOGL.cpp LoadOGLCore.cpp TwEventGLFW.c TwEventGLUT.c TwEventSDL.c TwEventSDL12.c TwEventSDL13.c TwEventSFML.cpp TwEventX11.c

# build object list from source files
OBJS_1 		= $(SRC_FILES:.c=.o)
OBJS 		= $(OBJS_1:.cpp=.o)


####### Build rules


#first:	depend all
first:	all

all: 	Makefile $(TARGET)

# append dependencies to this Makefile
#depend:
#	@echo "==== Make dependencies ====="
#	makedepend -Y
#	makedepend -a -Y -- $(CXXFLAGS) $(INCPATH) -- $(SRC_FILES) $(NO_STDERR)

$(TARGET): $(OBJS)
	@echo "===== Link $@ ====="
	$(LINK) $(LFLAGS) -shared -Wl,-soname,lib$(TARGET)$(SO_EXT).$(SO_VERSION) -o $(OUT_DIR)/lib$(TARGET)$(SO_EXT) $(OBJS) $(LIBS)
	$(SYMLINK) $(OUT_DIR)/lib$(TARGET)$(SO_EXT) $(OUT_DIR)/lib$(TARGET)$(SO_EXT).$(SO_VERSION)

.cpp.o:
	@echo "===== Compile $< ====="
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.c.o:
	@echo "===== Compile $< ====="
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

clean:
	@echo "===== Clean ====="
	-$(DEL_FILE) *.o
	-$(DEL_FILE) *~ core *.core *.stackdump

	
####### DEPENDENCIES

TwColors.o:    TwPrecomp.h TwColors.h
TwFonts.o:     TwPrecomp.h ../include/AntTweakBar.h TwFonts.h TwMgr.h TwColors.h TwGraph.h AntPerfTimer.h
TwOpenGL.o:    TwPrecomp.h ../include/AntTweakBar.h TwOpenGL.h LoadOGL.h TwGraph.h TwColors.h TwFonts.h TwMgr.h AntPerfTimer.h
TwOpenGLCore.o: TwPrecomp.h ../include/AntTweakBar.h TwOpenGLCore.h LoadOGLCore.h TwGraph.h TwColors.h TwFonts.h TwMgr.h AntPerfTimer.h
TwBar.o:       TwPrecomp.h ../include/AntTweakBar.h TwBar.h TwMgr.h TwColors.h TwFonts.h TwGraph.h AntPerfTimer.h
TwMgr.o:       TwPrecomp.h ../include/AntTweakBar.h TwMgr.h TwColors.h TwFonts.h TwGraph.h AntPerfTimer.h TwBar.h TwOpenGL.h res/TwXCursors.h
TwPrecomp.o:   TwPrecomp.h
LoadOGL.o:     TwPrecomp.h LoadOGL.h
TwEventGLFW.o: ../include/AntTweakBar.h MiniGLFW.h
TwEventGLUT.o: ../include/AntTweakBar.h MiniGLUT.h
TwEventSDL.o:  ../include/AntTweakBar.h
TwEventSDL12.o:  ../include/AntTweakBar.h MiniSDL12.h
TwEventSDL13.o:  ../include/AntTweakBar.h MiniSDL13.h
TwEventX11.o:  ../include/AntTweakBar.h
