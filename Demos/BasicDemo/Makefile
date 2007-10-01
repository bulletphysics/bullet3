__ARCH_BITS__ := 32

# define macros
RM=rm -f
OUTDIR=.
CELLSDKDIR=/opt/ibm/cell-sdk
ARCHITECTUREFLAG=-m$(__ARCH_BITS__)
USE_CESOF=1
ifeq "$(__ARCH_BITS__)" "64"
  DEFFLAGS= -DUSE_LIBSPE2 -DUSE_ADDR64 -DUSE_PARALLEL_DISPATCHER
  GCC=ppu-g++
else
  DEFFLAGS= -DUSE_LIBSPE2 -DUSE_PARALLEL_DISPATCHER
  GCC=ppu32-g++
endif
DEBUGFLAG=
ifeq "$(USE_CESOF)" "1"
  CFLAGS= $(ARCHITECTUREFLAG) $(DEBUGFLAG) -DUSE_CESOF -W -Wall -Winline -O3 -mabi=altivec -maltivec -include altivec.h -include stdbool.h -c
else
  CFLAGS= $(ARCHITECTUREFLAG) $(DEBUGFLAG) -W -Wall -Winline -O3 -mabi=altivec -maltivec -include altivec.h -include stdbool.h -c
endif
INCLUDEDIR= -I. -I$(CELLSDKDIR)/prototype/sysroot/usr/include -I../../src -I../OpenGL
LFLAGS= $(ARCHITECTUREFLAG) -Wl,-m,elf$(__ARCH_BITS__)ppc
ifeq "$(USE_CESOF)" "1"
  LIBRARIES= -lstdc++ -lsupc++ -lgcc -lgcov -lspe2 -lpthread -L../../out/linuxppc/optimize/libs \
  		-lbulletmultithreaded -lspu -lbulletdynamics -lbulletcollision -lbulletmath \
  		-L$(CELLSDKDIR)/prototype/sysroot/usr/lib$(__ARCH_BITS__) -R$(CELLSDKDIR)/prototype/sysroot/usr/lib
else
  LIBRARIES= -lstdc++ -lsupc++ -lgcc -lgcov -lspe2 -lpthread -L../../out/linuxppc/optimize/libs \
  		-lbulletmultithreaded -lbulletdynamics -lbulletcollision -lbulletmath \
  		-L$(CELLSDKDIR)/prototype/sysroot/usr/lib$(__ARCH_BITS__) -R$(CELLSDKDIR)/prototype/sysroot/usr/lib
endif

DemoApplication :
	$(GCC) $(DEFFLAGS) $(CFLAGS) $(INCLUDEDIR) -o $(OUTDIR)/$@.o ../OpenGL/$@.cpp


BasicDemo : DemoApplication
	$(GCC) $(DEFFLAGS) $(CFLAGS) $(INCLUDEDIR) -o $(OUTDIR)/$@.o $@.cpp



all : BasicDemo
	$(GCC) -o$(OUTDIR)/BasicDemo $(OUTDIR)/BasicDemo.o $(OUTDIR)/DemoApplication.o $(LFLAGS) $(LIBRARIES)


clean : 
	$(RM) $(OUTDIR)/BasicDemo ; $(RM) $(OUTDIR)/BasicDemo.o ; $(RM) $(OUTDIR)/DemoApplication.o

