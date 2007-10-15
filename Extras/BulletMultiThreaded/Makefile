__ARCH_BITS__ := 32

# define macros
NARROWPHASEDIR=./SpuNarrowPhaseCollisionTask
SPU_TASKFILE=$(NARROWPHASEDIR)/SpuGatheringCollisionTask

RM=rm -f 
OUTDIR=./out
DEBUGFLAG=-DNDEBUG
LIBOUTDIR=../../lib/ibmsdk
COLLISIONDIR=../../src/BulletCollision
MATHDIR=../../src/LinearMath
CELLSDKDIR=/opt/ibm/cell-sdk
ARCHITECTUREFLAG=-m$(__ARCH_BITS__)
ifeq "$(__ARCH_BITS__)" "64"
  SPU_DEFFLAGS= -DUSE_LIBSPE2 -D__SPU__ -DUSE_ADDR64
else
  SPU_DEFFLAGS= -DUSE_LIBSPE2 -D__SPU__
endif
SPU_GCC=/usr/bin/spu-gcc
SPU_INCLUDEDIR= -I. -I$(CELLSDKDIR)/prototype/sysroot/usr/spu/include -I../../src -I$(NARROWPHASEDIR)
#SPU_CFLAGS= $(DEBUGFLAG) -W -Wall -Winline -Os -c -include spu_intrinsics.h -include stdbool.h
SPU_CFLAGS= $(DEBUGFLAG) -W -Wall -Winline -O3 -mbranch-hints -fomit-frame-pointer -ftree-vectorize -finline-functions -ftree-vect-loop-version -ftree-loop-optimize -ffast-math -fno-rtti -fno-exceptions -c -include spu_intrinsics.h -include stdbool.h

SPU_LFLAGS= -Wl,-N
SPU_LIBRARIES=-lstdc++
SPU_EMBED=/usr/bin/ppu-embedspu
SPU_AR=/usr/bin/ar
SYMBOLNAME=spu_program

ifeq "$(__ARCH_BITS__)" "64"
  PPU_DEFFLAGS= -DUSE_LIBSPE2 -DUSE_ADDR64
  PPU_GCC=/usr/bin/ppu-gcc
else
  PPU_DEFFLAGS= -DUSE_LIBSPE2
  PPU_GCC=/usr/bin/ppu32-gcc
endif

PPU_CFLAGS= $(ARCHITECTUREFLAG) $(DEBUGFLAG) -W -Wall -Winline -O3 -c -mabi=altivec -maltivec -include altivec.h -include stdbool.h
PPU_INCLUDEDIR= -I. -I$(CELLSDKDIR)/prototype/sysroot/usr/include -I../../src -I$(NARROWPHASEDIR)
PPU_LFLAGS= $(ARCHITECTUREFLAG) -Wl,-m,elf$(__ARCH_BITS__)ppc
PPU_LIBRARIES= -lstdc++ -lsupc++ -lgcc -lgcov -lspe2 -lpthread -L../../lib/ibmsdk -lbulletcollision -lbulletdynamics -lbulletmath -L$(CELLSDKDIR)/prototype/sysroot/usr/lib$(__ARCH_BITS__) -R$(CELLSDKDIR)/prototype/sysroot/usr/lib
PPU_AR=/usr/bin/ar

MakeOut :
	rm -f -R $(OUTDIR) ; mkdir $(OUTDIR)

# SPU
SpuTaskFile : MakeOut
	$(SPU_GCC) $(SPU_DEFFLAGS) $(SPU_CFLAGS) $(SPU_INCLUDEDIR) -o $(OUTDIR)/SpuTaskFile.o $(SPU_TASKFILE).cpp

SpuFakeDma : MakeOut
	$(SPU_GCC) $(SPU_DEFFLAGS) $(SPU_CFLAGS) $(SPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $@.cpp

SpuContactManifoldCollisionAlgorithm_spu : MakeOut
	$(SPU_GCC) $(SPU_DEFFLAGS) $(SPU_CFLAGS) $(SPU_INCLUDEDIR) -o $(OUTDIR)/$@.o SpuContactManifoldCollisionAlgorithm.cpp

SpuContactResult : MakeOut
	$(SPU_GCC) $(SPU_DEFFLAGS) $(SPU_CFLAGS) $(SPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $(NARROWPHASEDIR)/$@.cpp

#SpuGatheringCollisionTask : MakeOut
#	$(SPU_GCC) $(SPU_DEFFLAGS) $(SPU_CFLAGS) $(SPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $(NARROWPHASEDIR)/$@.cpp

SpuGjkPairDetector: MakeOut
	$(SPU_GCC) $(SPU_DEFFLAGS) $(SPU_CFLAGS) $(SPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $(NARROWPHASEDIR)/$@.cpp

SpuMinkowskiPenetrationDepthSolver : MakeOut
	$(SPU_GCC) $(SPU_DEFFLAGS) $(SPU_CFLAGS) $(SPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $(NARROWPHASEDIR)/$@.cpp

SpuVoronoiSimplexSolver : MakeOut
	$(SPU_GCC) $(SPU_DEFFLAGS) $(SPU_CFLAGS) $(SPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $(NARROWPHASEDIR)/$@.cpp

#SpuLibspe2Support_spu : MakeOut
#	$(SPU_GCC) $(SPU_DEFFLAGS) $(SPU_CFLAGS) $(SPU_INCLUDEDIR) -o $(OUTDIR)/$@.o SpuLibspe2Support.cpp

## SPU-Bullet
btPersistentManifold : MakeOut
	$(SPU_GCC) $(SPU_DEFFLAGS) $(SPU_CFLAGS) $(SPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $(COLLISIONDIR)/NarrowPhaseCollision/$@.cpp

btOptimizedBvh : MakeOut
	$(SPU_GCC) $(SPU_DEFFLAGS) $(SPU_CFLAGS) $(SPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $(COLLISIONDIR)/CollisionShapes/$@.cpp

btCollisionObject : MakeOut
	$(SPU_GCC) $(SPU_DEFFLAGS) $(SPU_CFLAGS) $(SPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $(COLLISIONDIR)/CollisionDispatch/$@.cpp

btTriangleCallback : MakeOut
	$(SPU_GCC) $(SPU_DEFFLAGS) $(SPU_CFLAGS) $(SPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $(COLLISIONDIR)/CollisionShapes/$@.cpp

btTriangleIndexVertexArray : MakeOut
	$(SPU_GCC) $(SPU_DEFFLAGS) $(SPU_CFLAGS) $(SPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $(COLLISIONDIR)/CollisionShapes/$@.cpp

btStridingMeshInterface : MakeOut
	$(SPU_GCC) $(SPU_DEFFLAGS) $(SPU_CFLAGS) $(SPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $(COLLISIONDIR)/CollisionShapes/$@.cpp

btAlignedAllocator : MakeOut
	$(SPU_GCC) $(SPU_DEFFLAGS) $(SPU_CFLAGS) $(SPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $(MATHDIR)/$@.cpp


# PPU
SpuGatheringCollisionDispatcher : MakeOut
	$(PPU_GCC) $(PPU_DEFFLAGS) $(PPU_CFLAGS) $(PPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $@.cpp

SpuLibspe2Support: MakeOut
	$(PPU_GCC) $(PPU_DEFFLAGS) $(PPU_CFLAGS) $(PPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $@.cpp

btThreadSupportInterface: MakeOut
	$(PPU_GCC) $(PPU_DEFFLAGS) $(PPU_CFLAGS) $(PPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $@.cpp

SpuCollisionTaskProcess : MakeOut
	$(PPU_GCC) $(PPU_DEFFLAGS) $(PPU_CFLAGS) $(PPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $@.cpp

SpuContactManifoldCollisionAlgorithm : MakeOut
	$(PPU_GCC) $(PPU_DEFFLAGS) $(PPU_CFLAGS) $(PPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $@.cpp
	
SpuSampleTaskProcess : MakeOut
	$(PPU_GCC) $(PPU_DEFFLAGS) $(PPU_CFLAGS) $(PPU_INCLUDEDIR) -o $(OUTDIR)/$@.o $@.cpp
	


spu : SpuFakeDma SpuContactManifoldCollisionAlgorithm_spu SpuContactResult SpuTaskFile \
      SpuGjkPairDetector SpuMinkowskiPenetrationDepthSolver SpuVoronoiSimplexSolver \
      btPersistentManifold btOptimizedBvh btCollisionObject btTriangleCallback btTriangleIndexVertexArray \
      btStridingMeshInterface btAlignedAllocator
	$(SPU_GCC) -o $(OUTDIR)/spuCollision.elf \
                            $(OUTDIR)/SpuTaskFile.o \
                            $(OUTDIR)/SpuFakeDma.o \
                            $(OUTDIR)/SpuContactManifoldCollisionAlgorithm_spu.o \
                            $(OUTDIR)/SpuContactResult.o \
                            $(OUTDIR)/SpuGjkPairDetector.o \
                            $(OUTDIR)/SpuMinkowskiPenetrationDepthSolver.o \
                            $(OUTDIR)/SpuVoronoiSimplexSolver.o \
                            $(OUTDIR)/btPersistentManifold.o \
                            $(OUTDIR)/btTriangleCallback.o \
                            $(OUTDIR)/btTriangleIndexVertexArray.o \
                            $(OUTDIR)/btStridingMeshInterface.o \
                            $(OUTDIR)/btAlignedAllocator.o \
                            $(SPU_LFLAGS) $(SPU_LIBRARIES)

spu-embed : spu                            
	$(SPU_EMBED) $(ARCHITECTUREFLAG) $(SYMBOLNAME) $(OUTDIR)/spuCollision.elf $(OUTDIR)/$@.o
	$(SPU_AR) -qcs $(LIBOUTDIR)/libspu.a $(OUTDIR)/$@.o



ppu : SpuGatheringCollisionDispatcher SpuCollisionTaskProcess btThreadSupportInterface \
      SpuLibspe2Support SpuContactManifoldCollisionAlgorithm SpuSampleTaskProcess
	$(PPU_AR) -qcs $(LIBOUTDIR)/bulletmultithreaded.a \
                                                          $(OUTDIR)/SpuCollisionTaskProcess.o \
                                                          $(OUTDIR)/SpuSampleTaskProcess.o \
                                                          $(OUTDIR)/SpuGatheringCollisionDispatcher.o \
                                                          $(OUTDIR)/SpuLibspe2Support.o \
                                                          $(OUTDIR)/btThreadSupportInterface.o \
							  $(OUTDIR)/SpuContactManifoldCollisionAlgorithm.o


all : spu ppu 

all-embed : spu-embed ppu


clean :
	$(RM) $(OUTDIR)/* ; $(RM) $(LIBOUTDIR)/libspu.a ; $(RM) $(LIBOUTDIR)/bulletmultithreaded.a




