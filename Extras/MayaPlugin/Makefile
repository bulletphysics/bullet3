#MayaPlugin Makefile

## MAYA_LOCATION is the Maya installation directory. It should be already defined in your
# environment variables. If not, please change it to the appropriate directory
MAYA=$(MAYA_LOCATION)

## Change this if you want to change the installation directory
MAYA_PLUG_IN_PATH=/usr/maya-plugins

## Change this if you want to change the name of the final plugin 
LIBRARY=mayaplugin.so

##################################

BULLET=../..

CPP = g++412
LD = ld

CPPFLAGS = -DBits64_ -m64 -DUNIX -D_BOOL -DLINUX -DFUNCPROTO -D_GNU_SOURCE -DLINUX_64 -fPIC \
		-fno-strict-aliasing -DREQUIRE_IOSTREAM -Wno-deprecated -Wall \
		-Wno-multichar -Wno-comment -Wno-sign-compare -funsigned-char \
		-Wno-reorder -fno-gnu-keywords -ftemplate-depth-25 -pthread \
		-Wno-deprecated -fno-gnu-keywords \
		-g

LDFLAGS =-Wl,-Bsymbolic -shared

BULLET_INCLUDE=-I$(BULLET)/src
BULLET_LIB=-L$(BULLET)/out/linux/optimize/libs -L$(BULLET)/src \
               -lBulletColladaConverter -lcolladadom -llibxml -lGIMPACTUtils -lbulletdynamics \
                 -lbulletcollision -lbulletmath

GL_LIB=-lGL -lGLU 

MAYA_INCLUDE=-I$(MAYA)/include
MAYA_LIB=-L$(MAYA)/lib -lOpenMaya -lFoundation -lOpenMayaUI -lOpenMayaFX

SOURCES = bt_solver.cpp           dRigidBodyArrayCmd.cpp  pdbIO.cpp                \
	colladaExport.cpp       dRigidBodyCmd.cpp       pluginMain.cpp             \
	collisionShapeNode.cpp  dSolverCmd.cpp          rigidBodyArrayNode.cpp     \
	dNailConstraintCmd.cpp  dSolverNode.cpp         rigidBodyNode.cpp          \
	drawUtils.cpp           nailConstraintNode.cpp  solver.cpp                 



HEADERS = box_shape.h             collisionShapeNode.h      nail_constraint.h     \
	box_shape_impl.h        constraint.h              nail_constraint_impl.h  \
	bt_box_shape.h          constraint_impl.h         nailConstraintNode.h    \
	bt_collision_shape.h    convex_hull_shape.h       pdbIO.h                 \
	bt_constraint.h         convex_hull_shape_impl.h  plane_shape.h           \
	bt_convex_hull_shape.h  dNailConstraintCmd.h      plane_shape_impl.h      \
	bt_mesh_shape.h         drawUtils.h               rigidBodyArrayNode.h    \
	bt_nail_constraint.h    dRigidBodyArrayCmd.h      rigid_body.h            \
	bt_plane_shape.h        dRigidBodyCmd.h           rigid_body_impl.h       \
	bt_rigid_body.h         dSolverCmd.h              rigidBodyNode.h         \
	bt_solver.h             dSolverNode.h             shared_ptr.h            \
	bt_sphere_shape.h       mathUtils.h               solver.h                \
	colladaExport.h         mayaUtils.h               solver_impl.h           \
	collision_shape.h       mesh_shape.h              sphere_shape.h          \
	collision_shape_impl.h  mesh_shape_impl.h         sphere_shape_impl.h     



INCLUDE_FLAGS= $(GL_INCLUDE) $(BULLET_INCLUDE) $(MAYA_INCLUDE)
LIB_FLAGS=$(BULLET_LIB) $(MAYA_LIB) $(GL_LIB)

OBJECTS=$(SOURCES:.cpp=.o)

all: $(SOURCES) $(LIBRARY)

.cpp.o: $(SOURCES) $(HEADERS)
	$(CPP) -c $< $(CPPFLAGS) $(INCLUDE_FLAGS) -o $@ 

$(OBJECTS): $(HEADERS)

$(LIBRARY): $(OBJECTS) 
	$(CPP) $(OBJECTS) $(LDFLAGS) $(LIB_FLAGS) -o $@

install: $(LIBRARY)
	cp -f $(LIBRARY) $(MAYA_PLUG_IN_PATH)	
	cp -f scripts/*.mel $(MAYA_PLUG_IN_PATH)	
	cp -f icons/*.xpm $(MAYA_PLUG_IN_PATH)	

clean:
	rm -f *.o *.so

