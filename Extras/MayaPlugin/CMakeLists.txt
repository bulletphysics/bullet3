CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

#PROJECT(MayaPlugin)
if(COMMAND cmake_policy)
     cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)



SET(CMAKE_CXX_WARNING_LEVEL 4)

#SET (CMAKE_VERBOSE_MAKEFILE TRUE)
FIND_PATH(MAYA_BASE_DIR include/maya/MFn.h PATH /usr/autodesk/maya2008-x64 
			/usr/autodesk/maya "C:/Program Files/Autodesk/Maya2008" "C:/Program Files/Autodesk/Maya2009")

SET(MAYA_INC_DIR ${MAYA_BASE_DIR}/include)
SET(MAYA_LIB_DIR ${MAYA_BASE_DIR}/lib)

INCLUDE_DIRECTORIES(${MAYA_INC_DIR} ${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Extras/MayaPlugin )
LINK_DIRECTORIES(${MAYA_LIB_DIR})

ADD_DEFINITIONS(-D_BOOL)
ADD_DEFINITIONS(-DREQUIRE_IOSTREAM)

SET(TARGET_H 
 box_shape.h             bt_sphere_shape.h         dSolverNode.h         rigid_body_impl.h
	box_shape_impl.h        collision_shape.h         mathUtils.h           rigidBodyNode.h
	bt_box_shape.h          collision_shape_impl.h    mayaUtils.h           solver.h
	bt_collision_shape.h    collisionShapeNode.h      mesh_shape.h          solver_impl.h
	bt_convex_hull_shape.h  convex_hull_shape.h       mesh_shape_impl.h     sphere_shape.h
	bt_mesh_shape.h         convex_hull_shape_impl.h  plane_shape.h         sphere_shape_impl.h
	bt_plane_shape.h        dRigidBodyArrayCmd.h      plane_shape_impl.h
	bt_rigid_body.h         dRigidBodyCmd.h           rigidBodyArrayNode.h
	bt_solver.h             dSolverCmd.h              rigid_body.h 		pdbIO.h
	shared_ptr.h		drawUtils.h
)

SET(TARGET_SRC 
	pluginMain.cpp bulletExport.cpp  rigidBodyNode.cpp rigidBodyArrayNode.cpp collisionShapeNode.cpp
	  solver.cpp bt_solver.cpp dSolverNode.cpp dSolverCmd.cpp dRigidBodyCmd.cpp dRigidBodyArrayCmd.cpp
	  pdbIO.cpp drawUtils.cpp  constraint/dHingeConstraintCmd.cpp	constraint/dSliderConstraintCmd.cpp
	constraint/hingeConstraintNode.cpp	constraint/sixdofConstraintNode.cpp constraint/dNailConstraintCmd.cpp
	constraint/dsixdofConstraintCmd.cpp	constraint/nailConstraintNode.cpp	constraint/sliderConstraintNode.cpp
)	

ADD_LIBRARY(BulletMayaDynamica SHARED ${TARGET_H} ${TARGET_SRC})



IF (WIN32)
   SET(MAYA_LIBS OpenMaya Foundation OpenMayaAnim OpenMayaFX OpenMayaUI)
	IF (CMAKE_CL_64)
	SET_TARGET_PROPERTIES(BulletMayaDynamica PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS /MACHINE:X64")
   ELSE (CMAKE_CL_64)
   SET_TARGET_PROPERTIES(BulletMayaDynamica PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS /MACHINE:X86")
   ENDIF (CMAKE_CL_64)
   SET_TARGET_PROPERTIES(BulletMayaDynamica PROPERTIES SUFFIX ".mll")

   LINK_DIRECTORIES(${MAYA_LIB_DIR})
   TARGET_LINK_LIBRARIES(BulletMayaDynamica BulletDynamics BulletCollision LinearMath ${MAYA_LIBS} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
   SET(CMAKE_SHARED_LINKER_FLAGS "/export:initializePlugin /export:uninitializePlugin")  
ELSE (WIN32)
  SET(MAYA_LIBS OpenMayalib Foundation OpenMayaAnim OpenMayaFX OpenMayaUI)
  LINK_DIRECTORIES(${MAYA_LIB_DIR})
  TARGET_LINK_LIBRARIES(BulletMayaDynamica  BulletDynamics BulletCollision LinearMath ${MAYA_LIBS} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
ENDIF(WIN32)

#OPTION(BUILD_STANDALONE "Build a standalone exporter program")
#
#IF (BUILD_STANDALONE)
#  IF (WIN32)
#    
#  ELSE (WIN32)
#    SET(MAYA_LIBS OpenMayalib Foundation OpenMayaAnim)
#    ADD_EXECUTABLE(me ${TARGET_H} ${TARGET_SRC} standalone.cpp)
#    LINK_DIRECTORIES(${MAYA_LIB_DIR})
#    TARGET_LINK_LIBRARIES(me ${MAYA_LIBS} pthread m imf)
#  ENDIF (WIN32)      
#ENDIF (BUILD_STANDALONE)

