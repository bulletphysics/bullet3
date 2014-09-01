

INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src 
	${BULLET_PHYSICS_SOURCE_DIR}/btgui
	${BULLET_PHYSICS_SOURCE_DIR}/btgui/lua-5.2.3/src
)

SET(App_AllBullet2Demos_SRCS
	main.cpp
	BulletDemoInterface.h
	BulletDemoEntries.h
	GwenParameterInterface.cpp
	GwenParameterInterface.h
	GraphingTexture.h
        GwenParameterInterface.h    
	GwenProfileWindow.h         
	GwenTextureWindow.h         
	GraphingTexture.cpp         
	GwenProfileWindow.cpp 
	GwenTextureWindow.cpp
	../bullet2/BasicDemo/Bullet2RigidBodyDemo.cpp
	../bullet2/BasicDemo/Bullet2RigidBodyDemo.h
	../../Demos/BasicDemo/BasicDemoPhysicsSetup.cpp
	../../Demos/BasicDemo/BasicDemoPhysicsSetup.h
	../../Demos/CcdPhysicsDemo/CcdPhysicsSetup.cpp
	../../Demos/CcdPhysicsDemo/CcdPhysicsSetup.h
	../../Demos/SerializeDemo/SerializeSetup.cpp
        ../../Extras/Serialize/BulletFileLoader/bChunk.cpp
        ../../Extras/Serialize/BulletFileLoader/bDNA.cpp
        ../../Extras/Serialize/BulletFileLoader/bFile.cpp
        ../../Extras/Serialize/BulletFileLoader/btBulletFile.cpp
        ../../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.cpp
        ../../Extras/Serialize/BulletWorldImporter/btWorldImporter.cpp
	../bullet2/MultiBodyDemo/TestJointTorqueSetup.cpp
	../bullet2/ConstraintDemo/ConstraintPhysicsSetup.cpp
	../bullet2/ConstraintDemo/ConstraintPhysicsSetup.h
	../bullet2/FeatherstoneMultiBodyDemo/BulletMultiBodyDemos.cpp
	../bullet2/FeatherstoneMultiBodyDemo/BulletMultiBodyDemos.h
	../bullet2/FeatherstoneMultiBodyDemo/MultiDofDemo.cpp
	../bullet2/FeatherstoneMultiBodyDemo/MultiDofDemo.h
	../bullet2/BasicDemo/BasicDemo.cpp
	../bullet2/BasicDemo/BasicDemo.h
# the next few demos are not converted to 'newer' structure yet
# target is to convert all Bullet 2 demos in new structure but need to settle down on features
#	../bullet2/BasicDemo/HingeDemo.cpp
#	../bullet2/BasicDemo/HingeDemo.h
#	../bullet2/ChainDemo/ChainDemo.cpp
#	../bullet2/ChainDemo/ChainDemo.h
#	../bullet2/RagdollDemo/RagdollDemo.cpp
#	../bullet2/RagdollDemo/RagdollDemo.h
	../bullet2/LuaDemo/LuaPhysicsSetup.cpp
	../bullet2/LuaDemo/LuaPhysicsSetup.h
	../GpuDemos/gwenUserInterface.cpp
	../GpuDemos/gwenUserInterface.h
	../ImportURDFDemo/ImportURDFSetup.cpp
	../ImportURDFDemo/ImportURDFSetup.h
	../ImportObjDemo/ImportObjSetup.cpp
	 	../ImportSTLDemo/ImportSTLSetup.cpp
		../Wavefront/tiny_obj_loader.cpp
		../Wavefront/tiny_obj_loader.h
	../../btgui/Timing/b3Clock.cpp
	../../btgui/Timing/b3Clock.h
        ../../btgui/urdf/urdfdom/urdf_parser/src/pose.cpp
                        ../../btgui/urdf/urdfdom/urdf_parser/src/model.cpp
                        ../../btgui/urdf/urdfdom/urdf_parser/src/link.cpp
                        ../../btgui/urdf/urdfdom/urdf_parser/src/joint.cpp
                        ../../btgui/urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h
                        ../../btgui/urdf/urdfdom_headers/urdf_exception/include/urdf_exception/exception.h
                        ../../btgui/urdf/urdfdom_headers/urdf_model/include/urdf_model/pose.h
                        ../../btgui/urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h
                        ../../btgui/urdf/urdfdom_headers/urdf_model/include/urdf_model/link.h
                        ../../btgui/urdf/urdfdom_headers/urdf_model/include/urdf_model/joint.h
                        ../../btgui/tinyxml/tinystr.cpp
                        ../../btgui/tinyxml/tinyxml.cpp
                        ../../btgui/tinyxml/tinyxmlerror.cpp
                        ../../btgui/tinyxml/tinyxmlparser.cpp
                        ../../btgui/urdf/boost_replacement/lexical_cast.h
                        ../../btgui/urdf/boost_replacement/shared_ptr.h
                        ../../btgui/urdf/boost_replacement/printf_console.cpp
                        ../../btgui/urdf/boost_replacement/printf_console.h
                        ../../btgui/urdf/boost_replacement/string_split.cpp
                        ../../btgui/urdf/boost_replacement/string_split.h
	${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
)

LINK_LIBRARIES(
        lua-5.2.3 Bullet3Common BulletSoftBody BulletDynamics BulletCollision LinearMath OpenGLWindow gwen ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)

IF (WIN32)
	SET(App_AllBullet2Demos_SRCS ${App_AllBullet2Demos_SRCS}  ${App_AllBullet2Demos_Common_SRCS})
	INCLUDE_DIRECTORIES(
				${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
	)
	ADD_DEFINITIONS(-DGLEW_STATIC)
ELSE(WIN32)
	IF(APPLE)
                find_library(COCOA NAMES Cocoa)
                MESSAGE(${COCOA})
                link_libraries(${COCOA})
        ELSE(APPLE)
		ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
		ADD_DEFINITIONS("-DGLEW_STATIC")
	        ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")
		INCLUDE_DIRECTORIES(
                                ${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
	        )
		LINK_LIBRARIES( pthread dl)
	ENDIF(APPLE)
ENDIF(WIN32)
 
	


ADD_EXECUTABLE(App_AllBullet2Demos
		${App_AllBullet2Demos_SRCS}
)
	

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(App_AllBullet2Demos PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(App_AllBullet2Demos PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(App_AllBullet2Demos PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
