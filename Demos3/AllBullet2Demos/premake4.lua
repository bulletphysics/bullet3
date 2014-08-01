	
		project "App_AllBullet2Demos"

		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../bin"

  	includedirs {
                ".",
                "../../src",
                "../../btgui",
                "../../btgui/lua-5.2.3/src"
                }

			
		links{"gwen", "OpenGL_Window","OpenGL_TrueTypeFont","BulletSoftBody","BulletDynamics","BulletCollision","LinearMath","lua-5.2.3"}
		initOpenGL()
		initGlew()
	
		files {
		"**.cpp",
		"**.h",
		"../bullet2/BasicDemo/Bullet2RigidBodyDemo.cpp",
		"../bullet2/BasicDemo/Bullet2RigidBodyDemo.h",
		"../../Demos/BasicDemo/BasicDemoPhysicsSetup.cpp",
		"../../Demos/BasicDemo/BasicDemoPhysicsSetup.h",
		"../../Demos/CcdPhysicsDemo/CcdPhysicsSetup.cpp",
		"../../Demos/CcdPhysicsDemo/CcdPhysicsSetup.h",
		"../../Demos/ConstraintDemo/ConstraintPhysicsSetup.cpp",
  		"../../Demos/ConstraintDemo/ConstraintPhysicsSetup.h",
		"../ImportURDFDemo/ImportURDFSetup.cpp",
		"../../btgui/urdf/urdfdom/urdf_parser/src/pose.cpp",
		"../../btgui/urdf/urdfdom/urdf_parser/src/model.cpp",
                "../../btgui/urdf/urdfdom/urdf_parser/src/link.cpp",
                "../../btgui/urdf/urdfdom/urdf_parser/src/joint.cpp",
                "../../btgui/urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h",
                "../../btgui/urdf/urdfdom_headers/urdf_exception/include/urdf_exception/exception.h",
                "../../btgui/urdf/urdfdom_headers/urdf_model/include/urdf_model/pose.h",
                "../../btgui/urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h",
                "../../btgui/urdf/urdfdom_headers/urdf_model/include/urdf_model/link.h",
                "../../btgui/urdf/urdfdom_headers/urdf_model/include/urdf_model/joint.h",
                "../../btgui/tinyxml/tinystr.cpp",
                "../../btgui/tinyxml/tinyxml.cpp",
                "../../btgui/tinyxml/tinyxmlerror.cpp",
                "../../btgui/tinyxml/tinyxmlparser.cpp",
                "../../btgui/urdf/boost_replacement/lexical_cast.h",
                "../../btgui/urdf/boost_replacement/shared_ptr.h",
                "../../btgui/urdf/boost_replacement/printf_console.cpp",
                "../../btgui/urdf/boost_replacement/printf_console.h",
                "../../btgui/urdf/boost_replacement/string_split.cpp",
                "../../btgui/urdf/boost_replacement/string_split.h",
		"../bullet2/FeatherstoneMultiBodyDemo/BulletMultiBodyDemos.cpp",
		"../bullet2/FeatherstoneMultiBodyDemo/BulletMultiBodyDemos.h",
		"../bullet2/FeatherstoneMultiBodyDemo/MultiDofDemo.cpp",
		"../bullet2/FeatherstoneMultiBodyDemo/MultiDofDemo.h",
		"../bullet2/BasicDemo/BasicDemo.cpp",
		"../bullet2/BasicDemo/BasicDemo.h",
--		"../bullet2/BasicDemo/HingeDemo.cpp",
--		"../bullet2/BasicDemo/HingeDemo.h",
--		"../bullet2/ChainDemo/ChainDemo.cpp",
--		"../bullet2/ChainDemo/ChainDemo.h",
		
--		"../bullet2/RagdollDemo/RagdollDemo.cpp",
--		"../bullet2/RagdollDemo/RagdollDemo.h",
--		"../bullet2/LuaDemo/LuaDemo.cpp",
--		"../bullet2/LuaDemo/LuaDemo.h",
		
		
		"../../src/Bullet3Common/**.cpp",
 		"../../src/Bullet3Common/**.h",
		"../../btgui/Timing/b3Clock.cpp",
		"../../btgui/Timing/b3Clock.h",
		"../GpuDemos/gwenUserInterface.cpp",
		"../GpuDemos/gwenUserInterface.h"
		}

if os.is("Linux") then links{"X11","pthread"} end
if os.is("MacOSX") then
                        links{"Cocoa.framework"}
end
