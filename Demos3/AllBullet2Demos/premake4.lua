	
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

		initOpenGL()
		initGlew()
			
		links{"gwen", "OpenGL_Window","OpenGL_TrueTypeFont","BulletSoftBody","BulletDynamics","BulletCollision","LinearMath","lua-5.2.3"}
		
		files {
		"**.cpp",
		"**.h",
		"../bullet2/BasicDemo/Bullet2RigidBodyDemo.cpp",
		"../bullet2/BasicDemo/Bullet2RigidBodyDemo.h",
		"../bullet2/FeatherstoneMultiBodyDemo/BulletMultiBodyDemos.cpp",
		"../bullet2/FeatherstoneMultiBodyDemo/BulletMultiBodyDemos.h",
		"../bullet2/FeatherstoneMultiBodyDemo/MultiDofDemo.cpp",
		"../bullet2/FeatherstoneMultiBodyDemo/MultiDofDemo.h",
		"../bullet2/BasicDemo/BasicDemo.cpp",
		"../bullet2/BasicDemo/BasicDemo.h",
		"../bullet2/BasicDemo/HingeDemo.cpp",
		"../bullet2/BasicDemo/HingeDemo.h",
		"../bullet2/ChainDemo/ChainDemo.cpp",
		"../bullet2/ChainDemo/ChainDemo.h",
		
		"../bullet2/RagdollDemo/RagdollDemo.cpp",
		"../bullet2/RagdollDemo/RagdollDemo.h",
		"../bullet2/LuaDemo/LuaDemo.cpp",
		"../bullet2/LuaDemo/LuaDemo.h",
		
		
		"../../src/Bullet3Common/**.cpp",
 		"../../src/Bullet3Common/**.h",
		"../../btgui/Timing/b3Clock.cpp",
		"../../btgui/Timing/b3Clock.h",
		"../GpuDemos/gwenUserInterface.cpp",
		"../GpuDemos/gwenUserInterface.h"
		}

if os.is("Linux") then links{"X11"} end
if os.is("MacOSX") then
                        links{"Cocoa.framework"}
end
