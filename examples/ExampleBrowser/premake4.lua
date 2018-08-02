project "App_BulletExampleBrowser"

        language "C++"

        kind "ConsoleApp"
        
        if os.is("Linux") then
	        buildoptions{"-fPIC"}
	    end
        
        hasCL = findOpenCL("clew")

        if (hasCL) then
            initOpenCL("clew")
        end

        links{"BulletExampleBrowserLib","gwen", "OpenGL_Window","BulletSoftBody", "BulletInverseDynamicsUtils", "BulletInverseDynamics", "BulletDynamics","BulletCollision","LinearMath","BussIK", "Bullet3Common"}
        initOpenGL()
        initGlew()

        includedirs {
                ".",
                "../../src",
		"../../examples/SharedMemory",
                "../ThirdPartyLibs",
                }


        if os.is("MacOSX") then
                links{"Cocoa.framework"}
        end

                if (hasCL) then
                        links {
                                "Bullet3OpenCL_clew",
                                "Bullet3Dynamics",
                                "Bullet3Collision",
                                "Bullet3Geometry",
                                "Bullet3Common",
                        }
                end

	if _OPTIONS["audio"] then
			files {"../TinyAudio/*.cpp"}
			defines {"B3_ENABLE_TINY_AUDIO"}
			
			if os.is("Windows") then
				links {"winmm","Wsock32","dsound"}
				defines {"WIN32","__WINDOWS_MM__","__WINDOWS_DS__"}
			end
			
			if os.is("Linux") then initX11() 
			                defines  {"__OS_LINUX__","__LINUX_ALSA__"}
				links {"asound","pthread"}
			end


			if os.is("MacOSX") then
				links{"Cocoa.framework"}
				links{"CoreAudio.framework", "coreMIDI.framework", "Cocoa.framework"}
				defines {"__OS_MACOSX__","__MACOSX_CORE__"}
			end
		end
					
    if _OPTIONS["lua"] then
                includedirs{"../ThirdPartyLibs/lua-5.2.3/src"}
                links {"lua-5.2.3"}
                defines {"ENABLE_LUA"}
                files {"../LuaDemo/LuaPhysicsSetup.cpp"}
        end

	defines {"INCLUDE_CLOTH_DEMOS"}

        files {
        	
        "main.cpp",
        "ExampleEntries.cpp",
        "../InverseKinematics/*",
		"../TinyRenderer/geometry.cpp",
		"../TinyRenderer/model.cpp",
		"../TinyRenderer/tgaimage.cpp",
		"../TinyRenderer/our_gl.cpp",
		"../TinyRenderer/TinyRenderer.cpp",
		"../SharedMemory/IKTrajectoryHelper.cpp",
		"../SharedMemory/IKTrajectoryHelper.h",
		"../SharedMemory/PhysicsClientC_API.cpp",
		"../SharedMemory/PhysicsClientC_API.h",
		"../SharedMemory/PhysicsServerExample.cpp",
		"../SharedMemory/PhysicsServerExampleBullet2.cpp",
		"../SharedMemory/PhysicsClientExample.cpp",
		"../SharedMemory/PhysicsServer.cpp",
		"../SharedMemory/PhysicsServerSharedMemory.cpp",
		"../SharedMemory/PhysicsClientSharedMemory.cpp",
		"../SharedMemory/PhysicsClientSharedMemory_C_API.cpp",
		"../SharedMemory/PhysicsClientSharedMemory_C_API.h",
		"../SharedMemory/PhysicsClientSharedMemory2.cpp",
		"../SharedMemory/PhysicsClientSharedMemory2.h",
		"../SharedMemory/PhysicsClientSharedMemory2_C_API.cpp",
		"../SharedMemory/PhysicsClientSharedMemory2_C_API.h",
		"../SharedMemory/SharedMemoryCommandProcessor.cpp",
		"../SharedMemory/SharedMemoryCommandProcessor.h",
		"../SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp",
		"../SharedMemory/PhysicsClient.cpp",
		"../SharedMemory/PosixSharedMemory.cpp",
		"../SharedMemory/Win32SharedMemory.cpp",
		"../SharedMemory/InProcessMemory.cpp",
		"../SharedMemory/PhysicsDirect.cpp",
		"../SharedMemory/PhysicsDirect.h",
		"../SharedMemory/PhysicsDirectC_API.cpp",
		"../SharedMemory/PhysicsDirectC_API.h",
		"../SharedMemory/PhysicsLoopBack.cpp",
		"../SharedMemory/PhysicsLoopBack.h",
		"../SharedMemory/PhysicsLoopBackC_API.cpp",
		"../SharedMemory/PhysicsLoopBackC_API.h",
		"../SharedMemory/PhysicsServerCommandProcessor.cpp",
		"../SharedMemory/PhysicsServerCommandProcessor.h",
		"../SharedMemory/b3PluginManager.cpp",		
		"../SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp",
		"../SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.cpp",
		"../SharedMemory/plugins/pdControlPlugin/pdControlPlugin.cpp",
		"../SharedMemory/plugins/pdControlPlugin/pdControlPlugin.h",
		"../SharedMemory/SharedMemoryCommands.h",
		"../SharedMemory/SharedMemoryPublic.h",
		"../SharedMemory/b3RobotSimulatorClientAPI_NoGUI.cpp",
		"../SharedMemory/b3RobotSimulatorClientAPI_NoGUI.h",		
		"../SharedMemory/b3RobotSimulatorClientAPI_NoDirect.cpp",
		"../SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h",		
		"../MultiThreading/MultiThreadingExample.cpp",
		"../MultiThreading/b3PosixThreadSupport.cpp",
		"../MultiThreading/b3Win32ThreadSupport.cpp",
		"../MultiThreading/b3ThreadSupportInterface.cpp",
		"../InverseDynamics/InverseDynamicsExample.cpp",
		"../InverseDynamics/InverseDynamicsExample.h",
		"../RobotSimulator/b3RobotSimulatorClientAPI.cpp",
		"../RobotSimulator/b3RobotSimulatorClientAPI.h",		
		"../BasicDemo/BasicExample.*",
		"../Tutorial/*",
		"../ExtendedTutorials/*",
		"../Utils/RobotLoggingUtil.cpp",
		"../Utils/RobotLoggingUtil.h",
		"../Evolution/NN3DWalkers.cpp",
		"../Evolution/NN3DWalkers.h",
		"../Collision/*",
		"../RoboticsLearning/*",
		"../Collision/Internal/*",
		"../Benchmarks/*",
		"../MultiThreadedDemo/*",
		"../CommonInterfaces/*.h",
		"../ForkLift/ForkLiftDemo.*",
		"../Importers/**",
		"../../Extras/Serialize/BulletWorldImporter/*",
		"../../Extras/Serialize/BulletFileLoader/*",	
		"../Planar2D/Planar2D.*",
		"../RenderingExamples/*",
		"../VoronoiFracture/*",
		"../SoftDemo/*",
		"../RollingFrictionDemo/*",
		"../FractureDemo/*",
		"../DynamicControlDemo/*",
		"../Constraints/*",
		"../Vehicles/*",
		"../Raycast/*",
		"../MultiBody/MultiDofDemo.cpp",
		"../MultiBody/TestJointTorqueSetup.cpp",
		"../MultiBody/Pendulum.cpp",
		"../MultiBody/MultiBodySoftContact.cpp",
		"../MultiBody/MultiBodyConstraintFeedback.cpp",
		"../MultiBody/InvertedPendulumPDControl.cpp",
		"../RigidBody/RigidBodySoftContact.cpp",
		"../ThirdPartyLibs/stb_image/*",
		"../ThirdPartyLibs/Wavefront/tiny_obj_loader.*",
		"../ThirdPartyLibs/BussIK/*",
		"../GyroscopicDemo/GyroscopicSetup.cpp",
		"../GyroscopicDemo/GyroscopicSetup.h",
    "../ThirdPartyLibs/tinyxml2/tinyxml2.cpp",
    "../ThirdPartyLibs/tinyxml2/tinyxml2.h",
        }
if (hasCL and findOpenGL3()) then
			files {
				"../OpenCL/broadphase/*",
				"../OpenCL/CommonOpenCL/*",
				"../OpenCL/rigidbody/GpuConvexScene.cpp",
				"../OpenCL/rigidbody/GpuRigidBodyDemo.cpp",
			}
		end
		
if (_OPTIONS["enable_static_vr_plugin"]) then
		files {"../../examples/SharedMemory/plugins/vrSyncPlugin/vrSyncPlugin.cpp"}
end

if os.is("Linux") then
        initX11()
end


	
project "BulletExampleBrowserLib"

		hasCL = findOpenCL("clew")
	
		if (hasCL) then

				-- project ("App_Bullet3_OpenCL_Demos_" .. vendor)

				initOpenCL("clew")

		end

		language "C++"
				
		kind "StaticLib"

  	includedirs {
                ".",
                "../../src",
                "../ThirdPartyLibs",
                }
                
        if os.is("Linux") then
            buildoptions{"-fPIC"}
        end

	if _OPTIONS["lua"] then
		includedirs{"../ThirdPartyLibs/lua-5.2.3/src"}
		links {"lua-5.2.3"}
		defines {"ENABLE_LUA"}
		files {"../LuaDemo/LuaPhysicsSetup.cpp"}
	end

	
	
			
		initOpenGL()
		initGlew()

		defines {"INCLUDE_CLOTH_DEMOS"}
			


		files {
		"OpenGLExampleBrowser.cpp",
		"OpenGLGuiHelper.cpp",
		"OpenGLExampleBrowser.cpp",
		"../Utils/b3Clock.cpp",
		"../Utils/b3Clock.h",
		"../Utils/ChromeTraceUtil.cpp",
		"../Utils/ChromeTraceUtil.h",
		"*.h",
		"GwenGUISupport/*.cpp",
		"GwenGUISupport/*.h",
		"CollisionShape2TriangleMesh.cpp",
		"CollisionShape2TriangleMesh.h",
		"../Utils/b3ResourcePath.*",
		"GL_ShapeDrawer.cpp",
		"InProcessExampleBrowser.cpp",
	
   

		}
		
		

if os.is("Linux") then 
	initX11()
end

			

