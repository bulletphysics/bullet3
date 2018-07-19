
project "App_PhysicsServer_SharedMemory"

if _OPTIONS["ios"] then
	kind "WindowedApp"
else	
	kind "ConsoleApp"
end

includedirs {".","../../src", "../ThirdPartyLibs"}

links {
	"BulletSoftBody", "Bullet3Common","BulletInverseDynamicsUtils", "BulletInverseDynamics",	"BulletDynamics","BulletCollision", "LinearMath", "BussIK"
}
if os.is("Linux") then
    links{"dl"}
end

language "C++"

myfiles = 
{
	"b3RobotSimulatorClientAPI_NoDirect.cpp",
	"b3RobotSimulatorClientAPI_NoDirect.h",
	"IKTrajectoryHelper.cpp",
	"IKTrajectoryHelper.h",
	"PhysicsClient.cpp",
	"PhysicsClientSharedMemory.cpp",
	"PhysicsClientExample.cpp",
	"PhysicsServerExample.cpp",
	"PhysicsServerExampleBullet2.cpp",
	"PhysicsServerSharedMemory.cpp",
	"PhysicsServerSharedMemory.h",
	"PhysicsServer.cpp",
	"PhysicsServer.h",
	"PhysicsClientC_API.cpp",
	"SharedMemoryCommands.h",
	"SharedMemoryPublic.h",
	"PhysicsServer.cpp",
	"PosixSharedMemory.cpp",
	"Win32SharedMemory.cpp",
	"InProcessMemory.cpp",
	"PhysicsDirect.cpp",
	"PhysicsDirect.h",
	"PhysicsDirectC_API.cpp",
	"PhysicsDirectC_API.h",
	"PhysicsLoopBack.cpp",
	"PhysicsLoopBack.h",
	"PhysicsLoopBackC_API.cpp",
	"PhysicsLoopBackC_API.h",
	"PhysicsClientSharedMemory_C_API.cpp",
	"PhysicsClientSharedMemory_C_API.h",
	"PhysicsClientSharedMemory2_C_API.cpp",
	"PhysicsClientSharedMemory2_C_API.h",
	"PhysicsClientSharedMemory2.cpp",
	"PhysicsClientSharedMemory2.h",
	"SharedMemoryCommandProcessor.cpp",
	"SharedMemoryCommandProcessor.h",
	"PhysicsServerCommandProcessor.cpp",
	"PhysicsServerCommandProcessor.h",
	"b3PluginManager.cpp",
	"b3PluginManager.h",
	"plugins/pdControlPlugin/pdControlPlugin.cpp",
	"plugins/pdControlPlugin/pdControlPlugin.h",
	"../OpenGLWindow/SimpleCamera.cpp",
	"../OpenGLWindow/SimpleCamera.h",
	"../Importers/ImportURDFDemo/ConvertRigidBodies2MultiBody.h",
	"../Importers/ImportURDFDemo/MultiBodyCreationInterface.h",
	"../Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
	"../Importers/ImportURDFDemo/MyMultiBodyCreator.h",
	"../Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
	"../Importers/ImportURDFDemo/BulletUrdfImporter.h",
	"../Importers/ImportURDFDemo/UrdfParser.cpp",
 	"../Importers/ImportURDFDemo/urdfStringSplit.cpp",
	"../Importers/ImportURDFDemo/UrdfParser.cpp",
	"../Importers/ImportURDFDemo/UrdfParser.h",
	"../Importers/ImportURDFDemo/URDF2Bullet.cpp",
	"../Importers/ImportURDFDemo/URDF2Bullet.h",
	"../Importers/ImportMJCFDemo/BulletMJCFImporter.cpp",
	"../Importers/ImportMJCFDemo/BulletMJCFImporter.h",
	"../Utils/b3ResourcePath.cpp",
	"../Utils/b3Clock.cpp",
	"../Utils/RobotLoggingUtil.cpp",
	"../Utils/RobotLoggingUtil.h",
	"../Utils/ChromeTraceUtil.cpp",
	"../Utils/ChromeTraceUtil.h",
	"../../Extras/Serialize/BulletWorldImporter/*",
	"../../Extras/Serialize/BulletFileLoader/*",	
	"../Importers/ImportURDFDemo/URDFImporterInterface.h",
	"../Importers/ImportURDFDemo/URDFJointTypes.h",
	"../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
	"../Importers/ImportObjDemo/LoadMeshFromObj.cpp",
	"../Importers/ImportSTLDemo/ImportSTLSetup.h",
	"../Importers/ImportSTLDemo/LoadMeshFromSTL.h",
	"../Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
	"../Importers/ImportColladaDemo/ColladaGraphicsInstance.h",
	"../ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",	
	"../ThirdPartyLibs/tinyxml2/tinyxml2.cpp",
	"../Importers/ImportMeshUtility/b3ImportMeshUtility.cpp",
	"../ThirdPartyLibs/stb_image/stb_image.cpp",     

}


files {
	myfiles,
	"main.cpp",
}

if (_OPTIONS["enable_static_vr_plugin"]) then
	defines("STATIC_LINK_VR_PLUGIN")
	files {"plugins/vrSyncPlugin/vrSyncPlugin.cpp"}
end

if (not _OPTIONS["disable_static_tinyrenderer_plugin"]) then
	files 
		{
		"plugins/tinyRendererPlugin/tinyRendererPlugin.cpp",
		"plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp",
		"../TinyRenderer/geometry.cpp",
		"../TinyRenderer/model.cpp",
		"../TinyRenderer/tgaimage.cpp",
		"../TinyRenderer/our_gl.cpp",
		"../TinyRenderer/TinyRenderer.cpp"
		}
else
	defines("SKIP_STATIC_TINYRENDERER_PLUGIN")
end

files {
		"../MultiThreading/b3ThreadSupportInterface.cpp",
		"../MultiThreading/b3ThreadSupportInterface.h"
	}
	if os.is("Windows") then

		files {
                "../MultiThreading/b3Win32ThreadSupport.cpp",  
                "../MultiThreading/b3Win32ThreadSupport.h" 
		}
		--links {"winmm"}
		--defines {"__WINDOWS_MM__", "WIN32"}
	end

	if os.is("Linux") then 
		files {
                "../MultiThreading/b3PosixThreadSupport.cpp",  
                "../MultiThreading/b3PosixThreadSupport.h"    
        	}

		links {"pthread"}
	end

	if os.is("MacOSX") then
		files {
                "../MultiThreading/b3PosixThreadSupport.cpp",
                "../MultiThreading/b3PosixThreadSupport.h"    
                }

		links {"pthread"}
		--links{"CoreAudio.framework", "coreMIDI.framework", "Cocoa.framework"}
		--defines {"__MACOSX_CORE__"}
	end


project "App_PhysicsServer_SharedMemory_GUI"

if _OPTIONS["ios"] then
        kind "WindowedApp"
else
        kind "ConsoleApp"
end
defines {"B3_USE_STANDALONE_EXAMPLE"}

includedirs {"../../src", "../ThirdPartyLibs"}

links {
       "BulletSoftBody",  "BulletInverseDynamicsUtils", "BulletInverseDynamics", "BulletDynamics","BulletCollision", "LinearMath", "OpenGL_Window","Bullet3Common","BussIK"
}
	initOpenGL()
  initGlew()

language "C++"

	if _OPTIONS["midi"] then
	
		defines {"B3_USE_MIDI"}

			
					
			 includedirs{"../ThirdPartyLibs/midi"}
			
				 files {
	        	"../ThirdPartyLibs/midi/RtMidi.cpp",
	        	"../ThirdPartyLibs/midi/RtMidi.h",
	        	"../ThirdPartyLibs/midi/RtError.h",
        	} 
			if os.is("Windows") then
				links {"winmm"}
				defines {"__WINDOWS_MM__", "WIN32"}
			end

			if os.is("Linux") then 
				defines {"__LINUX_ALSA__"}
			  links {"asound","pthread"}
			end

			if os.is("MacOSX") then
				links{"CoreAudio.framework", "coreMIDI.framework", "Cocoa.framework"}
				defines {"__MACOSX_CORE__"}
			end
		
	end

if (not _OPTIONS["disable_static_tinyrenderer_plugin"]) then
	files 
		{
		"plugins/tinyRendererPlugin/tinyRendererPlugin.cpp",
		"plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp",
		"../TinyRenderer/geometry.cpp",
		"../TinyRenderer/model.cpp",
		"../TinyRenderer/tgaimage.cpp",
		"../TinyRenderer/our_gl.cpp",
		"../TinyRenderer/TinyRenderer.cpp"
		}
else
	defines("SKIP_STATIC_TINYRENDERER_PLUGIN")
end


files {
        myfiles,
        "../StandaloneMain/main_opengl_single_example.cpp",
				"../ExampleBrowser/OpenGLGuiHelper.cpp",
				"../ExampleBrowser/GL_ShapeDrawer.cpp",
				"../ExampleBrowser/CollisionShape2TriangleMesh.cpp",
}
if (_OPTIONS["enable_static_vr_plugin"]) then
	defines("STATIC_LINK_VR_PLUGIN")
	files {"plugins/vrSyncPlugin/vrSyncPlugin.cpp"}
end


if os.is("Linux") then initX11() end

if os.is("MacOSX") then
        links{"Cocoa.framework"}
end


files {
		"../MultiThreading/b3ThreadSupportInterface.cpp",
		"../MultiThreading/b3ThreadSupportInterface.h"
	}
if os.is("Windows") then

	files {
              "../MultiThreading/b3Win32ThreadSupport.cpp",  
              "../MultiThreading/b3Win32ThreadSupport.h" 
	}
	--links {"winmm"}
	--defines {"__WINDOWS_MM__", "WIN32"}
end

if os.is("Linux") then 
	files {
              "../MultiThreading/b3PosixThreadSupport.cpp",  
              "../MultiThreading/b3PosixThreadSupport.h"    
      	}

	links {"pthread"}
end

if os.is("MacOSX") then
	files {
              "../MultiThreading/b3PosixThreadSupport.cpp",
              "../MultiThreading/b3PosixThreadSupport.h"    
              }

	links {"pthread"}
	--links{"CoreAudio.framework", "coreMIDI.framework", "Cocoa.framework"}
	--defines {"__MACOSX_CORE__"}
end



if os.is("Windows") then 
	project "App_PhysicsServer_SharedMemory_VR"
	--for now, only enable VR under Windows, until compilation issues are resolved on Mac/Linux
	defines {"B3_USE_STANDALONE_EXAMPLE","BT_ENABLE_VR"}
	
	if _OPTIONS["ios"] then
		kind "WindowedApp"
	else	
		kind "ConsoleApp"
	end

	if _OPTIONS["midi"] then
	
		defines {"B3_USE_MIDI"}

			
					
			 includedirs{"../ThirdPartyLibs/midi"}
			
				 files {
	        	"../ThirdPartyLibs/midi/RtMidi.cpp",
	        	"../ThirdPartyLibs/midi/RtMidi.h",
	        	"../ThirdPartyLibs/midi/RtError.h",
        	} 
			if os.is("Windows") then
				links {"winmm"}
				defines {"__WINDOWS_MM__", "WIN32"}
			end

			if os.is("Linux") then 
				defines {"__LINUX_ALSA__"}
			  links {"asound","pthread"}
			end

			if os.is("MacOSX") then
				links{"CoreAudio.framework", "coreMIDI.framework", "Cocoa.framework"}
				defines {"__MACOSX_CORE__"}
			end
		
	end
	if _OPTIONS["audio"] then
			files {
				"../TinyAudio/b3ADSR.cpp",
				"../TinyAudio/b3AudioListener.cpp",
				"../TinyAudio/b3ReadWavFile.cpp",
				"../TinyAudio/b3SoundEngine.cpp",
				"../TinyAudio/b3SoundSource.cpp",
				"../TinyAudio/b3WriteWavFile.cpp",
				"../TinyAudio/RtAudio.cpp",
			}
			
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
	includedirs {
			".","../../src", "../ThirdPartyLibs",
			"../ThirdPartyLibs/openvr/headers",
			"../ThirdPartyLibs/openvr/samples/shared"
		}
						
	links {
		"BulletSoftBody", "BulletInverseDynamicsUtils", "BulletInverseDynamics","Bullet3Common",	"BulletDynamics","BulletCollision", "LinearMath","OpenGL_Window","openvr_api","BussIK"
	}
	
	
	language "C++"
	
	
		initOpenGL()
	  initGlew()
	
	if (not _OPTIONS["disable_static_tinyrenderer_plugin"]) then
	files 
		{
		"plugins/tinyRendererPlugin/tinyRendererPlugin.cpp",
		"plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp",
		"../TinyRenderer/geometry.cpp",
		"../TinyRenderer/model.cpp",
		"../TinyRenderer/tgaimage.cpp",
		"../TinyRenderer/our_gl.cpp",
		"../TinyRenderer/TinyRenderer.cpp"
		}
else
	defines("SKIP_STATIC_TINYRENDERER_PLUGIN")
end


	files
	{
		myfiles,
		 "../StandaloneMain/hellovr_opengl_main.cpp",
					"../ExampleBrowser/OpenGLGuiHelper.cpp",
					"../ExampleBrowser/GL_ShapeDrawer.cpp",
					"../ExampleBrowser/CollisionShape2TriangleMesh.cpp",
					"../RenderingExamples/TinyVRGui.cpp",
					"../RenderingExamples/TimeSeriesCanvas.cpp",
					"../RenderingExamples/TimeSeriesFontData.cpp",
					"../ThirdPartyLibs/openvr/samples/shared/lodepng.cpp",
					"../ThirdPartyLibs/openvr/samples/shared/lodepng.h",
					"../ThirdPartyLibs/openvr/samples/shared/Matrices.cpp",
					"../ThirdPartyLibs/openvr/samples/shared/Matrices.h",
					"../ThirdPartyLibs/openvr/samples/shared/strtools.cpp",
					"../ThirdPartyLibs/openvr/samples/shared/pathtools.cpp",
					"../ThirdPartyLibs/openvr/samples/shared/pathtools.h",
					"../ThirdPartyLibs/openvr/samples/shared/Vectors.h",
	}
if (_OPTIONS["enable_static_vr_plugin"]) then
	defines("STATIC_LINK_VR_PLUGIN")
	files {"plugins/vrSyncPlugin/vrSyncPlugin.cpp"}
end

	if os.is("Windows") then 
		configuration {"x32"}
			libdirs {"../ThirdPartyLibs/openvr/lib/win32"}
		configuration {"x64"}
			libdirs {"../ThirdPartyLibs/openvr/lib/win64"}
		configuration{}
	end
	
	if os.is("Linux") then initX11() end
	
	if os.is("MacOSX") then
	        links{"Cocoa.framework"}
	end
	
	
	files {
			"../MultiThreading/b3ThreadSupportInterface.cpp",
			"../MultiThreading/b3ThreadSupportInterface.h"
		}
		if os.is("Windows") then
	
			files {
	                "../MultiThreading/b3Win32ThreadSupport.cpp",  
	                "../MultiThreading/b3Win32ThreadSupport.h" 
			}
			--links {"winmm"}
			--defines {"__WINDOWS_MM__", "WIN32"}
		end
	
		if os.is("Linux") then 
			files {
	                "../MultiThreading/b3PosixThreadSupport.cpp",  
	                "../MultiThreading/b3PosixThreadSupport.h"    
	        	}
	
			links {"pthread"}
		end
	
		if os.is("MacOSX") then
			files {
	                "../MultiThreading/b3PosixThreadSupport.cpp",
	                "../MultiThreading/b3PosixThreadSupport.h"    
	                }
	
			links {"pthread"}
			--links{"CoreAudio.framework", "coreMIDI.framework", "Cocoa.framework"}
			--defines {"__MACOSX_CORE__"}
		end


end


include "udp"
include "tcp"
include "plugins/testPlugin"
include "plugins/vrSyncPlugin"
include "plugins/tinyRendererPlugin"

include "plugins/pdControlPlugin"
include "plugins/collisionFilterPlugin"


