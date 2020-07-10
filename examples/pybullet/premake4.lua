		

project ("pybullet")
		language "C++"
		kind "SharedLib"

		if _OPTIONS["enable_grpc"] then
				initGRPC()
				
				 files {
                  "../../examples/SharedMemory/PhysicsClientGRPC.cpp",
                  "../../examples/SharedMemory/PhysicsClientGRPC.h",
                  "../../examples/SharedMemory/PhysicsClientGRPC_C_API.cpp",
                  "../../examples/SharedMemory/PhysicsClientGRPC_C_API.h",
                }
		end
		
		includedirs {"../../src", "../../examples",
		"../../examples/ThirdPartyLibs",
		"../../Extras/VHACD/inc", "../../Extras/VHACD/public",
		}
		defines {"BT_ENABLE_VHACD"}
		
		defines {"PHYSICS_IN_PROCESS_EXAMPLE_BROWSER"}
		files 
		{
			"../../Extras/VHACD/test/src/main_vhacd.cpp",
			"../../Extras/VHACD/src/VHACD.cpp",
			"../../Extras/VHACD/src/vhacdICHull.cpp",
			"../../Extras/VHACD/src/vhacdManifoldMesh.cpp",
			"../../Extras/VHACD/src/vhacdMesh.cpp",
			"../../Extras/VHACD/src/vhacdVolume.cpp",
		}
		
		
	hasCL = findOpenCL("clew")

	links{ "BulletExampleBrowserLib","gwen", "BulletFileLoader","BulletWorldImporter","OpenGL_Window","BulletSoftBody", "BulletInverseDynamicsUtils", "BulletInverseDynamics", "BulletDynamics","BulletCollision","LinearMath","BussIK", "Bullet3Common"}
	initOpenGL()
	initGlew()

  	includedirs {
                ".",
                "../../src",
                "../ThirdPartyLibs",
                }

	if os.is("MacOSX") then
--		targetextension {"so"}
		links{"Cocoa.framework","Python"}
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

if not _OPTIONS["no-enet"] then

		includedirs {"../../examples/ThirdPartyLibs/enet/include"}
	
		if os.is("Windows") then 
--			targetextension {"dylib"}
			defines { "WIN32" }
			links {"Ws2_32","Winmm"}
		end
		if os.is("Linux") then
		end
		if os.is("MacOSX") then
		end		
		
		links {"enet"}		

		files {
			"../../examples/SharedMemory/PhysicsClientUDP.cpp",
			"../../examples/SharedMemory/PhysicsClientUDP.h",
			"../../examples/SharedMemory/PhysicsClientUDP_C_API.cpp",
			"../../examples/SharedMemory/PhysicsClientUDP_C_API.h",
		}	
		defines {"BT_ENABLE_ENET"}
	end

	if not _OPTIONS["no-clsocket"] then

                includedirs {"../../examples/ThirdPartyLibs/clsocket/src"}

		 if os.is("Windows") then
                	defines { "WIN32" }
                	links {"Ws2_32","Winmm"}
       		 end
        	if os.is("Linux") then
                	defines {"_LINUX"}
        	end
        	if os.is("MacOSX") then
                	defines {"_DARWIN"}
        	end

                links {"clsocket"}

                files {
                        "../../examples/SharedMemory/PhysicsClientTCP.cpp",
                        "../../examples/SharedMemory/PhysicsClientTCP.h",
                        "../../examples/SharedMemory/PhysicsClientTCP_C_API.cpp",
                        "../../examples/SharedMemory/PhysicsClientTCP_C_API.h",
                }
                defines {"BT_ENABLE_CLSOCKET"}
        end


		files {
			"pybullet.c",
			"../../examples/SharedMemory/IKTrajectoryHelper.cpp",
			"../../examples/SharedMemory/IKTrajectoryHelper.h",
			"../../examples/ExampleBrowser/InProcessExampleBrowser.cpp",
			"../../examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.cpp",
			"../../examples/SharedMemory/plugins/tinyRendererPlugin/tinyRendererPlugin.h",
			"../../examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp",
			"../../examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.h",
			"../../examples/TinyRenderer/geometry.cpp",
			"../../examples/TinyRenderer/model.cpp",
			"../../examples/TinyRenderer/tgaimage.cpp",
			"../../examples/TinyRenderer/our_gl.cpp",
			"../../examples/TinyRenderer/TinyRenderer.cpp",
			"../../examples/SharedMemory/InProcessMemory.cpp",
			"../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.cpp",
			"../../examples/SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h",
			"../../examples/SharedMemory/PhysicsClient.cpp",
			"../../examples/SharedMemory/PhysicsClient.h",
			"../../examples/SharedMemory/PhysicsServer.cpp",
			"../../examples/SharedMemory/PhysicsServer.h",
			"../../examples/SharedMemory/PhysicsServerExample.cpp",
			"../../examples/SharedMemory/PhysicsServerExampleBullet2.cpp",
			"../SharedMemory/GraphicsClientExample.cpp",
      "../SharedMemory/GraphicsClientExample.h",
      "../SharedMemory/GraphicsServerExample.cpp",
    	"../SharedMemory/GraphicsServerExample.h",
 	   	"../SharedMemory/GraphicsSharedMemoryBlock.h",
   	 	"../SharedMemory/GraphicsSharedMemoryCommands.h",
    	"../SharedMemory/GraphicsSharedMemoryPublic.h",
    	"../SharedMemory/RemoteGUIHelper.cpp",
    	"../SharedMemory/RemoteGUIHelperTCP.cpp",
    	"../SharedMemory/RemoteGUIHelper.h",
			"../../examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp",
			"../../examples/SharedMemory/PhysicsServerSharedMemory.cpp",
			"../../examples/SharedMemory/PhysicsServerSharedMemory.h",
			"../../examples/SharedMemory/PhysicsDirect.cpp",
			"../../examples/SharedMemory/PhysicsDirect.h",
			"../../examples/SharedMemory/PhysicsDirectC_API.cpp",
			"../../examples/SharedMemory/PhysicsDirectC_API.h",
			"../../examples/SharedMemory/PhysicsServerCommandProcessor.cpp",
			"../../examples/SharedMemory/PhysicsServerCommandProcessor.h",
			"../../examples/SharedMemory/b3PluginManager.cpp",
			"../../examples/SharedMemory/b3PluginManager.h",
			"../../examples/SharedMemory/PhysicsClientSharedMemory.cpp",
			"../../examples/SharedMemory/PhysicsClientSharedMemory.h",
			"../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.cpp",
			"../../examples/SharedMemory/PhysicsClientSharedMemory_C_API.h",
			"../../examples/SharedMemory/PhysicsClientC_API.cpp",
			"../../examples/SharedMemory/PhysicsClientC_API.h",
			"../../examples/SharedMemory/Win32SharedMemory.cpp",
			"../../examples/SharedMemory/Win32SharedMemory.h",
			"../../examples/SharedMemory/PosixSharedMemory.cpp",
			"../../examples/SharedMemory/PosixSharedMemory.h",
			"../../examples/SharedMemory/SharedMemoryCommands.h",
			"../../examples/SharedMemory/SharedMemoryPublic.h",
			"../../examples/Utils/RobotLoggingUtil.cpp",
			"../../examples/Utils/RobotLoggingUtil.h",
			"../../examples/ThirdPartyLibs/tinyxml2/tinyxml2.cpp",
			"../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
			"../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.h",
			"../../examples/ThirdPartyLibs/stb_image/stb_image.cpp",
			"../../examples/ThirdPartyLibs/stb_image/stb_image_write.cpp",
			"../../examples/Importers/ImportColladaDemo/LoadMeshFromCollada.cpp",
			"../../examples/Importers/ImportObjDemo/LoadMeshFromObj.cpp",
			"../../examples/Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp",
			"../../examples/Importers/ImportMJCFDemo/BulletMJCFImporter.cpp",
			"../../examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp",
			"../../examples/Importers/ImportURDFDemo/MyMultiBodyCreator.cpp",
			"../../examples/Importers/ImportURDFDemo/URDF2Bullet.cpp",
			"../../examples/Importers/ImportURDFDemo/UrdfParser.cpp",
			"../../examples/Importers/ImportURDFDemo/urdfStringSplit.cpp",
			"../../examples/Importers/ImportMeshUtility/b3ImportMeshUtility.cpp",
			"../../examples/MultiThreading/b3PosixThreadSupport.cpp",
			"../../examples/MultiThreading/b3Win32ThreadSupport.cpp",
			"../../examples/MultiThreading/b3ThreadSupportInterface.cpp",
			"../../examples/SharedMemory/plugins/collisionFilterPlugin/collisionFilterPlugin.cpp",
			"../../examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.cpp",
			"../../examples/SharedMemory/plugins/pdControlPlugin/pdControlPlugin.h",
		}
		
	defines {"B3_ENABLE_FILEIO_PLUGIN", "B3_USE_ZIPFILE_FILEIO"}	
  files {
  	"../../examples/SharedMemory/plugins/fileIOPlugin/fileIOPlugin.cpp",
  	"../../examples/ThirdPartyLibs/minizip/ioapi.c",
    "../../examples/ThirdPartyLibs/minizip/unzip.c",
    "../../examples/ThirdPartyLibs/minizip/zip.c",
    "../../examples/ThirdPartyLibs/zlib/adler32.c",
    "../../examples/ThirdPartyLibs/zlib/compress.c",
    "../../examples/ThirdPartyLibs/zlib/crc32.c",
    "../../examples/ThirdPartyLibs/zlib/deflate.c",
    "../../examples/ThirdPartyLibs/zlib/gzclose.c",
    "../../examples/ThirdPartyLibs/zlib/gzlib.c",
    "../../examples/ThirdPartyLibs/zlib/gzread.c",
    "../../examples/ThirdPartyLibs/zlib/gzwrite.c",
    "../../examples/ThirdPartyLibs/zlib/infback.c",
    "../../examples/ThirdPartyLibs/zlib/inffast.c",
    "../../examples/ThirdPartyLibs/zlib/inflate.c",
    "../../examples/ThirdPartyLibs/zlib/inftrees.c",
    "../../examples/ThirdPartyLibs/zlib/trees.c",
    "../../examples/ThirdPartyLibs/zlib/uncompr.c",
    "../../examples/ThirdPartyLibs/zlib/zutil.c",
  }

	if _OPTIONS["enable_stable_pd"] then
		defines {"STATIC_LINK_SPD_PLUGIN"}
		files {
			"../../examples/SharedMemory/plugins/stablePDPlugin/SpAlg.cpp",
			"../../examples/SharedMemory/plugins/stablePDPlugin/SpAlg.h",
			"../../examples/SharedMemory/plugins/stablePDPlugin/Shape.cpp",
			"../../examples/SharedMemory/plugins/stablePDPlugin/Shape.h",
			"../../examples/SharedMemory/plugins/stablePDPlugin/RBDUtil.cpp",
			"../../examples/SharedMemory/plugins/stablePDPlugin/RBDUtil.h",
			"../../examples/SharedMemory/plugins/stablePDPlugin/RBDModel.cpp",
			"../../examples/SharedMemory/plugins/stablePDPlugin/RBDModel.h",
			"../../examples/SharedMemory/plugins/stablePDPlugin/MathUtil.cpp",
			"../../examples/SharedMemory/plugins/stablePDPlugin/MathUtil.h",
			"../../examples/SharedMemory/plugins/stablePDPlugin/KinTree.cpp",
			"../../examples/SharedMemory/plugins/stablePDPlugin/KinTree.h",
			"../../examples/SharedMemory/plugins/stablePDPlugin/BulletConversion.cpp",
			"../../examples/SharedMemory/plugins/stablePDPlugin/BulletConversion.h",
			}
		end
		
		
	if _OPTIONS["enable_physx"] then
  	defines {"BT_ENABLE_PHYSX","PX_PHYSX_STATIC_LIB", "PX_FOUNDATION_DLL=0"}
		
		configuration {"x64", "debug"}			
				defines {"_DEBUG"}
		configuration {"x86", "debug"}
				defines {"_DEBUG"}
		configuration {"x64", "release"}
				defines {"NDEBUG"}
		configuration {"x86", "release"}
				defines {"NDEBUG"}
		configuration{}

		includedirs {
                ".",
                "../../src/PhysX/physx/include",
						    "../../src/PhysX/physx/include/characterkinematic",
						    "../../src/PhysX/physx/include/common",
						    "../../src/PhysX/physx/include/cooking",
						    "../../src/PhysX/physx/include/extensions",
						    "../../src/PhysX/physx/include/geometry",
						    "../../src/PhysX/physx/include/geomutils",
						    "../../src/PhysX/physx/include/vehicle",
						    "../../src/PhysX/pxshared/include",
                }
		links {
				"PhysX",
			}
			
			files {
				"../../examples/SharedMemory/plugins/eglPlugin/eglRendererPlugin.cpp",
				"../../examples/SharedMemory/plugins/eglPlugin/eglRendererPlugin.h",
				"../../examples/SharedMemory/plugins/eglPlugin/eglRendererVisualShapeConverter.cpp",
				"../../examples/SharedMemory/plugins/eglPlugin/eglRendererVisualShapeConverter.h",
				"../../examples/SharedMemory/physx/PhysXC_API.cpp",
				"../../examples/SharedMemory/physx/PhysXServerCommandProcessor.cpp",
				"../../examples/SharedMemory/physx/PhysXUrdfImporter.cpp",
				"../../examples/SharedMemory/physx/URDF2PhysX.cpp",
				"../../examples/SharedMemory/physx/PhysXC_API.h",
				"../../examples/SharedMemory/physx/PhysXServerCommandProcessor.h",
				"../../examples/SharedMemory/physx/PhysXUrdfImporter.h",
				"../../examples/SharedMemory/physx/URDF2PhysX.h",
				"../../examples/SharedMemory/physx/PhysXUserData.h",
				}
  end
  			
if (_OPTIONS["enable_static_vr_plugin"]) then
		files {"../../examples/SharedMemory/plugins/vrSyncPlugin/vrSyncPlugin.cpp"}
end

	
	includedirs {
		_OPTIONS["python_include_dir"],
	}
	libdirs {
		_OPTIONS["python_lib_dir"]
	}
	
	if os.is("Linux") then
       		initX11()
	end

	
