	
		project "App_AllBullet2Demos"

		language "C++"
				
		kind "ConsoleApp"

  	includedirs {
                ".",
                "../../src",
                "../../btgui",
                "../../btgui/lua-5.2.3/src"
                }

			
		links{"gwen", "OpenGL_Window","BulletDynamics","BulletCollision","LinearMath","Bullet3Common","lua-5.2.3"}
		initOpenGL()
		initGlew()

     if _OPTIONS["midi"] then
                 if os.is("Windows") then
                         files {"../../btgui/MidiTest/RtMidi.cpp"}
                         links {"winmm"}
                         defines {"__WINDOWS_MM__", "WIN32","B3_USE_MIDI"}
                 end

                 if os.is("Linux") then
                       defines {"__LINUX_ALSA__"}
                       links {"asound","pthread"}
                 end

                 if os.is("MacOSX") then
                         files {"../../btgui/MidiTest/RtMidi.cpp"}
                         links{"CoreAudio.framework", "coreMIDI.framework", "Cocoa.framework"}
                         defines {"__MACOSX_CORE__","B3_USE_MIDI"}
                 end
         end
	
		files {
		"**.cpp",
		"**.h",
		"../bullet2/BasicDemo/Bullet2RigidBodyDemo.cpp",
		"../bullet2/BasicDemo/Bullet2RigidBodyDemo.h",
		"../bullet2/LuaDemo/LuaPhysicsSetup.cpp",
		"../bullet2/LuaDemo/LuaPhysicsSetup.h",
		"../bullet2/MultiBodyDemo/TestJointTorqueSetup.cpp",
		"../bullet2/MultiBodyDemo/TestJointTorqueSetup.h",
	--	"../DifferentialGearDemo/DifferentialGearSetup.cpp",
--		"../DifferentialGearDemo/DifferentialGearSetup.h",
		"../../Demos/BasicDemo/BasicDemoPhysicsSetup.cpp",
		"../../Demos/BasicDemo/BasicDemoPhysicsSetup.h",
		"../../Demos/CcdPhysicsDemo/CcdPhysicsSetup.cpp",
		"../../Demos/CcdPhysicsDemo/CcdPhysicsSetup.h",
		"../../Demos/SerializeDemo/SerializeSetup.cpp",
		"../../Extras/Serialize/BulletFileLoader/bChunk.cpp",
		"../../Extras/Serialize/BulletFileLoader/bDNA.cpp",
		"../../Extras/Serialize/BulletFileLoader/bFile.cpp",
		"../../Extras/Serialize/BulletFileLoader/btBulletFile.cpp",	
		"../../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.cpp",
		"../../Extras/Serialize/BulletWorldImporter/btWorldImporter.cpp",	
		"../bullet2/ConstraintDemo/ConstraintPhysicsSetup.cpp",
  		"../bullet2/ConstraintDemo/ConstraintPhysicsSetup.h",
		"../ImportURDFDemo/ImportURDFSetup.cpp",
		"../ImportObjDemo/ImportObjSetup.cpp",
	 	"../ImportSTLDemo/ImportSTLSetup.cpp",	
		"../Wavefront/tiny_obj_loader.cpp",
		"../Wavefront/tiny_obj_loader.h",	
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
		
		
		"../../btgui/Timing/b3Clock.cpp",
		"../../btgui/Timing/b3Clock.h",
		"../GpuDemos/gwenUserInterface.cpp",
		"../GpuDemos/gwenUserInterface.h"
		}

if os.is("Linux") then 
	initX11()
end
if os.is("MacOSX") then
	links{"Cocoa.framework"}
end
