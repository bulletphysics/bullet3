
  solution "0MySolution"

	-- Multithreaded compiling
	if _ACTION == "vs2010" or _ACTION=="vs2008" then
		buildoptions { "/MP"  }
	end 
	
	act = ""
    
    if _ACTION then
        act = _ACTION
    end


	newoption 
	{
    		trigger     = "ios",
    		description = "Enable iOS target (requires xcode4)"
  	}
	
	newoption
	{
		trigger = "bullet2gpu",
		description = "Enable Bullet 2.x GPU using b3GpuDynamicsWorld bridge to Bullet 3.x"
	}

	newoption
	{
		trigger = "enet",
		description = "Enable enet NAT punchthrough test"
	}
  
	configurations {"Release", "Debug"}
	configuration "Release"
		flags { "Optimize", "EnableSSE","StaticRuntime", "NoMinimalRebuild", "FloatFast"}
	configuration "Debug"
		defines {"_DEBUG=1"}
		flags { "Symbols", "StaticRuntime" , "NoMinimalRebuild", "NoEditAndContinue" ,"FloatFast"}
		
	if os.is("Linux") then
		if os.is64bit() then
			platforms {"x64"}
		else
			platforms {"x32"}
		end
	else
		platforms {"x32", "x64"}
	end

	configuration {"x32"}
		targetsuffix ("_" .. act)
	configuration "x64"		
		targetsuffix ("_" .. act .. "_64" )
	configuration {"x64", "debug"}
		targetsuffix ("_" .. act .. "_x64_debug")
	configuration {"x64", "release"}
		targetsuffix ("_" .. act .. "_x64_release" )
	configuration {"x32", "debug"}
		targetsuffix ("_" .. act .. "_debug" )
	
	configuration{}

	postfix=""

	if _ACTION == "xcode4" then
		if _OPTIONS["ios"] then
      			postfix = "ios";
      			xcodebuildsettings
      			{
              		'CODE_SIGN_IDENTITY = "iPhone Developer"',
              		"SDKROOT = iphoneos",
              		'ARCHS = "armv7"',
              		'TARGETED_DEVICE_FAMILY = "1,2"',
              		'VALID_ARCHS = "armv7"',
      			}      
      		else
      			xcodebuildsettings
      			{
              		'ARCHS = "$(ARCHS_STANDARD_32_BIT) $(ARCHS_STANDARD_64_BIT)"',
              		'VALID_ARCHS = "x86_64 i386"',
      			}
    		end
	end

	
	flags { "NoRTTI", "NoExceptions"}
	defines { "_HAS_EXCEPTIONS=0" }
	targetdir "../bin"
	location("./" .. act .. postfix)

	
	projectRootDir = os.getcwd() .. "/../"
	print("Project root directroy: " .. projectRootDir);

	dofile ("findOpenCL.lua")
	dofile ("findDirectX11.lua")
	dofile ("findOpenGLGlewGlut.lua")
	
	language "C++"
	
	include "../Demos3/Wavefront"
	include "../btgui/MultiThreading"

	if not _OPTIONS["ios"] then
--		include "../demo/gpudemo"
--	include "../btgui/MidiTest"
--		include "../opencl/vector_add_simplified"
--		include "../opencl/vector_add"
		include "../btgui/Gwen"
		include "../btgui/GwenOpenGLTest"
		include "../test/clew"
		include "../Demos3/GpuGuiInitialize"
		
		include "../test/OpenCL/BasicInitialize"
--		include "../test/OpenCL/BroadphaseCollision"
--		include "../test/OpenCL/NarrowphaseCollision"
		include "../test/OpenCL/ParallelPrimitives"
		include "../test/OpenCL/RadixSortBenchmark"
		include "../test/OpenCL/BitonicSort"

		include "../src/Bullet3Dynamics"
		include "../src/Bullet3Common"
		include "../src/Bullet3Geometry"
		include "../src/Bullet3Collision"
		include "../src/Bullet3Serialize/Bullet2FileLoader"
	
		include "../src/Bullet3OpenCL"
		include "../Demos3/GpuDemos"
		
			
--		include "../demo/gpu_initialize"
--		include "../opencl/lds_bank_conflict"
--		include "../opencl/reduce"
			include "../btgui/OpenGLTrueTypeFont"
--		include "../btgui/OpenGLWindow"
--		include "../demo/ObjLoader"

		
--		include "../test/b3DynamicBvhBroadphase"
		
	if _OPTIONS["enet"] then
		include "../btgui/enet"
		include "../test/enet/server"
		include "../test/enet/client"
	end
	

	if _OPTIONS["bullet2gpu"] then
		include "../src/LinearMath"	
	include "../src/BulletCollision"	
	include "../src/BulletDynamics"	
	include "../src/BulletSoftBody"		
	include "../Demos/HelloWorld"
	
		include "../Demos3"
	end

		end
