solution "0MySolution"

	-- Multithreaded compiling
	if _ACTION == "vs2010" then
		buildoptions { "/MP"  }
	end 
	

  
	configurations {"Release", "Debug"}
	configuration "Release"
		flags { "Optimize", "StaticRuntime", "NoMinimalRebuild", "FloatFast"}
	configuration "Debug"
		flags { "Symbols", "StaticRuntime" , "NoMinimalRebuild", "NoEditAndContinue" ,"FloatFast"}
		
	platforms {"x32", "x64"}

	configuration "x64"		
		targetsuffix "_64"
	configuration {"x64", "debug"}
		targetsuffix "_x64_debug"
	configuration {"x64", "release"}
		targetsuffix "_x64"
	configuration {"x32", "debug"}
		targetsuffix "_debug"

	configuration{}

		flags { "NoRTTI", "NoExceptions"}
		defines { "_HAS_EXCEPTIONS=0" }
		targetdir "../bin"
	  location("./" .. _ACTION)


	projectRootDir = os.getcwd() .. "/../"
	print("Project root directroy: " .. projectRootDir);

	dofile ("findOpenCL.lua")
	dofile ("findDirectX11.lua")
	dofile ("findOpenGLGlewGlut.lua")
	
	language "C++"
	
	include "../opencl/gpu_rigidbody_pipeline2"
	include "../opencl/gpu_rigidbody_pipeline"
		
	include "../opencl/basic_initialize"
	include "../opencl/vector_add"
	
	include "../opencl/primitives/AdlTest"
	include "../opencl/primitives/benchmark"
	include "../opencl/3dGridBroadphase"
	include "../opencl/broadphase_benchmark"

	