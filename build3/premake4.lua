
  solution "0_Bullet3Solution"

	local osversion = os.getversion()
		print(string.format(" %d.%d.%d (%s)", 
   		osversion.majorversion, osversion.minorversion, osversion.revision,
   		osversion.description))
	

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
                trigger = "force_dlopen_opengl",
                description = "Dynamically load OpenGL (instead of static/dynamic linking)"
        }

	newoption
        {
                trigger = "force_dlopen_x11",
                description = "Dynamically load OpenGL (instead of static/dynamic linking)"
        }


	newoption
	{
		trigger = "midi",
		description = "Use Midi controller to control parameters"
	}

--	_OPTIONS["midi"] = "1";

	newoption
	{
		trigger = "bullet2demos",
		description = "Compile the Bullet 2 demos (Demo/Extra folder)"
	}

	newoption
	{
		trigger = "enet",
		description = "Enable enet NAT punchthrough test"
	}

	newoption
	{
		trigger = "without-gtest",
		description = "Disable unit tests using gtest"
	}

	configurations {"Release", "Debug"}
	configuration "Release"
		flags { "Optimize", "EnableSSE2","StaticRuntime", "NoMinimalRebuild", "FloatFast"}
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
			xcodebuildsettings
			{
        		'ARCHS = "$(ARCHS_STANDARD_32_BIT) $(ARCHS_STANDARD_64_BIT)"',
        		'VALID_ARCHS = "x86_64 i386"',
			}
	end

-- comment-out for now, URDF reader needs exceptions
--	flags { "NoRTTI", "NoExceptions"}
--	defines { "_HAS_EXCEPTIONS=0" }
	targetdir "../bin"
	location("./" .. act .. postfix)


	projectRootDir = os.getcwd() .. "/../"
	print("Project root directory: " .. projectRootDir);

	dofile ("findOpenCL.lua")
	dofile ("findDirectX11.lua")
	dofile ("findOpenGLGlewGlut.lua")

	language "C++"

	if not _OPTIONS["without-gtest"] then
		include "../test/gtest-1.7.0"
--		include "../test/hello_gtest"
		include "../test/TestBullet3OpenCL"
	end

if findOpenGL3() then
	include "../Demos3/AllBullet2Demos"
	include "../Demos3/GpuDemos"
	include"../Demos3/BasicDemoConsole"
	include"../Demos3/BasicDemoCustomOpenGL2"
	
--	include "../Demos3/CpuDemos"
--	include "../Demos3/Wavefront"
--	include "../btgui/MultiThreading"

	include "../btgui/OpenGLWindow"

--	include "../Demos3/ImplicitCloth"
	include "../Demos3/SimpleOpenGL3"
	include "../btgui/urdf"

	include "../btgui/lua-5.2.3"
	include "../test/lua"
    include "../btgui/Gwen"
    include "../btgui/GwenOpenGLTest"
end


--		include "../demo/gpudemo"
if _OPTIONS["midi"] then
                include "../btgui/MidiTest"
end

--		include "../opencl/vector_add_simplified"
--		include "../opencl/vector_add"
--		include "../test/clew"
--		include "../Demos3/GpuGuiInitialize"

--		include "../test/OpenCL/BasicInitialize"
		include "../test/OpenCL/KernelLaunch"--
--		include "../test/OpenCL/BroadphaseCollision"
--		include "../test/OpenCL/NarrowphaseCollision"
		include "../test/OpenCL/ParallelPrimitives"
		include "../test/OpenCL/RadixSortBenchmark"

		include "../src/BulletSoftBody"
		include "../src/BulletDynamics"
		include "../src/BulletCollision"
		include "../src/LinearMath"

		include "../src/Bullet3Dynamics"
		include "../src/Bullet3Common"
		include "../src/Bullet3Geometry"
		include "../src/Bullet3Collision"
		include "../src/Bullet3Serialize/Bullet2FileLoader"
		include "../src/Bullet3OpenCL"

--		include "../demo/gpu_initialize"
--		include "../opencl/lds_bank_conflict"
--		include "../opencl/reduce"
--		include "../btgui/OpenGLWindow"
--		include "../demo/ObjLoader"
--		include "../test/b3DynamicBvhBroadphase"

	if _OPTIONS["enet"] then
		include "../btgui/enet"
		include "../test/enet/server"
		include "../test/enet/client"
	end

	if _OPTIONS["bullet2demos"] then
		include "../Extras"
		if findOpenGL() then
        		include "../Demos"
		end
	end

