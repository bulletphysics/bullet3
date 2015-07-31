
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

	  newoption {
    		trigger     = "ios",
    		description = "Enable iOS target (requires xcode4)"
	}

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
		trigger = "noopengl3",
		description = "Don't compile any OpenGL3+ code"
	}

	newoption
	{
		trigger = "midi",
		description = "Use Midi controller to control parameters"
	}

--	--_OPTIONS["midi"] = "1";

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
		trigger = "lua",
		description = "Enable Lua scipting support in Example Browser"
	}

	 newoption {
    trigger     = "targetdir",
    value       = "path such as ../bin",
    description = "Set the output location for the generated project files"
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
	
	if os.is("Linux") or os.is("macosx") then
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
                                'INFOPLIST_FILE = "../../test/Bullet2/Info.plist"',
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
			'SDKROOT = "macosx10.9"',
			}
			end
	end

-- comment-out for now, URDF reader needs exceptions
--	flags { "NoRTTI", "NoExceptions"}
--	defines { "_HAS_EXCEPTIONS=0" }
--printf ( _OPTIONS["targetdir"] )

	targetdir( _OPTIONS["targetdir"] or "../bin" )
	location("./" .. act .. postfix)


	projectRootDir = os.getcwd() .. "/../"
	print("Project root directory: " .. projectRootDir);

	dofile ("findOpenCL.lua")
	dofile ("findDirectX11.lua")
	dofile ("findOpenGLGlewGlut.lua")
	
	if (not findOpenGL3()) then
		defines {"NO_OPENGL3"}
	end

	language "C++"

if not _OPTIONS["ios"] then

	include "../examples/ExampleBrowser"
	include "../examples/OpenGLWindow"
	include "../examples/SharedMemory"	
	include "../examples/ThirdPartyLibs/Gwen"
	include "../Extras"

	include "../examples/HelloWorld"
	include "../examples/BasicDemo"
	include "../test/SharedMemory"
	
	if _OPTIONS["enet"] then
		include "../examples/ThirdPartyLibs/enet"
		include "../test/enet/client"
		include "../test/enet/server"	
	end

	if _OPTIONS["lua"] then
		include "../examples/ThirdPartyLibs/lua-5.2.3"
	end
	
	include "../src/Bullet3Common"
	include "../src/Bullet3Geometry"
	include "../src/Bullet3Collision"
	include "../src/Bullet3Dynamics"
	include "../src/Bullet3OpenCL"
	include "../src/Bullet3Serialize/Bullet2FileLoader"

 	if not _OPTIONS["without-gtest"] then
                include "../test/gtest-1.7.0"
--              include "../test/hello_gtest"
                include "../test/collision"
                include "../test/TestBullet3OpenCL"
                include "../test/GwenOpenGLTest"
        end
end

	include "../test/Bullet2"	
 	include "../src/BulletSoftBody"
        include "../src/BulletDynamics"
        include "../src/BulletCollision"
        include "../src/LinearMath"
	
