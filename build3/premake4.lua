
	solution "0_Bullet3Solution"

	local osversion = os.getversion()
	print(string.format(" %d.%d.%d (%s)",
   		osversion.majorversion, osversion.minorversion, osversion.revision,
   		osversion.description))

	if _ACTION == "vs2010" or _ACTION=="vs2008" then
		buildoptions
		{
			-- Multithreaded compiling
			"/MP",
			-- Disable a few useless warnings
			"/wd4244",
			"/wd4267"
		}
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
		trigger = "enable_system_opengl",
		description = "Try to link and use the system OpenGL headers version instead of dynamically loading OpenGL (dlopen is default)"
	}

	newoption
	{
		trigger = "enable_openvr",
		description = "Enable experimental Virtual Reality examples, using OpenVR for HTC Vive and Oculus Rift"
	}
	newoption
	{
		trigger = "enable_system_x11",
		description = "Try to link and use system X11 headers instead of dynamically loading X11 (dlopen is default)"
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

-- _OPTIONS["midi"] = "1";

	newoption
	{
		trigger = "no-demos",
		description = "Don't build demos"
	}

	newoption
	{
		trigger = "no-extras",
		description = "Don't build Extras"
	}

	newoption
	{
		trigger = "no-enet",
		description = "Disable enet and enet tests"
	}

	newoption
	{
		trigger = "lua",
		description = "Enable Lua scipting support in Example Browser"
	}

	newoption
        {
                trigger = "enable_pybullet",
                description = "Enable high-level Python scripting of Bullet with URDF/SDF import and synthetic camera."
        }

if os.is("Linux") then
 		default_python_include_dir = "/usr/include/python2.7"
 		default_python_lib_dir = "/usr/local/lib/"
end

		
if os.is("Windows") then
 		default_python_include_dir = "C:/Python-3.5.2/include"
 		default_python_lib_dir = "C:/Python-3.5.2/libs"
end

		newoption
    {
			trigger     = "python_include_dir",
			value       = default_python_include_dir,
			description = "Python (2.x or 3.x) include directory"
    }
    
    newoption
    {
			trigger     = "python_lib_dir",
			value       = default_python_lib_dir,
			description = "Python (2.x or 3.x) library directory "
    }

	
	newoption {
		trigger     = "targetdir",
		value       = "path such as ../bin",
		description = "Set the output location for the generated project files"
	}

	newoption
	{
		trigger = "no-test",
		description = "Disable all tests"
	}

	newoption
	{
		trigger = "no-gtest",
		description = "Disable unit tests using gtest"
	}

	newoption
	{
		trigger = "no-bullet3",
		description = "Do not build bullet3 libs"
	}

	newoption
	{
		trigger = "double",
		description = "Double precision version of Bullet"
	}
	
	if _OPTIONS["double"] then
		defines {"BT_USE_DOUBLE_PRECISION"}
	end

	
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
		platforms {"x32","x64"}
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
			_OPTIONS["no-bullet3"] = "1"
			_OPTIONS["no-gtest"] = "1"

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
--			'SDKROOT = "macosx10.9"',
			}
		end
	end

-- comment-out for now, URDF reader needs exceptions
--	flags { "NoRTTI", "NoExceptions"}
--	defines { "_HAS_EXCEPTIONS=0" }
--printf ( _OPTIONS["targetdir"] )

	targetdir( _OPTIONS["targetdir"] or "../bin" )
	location("./" .. act .. postfix)

	if not _OPTIONS["python_include_dir"] then
			_OPTIONS["python_include_dir"] = default_python_include_dir
	end
	
	if not _OPTIONS["python_lib_dir"] then
			_OPTIONS["python_lib_dir"] = default_python_lib_dir
	end
	

	projectRootDir = os.getcwd() .. "/../"
	print("Project root directory: " .. projectRootDir);

	dofile ("findOpenCL.lua")
	dofile ("findDirectX11.lua")
	dofile ("findOpenGLGlewGlut.lua")

	if (not findOpenGL3()) then
		defines {"NO_OPENGL3"}
	end

	language "C++"


	if not _OPTIONS["no-demos"] then
		include "../examples/ExampleBrowser"
		include "../examples/OpenGLWindow"
		include "../examples/ThirdPartyLibs/Gwen"
		include "../examples/SimpleOpenGL3"
		include "../examples/TinyRenderer"

		include "../examples/HelloWorld"
		include "../examples/BasicDemo"
		include "../examples/InverseDynamics"
		include "../examples/ExtendedTutorials"
		include "../examples/SharedMemory"
		include "../examples/ThirdPartyLibs/BussIK"
		include "../examples/MultiThreading"

		if _OPTIONS["lua"] then
		   include "../examples/ThirdPartyLibs/lua-5.2.3"
		end
		if _OPTIONS["enable_pybullet"] then
		  include "../examples/pybullet"
		end

		if not _OPTIONS["no-test"] then
			include "../test/SharedMemory"
			
		end
	end

	if _OPTIONS["midi"] then
		include "../examples/ThirdPartyLibs/midi"
	end
	
	if not _OPTIONS["no-enet"] then
				include "../examples/ThirdPartyLibs/enet"
				include "../test/enet/nat_punchthrough/client"
				include "../test/enet/nat_punchthrough/server"
				include "../test/enet/chat/client"
				include "../test/enet/chat/server"
				defines {"BT_ENABLE_ENET"}
	end

	 if _OPTIONS["no-bullet3"] then
                print "--no-bullet3 implies --no-demos"
                _OPTIONS["no-demos"] = "1"
        else
                include "../src/Bullet3Common"
                include "../src/Bullet3Geometry"
                include "../src/Bullet3Collision"
                include "../src/Bullet3Dynamics"
                include "../src/Bullet3OpenCL"
                include "../src/Bullet3Serialize/Bullet2FileLoader"
        end

        if _OPTIONS["no-extras"] then
                print "--no-extras implies --no-demos"
                _OPTIONS["no-demos"] = "1"
        else
                include "../Extras"
        end

	if not _OPTIONS["no-test"] then
		include "../test/Bullet2"

		if not _OPTIONS["no-gtest"] then
			include "../test/gtest-1.7.0"
--			include "../test/hello_gtest"
			include "../test/collision"
			include "../test/BulletDynamics/pendulum"
			if not _OPTIONS["no-bullet3"] then
				if not _OPTIONS["no-extras"] then
					include "../test/InverseDynamics"
				end
				include "../test/TestBullet3OpenCL"
			end
			if not _OPTIONS["no-demos"] then
				-- Gwen is only used for demos
				include "../test/GwenOpenGLTest"
			end
		end
	end

	include "../src/BulletInverseDynamics"
 	include "../src/BulletSoftBody"
	include "../src/BulletDynamics"
	include "../src/BulletCollision"
	include "../src/LinearMath"

