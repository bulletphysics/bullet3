solution "00_MySolution"

	configurations {"Debug", "Release"}
	configuration "Release"
		flags { "Optimize", "StaticRuntime", "NoRTTI", "NoExceptions"}
	configuration "Debug"
		flags { "Symbols", "StaticRuntime" , "NoRTTI", "NoExceptions"}
	platforms {"x32", "x64"}
	configuration "x32"
		libdirs {"$(ATISTREAMSDKROOT)/lib/x86"}
	configuration "x64"
		libdirs {"$(ATISTREAMSDKROOT)/lib/x86_64"}
		targetsuffix "_64"

	configuration {"x64", "debug"}
		targetsuffix "_x64_debug"
	configuration {"x64", "release"}
		targetsuffix "_x64"
	configuration {"x32", "debug"}
		targetsuffix "_debug"


	language "C++"
	location "build"
	targetdir "bin"

	include "../src/base_level"
	include "../src/low_level"
	include "../src/util"
	
	include "../sample/api_physics_effects/0_console"
	include "../sample/api_physics_effects/1_simple"
	include "../sample/api_physics_effects/2_stable"
	include "../sample/api_physics_effects/3_sleep"
	include "../sample/api_physics_effects/4_motion_type"
	include "../sample/api_physics_effects/5_raycast"
	include "../sample/api_physics_effects/6_joint"
	
	
	