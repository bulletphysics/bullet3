	project "0_console_sample"
		
	kind "ConsoleApp"
	targetdir "../../../bin"
	includedirs {"../../../include"}
		
	links {
		"physicseffects2_lowlevel",
		"physicseffects2_baselevel",
		"physicseffects2_util"
	}
	
	files {
		"main.cpp",
		"physics_func.cpp",
		"physics_func.h",
		"../common/perf_func.win32.cpp"		
	}