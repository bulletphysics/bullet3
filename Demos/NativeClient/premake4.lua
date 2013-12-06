	project "NativeClientTumbler"
		
	kind "ConsoleApp"
	
	targetdir "bin_html"
	
	 includedirs { "."	}
	 
	--libdirs {}

	links {
		"ppapi_gles2",
		"ppapi",
		"ppapi_cpp",
		"ppruntime"
	}
	
	
	files {
				"cube.cc",
        "opengl_context.cc",
        "scripting_bridge.cc",
        "shader_util.cc",
        "transforms.cc",
        "tumbler.cc",
        "tumbler_module.cc"
	}