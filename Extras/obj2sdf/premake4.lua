project ("App_obj2sdf")

		language "C++"
		kind "ConsoleApp"

		includedirs {"../../src",
		"../../examples/ThirdPartyLibs"}
	
	
	links{"Bullet3Common"}


		files {
		"obj2sdf.cpp",
			"../../examples/Utils/b3ResourcePath.cpp",
			"../../examples/Utils/b3ResourcePath.h",
			"../../examples/ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp",
	}
