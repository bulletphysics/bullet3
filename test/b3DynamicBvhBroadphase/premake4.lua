

project ("Test_b3DynamicBvhBroadphase_test")

	language "C++"
			
	kind "ConsoleApp"
	targetdir "../../bin"
	includedirs {"../../src"}
	
	links {"Bullet3Common", "Bullet3Collision"}		
	
	files {
		"main.cpp",
	}

