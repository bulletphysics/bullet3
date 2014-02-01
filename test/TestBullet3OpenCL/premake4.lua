function createProject(vendor)

	project "Test_OpenCL_Bullet3"
		
	kind "ConsoleApp"
	
--	defines {  }
	
	targetdir "../../bin"
	
	initOpenCL(vendor)
	
	includedirs 
	{
		".","../gtest-1.7.0/include",
		"../../src",
	}

	links {"gtest",
--			"Bullet2FileLoader",
			 "Bullet3OpenCL_" .. vendor,
--			"Bullet3Dynamics",
--			"Bullet3Collision",
--			"Bullet3Geometry",
			"Bullet3Common"
	}
			
	
	files {
		"**.cpp",
		"**.h",
		"../OpenCL/BasicInitialize/testInitOpenCL.cpp",
		"../OpenCL/AllBullet3Kernels/*.cpp"
		
	}
	if os.is("Windows") then
	end

	if os.is("Linux") then 
		links {"pthread"}
	end

	if os.is("MacOSX") then
	end
end

createProject("clew")
