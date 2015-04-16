function createProject(vendor)

	project "Test_OpenCL_Bullet3"
		
	kind "ConsoleApp"
	
--	defines {  }
	
	if os.is("Windows") then
		--see http://stackoverflow.com/questions/12558327/google-test-in-visual-studio-2012
		defines {"_VARIADIC_MAX=10"}
	end
	
	
	targetdir "../../bin"
	
	initOpenCL(vendor)
	
	includedirs 
	{
		".","../gtest-1.7.0/include",
		"../../src",
		"../../examples/ThirdPartyLibs", --for unzip
	}

	links {"gtest",
--			"Bullet2FileLoader",
			 "Bullet3OpenCL_" .. vendor,
--			"Bullet3Dynamics",
--			"Bullet3Collision",
--			"Bullet3Geometry",
			"Bullet3Common"
	}

	--you can comment out the following few lines, then you need to unzip the untest_data.zip manually
	defines {"B3_USE_ZLIB"}		
	files {
		"../../examples/ThirdPartyLibs/minizip/*.c",
		"../../examples/ThirdPartyLibs/zlib/*.c",
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
