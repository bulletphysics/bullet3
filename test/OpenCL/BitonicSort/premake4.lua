function createProject(vendor)
	
	hasCL = findOpenCL(vendor)
	
	if (hasCL) then

		project ("Test_BitonicSort_" .. vendor)

		initOpenCL(vendor)
	
		language "C++"
				
		
		kind "ConsoleApp"
		targetdir "../../../bin"

		includedirs {"../../../src"}
		
		files {
			"main.cpp",
			"b3BitonicSort.cpp",
			"../../../src/Bullet3Common/b3AlignedAllocator.cpp",
			"../../../src/Bullet3Common/b3AlignedAllocator.h",
			"../../../src/Bullet3Common/b3Quickprof.cpp",
			"../../../src/Bullet3Common/b3Quickprof.h",
			"../../../src/Bullet3OpenCL/Initialize/b3OpenCLUtils.cpp",
			"../../../src/Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
		}
		
	end
end
	
createProject("Apple")
createProject("AMD")
createProject("Intel")
createProject("NVIDIA")
