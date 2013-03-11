function createProject(vendor)
	
	hasCL = findOpenCL(vendor)
	
	if (hasCL) then

		project ("OpenCL_VectorAdd_" .. vendor)

		initOpenCL(vendor)
	
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../bin"

		files {
			"main.cpp",
			"../basic_initialize/btOpenCLUtils.cpp",
			"../basic_initialize/btOpenCLUtils.h"
		}
		
	end
end
	
createProject("AMD")
createProject("Intel")
createProject("NVIDIA")
createProject("Apple")
