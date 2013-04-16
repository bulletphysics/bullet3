function createProject(vendor)
	
	hasCL = findOpenCL(vendor)
	
	if (hasCL) then

		project ("OpenCL_intialize_" .. vendor)

		initOpenCL(vendor)
	
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../bin"

		files {
			"main.cpp",
			"b3OpenCLUtils.cpp",
			"b3OpenCLUtils.h"
		}
		
	end
end
	
createProject("Apple")
createProject("AMD")
createProject("Intel")
createProject("NVIDIA")
