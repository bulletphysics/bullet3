	
	hasCL = findOpenCL_Intel()
	
	if (hasCL) then

		project "OpenCL_intialize_Intel"

		initOpenCL_Intel()
	
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../../bin"

--		includedirs {"..","../../../../include/gpu_research"}
		
		files {
			"../main.cpp",
			"../btOpenCLUtils.cpp",
			"../btOpenCLUtils.h"
		}
		
	end