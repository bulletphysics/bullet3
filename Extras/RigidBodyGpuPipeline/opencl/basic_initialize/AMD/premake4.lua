	
	hasCL = findOpenCL_AMD()
	
	if (hasCL) then

		project "OpenCL_intialize_AMD"

		initOpenCL_AMD()
	
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