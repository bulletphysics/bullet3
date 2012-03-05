	
	hasCL = findOpenCL_AMD()
	
	if (hasCL) then

		project "OpenCL_global_atomics_AMD"

		initOpenCL_AMD()
	
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../../bin"

--		includedirs {"..","../../../../include/gpu_research"}
		
		files {
			"../main.cpp",
			"../../basic_initialize/btOpenCLUtils.cpp",
			"../../basic_initialize/btOpenCLUtils.h"
		}
		
	end