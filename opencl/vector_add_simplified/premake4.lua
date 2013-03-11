
function createProject (vendor)

	local hasCL = findOpenCL(vendor)
	
	if (hasCL) then

		project ( "OpenCL_vector_add_simplified_" .. vendor)

		initOpenCL(vendor)
	
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../bin"

		links {
			"OpenCL_lib_parallel_primitives_host_" .. vendor
		}

		includedirs {
			"../basic_initialize"
		}
		
		files {
			"main.cpp",
			"../basic_initialize/btOpenCLUtils.cpp",
			"../basic_initialize/btOpenCLUtils.h"
		}
	end
	
end

createProject("AMD")
createProject("NVIDIA")
createProject("Intel")
createProject("Apple")
