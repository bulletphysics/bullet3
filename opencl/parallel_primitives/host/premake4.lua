function createProject(vendor)
	hasCL = findOpenCL(vendor)
	
	if (hasCL) then
		
		project ("OpenCL_lib_parallel_primitives_host_" .. vendor)
	
		initOpenCL(vendor)
			
		kind "StaticLib"
		
		targetdir "../../../lib"
		includedirs {
			".","../../../src"
		}
		
		files {
			"**.cpp",
			"**.h"
		}
		
	end
end

createProject("AMD")
createProject("Intel")
createProject("NVIDIA")
createProject("Apple")