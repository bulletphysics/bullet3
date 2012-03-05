	
	hasCL = findOpenCL_AMD()
	
	if (hasCL) then

		project "OpenCL_bt3dGridBroadphase_AMD"

		initOpenCL_AMD()
	
		language "C++"
				
		kind "StaticLib"
		targetdir "../../../bin"

		libdirs {"../../../rendering/GlutGlewWindows"}

			includedirs {
--		"../../../rendering/GlutGlewWindows",
		"../../../opencl/3dGridBroadphase/Shared",
		"../../../../../src",
		"../../primitives"
		}
		
		files {
			"../Shared/*.cpp",
			"../Shared/*.h"
		}
		
	end