	
	hasCL = findOpenCL_Intel()
	hasDX11 = findDirectX11()
	
	if (hasCL) then

		project "OpenCL_DX11_primitives_test_Intel"

		initOpenCL_Intel()

		if (hasDX11) then
			initDirectX11()
		end
		
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../../../bin"
		includedirs {"..","../.."}
		
		links {
		"OpenCL"
		}
		
		files {
			"../main.cpp",
			"../RadixSortBenchmark.h",
			"../UnitTests.h"
		}
		
	end