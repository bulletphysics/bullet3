	
	hasCL = findOpenCL_NVIDIA()
	hasDX11 = findDirectX11()
	
	if (hasCL) then

		project "OpenCL_DX11_radixsort_benchmark_NVIDIA"

		initOpenCL_NVIDIA()

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
			"../test_large_problem_sorting.cpp"
		}
		
	end