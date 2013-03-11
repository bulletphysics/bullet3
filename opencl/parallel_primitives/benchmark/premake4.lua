function createProject(vendor)
	hasCL = findOpenCL(vendor)
	
	if (hasCL) then

		project ("OpenCL_radixsort_benchmark_" .. vendor)

		initOpenCL(vendor)
		
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../../bin"
		includedirs {".."}
		
		links {
			("OpenCL_lib_parallel_primitives_host_" .. vendor)
		}
		
		files {
			"test_large_problem_sorting.cpp",
			"../../basic_initialize/btOpenCLUtils.cpp",
			"../../basic_initialize/btOpenCLUtils.h",
			"../host/btFillCL.cpp",
			"../host/btPrefixScanCL.cpp",
			"../host/btRadixSort32CL.cpp",
		}
		
	end
end

createProject("AMD")
createProject("Intel")
createProject("NVIDIA")
createProject("Apple")