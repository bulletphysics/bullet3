function createProject(vendor)
	hasCL = findOpenCL(vendor)
	
	if (hasCL) then

		project ("OpenCL_radixsort_benchmark_" .. vendor)

		initOpenCL(vendor)
		
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../../bin"
		includedirs {"..","../../../src"}
		
		links {
			("OpenCL_lib_parallel_primitives_host_" .. vendor)
		}
		
		files {
			"test_large_problem_sorting.cpp",
			"../../basic_initialize/b3OpenCLUtils.cpp",
			"../../basic_initialize/b3OpenCLUtils.h",
			"../host/b3FillCL.cpp",
			"../host/b3PrefixScanCL.cpp",
			"../host/b3RadixSort32CL.cpp",
			"../../../src/Bullet3Common/b3AlignedAllocator.cpp",
			"../../../src/Bullet3Common/b3AlignedAllocator.h",
			"../../../src/Bullet3Common/b3AlignedObjectArray.h",
			"../../../src/Bullet3Common/b3Quickprof.cpp",
			"../../../src/Bullet3Common/b3Quickprof.h",
		}
		
	end
end

createProject("AMD")
createProject("Intel")
createProject("NVIDIA")
createProject("Apple")