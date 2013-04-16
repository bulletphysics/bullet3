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
			"../host/btFillCL.cpp",
			"../host/btPrefixScanCL.cpp",
			"../host/btRadixSort32CL.cpp",
			"../../../src/BulletCommon/btAlignedAllocator.cpp",
			"../../../src/BulletCommon/btAlignedAllocator.h",
			"../../../src/BulletCommon/btAlignedObjectArray.h",
			"../../../src/BulletCommon/btQuickprof.cpp",
			"../../../src/BulletCommon/btQuickprof.h",
		}
		
	end
end

createProject("AMD")
createProject("Intel")
createProject("NVIDIA")
createProject("Apple")