function createProject(vendor)	
	hasCL = findOpenCL(vendor)
	
	if (hasCL) then

		project ("OpenCL_broadphase_test_" .. vendor)

		initOpenCL(vendor)

		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../../bin"
		includedirs {"..","../..","../../../src"}
		
		
		files {
			"main.cpp",
			"../../basic_initialize/b3OpenCLInclude.h",
			"../../basic_initialize/b3OpenCLUtils.cpp",
			"../../basic_initialize/b3OpenCLUtils.h",
			"../host/b3GpuSapBroadphase.cpp",
			"../host/b3GpuSapBroadphase.h",
			"../../parallel_primitives/host/btFillCL.cpp",
			"../../parallel_primitives/host/btFillCL.h",
			"../../parallel_primitives/host/btBoundSearchCL.cpp",
			"../../parallel_primitives/host/btBoundSearchCL.h",
			"../../parallel_primitives/host/btPrefixScanCL.cpp",
			"../../parallel_primitives/host/btPrefixScanCL.h",
			"../../parallel_primitives/host/btRadixSort32CL.cpp",
			"../../parallel_primitives/host/btRadixSort32CL.h",
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