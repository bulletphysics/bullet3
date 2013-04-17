function createProject(vendor)	
	hasCL = findOpenCL(vendor)
	
	if (hasCL) then

		project ("OpenCL_sat_test_" .. vendor)

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
			"../host/**.cpp",
			"../host/**.h",
			"../../parallel_primitives/host/btFillCL.cpp",
			"../../parallel_primitives/host/btFillCL.h",
			"../../parallel_primitives/host/btBoundSearchCL.cpp",
			"../../parallel_primitives/host/btBoundSearchCL.h",
			"../../parallel_primitives/host/btPrefixScanCL.cpp",
			"../../parallel_primitives/host/btPrefixScanCL.h",
			"../../parallel_primitives/host/btRadixSort32CL.cpp",
			"../../parallel_primitives/host/btRadixSort32CL.h",
			"../../../src/BulletCommon/b3AlignedAllocator.cpp",
			"../../../src/BulletCommon/b3AlignedAllocator.h",
			"../../../src/BulletCommon/b3AlignedObjectArray.h",
			"../../../src/BulletCommon/b3Quickprof.cpp",
			"../../../src/BulletCommon/b3Quickprof.h",
			"../../../src/BulletGeometry/**.cpp",
			"../../../src/BulletGeometry/**.h",
			

		}
		
	end
end

createProject("AMD")
createProject("Intel")
createProject("NVIDIA")
createProject("Apple")