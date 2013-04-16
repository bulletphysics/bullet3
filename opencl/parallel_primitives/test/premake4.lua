function createProject(vendor)	
	hasCL = findOpenCL(vendor)
	
	if (hasCL) then

		project ("OpenCL_primitives_test_" .. vendor)

		initOpenCL(vendor)

		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../../bin"
		includedirs {".","..","../../../src"}
		
		
		files {
			"main.cpp",
			"../../basic_initialize/b3OpenCLInclude.h",
			"../../basic_initialize/b3OpenCLUtils.cpp",
			"../../basic_initialize/b3OpenCLUtils.h",
			"../host/btFillCL.cpp",
			"../host/btFillCL.h",
			"../host/btBoundSearchCL.cpp",
			"../host/btBoundSearchCL.h",
			"../host/btPrefixScanCL.cpp",
			"../host/btPrefixScanCL.h",
			"../host/btRadixSort32CL.cpp",
			"../host/btRadixSort32CL.h",
			"../../../src/BulletCommon/btAlignedAllocator.cpp",
			"../../../src/BulletCommon/btAlignedAllocator.h",
			"../../../src/BulletCommon/btAlignedObjectArray.h",
		}
		
	end
end

createProject("AMD")
createProject("Intel")
createProject("NVIDIA")
createProject("Apple")