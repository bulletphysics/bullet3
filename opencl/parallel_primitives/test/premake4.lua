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
			"../host/b3FillCL.cpp",
			"../host/b3FillCL.h",
			"../host/b3BoundSearchCL.cpp",
			"../host/b3BoundSearchCL.h",
			"../host/b3PrefixScanCL.cpp",
			"../host/b3PrefixScanCL.h",
			"../host/b3RadixSort32CL.cpp",
			"../host/b3RadixSort32CL.h",
			"../../../src/Bullet3Common/b3AlignedAllocator.cpp",
			"../../../src/Bullet3Common/b3AlignedAllocator.h",
			"../../../src/Bullet3Common/b3AlignedObjectArray.h",
		}
		
	end
end

createProject("AMD")
createProject("Intel")
createProject("NVIDIA")
createProject("Apple")