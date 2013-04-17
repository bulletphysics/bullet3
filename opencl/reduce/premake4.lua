
function createProject (vendor)

	local hasCL = findOpenCL(vendor)
	
	if (hasCL) then

		project ( "OpenCL_reduce_" .. vendor)

		initOpenCL(vendor)
	
		language "C++"
				
		kind "ConsoleApp"
		targetdir "../../bin"

		links {
			"OpenCL_lib_parallel_primitives_host_" .. vendor
		}

		includedirs {
			"../basic_initialize",
			"../../src"
		}
		
		files {
			"main.cpp",
			"../basic_initialize/b3OpenCLUtils.cpp",
			"../basic_initialize/b3OpenCLUtils.h",
			"../../src/BulletCommon/b3AlignedAllocator.cpp",
			"../../src/BulletCommon/b3AlignedAllocator.h",
			"../../src/BulletCommon/b3AlignedObjectArray.h",
		}
	end
	
end

createProject("AMD")
createProject("NVIDIA")
createProject("Intel")
createProject("Apple")
