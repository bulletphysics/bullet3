
function createProject (vendor)

	local hasCL = findOpenCL(vendor)
	
	if (hasCL) then

		project ( "OpenCL_lds_bank_conflict_" .. vendor)

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
			"../../src/Bullet3Common/b3AlignedAllocator.cpp",
			"../../src/Bullet3Common/b3AlignedAllocator.h",
			"../../src/Bullet3Common/b3AlignedObjectArray.h",
			"../../src/Bullet3Common/b3Quickprof.cpp",
			"../../src/Bullet3Common/b3Quickprof.h",
			
		}
	end
	
end

createProject("AMD")
createProject("NVIDIA")
createProject("Intel")
createProject("Apple")
