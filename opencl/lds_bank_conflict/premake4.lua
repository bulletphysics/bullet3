
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
			"../basic_initialize/btOpenCLUtils.cpp",
			"../basic_initialize/btOpenCLUtils.h",
			"../../src/BulletCommon/btAlignedAllocator.cpp",
			"../../src/BulletCommon/btAlignedAllocator.h",
			"../../src/BulletCommon/btAlignedObjectArray.h",
			"../../src/BulletCommon/btQuickprof.cpp",
			"../../src/BulletCommon/btQuickprof.h",
			
		}
	end
	
end

createProject("AMD")
createProject("NVIDIA")
createProject("Intel")
createProject("Apple")
