function createProject(vendor)
	hasCL = findOpenCL(vendor)

	if (hasCL) then

		project ("Bullet3OpenCL_" .. vendor)

		initOpenCL(vendor)

		configuration { "*" }
			kind "StaticLib"
		configuration { "*DLL" }
			kind "SharedLib"
		configuration {}

		includedirs {
			".",".."
		}

		files {
			"**.cpp",
			"**.h"
		}

	end
end

createProject("clew")
createProject("AMD")
createProject("Intel")
createProject("NVIDIA")
createProject("Apple")
