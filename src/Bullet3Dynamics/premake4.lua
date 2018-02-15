	project "Bullet3Dynamics"

	language "C++"
				
	kind "StaticLib"

	includedirs {
		".."
	}		
	
    if os.is("Linux") then
        buildoptions{"-fPIC"}
    end

	files {
		"**.cpp",
		"**.h"
	}