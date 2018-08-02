	project "Bullet3Geometry"

	language "C++"
				
	kind "StaticLib"
		
	includedirs {".."}
	
    if os.is("Linux") then
        buildoptions{"-fPIC"}
    end

	files {
		"**.cpp",
		"**.h"
	}