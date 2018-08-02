	project "Bullet3Common"

	language "C++"
				
	kind "StaticLib"
		
	if os.is("Linux") then
	    buildoptions{"-fPIC"}
	end

	includedirs {".."}

	files {
		"*.cpp",
		"*.h"
	}
