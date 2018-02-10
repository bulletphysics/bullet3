	project "BulletFileLoader"
		
	kind "StaticLib"
	
	if os.is("Linux") then
	    buildoptions{"-fPIC"}
	end
	 
	includedirs {
		"../../../src"
	}
	 
	files {
		"**.cpp",
		"**.h"
	}