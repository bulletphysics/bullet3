	project "BulletWorldImporter"
		
	kind "StaticLib"
	
	includedirs {
		"../BulletFileLoader",
		"../../../src"
	}

    if os.is("Linux") then
        buildoptions{"-fPIC"}
    end
    	 
	files {
		"**.cpp",
		"**.h"
	}