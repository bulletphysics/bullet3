	project "BulletXmlWorldImporter"
		
	kind "StaticLib"
	--targetdir "../../lib"
	includedirs {
		"../BulletWorldImporter",
		"../BulletFileLoader",
		"../../../src",
		"../../../examples/ThirdPartyLibs/tinyxml2"
	}
	 
	files {
		"**.cpp",
		"**.h",
		"../../../examples/ThirdPartyLibs/tinyxml2/tinyxml2.cpp",
	}
