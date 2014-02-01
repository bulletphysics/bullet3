	project "gtest"
		
	kind "StaticLib"

	files{"src/gtest-all.cc"}

	--defines {"GTEST_HAS_PTHREAD=1"}	
	--targetdir "../../lib"	

	includedirs {
		".","include"
	}
