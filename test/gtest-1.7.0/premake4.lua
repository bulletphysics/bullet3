	project "gtest"
		
	kind "StaticLib"

	files{"src/gtest-all.cc"}

	--defines {"GTEST_HAS_PTHREAD=1"}	
		
	if os.is("Windows") then
		--see http://stackoverflow.com/questions/12558327/google-test-in-visual-studio-2012
		defines {"_VARIADIC_MAX=10"}
	end
	
	

	includedirs {
		".","include"
	}
