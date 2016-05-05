
	project "hello_gtest"
		
	kind "ConsoleApp"
	
--	defines {  }
	
	targetdir "../../bin"
	
	includedirs 
	{
		".","../gtest-1.7.0/include"
	}

	if os.is("Windows") then
		--see http://stackoverflow.com/questions/12558327/google-test-in-visual-studio-2012
		defines {"_VARIADIC_MAX=10"}
	end
	
--        linkLib "gtest"			
	links "gtest"
	
	
	files {
		"**.cpp",
		"**.h",
	--	"../gtest-1.7.0/src/gtest_main.cc"
	}
	if os.is("Windows") then
	end

	if os.is("Linux") then 
	end

	if os.is("MacOSX") then
	end
