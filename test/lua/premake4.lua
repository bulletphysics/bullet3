

project ("Test_Lua")

	language "C++"
			
	kind "ConsoleApp"
	targetdir "../../bin"
	includedirs {"../../btgui/lua-5.2.3/src"}
	
	if os.is("Windows") then 
	end
	if os.is("Linux") then
	end
	if os.is("MacOSX") then
	end		
		
	links {"lua-5.2.3"}		
	
	files {
		"main.cpp",
	}

