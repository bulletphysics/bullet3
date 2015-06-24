

project ("App_LuaCompiler")

	language "C++"
			
	kind "ConsoleApp"
	targetdir "../../../../bin"
	includedirs {"../src"}
	
	if os.is("Windows") then 
	end
	if os.is("Linux") then
	end
	if os.is("MacOSX") then
	end		
		
	links {"lua-5.2.3"}		
	
	files {
		"luac.c",
	}

