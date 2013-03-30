
	project "rtMidiTest"
		
	kind "ConsoleApp"
	
--	defines {  }
	
	targetdir "../../bin"
	
	includedirs 
	{
		".",
	}

			
--	links { }
	
	
	files {
		"**.cpp",
		"**.h"
	}
	if os.is("Windows") then
		links {"winmm"}
		defines {"__WINDOWS_MM__", "WIN32"}
	end

	if os.is("Linux") then 
	end

	if os.is("MacOSX") then
		links{"Cocoa.framework"}
	end
