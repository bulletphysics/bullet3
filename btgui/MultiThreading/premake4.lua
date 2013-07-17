
	project "App_ThreadingTest"
		
	kind "ConsoleApp"
	
--	defines {  }
	
	targetdir "../../bin"
	
	includedirs 
	{
		".","../../src"
	}

			
	links { "Bullet3Common" }
	
	
	files {
		"**.cpp",
		"**.h"
	}
	if os.is("Windows") then
		--links {"winmm"}
		--defines {"__WINDOWS_MM__", "WIN32"}
	end

	if os.is("Linux") then 
		links {"pthread"}
	end

	if os.is("MacOSX") then
		links {"pthread"}
		--links{"CoreAudio.framework", "coreMIDI.framework", "Cocoa.framework"}
		--defines {"__MACOSX_CORE__"}
	end
