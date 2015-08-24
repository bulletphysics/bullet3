
	project "App_ThreadingTest"
		
	kind "ConsoleApp"
	
--	defines {  }
	
	
	includedirs 
	{
		".","../../src"
	}

			
	links { "Bullet3Common" }
	
	
	files {
		"b3ThreadSupportInterface.cpp",
		"main.cpp",
		"b3ThreadSupportInterface.h"
	}
	if os.is("Windows") then

		files {
                "b3Win32ThreadSupport.cpp",  
                "b3Win32ThreadSupport.h" 
		}
		--links {"winmm"}
		--defines {"__WINDOWS_MM__", "WIN32"}
	end

	if os.is("Linux") then 
		files {
                "b3PosixThreadSupport.cpp",  
                "b3PosixThreadSupport.h"    
        	}

		links {"pthread"}
	end

	if os.is("MacOSX") then
		files {
                "b3PosixThreadSupport.cpp",
                "b3PosixThreadSupport.h"    
                }

		links {"pthread"}
		--links{"CoreAudio.framework", "coreMIDI.framework", "Cocoa.framework"}
		--defines {"__MACOSX_CORE__"}
	end
