	
		project "App_TinyAudioExample"

		language "C++"
				
		kind "ConsoleApp"

  	includedirs {
                ".",
                "../../src",
                }
			
		defines {"B3_USE_STANDALONE_EXAMPLE", "__STK_REALTIME__"}	
		files {
		"**.cpp",
		"**.h",
		"../StandaloneMain/main_console_single_example.cpp",
		"../Utils/b3ResourcePath.cpp"
		}

		links {"Bullet3Common"}

		if os.is("Windows") then
			links {"winmm","Wsock32","dsound"}
			defines {"WIN32","__WINDOWS_MM__","__WINDOWS_DS__"}
		end

		
if os.is("Linux") then initX11() 
                defines  {"__OS_LINUX__","__LINUX_ALSA__"}
	links {"asound","pthread"}
end


if os.is("MacOSX") then
	links{"Cocoa.framework"}
	links{"CoreAudio.framework", "coreMIDI.framework", "Cocoa.framework"}

	defines {"__OS_MACOSX__", "__MACOSX_CORE__"}
end

