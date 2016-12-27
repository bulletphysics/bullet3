

project ("Test_enet_chat_server")

	language "C++"
			
	kind "ConsoleApp"
	
	includedirs {"../../../../examples/ThirdPartyLibs/enet/include"}
	
	if os.is("Windows") then 
			defines { "WIN32" }

		links {"Ws2_32","Winmm"}
	end
	if os.is("Linux") then
	end
	if os.is("MacOSX") then
	end		
		
	links {"enet"}		
	
	files {
		"main.cpp",
	}

