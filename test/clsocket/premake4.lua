

project ("Test_clsocket_EchoServer")

	language "C++"
			
	kind "ConsoleApp"
	
	includedirs {"../../examples/ThirdPartyLibs/clsocket/src"}
	
	if os.is("Windows") then 
		defines { "WIN32" }
		links {"Ws2_32","Winmm"}
	end
	if os.is("Linux") then
		defines {"_LINUX"}
	end
	if os.is("MacOSX") then
		defines {"_DARWIN"}
	end		
		
	links {"clsocket"}		
	
	files {
		"EchoServer.cpp",
	}




project ("Test_clsocket_QueryDayTime")

	language "C++"
			
	kind "ConsoleApp"
	
	includedirs {"../../examples/ThirdPartyLibs/clsocket/src"}
	
	if os.is("Windows") then 
		defines { "WIN32" }
		links {"Ws2_32","Winmm"}
	end

	if os.is("Linux") then
                defines {"_LINUX"}
        end
        if os.is("MacOSX") then
                defines {"_DARWIN"}
        end

	links {"clsocket"}		
	
	files {
		"QueryDayTime.cpp",
	}

