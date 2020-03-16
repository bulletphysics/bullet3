
project "test_vhacd"

if _OPTIONS["ios"] then
	kind "WindowedApp"
else	
	kind "ConsoleApp"
end

includedirs {"../../public",
"../../../src"}

links {
	"vhacd", "LinearMath"
}

language "C++"

files {
	"main.cpp",
	"main_vhacd.cpp",
}


if os.is("Linux") then 
		links {"pthread"}
end