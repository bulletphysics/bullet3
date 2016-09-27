
project "test_vhacd"

if _OPTIONS["ios"] then
	kind "WindowedApp"
else	
	kind "ConsoleApp"
end

includedirs {"../../public"}

links {
	"vhacd"
}

language "C++"

files {
	"main.cpp",
}


if os.is("Linux") then 
		links {"pthread"}
end