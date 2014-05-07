
project "AppUnitTest"

if _OPTIONS["ios"] then
	kind "WindowedApp"
else	
	kind "ConsoleApp"
end
targetdir "bin"

includedirs {"../src","Source", "Source/Tests"}

links {
	"BulletDynamics","BulletCollision", "LinearMath"
}

language "C++"

files {
	"Source/**.cpp",
	"Source/**.h",
}

