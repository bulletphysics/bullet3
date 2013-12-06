
project "AppBenchmarks"

if _OPTIONS["ios"] then
	kind "WindowedApp"
else	
	kind "ConsoleApp"
end

includedirs {"../../src"}

links {
	"BulletDynamics","BulletCollision", "LinearMath"
}

language "C++"

files {
	"**.cpp",
	"**.h",
}

