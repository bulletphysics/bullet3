
project "App_SharedMemoryPhysics"

if _OPTIONS["ios"] then
	kind "WindowedApp"
else	
	kind "ConsoleApp"
end

includedirs {"../../src"}

links {
	"Bullet3Common",	"BulletDynamics","BulletCollision", "LinearMath"
}

language "C++"

files {
	"**.cpp",
	"**.h",
}

