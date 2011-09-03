--add the 0 so the solution comes first in the directory (when sorted on name)

solution "0BulletSolution"

	-- Multithreaded compiling
	if _ACTION == "vs2010" or _ACTION=="vs2008" then
		buildoptions { "/MP"  }
	end 
	
	newoption {
    trigger     = "with-nacl",
    description = "Enable Native Client build"
  }
  
  newoption {
    trigger     = "with-dx11",
    description = "Enable DirectX11 build"
  }
  
  
	configurations {"Release", "Debug"}
	configuration "Release"
		flags { "Optimize", "StaticRuntime", "NoMinimalRebuild", "FloatFast"}
	configuration "Debug"
		flags { "Symbols", "StaticRuntime" , "NoMinimalRebuild", "NoEditAndContinue" ,"FloatFast"}
		
	platforms {"x32", "x64"}

	configuration "x64"		
		targetsuffix "_64"
	configuration {"x64", "debug"}
		targetsuffix "_x64_debug"
	configuration {"x64", "release"}
		targetsuffix "_x64"
	configuration {"x32", "debug"}
		targetsuffix "_debug"

  configuration {"Windows"}
  	defines { "_CRT_SECURE_NO_WARNINGS","_CRT_SECURE_NO_DEPRECATE"}
  
	configuration{}



if not _OPTIONS["with-nacl"] then
		flags { "NoRTTI", "NoExceptions"}
		defines { "_HAS_EXCEPTIONS=0" }
		targetdir "../bin"
else
	targetdir "../bin_html"
end


	dofile ("findOpenCL.lua")
	dofile ("findDirectX11.lua")
	
	language "C++"
	
	location("./" .. _ACTION)

	if _OPTIONS["with-dx11"] then
		include "../Demos/DX11ClothDemo"
		include "../src/BulletMultiThreaded/GpuSoftBodySolvers/DX11"
	end


  
	if not _OPTIONS["with-dx11"] and not _OPTIONS["with-nacl"] then
		include "../Demos"
  	include "../Extras"
  end
  
   if _OPTIONS["with-nacl"] then
  	include "../Demos/NativeClient"
  else
  	include "../src/LinearMath"	
		include "../src/BulletCollision"	
		include "../src/BulletDynamics"	
		include "../src/BulletSoftBody"	
	end
