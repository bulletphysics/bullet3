	
	hasDX11 = findDirectX11()
	
	if (hasDX11) then

		project "App_DX11ClothDemo"

		initDirectX11()
		
		language "C++"
				
		defines { "UNICODE","_UNICODE"}
				
		kind "WindowedApp"
		flags { "WinMain" }
		
		targetdir "../.."
		includedirs {
			"../../src",
			"DXUT/Core",
			"DXUT/Optional"
		}
		
		links { 
			"LinearMath","BulletCollision","BulletDynamics", "BulletSoftBody", "BulletSoftBodyDX11Solvers"
		}
		files {
			"DXUT/Core/DXUT.cpp",
			"DXUT/Optional/DXUTcamera.cpp",
			"DXUT/Core/DXUTDevice11.cpp",
			"DXUT/Core/DXUTDevice9.cpp",
			"DXUT/Optional/DXUTgui.cpp",
			"DXUT/Core/DXUTmisc.cpp",
			"DXUT/Optional/DXUTres.cpp",
			"DXUT/Optional/DXUTsettingsdlg.cpp",
			"DXUT/Optional/SDKmesh.cpp",
			"DXUT/Optional/SDKmisc.cpp",	
			"cloth_renderer.cpp"
		}
		
	end