		

project ("pybullet")

		language "C++"
		kind "SharedLib"
		targetsuffix ("")
		targetprefix ("")
		targetextension (".so")
		includedirs {"../../src", "../../examples",
		"../../examples/ThirdPartyLibs"}
		defines {"PHYSICS_IN_PROCESS_EXAMPLE_BROWSER"}
	hasCL = findOpenCL("clew")

	links{"BulletExampleBrowserLib","gwen", "OpenGL_Window","BulletSoftBody", "BulletInverseDynamicsUtils", "BulletInverseDynamics", "BulletDynamics","BulletCollision","LinearMath","Bullet3Common"}
	initOpenGL()
	initGlew()

  	includedirs {
                ".",
                "../../src",
                "../ThirdPartyLibs",
		"/usr/include/python2.7",
                }


	if os.is("MacOSX") then
		links{"Cocoa.framework","Python"}
	end

		if (hasCL) then
			links {
				"Bullet3OpenCL_clew",
				"Bullet3Dynamics",
				"Bullet3Collision",
				"Bullet3Geometry",
				"Bullet3Common",
			}
		end

		files {
			"pybullet.c",
			"../../examples/ExampleBrowser/ExampleEntries.cpp",
			}
	if os.is("Linux") then
       		initX11()
	end

	
