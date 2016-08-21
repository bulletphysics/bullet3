project ("Swig")

	language "C++"
	kind "SharedLib"
	targetsuffix ("")
	targetprefix ("")
	targetname("_bullet")
	targetextension(".so")
	targetdir ("../../examples/Swig")

	if _ACTION == "clean" then
	    os.rm("bullet.py")
	    os.rm("bullet.pyc")
	    os.rm("bullet_wrap.cxx")
	end

	-- RUN SWIG
	if os.is("MacOSX") then
	  prebuildcommands { "swig -python -c++ -D__APPLE__ -D__x86_64__ -I../../src ../../examples/Swig/bullet.i " }
	else
	  prebuildcommands { "swig -python -c++ -D__x86_64__ -I../../src ../../examples/Swig/bullet.i " }
	end

	links{"BulletInverseDynamics", "BulletDynamics","BulletCollision","LinearMath","Bullet3Common"}

  	includedirs {
                ".",
                "../../src",
                }

	if os.is("MacOSX") then
		links{"Python"}
	end

	files {
		  "bullet_wrap.cxx",
		  }

	includedirs {
		_OPTIONS["python_include_dir"],
	}
	libdirs {
		_OPTIONS["python_lib_dir"]
	}
