	project "PhysX"

	kind "StaticLib"
	if os.is("Linux") then
	    buildoptions{"-fPIC"}
	end

	defines {
	"PX_PHYSX_STATIC_LIB",
	"PX_COOKING",
	"PX_FOUNDATION_DLL=0"
	}
    
	configuration {"x64", "debug"}			
			defines {"_DEBUG"}
	configuration {"x86", "debug"}
			defines {"_DEBUG"}
	configuration {"x64", "release"}
			defines {"NDEBUG"}
	configuration {"x86", "release"}
			defines {"NDEBUG"}
	configuration{}
				
	includedirs {
		"physx/source/common/include",
    "physx/source/common/src",
    "physx/source/fastxml/include",
    "physx/source/filebuf/include",
    "physx/source/foundation/include",
    "physx/source/geomutils/include",
    "physx/source/geomutils/src",
    "physx/source/geomutils/src/ccd",
    "physx/source/geomutils/src/common",
    "physx/source/geomutils/src/contact",
    "physx/source/geomutils/src/convex",
    "physx/source/geomutils/src/distance",
    "physx/source/geomutils/src/gjk",
    "physx/source/geomutils/src/hf",
    "physx/source/geomutils/src/intersection",
    "physx/source/geomutils/src/mesh",
    "physx/source/geomutils/src/pcm",
    "physx/source/geomutils/src/sweep",
    "physx/source/lowlevel/api/include",
    "physx/source/lowlevel/common/include",
    "physx/source/lowlevel/common/include/collision",
    "physx/source/lowlevel/common/include/pipeline",
    "physx/source/lowlevel/common/include/utils",
    "physx/source/lowlevel/software/include",
    "physx/source/lowlevelaabb/include",
    "physx/source/lowleveldynamics/include",
    "physx/source/physx/src",
    "physx/source/physx/src/buffering",
    "physx/source/physx/src/device",
    "physx/source/physxcooking/src",
    "physx/source/physxcooking/src/convex",
    "physx/source/physxcooking/src/mesh",
    "physx/source/physxextensions/src",
    "physx/source/physxextensions/src/serialization/Binary",
    "physx/source/physxextensions/src/serialization/File",
    "physx/source/physxextensions/src/serialization/Xml",
    "physx/source/physxmetadata/core/include",
    "physx/source/physxmetadata/extensions/include",
    "physx/source/physxvehicle/src",
    "physx/source/physxvehicle/src/physxmetadata/include",
    "physx/source/pvd/include",
    "physx/source/scenequery/include",
    "physx/source/simulationcontroller/include",
    "physx/source/simulationcontroller/src",
--public
    "physx/include",
    "physx/include/characterkinematic",
    "physx/include/common",
    "physx/include/cooking",
    "physx/include/extensions",
    "physx/include/geometry",
    "physx/include/geomutils",
    "physx/include/vehicle",
    "pxshared/include",
	}
	
	if os.is("Windows") then
		files {
			"physx/source/common/src/windows/*.cpp",
			"physx/source/foundation/src/windows/*.cpp",
			"physx/source/physx/src/device/windows/*.cpp",
			"physx/source/physx/src/windows/NpWindowsDelayLoadHook.cpp",
		}
		includedirs {
			"physx/include/gpu",
			"physx/source/physxgpu/include",
		}
	else
--		files {
--			"physx/source/foundation/src/unix/*.cpp",
--			"physx/source/physx/src/device/linux/*.cpp",
--		}
	end
	  
	files {
	 "physx/source/common/src/*.cpp",
    "physx/source/fastxml/src/*.cpp",
    "physx/source/foundation/src/*.cpp",
    "physx/source/geomutils/src/**.cpp",
    "physx/source/immediatemode/src/*.cpp",
    "physx/source/lowlevel/api/src/*.cpp",
    "physx/source/lowlevel/common/src/**.cpp",
    "physx/source/lowlevel/software/src/*.cpp",
    "physx/source/lowlevelaabb/src/*.cpp",
    "physx/source/lowleveldynamics/src/*.cpp",
    "physx/source/physx/src/*.cpp",
    "physx/source/physx/src/buffering/*.cpp",
    "physx/source/physx/src/gpu/*.cpp",
    "physx/source/physxcharacterkinematic/src/*.cpp",
    "physx/source/physxcooking/src/*.cpp",
    "physx/source/physxcooking/src/convex/*.cpp",
    "physx/source/physxcooking/src/mesh/*.cpp",
    "physx/source/physxextensions/src/*.cpp",
    "physx/source/physxextensions/src/**.cpp",
    "physx/source/physxmetadata/core/src/*.cpp",
    "physx/source/physxmetadata/extensions/src/*.cpp",
    "physx/source/physxvehicle/src/*.cpp",
    "physx/source/physxvehicle/src/physxmetadata/src/*.cpp",
    "physx/source/pvd/src/*.cpp",
    "physx/source/scenequery/src/*.cpp",
    "physx/source/simulationcontroller/src/*.cpp",
    "physx/source/task/src/*.cpp",	
	}
	