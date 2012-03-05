if os.is("Windows") then
	
		hasCL = findOpenCL_AMD()
	
		if (hasCL) then
	
		project "basic_bullet2_demo_AMD"

		initOpenCL_AMD()
				
		language "C++"
		
		kind "ConsoleApp"
		targetdir "../../../bin"

  		includedirs {
                "..",
                "../../../bullet2",
                "../../testbed",
                "../../../rendering/Gwen",
                "../../../opencl/basic_initialize",
                "../../../opencl/primitives"
                }
		

		links { "testbed",
			"bullet2",
			"gwen"
		}
		
	
		initOpenGL()
		initGlut()

	
		files {
		"../**.cpp",
		"../**.h",
		"../../../opencl/basic_initialize/btOpenCLUtils.cpp",
		"../../../opencl/basic_initialize/btOpenCLUtils.h"
		}

	end
	
end
