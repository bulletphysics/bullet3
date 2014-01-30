
	function findOpenCL_clew()
		return true;
	end

	function findOpenCL_Apple()
--		if os.is("macosx") then
--			return true	
--		else
			return false
--		end
	end

	
	function findOpenCL_AMD()
--		local amdopenclpath = os.getenv("AMDAPPSDKROOT")
--		if (amdopenclpath) then
--			return true
--		end
		return false
	end

	function findOpenCL_NVIDIA()
--		local nvidiaopenclpath = os.getenv("CUDA_PATH")
--		if (nvidiaopenclpath) then
--			return true
--		end
		return false
	end

	function findOpenCL_Intel()
--		if os.is("Windows") then
--			local intelopenclpath = os.getenv("INTELOCLSDKROOT")
--			if (intelopenclpath) then
--			return true
--			end
--		end
--		if os.is("Linux") then
--			local intelsdk = io.open("/usr/include/CL/opencl.h","r")
--			if (intelsdk) then
--				return true;
--			end
--		end
		return false
	end
		
	function initOpenCL_clew()
		configuration{}
		includedirs {
			projectRootDir .. "src/clew"
		}
		defines {"B3_USE_CLEW"}
		files {
			projectRootDir .. "src/clew/clew.c",
			projectRootDir .. "src/clew/clew.h"
		}
		 if os.is("Linux") then
        	        links {"dl"}
        	end
	end

	function initOpenCL_Apple()
		configuration{}
		includedirs {
			"/System/Library/Frameworks/OpenCL.framework"
		}
		libdirs "/System/Library/Frameworks/OpenCL.framework"
		links
		{
			"OpenCL.framework"
		}
	end
	
	function initOpenCL_AMD()
		configuration {}
		local amdopenclpath = os.getenv("AMDAPPSDKROOT")
		if (amdopenclpath) then
			defines { "ADL_ENABLE_CL" , "CL_PLATFORM_AMD"}
			includedirs {
				"$(AMDAPPSDKROOT)/include"				
			}
			configuration "x32"
				libdirs {"$(AMDAPPSDKROOT)/lib/x86"}
			configuration "x64"
				libdirs {"$(AMDAPPSDKROOT)/lib/x86_64"}
			configuration {}
			links {"OpenCL"}
			return true
		end
		return false
	end


	function initOpenCL_NVIDIA()
		configuration {}
		local nvidiaopenclpath = os.getenv("CUDA_PATH")
		if (nvidiaopenclpath) then
			defines { "ADL_ENABLE_CL" , "CL_PLATFORM_NVIDIA"}
			includedirs {
				"$(CUDA_PATH)/include"				
			}
			configuration "x32"
				libdirs {"$(CUDA_PATH)/lib/Win32"}
			configuration "x64"
				libdirs {"$(CUDA_PATH)/lib/x64"}
			configuration {}
			links {"OpenCL"}
			return true
		end
		return false
	end

	function initOpenCL_Intel()
		configuration {}
		if os.is("Windows") then
		local intelopenclpath = os.getenv("INTELOCLSDKROOT")
		if (intelopenclpath) then
			defines { "ADL_ENABLE_CL" , "CL_PLATFORM_INTEL"}
			includedirs {
				"$(INTELOCLSDKROOT)/include"				
			}
			configuration "x32"
				libdirs {"$(INTELOCLSDKROOT)/lib/x86"}
			configuration "x64"
				libdirs {"$(INTELOCLSDKROOT)/lib/x64"}
			configuration {}
			links {"OpenCL"}
			return true
		end
		end
		if os.is("Linux") then
			defines { "ADL_ENABLE_CL" , "CL_PLATFORM_INTEL"}
                        configuration {}
                        links {"OpenCL"}
		end
		return false
	end
	
	function findOpenCL (vendor )
		if vendor=="clew" then
			return findOpenCL_clew()
		end
		if vendor=="AMD" then
			return findOpenCL_AMD()
		end
		if vendor=="NVIDIA" then
			return findOpenCL_NVIDIA()
		end
			if vendor=="Intel" then
			return findOpenCL_Intel()
		end
		if vendor=="Apple" then
			return findOpenCL_Apple()
		end
		return false
	end
	
	function initOpenCL ( vendor )
		if vendor=="clew" then
			initOpenCL_clew()
		end
		if vendor=="AMD" then
			initOpenCL_AMD()
		end
		if vendor=="NVIDIA" then
			return initOpenCL_NVIDIA()
		end
		if vendor=="Intel" then
			initOpenCL_Intel()
		end
		if vendor=="Apple" then
			return initOpenCL_Apple()
		end
	end
	
