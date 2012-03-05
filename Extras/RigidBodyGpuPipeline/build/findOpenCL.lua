	-- todo: add Apple OpenCL environment vars

	function findOpenCL_AMD()
		local amdopenclpath = os.getenv("AMDAPPSDKROOT")
		if (amdopenclpath) then
			return true
		end
		return false
	end

	function findOpenCL_NVIDIA()
		local nvidiaopenclpath = os.getenv("CUDA_PATH")
		if (nvidiaopenclpath) then
			return true
		end
		return false
	end

	function findOpenCL_Intel()
		local intelopenclpath = os.getenv("INTELOCLSDKROOT")
		if (intelopenclpath) then
			return true
		end
		return false
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
		return false
	end
	