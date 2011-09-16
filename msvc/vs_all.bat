rem this script is mainly to create distributable visual studio project file


premake4 --with-opencl-nvidia vs2008
rename vs2008 vs2008_opencl_nvidia

premake4 --with-opencl-intel vs2008
rename vs2008 vs2008_opencl_intel

premake4 --with-opencl-amd vs2008
rename vs2008 vs2008_opencl_amd

premake4 --with-opencl-nvidia vs2010
rename vs2010 vs2010_opencl_nvidia

premake4 --with-opencl-intel vs2010
rename vs2010 vs2010_opencl_intel

premake4 --with-opencl-amd vs2010
rename vs2010 vs2010_opencl_amd

premake4 --with-dx11 vs2008
rename vs2008 vs2008_dx11

premake4 --with-dx11 vs2010
rename vs2010 vs2010_dx11

premake4 --with-dx11 vs2005
rename vs2005 vs2005_dx11

premake4 vs2005
premake4 vs2008
premake4 vs2010


pause