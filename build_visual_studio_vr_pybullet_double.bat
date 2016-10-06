
IF NOT EXIST bin mkdir bin
IF NOT EXIST bin\openvr_api.dll  copy examples\ThirdPartyLibs\openvr\bin\win32\openvr_api.dll bin

cd build3

premake4   --double --enable_openvr --enable_pybullet --python_include_dir="C:/Python-3.5.2/include" --python_lib_dir="C:/Python-3.5.2/libs"   --targetdir="../bin" vs2010 
start vs2010
