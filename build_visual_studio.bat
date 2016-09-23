
cd build3
rem premake4   --enable_openvr  --targetdir="../bin" vs2010 

premake4   --double --enable_pybullet --python_include_dir="C:/Python-3.5.2/include" --python_lib_dir="C:/Python-3.5.2/libs" --enable_openvr  --targetdir="../bin" vs2010 
pause
