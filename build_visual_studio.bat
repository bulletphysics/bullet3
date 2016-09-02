
rem premake4 --with-pe  vs2010
rem premake4  --bullet2demos vs2010
cd build3
premake4   --enable_openvr  --targetdir="../bin" vs2010 

rem premake4   --enable_pybullet --python_include_dir="c:/python-3.5.2/include" --python_lib_dir="c:/python-3.5.2/libs" --enable_openvr  --targetdir="../bin" vs2010 
rem premake4   --enable_openvr --enable_pybullet --targetdir="../bin" vs2010 
rem premake4   --targetdir="../server2bin" vs2010 
rem cd vs2010
rem rename 0_Bullet3Solution.sln 0_server.sln
rem cd ..
rem rename vs2010 vs2010_server
rem 
rem premake4   --targetdir="../client2bin" vs2010 
rem cd vs2010
rem rename 0_Bullet3Solution.sln 0_client.sln
rem cd ..
rem rename vs2010 vs2010_client
rem start vs2010/0_Bullet3Solution.sln

pause
