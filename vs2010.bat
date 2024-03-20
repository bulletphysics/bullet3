IF NOT EXIST bin mkdir bin

rem find a python version (hopefully just 1) and use this
dir c:\python* /b /ad > tmp1234.txt

set /p myvar1= < tmp1234.txt
set myvar=c:/%myvar1%
del tmp1234.txt

rem you can also override and hardcode the Python path like this (just remove the # hashmark in next line)
rem SET myvar=c:\python-3.5.2

cd build3

premake4 --double --enable_multithreading --midi --enable_pybullet --python_include_dir="%myvar%/include" --python_lib_dir="%myvar%/libs" --targetdir="../bin" vs2010

start vs2010

