
Bullet 3.x GPU rigid body pipeline using OpenCL.

The entire collision detection and rigid body dynamics is executed on the GPU.

Requirements:
A high-end desktop GPU, such as an AMD Radeon 7970 or NVIDIA GTX 680 or similar.
We succesfully tested the software under Windows, Linux and Mac OSX.
The software currently doesn't work on OpenCL CPU devices. It might run
on a laptop GPU but performance is likely not very good.


License
All source code files are licensed under the permissive zlib license
(http://opensource.org/licenses/Zlib) unless marked differently in a particular folder/file.


Build instructions:

Windows:
Click on build3/vs2010.bat and open build3/vs2010/0MySolution.sln

Linux and Mac OSX gnu make
In a terminal type:

cd build3

Dependend on your system (Linux 32bit, 64bit or Mac OSX) use one of the following lines
./premake_linux gmake
./premake_linux64 gmake
./premake_osx gmake

Then

cd gmake
make

Mac OSX Xcode
Click on build3/xcode4.command or in a terminal window execute
./premake_osx xcode4

Usage:

The main demo executable will be located in the bin folder.
The demo starts with App_Bullet3_OpenCL_Demos_*

You can just run it though a terminal/command prompt, or by clicking it.


There are some command-line options, you can see using the --help option. For example, this will perform a benchmark writing to some files:

./App_Bullet3_OpenCL_Demos_clew_gmake --benchmark

You can use the mouse to grab objects. When holding the ALT of CONTROL key, you have Maya style camera mouse controls.
Press F1 to create a screenshot.
Hit ESCAPE to exit the demo app.

See docs folder for further information.

