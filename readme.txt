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

./App_Bullet3_OpenCL_Demos_clew_gmake --cl_platform=1 --cl_device=1 --

[--selected_demo=<int>]   Start with a selected demo: 
[--benchmark]             Run benchmark and export results to file 
[--maxFrameCount=<int>]   Run the benchmark for <int> frames
[--dump_timings]          Print the profile timings to console
[--disable_opencl]        Don't use OpenCL (ignore this option for now)
[--cl_device=<int>]       Choose a certain OpenCL device, if the platform has more than 0 devices
[--cl_platform=<int>]     Choose a certain OpenCL platform, if the machine has multiple platforms
[--disable_cached_cl_kernels] Disable loading cached binary OpenCL kernels, useful for CodeXL debugging
[--x_dim=<int>]               Change default demo settings (same for y, z and x_gap etc)
[--pair_benchmark_file=<filename>] Load AABB's from disk for the PairBench
[--load_cl_kernels_from_disk] Force loading OpenCL kernels from disk (this feature is currently broken)
[--no_instanced_collision_shapes]  Disable collision shape instancing (for tests)


You can use mouse picking to grab objects. When holding the ALT of CONTROL key, you have Maya style camera mouse controls.
Press F1 to create a screenshot. Hit ESCAPE to exit the demo app.

See docs folder for further information and see the SIGGRAPH 2013 course notes at
http://www.multithreadingandvfx.org/course_notes


