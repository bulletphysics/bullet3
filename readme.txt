
Bullet 3.x GPU rigid body pipeline using OpenCL.

Note that the Bullet 2.x svn repository from http://bullet.googlecode.com
is being merged into this repository. 

1) The old Bullet2 demos are moved from ObsoleteDemos to AllBullet2Demos
2) A new Bullet 3 API is created
3) All Bullet2 functionality is added to Bullet 3
Until this is done, you can use the Demos3/BasicGpuDemo/b3GpuDynamicsWorld
or explore the Demos3/GpuDemos to check out Bullet 3.

You can still use svn or svn externals using the github git repository: use svn co https://github.com/erwincoumans/bullet3/trunk

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
./premake4_linux gmake
./premake4_linux64 gmake
./premake4_osx gmake

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

[--selected_demo=<int>]             Start with a selected demo
[--benchmark]                       Run benchmark and export results to file 
[--maxFrameCount=<int>]             Run the benchmark for <int> frames
[--dump_timings]                    Print the profile timings to console
[--cl_device=<int>]                 Choose a certain OpenCL device
[--cl_platform=<int>]               Choose a certain OpenCL platform
[--disable_cached_cl_kernels]       Disable loading cached binary OpenCL kernels
[--x_dim=<int>]                     Change default demo settings (x,y,z)
[--pair_benchmark_file=<filename>]  Load AABB's from disk for the PairBench
[--no_instanced_collision_shapes]   Disable collision shape instancing (for tests)
[--no_shadow_map]                   Disable shadowmap rendering
[--shadowmap_resolution=<int>]      Change the resolution of the shadowmap
[--shadowmap_size=<int>]            Change the worldspace size of the shadowmap
[--use_uniform_grid]                Use uniform grid broadphase (no all scenes work)
[--use_jacobi]                      Use GPU parallel Jacobi solver (instead of PGS)
[--use_large_batches]               Use a different strategy for the constrains solver
[--debug_kernel_launch]             Show debug info at start/end of each kernel launch
[--use_dbvt]                        Use the CPU dynamic BVH tree broadphase
[--allow_opencl_cpu]                Allow to use an OpenCL CPU device


You can use mouse picking to grab objects. When holding the ALT of CONTROL key, you have Maya style camera mouse controls.
Press F1 to create a screenshot. Hit ESCAPE to exit the demo app.


Bullet 3.x only implements a small set of collision shapes and constraints:

Static plane 
Static concave triangle mesh
Sphere
Convex Polyhedron
Compound of Convex Polyhedra

Bullet 3.x uses the separating axis test (SAT) between convex polyhedra, testing all vertex - face and edge - edge combinations. For performance it is best to keep the number of edges in a convex polyhedron low, using simple shapes such as a tetrahedron or a box.

The constraint solver currently supports two constraints:

point to point constraint (ball-socket
fixed constraint

It can be extended to other constraint types. The constraint solver uses the same algorithm as Bullet 2.x.

It is possibly to try out Bullet 3.x using the Bullet 2.x API, using the b3GpuDynamicsWorld bridge:
Copy the source code of both versions in the same location, and click on build3/vs2010_bullet2gpu.bat to generate Visual Studio project files.

See docs folder for further information.

