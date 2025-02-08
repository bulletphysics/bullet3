# Bullet Physics SDK

This is the official C++ source code repository of the Bullet Physics SDK: real-time collision detection and multi-physics simulation for VR, games, visual effects, robotics, machine learning etc.

![PyBullet](https://pybullet.org/wordpress/wp-content/uploads/2019/03/cropped-pybullet.png)

[![Travis Build Status](https://api.travis-ci.org/bulletphysics/bullet3.png?branch=master)](https://travis-ci.org/bulletphysics/bullet3)
[![Appveyor Build status](https://ci.appveyor.com/api/projects/status/6sly9uxajr6xsstq)](https://ci.appveyor.com/project/erwincoumans/bullet3)

## Note
At the current moment in time this is the home of the bullet physics the c++ library as well as [pybullet](https://pypi.org/project/pybullet/) the python package that has bindings for the c++ code.

## Getting help
* If you have a github account, feel free to use the ![built-in discussions](https://github.com/bulletphysics/bullet3/discussions)
* You can also use the long standing ![forum](https://pybullet.org/Bullet/phpBB3/)

## Index
* [Bullet C++](#bullet-c)
* [pybullet](#pybullet)
* [Running Examples](#running-examples)
* [Contributing](#contributing)
* [License](#license)

---

# Bullet C++

## Building

### Requirements 

A C++ compiler for C++ 2003. The library is tested on Windows, Linux, Mac OSX, iOS, Android,
but should likely work on any platform with C++ compiler. 
Some optional demos require OpenGL 2 or OpenGL 3, there are some non-graphical demos and unit tests too.


### [cmake](https://cmake.org)
While inside the root of this repository:
```
mkdir my_build
cmake -S . -B my_build
cd my_build
cmake --build .
```

### [vcpkg](https://github.com/Microsoft/vcpkg/)
```
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
./vcpkg integrate install
./vcpkg install bullet3
```

The Bullet port in vcpkg is kept up to date by Microsoft team members and community contributors. If the version is out of date, please [create an issue or pull request](https://github.com/Microsoft/vcpkg) on the vcpkg repository.



## Examples




---

# PyBullet
It is highly recommended to use PyBullet Python bindings for improved support for robotics, reinforcement learning and VR. Use pip install pybullet and checkout the [PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3).

## Installation

```
pip3 install pybullet --upgrade --user
python3 -m pybullet_envs.examples.enjoy_TF_AntBulletEnv_v0_2017may
python3 -m pybullet_envs.examples.enjoy_TF_HumanoidFlagrunHarderBulletEnv_v1_2017jul
python3 -m pybullet_envs.deep_mimic.testrl --arg_file run_humanoid3d_backflip_args.txt
```

If you use PyBullet in your research, please cite it like this:

```
@MISC{coumans2021,
author =   {Erwin Coumans and Yunfei Bai},
title =    {PyBullet, a Python module for physics simulation for games, robotics and machine learning},
howpublished = {\url{http://pybullet.org}},
year = {2016--2021}
}
```

## Building

**Windows**

Click on build_visual_studio_vr_pybullet_double.bat and open build3/vs2010/0_Bullet3Solution.sln
When asked, convert the projects to a newer version of Visual Studio.
If you installed Python in the C:\ root directory, the batch file should find it automatically.
Otherwise, edit this batch file to choose where Python include/lib directories are located.

**Linux**

Make sure gcc and cmake is installed (`sudo apt-get install build-essential` and `sudo apt-get install cmake` for Linux, `brew install cmake` for Mac, or https://cmake.org)

In a terminal type:
```
./build_cmake_pybullet_double.sh
```
This script will invoke cmake and build in the build_cmake directory. You can find pybullet in Bullet/examples/pybullet.

Note: On Linux, you need to use cmake to build pybullet, since the compiler has issues of mixing shared and static libraries.

**Mac OSX**

Follow the Linux instructions.

Alternatively if you use Xcode, then click on build3/xcode4.command or in a terminal window execute
```	
./premake_osx xcode4
```

<details>
<summary>other systems</summary
**Windows Virtual Reality sandbox for HTC Vive and Oculus Rift**

Build and run the App_SharedMemoryPhysics_VR project, preferably in Release/optimized build.
You can connect from Python pybullet to the sandbox using:

```
import pybullet as p
p.connect(p.SHARED_MEMORY) #or (p.TCP, "localhost", 6667) or (p.UDP, "192.168.86.10",1234)
```
</details>


# Running Examples

The main thing you'll want to look at is the BulletExampleBrowser executable which can be found under examples/ExampleBrowser.
It contains a gui allowing you to select various other examples and view them.

The App_ExampleBrowser executables will be located in the bin folder.
You can just run it though a terminal/command prompt, or by clicking it.

```
[--start_demo_name="Demo Name"]     Start with a selected demo  
[--mp4=moviename.mp4]               Create a mp4 movie of the window, requires ffmpeg installed
[--mouse_move_multiplier=0.400000]  Set the mouse move sensitivity
[--mouse_wheel_multiplier=0.01]     Set the mouse wheel sensitivity
[--background_color_red= 0.9]       Set the red component for background color. Same for green and blue
[--fixed_timestep= 0.0]             Use either a real-time delta time (0.0) or a fixed step size (0.016666)
```

You can use mouse picking to grab objects. When holding the ALT or CONTROL key, you have Maya style camera mouse controls.
Press F1 to create a series of screenshots. Hit ESCAPE to exit the demo app.

# Experimental [OpenCL](https://www.khronos.org/opencl/) [GPGPU](https://en.wikipedia.org/wiki/General-purpose_computing_on_graphics_processing_units) support

The entire collision detection and rigid body dynamics can be executed on the GPU.

A high-end desktop GPU, such as an AMD Radeon 7970 or NVIDIA GTX 680 or better.
We succesfully tested the software under Windows, Linux and Mac OSX.
The software currently doesn't work on OpenCL CPU devices. It might run
on a laptop GPU but performance will not likely be very good. Note that
often an OpenCL drivers fails to compile a kernel. Some unit tests exist to
track down the issue, but more work is required to cover all OpenCL kernels.

# Contributing

Please read: https://docs.google.com/document/d/1u9vyzPtrVoVhYqQOGNWUgjRbfwfCdIts_NzmvgiJ144/edit


# License

All source code files are licensed under the permissive zlib license
(http://opensource.org/licenses/Zlib) unless marked differently in a particular folder/file.


Check out the docs folder and the Bullet physics forums for further information.
