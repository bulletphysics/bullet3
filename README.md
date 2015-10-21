
[![Travis Build Status](https://api.travis-ci.org/bulletphysics/bullet3.png?branch=master)](https://travis-ci.org/bulletphysics/bullet3)
[![Appveyor Build status](https://ci.appveyor.com/api/projects/status/6sly9uxajr6xsstq)](https://ci.appveyor.com/project/erwincoumans/bullet3)

# Bullet 2.x including experimental Bullet 3 OpenCL.

This is the official repository of Bullet 2.x, moved from http://bullet.googlecode.com
It includes the optional work-in-progress Bullet 3 GPU pipeline.

The Bullet 2 API will stay default and up-to-date while slowly moving to Bullet 3.
The steps towards Bullet 3 are in a nutshell:

1. The old Bullet2 demos are being merged into the examples/ExampleBrowser
2. A new Bullet 3 API is created
3. All Bullet2 functionality is added to Bullet 3.
4. The OpenCL examples in the ExampleBrowser can be enabled using --enable_experimental_opencl

You can still use svn or svn externals using the github git repository: use svn co https://github.com/bulletphysics/bullet3/trunk

## Requirements for Bullet 2

A C++ compiler for C++ 2003. The library is tested on Windows, Linux, Mac OSX, iOS, Android,
but should likely work on any platform with C++ compiler. 
Some optional demos require OpenGL 2 or OpenGL 3, there are some non-graphical demos and unit tests too.

## Contributors and Coding Style information

https://docs.google.com/document/d/1u9vyzPtrVoVhYqQOGNWUgjRbfwfCdIts_NzmvgiJ144/edit

## Requirements for Bullet 3

The entire collision detection and rigid body dynamics is executed on the GPU.

A high-end desktop GPU, such as an AMD Radeon 7970 or NVIDIA GTX 680 or better.
We succesfully tested the software under Windows, Linux and Mac OSX.
The software currently doesn't work on OpenCL CPU devices. It might run
on a laptop GPU but performance is likely not very good. Note that
often an OpenCL drivers fails to compile a kernel. Some unit tests exist to
track down the issue, but more work is required to cover all OpenCL kernels.

## License

All source code files are licensed under the permissive zlib license
(http://opensource.org/licenses/Zlib) unless marked differently in a particular folder/file.

## Build instructions for Bullet using premake. You can also use cmake instead.

**Windows**

Click on build3/vs2010.bat and open build3/vs2010/0MySolution.sln

**Linux and Mac OSX gnu make**

In a terminal type:

	cd build3

Dependend on your system (Linux 32bit, 64bit or Mac OSX) use one of the following lines

	./premake4_linux gmake
	./premake4_linux64 gmake
	./premake4_osx gmake

Then

	cd gmake
	make

**Mac OSX Xcode**
	
Click on build3/xcode4.command or in a terminal window execute
	
	./premake_osx xcode4

## Usage

The App_ExampleBrowser executables will be located in the bin folder.
You can just run it though a terminal/command prompt, or by clicking it.


```
[--start_demo_name="Demo Name"]     Start with a selected demo  
[--enable_experimental_opencl]      Enable some experimental OpenCL examples
[--mp4=moviename.mp4]               Create a mp4 movie of the window, requires ffmpeg installed
[--mouse_move_multiplier=0.400000]  Set the mouse move sensitivity
[--mouse_wheel_multiplier=0.01]     Set the mouse wheel sensitivity
[--background_color_red= 0.9]       Set the red component for background color. Same for green and blue.
[--fixed_timestep= 0.0]             Use either a real-time delta time (0.0) or a fixed step size (0.016666)
```

You can use mouse picking to grab objects. When holding the ALT of CONTROL key, you have Maya style camera mouse controls.
Press F1 to create series of screenshot. Hit ESCAPE to exit the demo app.

See docs folder for further information.
