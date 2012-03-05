/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Takahiro Harada



//ADL_ENABLE_CL and ADL_ENABLE_DX11 can be set in the build system using C/C++ preprocessor defines
//#define ADL_ENABLE_CL
//#define ADL_ENABLE_DX11

//#define ADL_CL_FORCE_UNCACHE_KERNEL
#define ADL_CL_DUMP_MEMORY_LOG

//load the kernels from string instead of loading them from file
#define ADL_LOAD_KERNEL_FROM_STRING
#define ADL_DUMP_DX11_ERROR
