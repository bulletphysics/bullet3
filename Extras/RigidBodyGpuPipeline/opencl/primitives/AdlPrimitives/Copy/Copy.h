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


#pragma once

#include <Adl/Adl.h>
#include <AdlPrimitives/Math/Math.h>

namespace adl
{

class CopyBase
{
	public:
		enum Option
		{
			PER_WI_1, 
			PER_WI_2, 
			PER_WI_4, 
		};
};

template<DeviceType TYPE>
class Copy : public CopyBase
{
	public:
		typedef Launcher::BufferInfo BufferInfo;

		struct Data
		{
			const Device* m_device;
			Kernel* m_copy1F4Kernel;
			Kernel* m_copy2F4Kernel;
			Kernel* m_copy4F4Kernel;
			Kernel* m_copyF1Kernel;
			Kernel* m_copyF2Kernel;
			Buffer<int4>* m_constBuffer;
		};

		static
		Data* allocate(const Device* deviceData);

		static
		void deallocate(Data* data);

		static
		void execute( Data* data, Buffer<float4>& dst, Buffer<float4>& src, int n, Option option = PER_WI_1);

		static
		void execute( Data* data, Buffer<float2>& dst, Buffer<float2>& src, int n);

		static
		void execute( Data* data, Buffer<float>& dst, Buffer<float>& src, int n);
};


#include <AdlPrimitives/Copy/CopyHost.inl>
#include <AdlPrimitives/Copy/Copy.inl>

};
