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

class FillBase
{
	public:
		enum Option
		{

		};
};

template<DeviceType TYPE>
class Fill
{
	public:
		typedef Launcher::BufferInfo BufferInfo;

		struct ConstData
		{
			int4 m_data;
			int m_offset;
			int m_n;
			int m_padding[2];
		};

		struct Data
		{
			const Device* m_device;
			Kernel* m_fillIntKernel;
			Kernel* m_fillInt2Kernel;
			Kernel* m_fillInt4Kernel;
			Buffer<ConstData>* m_constBuffer;
		};

		static
		Data* allocate(const Device* deviceData);

		static
		void deallocate(Data* data);

		static
		void execute(Data* data, Buffer<int>& src, const int& value, int n, int offset = 0);

		static
		void execute(Data* data, Buffer<int2>& src, const int2& value, int n, int offset = 0);

		static
		void execute(Data* data, Buffer<int4>& src, const int4& value, int n, int offset = 0);

};


#include <AdlPrimitives/Fill/FillHost.inl>
#include <AdlPrimitives/Fill/Fill.inl>

};
