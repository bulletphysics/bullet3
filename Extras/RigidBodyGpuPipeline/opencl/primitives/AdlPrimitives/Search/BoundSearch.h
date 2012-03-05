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
#include <AdlPrimitives/Sort/SortData.h>
#include <AdlPrimitives/Fill/Fill.h>

namespace adl
{

class BoundSearchBase
{
	public:
		enum Option
		{
			BOUND_LOWER,
			BOUND_UPPER,
			COUNT,
		};
};

template<DeviceType TYPE>
class BoundSearch : public BoundSearchBase
{
	public:
		typedef Launcher::BufferInfo BufferInfo;

		struct Data
		{
			const Device* m_device;
			Kernel* m_lowerSortDataKernel;
			Kernel* m_upperSortDataKernel;
			Kernel* m_subtractKernel;
			Buffer<int4>* m_constBuffer;
			Buffer<u32>* m_lower;
			Buffer<u32>* m_upper;
			typename Fill<TYPE>::Data* m_fillData;
		};

		static
		Data* allocate(const Device* deviceData, int maxSize = 0);

		static
		void deallocate(Data* data);

		//	src has to be src[i].m_key <= src[i+1].m_key
		static
		void execute(Data* data, Buffer<SortData>& src, u32 nSrc, Buffer<u32>& dst, u32 nDst, Option option = BOUND_LOWER );

//		static
//		void execute(Data* data, Buffer<u32>& src, Buffer<u32>& dst, int n, Option option = );
};

#include <AdlPrimitives/Search/BoundSearchHost.inl>
#include <AdlPrimitives/Search/BoundSearch.inl>

};
