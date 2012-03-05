/*
		2011 Takahiro Harada
*/

#pragma once

#include <Adl/Adl.h>
#include <AdlPrimitives/Math/Math.h>
#include <AdlPrimitives/Sort/SortData.h>
#include <AdlPrimitives/Scan/PrefixScan.h>

namespace adl
{

class RadixSortBase
{
	public:
		enum Option
		{
			SORT_SIMPLE,
			SORT_STANDARD, 
			SORT_ADVANCED
		};
};

template<DeviceType TYPE>
class RadixSort : public RadixSortBase
{
	public:
		struct Data
		{
			Option m_option;
			const Device* m_deviceData;
			typename PrefixScan<TYPE>::Data* m_scanData;
			int m_maxSize;
		};
		

		static
		Data* allocate(const Device* deviceData, int maxSize, Option option = SORT_STANDARD);

		static
		void deallocate(Data* data);

		static
		void execute(Data* data, Buffer<SortData>& inout, int n, int sortBits = 32);
};


#include <AdlPrimitives/Sort/RadixSort.inl>
#include <AdlPrimitives/Sort/RadixSortHost.inl>

};
