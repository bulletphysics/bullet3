/*
		2011 Takahiro Harada
*/

#pragma once

#include <Adl/Adl.h>
#include <AdlPrimitives/Math/Math.h>
#include <AdlPrimitives/Copy/Copy.h>
#include <AdlPrimitives/Sort/SortData.h>

namespace adl
{

class RadixSort32Base
{
	public:
// 		enum Option
// 		{
// 			SORT_SIMPLE,
// 			SORT_STANDARD, 
// 			SORT_ADVANCED
// 		};
};

template<DeviceType TYPE>
class RadixSort32 : public RadixSort32Base
{
	public:
		typedef Launcher::BufferInfo BufferInfo;

		enum
		{
			DATA_ALIGNMENT = 256,
			WG_SIZE = 64,
			ELEMENTS_PER_WORK_ITEM = (256/WG_SIZE),
			BITS_PER_PASS = 4,

			//	if you change this, change nPerWI in kernel as well
			NUM_WGS = 20*6,	//	cypress
//			NUM_WGS = 24*6,	//	cayman
//			NUM_WGS = 32*4,	//	nv
		};

		struct ConstData
		{
			int m_n;
			int m_nWGs;
			int m_startBit;
			int m_nBlocksPerWG;
		};

		struct Data
		{
			const Device* m_device;
			int m_maxSize;

			Kernel* m_streamCountKernel;
			Kernel* m_streamCountSortDataKernel;
			Kernel* m_prefixScanKernel;
			Kernel* m_sortAndScatterKernel;
			Kernel* m_sortAndScatterKeyValueKernel;
			Kernel* m_sortAndScatterSortDataKernel;

			Buffer<u32>* m_workBuffer0;
			Buffer<u32>* m_workBuffer1;
			Buffer<u32>* m_workBuffer2;
			Buffer<SortData>* m_workBuffer3;

			Buffer<ConstData>* m_constBuffer[32/BITS_PER_PASS];

			typename Copy<TYPE>::Data* m_copyData;
		};

		static
		Data* allocate(const Device* device, int maxSize);

		static
		void deallocate(Data* data);

		static
		void execute(Data* data, Buffer<u32>& inout, int n, int sortBits = 32);

		static
		void execute(Data* data, Buffer<u32>& in, Buffer<u32>& out, int n, int sortBits = 32);

		static
		void execute(Data* data, Buffer<u32>& keysIn, Buffer<u32>& keysOut, Buffer<u32>& valuesIn, Buffer<u32>& valuesOut, int n, int sortBits = 32);
		
		static
		void execute(Data* data, Buffer<SortData>& keyValuesInOut, int n, int sortBits = 32 );
};


#include <AdlPrimitives/Sort/RadixSort32Host.inl>
#include <AdlPrimitives/Sort/RadixSort32.inl>

};
