
#ifndef BT_RADIXSORT32_H
#define BT_RADIXSORT32_H

#include "btOpenCLArray.h"

struct btSortData
{
	int m_key;
	int m_value;
};
#include "btBufferInfoCL.h"

class  btRadixSort32CL
{

		btOpenCLArray<unsigned int>* m_workBuffer1;
		btOpenCLArray<unsigned int>* m_workBuffer2;
		
		btOpenCLArray<btSortData>*	m_workBuffer3;
		btOpenCLArray<btSortData>*	m_workBuffer4;

		btOpenCLArray<unsigned int>* m_workBuffer3a;
		btOpenCLArray<unsigned int>* m_workBuffer4a;

		cl_command_queue	m_commandQueue;

		cl_kernel m_streamCountSortDataKernel;
		cl_kernel m_streamCountKernel;

		cl_kernel m_prefixScanKernel;
		cl_kernel m_sortAndScatterSortDataKernel;
		cl_kernel m_sortAndScatterKernel;


		bool	m_deviceCPU;

		class btPrefixScanCL* m_scan;
		class btFillCL*	m_fill;

public:
	struct btConstData
		{
			int m_n;
			int m_nWGs;
			int m_startBit;
			int m_nBlocksPerWG;
		};
	enum
		{
			DATA_ALIGNMENT = 256,
			WG_SIZE = 64,
            BLOCK_SIZE = 256,
			ELEMENTS_PER_WORK_ITEM = (BLOCK_SIZE/WG_SIZE),
			BITS_PER_PASS = 4,
			NUM_BUCKET=(1<<BITS_PER_PASS),
			//	if you change this, change nPerWI in kernel as well
			NUM_WGS = 20*6,	//	cypress
//			NUM_WGS = 24*6,	//	cayman
//			NUM_WGS = 32*4,	//	nv
		};


private:
		

public:

		btRadixSort32CL(cl_context ctx, cl_device_id device, cl_command_queue queue, int initialCapacity =0);

		virtual ~btRadixSort32CL();

		void execute(btOpenCLArray<unsigned int>& keysIn, btOpenCLArray<unsigned int>& keysOut, btOpenCLArray<unsigned int>& valuesIn, 
								btOpenCLArray<unsigned int>& valuesOut, int n, int sortBits = 32);

		///keys only
		void execute(btOpenCLArray<unsigned int>& keysInOut, int sortBits  = 32 );

		void execute(btOpenCLArray<btSortData>& keyValuesInOut, int sortBits  = 32 );
		void executeHost(btOpenCLArray<btSortData>& keyValuesInOut, int sortBits = 32);
		void executeHost(b3AlignedObjectArray<btSortData>& keyValuesInOut, int sortBits = 32);

};
#endif //BT_RADIXSORT32_H

