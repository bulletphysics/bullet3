#ifndef BT_FILL_CL_H
#define BT_FILL_CL_H

#include "btOpenCLArray.h"
#include "Bullet3Common/b3Scalar.h"

#include "btInt2.h"
#include "btInt4.h"


class btFillCL
{
	
	cl_command_queue	m_commandQueue;
	
	cl_kernel			m_fillKernelInt2;
	cl_kernel			m_fillIntKernel;
	cl_kernel			m_fillUnsignedIntKernel;
	cl_kernel			m_fillFloatKernel;

	public:
		
		struct btConstData
		{
			union
			{
				btInt4 m_data;
				btUnsignedInt4 m_UnsignedData;
			};
			int m_offset;
			int m_n;
			int m_padding[2];
		};

protected:

public:

		btFillCL(cl_context ctx, cl_device_id device, cl_command_queue queue);

		virtual ~btFillCL();

		void execute(btOpenCLArray<unsigned int>& src, const unsigned int value, int n, int offset = 0);
	
		void execute(btOpenCLArray<int>& src, const int value, int n, int offset = 0);

		void execute(btOpenCLArray<float>& src, const float value, int n, int offset = 0);

		void execute(btOpenCLArray<btInt2>& src, const btInt2& value, int n, int offset = 0);

		void executeHost(b3AlignedObjectArray<btInt2> &src, const btInt2 &value, int n, int offset);

		void executeHost(b3AlignedObjectArray<int> &src, const int value, int n, int offset);

	//	void execute(btOpenCLArray<btInt4>& src, const btInt4& value, int n, int offset = 0);

};
		
		
		
	

#endif //BT_FILL_CL_H
