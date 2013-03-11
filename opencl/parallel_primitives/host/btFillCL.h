#ifndef BT_FILL_CL_H
#define BT_FILL_CL_H

#include "btOpenCLArray.h"
#include "btScalar.h"

ATTRIBUTE_ALIGNED16(struct) btUnsignedInt4
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	union
	{
		struct
		{
			unsigned int x,y,z,w;
		};
		struct
		{
			unsigned int s[4];
		};
	};
};

ATTRIBUTE_ALIGNED16(struct) btInt4
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	union
	{
		struct
		{
			int x,y,z,w;
		};
		struct
		{
			int s[4];
		};
	};
};

struct btUnsignedInt2
{
	union
	{
		struct
		{
			unsigned int x,y;
		};
		struct
		{
			unsigned int s[2];
		};
	};
};

struct btInt2
{
	union
	{
		struct
		{
			int x,y;
		};
		struct
		{
			int s[2];
		};
	};
};

SIMD_FORCE_INLINE btInt4 btMakeInt4(int x, int y, int z, int w = 0)
{
	btInt4 v;
	v.s[0] = x; v.s[1] = y; v.s[2] = z; v.s[3] = w;
	return v;
}

SIMD_FORCE_INLINE btUnsignedInt4 btMakeUnsignedInt4(unsigned int x, unsigned int y, unsigned int z, unsigned int w = 0)
{
	btUnsignedInt4 v;
	v.s[0] = x; v.s[1] = y; v.s[2] = z; v.s[3] = w;
	return v;
}

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

		void executeHost(btAlignedObjectArray<btInt2> &src, const btInt2 &value, int n, int offset);

		void executeHost(btAlignedObjectArray<int> &src, const int value, int n, int offset);

	//	void execute(btOpenCLArray<btInt4>& src, const btInt4& value, int n, int offset = 0);

};
		
		
		
	

#endif //BT_FILL_CL_H
