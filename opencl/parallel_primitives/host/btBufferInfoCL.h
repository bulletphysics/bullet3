
#ifndef BT_BUFFER_INFO_CL_H
#define BT_BUFFER_INFO_CL_H

#include "btOpenCLArray.h"


struct btBufferInfoCL
{
	//btBufferInfoCL(){}

//	template<typename T>
	btBufferInfoCL(cl_mem buff, bool isReadOnly = false): m_clBuffer(buff), m_isReadOnly(isReadOnly){}

	cl_mem m_clBuffer;
	bool m_isReadOnly;
};

#endif //BT_BUFFER_INFO_CL_H
