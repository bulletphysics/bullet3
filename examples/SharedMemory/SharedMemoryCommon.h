#ifndef SHARED_MEMORY_COMMON_H
#define SHARED_MEMORY_COMMON_H

#include "../CommonInterfaces/CommonMultiBodyBase.h"

class SharedMemoryCommon : public CommonMultiBodyBase
{
public:
	SharedMemoryCommon(GUIHelperInterface* helper)
	:CommonMultiBodyBase(helper)
	{
	}

	virtual bool wantsTermination()=0;
};
#endif//

