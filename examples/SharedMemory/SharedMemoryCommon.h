#ifndef SHARED_MEMORY_COMMON_H
#define SHARED_MEMORY_COMMON_H

#include "../CommonInterfaces/CommonMultiBodyBase.h"

class SharedMemoryCommon : public CommonExampleInterface
{
protected:
	struct GUIHelperInterface* m_guiHelper;

public:
	SharedMemoryCommon(GUIHelperInterface* helper)
		: m_guiHelper(helper)
	{
	}

	virtual void setSharedMemoryKey(int key) = 0;
	virtual bool wantsTermination() = 0;
	virtual bool isConnected() = 0;
};
#endif  //
