
#ifndef TINY_VR_GUI_H
#define TINY_VR_GUI_H

#include "Bullet3Common/b3Transform.h"

class TinyVRGui
{
	struct TinyVRGuiInternalData* m_data;

public:

	TinyVRGui(struct ComboBoxParams& params, struct CommonRenderInterface*	renderer);
	virtual ~TinyVRGui();
	
	bool init();
	void tick(b3Scalar deltaTime, const b3Transform& guiWorldTransform);

	void clearTextArea();
	void grapicalPrintf(const char* str,int rasterposx,int rasterposy,unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha);

};


#endif //TINY_VR_GUI_H
