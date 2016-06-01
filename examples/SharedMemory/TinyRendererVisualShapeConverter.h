#ifndef TINY_RENDERER_VISUAL_SHAPE_CONVERTER_H
#define TINY_RENDERER_VISUAL_SHAPE_CONVERTER_H


#include "../Importers/ImportURDFDemo/LinkVisualShapesConverter.h"

struct TinyRendererVisualShapeConverter : public LinkVisualShapesConverter
{
	
	struct TinyRendererVisualShapeConverterInternalData* m_data;
	
	TinyRendererVisualShapeConverter();
	
	virtual ~TinyRendererVisualShapeConverter();
	
	virtual void convertVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame, const UrdfModel& model, class btCollisionObject* colShape);
	
	void setUpAxis(int axis);
	
    void resetCamera(float camDist, float pitch, float yaw, float camPosX,float camPosY, float camPosZ);
	
    void clearBuffers(struct TGAColor& clearColor);

	void resetAll();

	void render();
	
};




#endif //TINY_RENDERER_VISUAL_SHAPE_CONVERTER_H
