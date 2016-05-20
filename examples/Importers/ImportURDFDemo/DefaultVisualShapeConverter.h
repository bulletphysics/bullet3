#ifndef DEFAULT_VISUAL_SHAPE_CONVERTER_H
#define DEFAULT_VISUAL_SHAPE_CONVERTER_H


#include "LinkVisualShapesConverter.h"

struct DefaultVisualShapeConverter : public LinkVisualShapesConverter
{
	
	struct DefaultVisualShapeConverterInternalData* m_data;
	
	DefaultVisualShapeConverter(struct GUIHelperInterface* guiHelper);
	
	virtual ~DefaultVisualShapeConverter();
	
	virtual int convertVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame, const UrdfModel& model, class btCollisionShape* colShape);
	
	virtual bool getLinkColor(int linkIndex, btVector4& colorRGBA) const;
	
	
};




#endif //DEFAULT_VISUAL_SHAPE_CONVERTER_H