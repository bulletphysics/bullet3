#ifndef LINK_VISUAL_SHAPES_CONVERTER_H
#define LINK_VISUAL_SHAPES_CONVERTER_H

struct LinkVisualShapesConverter
{
	virtual int convertVisualShapes(int linkIndex, const char* pathPrefix, const class btTransform& localInertiaFrame, const struct UrdfModel& model, class btCollisionShape* colShape)=0;
	virtual bool getLinkColor(int linkIndex, class btVector4& colorRGBA) const = 0;
};

#endif //LINK_VISUAL_SHAPES_CONVERTER_H
