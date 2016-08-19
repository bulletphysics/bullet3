#ifndef LINK_VISUAL_SHAPES_CONVERTER_H
#define LINK_VISUAL_SHAPES_CONVERTER_H

struct LinkVisualShapesConverter
{
	virtual void convertVisualShapes(int linkIndex, const char* pathPrefix, const class btTransform& localInertiaFrame, const struct UrdfModel& model, class btCollisionObject* colObj, int objectIndex)=0;
};

#endif //LINK_VISUAL_SHAPES_CONVERTER_H
