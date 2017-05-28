#ifndef LINK_VISUAL_SHAPES_CONVERTER_H
#define LINK_VISUAL_SHAPES_CONVERTER_H

struct UrdfLink;
struct UrdfModel;
class btTransform;
class btCollisionObject;

struct LinkVisualShapesConverter
{
	virtual void convertVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame, const UrdfLink* linkPtr, const UrdfModel* model, class btCollisionObject* colShape, int objectIndex) =0;
	
	virtual void removeVisualShape(class btCollisionObject* colObj)=0;

};

#endif //LINK_VISUAL_SHAPES_CONVERTER_H
