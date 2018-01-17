#ifndef URDF_RENDERING_INTERFACE_H
#define URDF_RENDERING_INTERFACE_H

struct UrdfLink;
struct UrdfModel;
class btTransform;


struct UrdfRenderingInterface
{
	virtual void convertVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame, const UrdfLink* linkPtr, const UrdfModel* model, int shapeUid, int objectIndex) =0;
	
	virtual void removeVisualShape(int shapeUid)=0;

	virtual void syncTransform(int shapeUid, const class btTransform& worldTransform, const class btVector3& localScaling)=0;

	virtual int getNumVisualShapes(int bodyUniqueId)=0;

	virtual int getVisualShapesData(int bodyUniqueId, int shapeIndex, struct b3VisualShapeData* shapeData)=0;
	
	virtual void changeRGBAColor(int bodyUniqueId, int linkIndex, const double rgbaColor[4])=0;

	virtual void setUpAxis(int axis)=0;
	
    virtual void resetCamera(float camDist, float yaw, float pitch, float camPosX,float camPosY, float camPosZ)=0;
	
    virtual void clearBuffers(struct TGAColor& clearColor)=0;

	virtual void resetAll()=0;

    virtual void getWidthAndHeight(int& width, int& height)=0;
	virtual void setWidthAndHeight(int width, int height)=0;
	virtual void setLightDirection(float x, float y, float z)=0;
    virtual void setLightColor(float x, float y, float z)=0;
    virtual void setLightDistance(float dist)=0;
    virtual void setLightAmbientCoeff(float ambientCoeff)=0;
    virtual void setLightDiffuseCoeff(float diffuseCoeff)=0;
    virtual void setLightSpecularCoeff(float specularCoeff)=0;
    virtual void setShadow(bool hasShadow)=0;
	virtual void setFlags(int flags)=0;

    virtual void copyCameraImageData(unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels, float* depthBuffer, int depthBufferSizeInPixels,int* segmentationMaskBuffer, int segmentationMaskSizeInPixels,  int startPixelIndex, int* widthPtr, int* heightPtr, int* numPixelsCopied)=0;
    
	virtual void render()=0;
	virtual void render(const float viewMat[16], const float projMat[16])=0;
    
    virtual int loadTextureFile(const char* filename)=0;
    virtual int registerTexture(unsigned char* texels, int width, int height)=0;
   
    virtual void activateShapeTexture(int objectUniqueId, int jointIndex, int shapeIndex, int textureUniqueId)=0;

};

#endif //LINK_VISUAL_SHAPES_CONVERTER_H
