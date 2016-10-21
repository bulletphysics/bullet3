#ifndef TINY_RENDERER_VISUAL_SHAPE_CONVERTER_H
#define TINY_RENDERER_VISUAL_SHAPE_CONVERTER_H


#include "../Importers/ImportURDFDemo/LinkVisualShapesConverter.h"




struct TinyRendererVisualShapeConverter : public LinkVisualShapesConverter
{
	
	struct TinyRendererVisualShapeConverterInternalData* m_data;
	
	TinyRendererVisualShapeConverter();
	
	virtual ~TinyRendererVisualShapeConverter();
	
	virtual void convertVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame, const UrdfModel& model, class btCollisionObject* colShape, int objectIndex);
	
	virtual int getNumVisualShapes(int bodyUniqueId);

	virtual int getVisualShapesData(int bodyUniqueId, int shapeIndex, struct b3VisualShapeData* shapeData);
	
	void setUpAxis(int axis);
	
    void resetCamera(float camDist, float pitch, float yaw, float camPosX,float camPosY, float camPosZ);
	
    void clearBuffers(struct TGAColor& clearColor);

	void resetAll();

    void getWidthAndHeight(int& width, int& height);
	void setWidthAndHeight(int width, int height);
    
    void copyCameraImageData(unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels, float* depthBuffer, int depthBufferSizeInPixels,int* segmentationMaskBuffer, int segmentationMaskSizeInPixels,  int startPixelIndex, int* widthPtr, int* heightPtr, int* numPixelsCopied);
    
	void render();
	void render(const float viewMat[16], const float projMat[16]);
    
    void loadTextureFile(const char* filename);
    int registerTexture(unsigned char* texels, int width, int height);
    void activateShapeTexture(int shapeUniqueId, int textureUniqueId);
    void activateShapeTexture(int objectUniqueId, int jointIndex, int shapeIndex, int textureUniqueId);
	
};




#endif //TINY_RENDERER_VISUAL_SHAPE_CONVERTER_H
