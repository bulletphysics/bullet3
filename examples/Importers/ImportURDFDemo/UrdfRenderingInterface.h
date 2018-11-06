#ifndef URDF_RENDERING_INTERFACE_H
#define URDF_RENDERING_INTERFACE_H

///UrdfLink and UrdfModel are the main URDF structures to define a robot
struct UrdfLink;
struct UrdfModel;
///btTransform is a position and 3x3 matrix, as defined in Bullet/src/LinearMath/btTransform
class btTransform;

///UrdfRenderingInterface is a simple rendering interface, mainly for URDF-based robots.
///There is an implementation in
///bullet3\examples\SharedMemory\plugins\tinyRendererPlugin\TinyRendererVisualShapeConverter.cpp
struct UrdfRenderingInterface
{
	virtual ~UrdfRenderingInterface() {}
	///given a URDF link, convert all visual shapes into internal renderer (loading graphics meshes, textures etc)
	///use the collisionObjectUid as a unique identifier to synchronize the world transform and to remove the visual shape.
	virtual void convertVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame, const UrdfLink* linkPtr, const UrdfModel* model, int collisionObjectUniqueId, int bodyUniqueId, struct CommonFileIOInterface* fileIO) = 0;

	///remove a visual shapes, based on the shape unique id (shapeUid)
	virtual void removeVisualShape(int collisionObjectUid) = 0;

	///update the world transform + scaling of the visual shape, using the shapeUid
	virtual void syncTransform(int collisionObjectUid, const class btTransform& worldTransform, const class btVector3& localScaling) = 0;

	///return the number of visual shapes, for a particular body unique id
	virtual int getNumVisualShapes(int bodyUniqueId) = 0;

	///return the visual shape information, for a particular body unique id and 'shape index'
	virtual int getVisualShapesData(int bodyUniqueId, int shapeIndex, struct b3VisualShapeData* shapeData) = 0;

	///change the RGBA color for some visual shape.
	virtual void changeRGBAColor(int bodyUniqueId, int linkIndex, int shapeIndex, const double rgbaColor[4]) = 0;

	///select a given texture for some visual shape.
	virtual void changeShapeTexture(int objectUniqueId, int linkIndex, int shapeIndex, int textureUniqueId) = 0;

	///pick the up-axis, either Y (1) or Z (2)
	virtual void setUpAxis(int axis) = 0;

	///compute the view matrix based on those parameters
	virtual void resetCamera(float camDist, float yaw, float pitch, float camPosX, float camPosY, float camPosZ) = 0;

	///clear the render buffer with a particular color.
	virtual void clearBuffers(struct TGAColor& clearColor) = 0;

	///remove all visual shapes.
	virtual void resetAll() = 0;

	///return the frame buffer width and height for the renderer
	virtual void getWidthAndHeight(int& width, int& height) = 0;

	///set the frame buffer width and height for the renderer
	virtual void setWidthAndHeight(int width, int height) = 0;

	///set the light direction, in world coordinates
	virtual void setLightDirection(float x, float y, float z) = 0;

	///set the ambient light color, in world coordinates
	virtual void setLightColor(float x, float y, float z) = 0;

	///set the light distance
	virtual void setLightDistance(float dist) = 0;

	///set the light ambient coefficient
	virtual void setLightAmbientCoeff(float ambientCoeff) = 0;

	///set the light diffuse coefficient
	virtual void setLightDiffuseCoeff(float diffuseCoeff) = 0;

	///set the light specular coefficient
	virtual void setLightSpecularCoeff(float specularCoeff) = 0;

	///enable or disable rendering of shadows
	virtual void setShadow(bool hasShadow) = 0;

	///some undocumented flags for experimentation (todo: document)
	virtual void setFlags(int flags) = 0;

	///provide the image pixels as a part of a stream.
	virtual void copyCameraImageData(unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels, float* depthBuffer, int depthBufferSizeInPixels, int* segmentationMaskBuffer, int segmentationMaskSizeInPixels, int startPixelIndex, int* widthPtr, int* heightPtr, int* numPixelsCopied) = 0;

	///render an image, using some arbitraty view and projection matrix
	virtual void render() = 0;

	///render an image using the provided view and projection matrix
	virtual void render(const float viewMat[16], const float projMat[16]) = 0;

	///load a texture from file, in png or other popular/supported format
	//virtual int loadTextureFile(const char* filename) = 0;
	virtual int loadTextureFile(const char* filename, struct CommonFileIOInterface* fileIO)=0;
	
	///register a texture using an in-memory pixel buffer of a given width and height
	virtual int registerTexture(unsigned char* texels, int width, int height) = 0;

	virtual void setProjectiveTextureMatrices(const float viewMatrix[16], const float projectionMatrix[16]) {}
	virtual void setProjectiveTexture(bool useProjectiveTexture) {}
};

#endif  //LINK_VISUAL_SHAPES_CONVERTER_H
