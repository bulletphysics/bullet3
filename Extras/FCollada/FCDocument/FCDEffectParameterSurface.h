/*
	Copyright (C) 2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FCDEffectParameterSurface.h
	This file contains the FCDEffectParameterSurface class,
	the FCDEffectParameterSurfaceInit interface and its derivate classes.
*/

#ifndef _FCD_EFFECT_PARAMETER_SURFACE_H_
#define _FCD_EFFECT_PARAMETER_SURFACE_H_

#include "FCDocument/FCDEffectParameter.h"

class FCDEffectParameterSurfaceInit;

/** A dynamically-sized array of COLLADA images */
typedef std::vector<FCDImage*> FCDImageList;

/**
	A COLLADA surface parameter.
	This parameters hold the texture loading information. The texture
	placement information should be held by the sampler parameter.

	@see FCDEffectParameterSampler

	@ingroup FCDEffect
*/
class FCOLLADA_EXPORT FCDEffectParameterSurface : public FCDEffectParameter
{
private:
	StringList names;
	FCDImageList images;
	FCDEffectParameterSurfaceInit* initMethod;
	FMVector3 size;
	float viewportRatio;
	uint16 mipLevelCount;
	bool generateMipmaps;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDEffectParameterList::AddParameter function.
		@param document The COLLADA document that owns the effect parameter. */
	FCDEffectParameterSurface(FCDocument* document);

	/** Destructor: do not use directly.
		Instead, use the FCDEffectParameterList::ReleaseParameter function.
		When released, the effect parameter list will also release all
		its parameters, if it owns them. */
	virtual ~FCDEffectParameterSurface();
	
	/** Retrieves the type of effect parameter class.
		@return The parameter class type: SURFACE. */
	virtual Type GetType() const { return SURFACE; }

	/** Retrieves the initialization method for the surface parameter.
		The initialization method is a powerful method of describing how
		to build complex textures, such as cube maps, from one or
		multiple image files.
		@return The surface initialization method. This pointer will be NULL,
			if no initialization method is provided. */
	FCDEffectParameterSurfaceInit* GetInitMethod() { return initMethod; }
	const FCDEffectParameterSurfaceInit* GetInitMethod() const { return initMethod; } /**< See above. */

	/** Sets the initialization method for the surface parameter.
		The initialization method is a powerful method of describing how
		to build complex textures, such as cube maps, from one or
		multiple image files.
		@param method The new initialization method.
			The old initialization method will be released.
			You should create a new initialization method
			for each surface parameter. */
	void SetInitMethod(FCDEffectParameterSurfaceInit* method);

	/** Retrieves the number of COLLADA images that make up this surface.
		There should never be more than six images to build a surface.
		In the large majority of cases, expect one image.
		@return The number of images. */
	size_t GetImageCount() const { return images.size(); }

	/** Retrieves the list of images that make up this surface.
		There should never be more than six images to build a surface.
		In the large majority of cases, expect one image.
		@return The	list of images. */
	FCDImageList& GetImages() { return images; }
	const FCDImageList& GetImages() const { return images; } /**< See above. */

	/** Retrieves a specific image.
		@param index The index of the image.
		@return The image. This pointer will be NULL if the index is out-of-bounds. */
	FCDImage* GetImage(size_t index = 0) { return index < images.size() ? images.at(index) : NULL; }
	const FCDImage* GetImage(size_t index = 0) const { return index < images.size() ? images.at(index) : NULL; } /**< See above. */

	/** Retrieves the index that matches the given image.
		@param image The image to match.
		@return The index within the list for this image.
			This index may be -1 if no match was found. */
	size_t FindImage(const FCDImage* image) const;

	/** Adds an image to the list.
		The initialization method indexes the images from this list.
		This function will verify that this image does not already exist within the list,
		so use the returned index.
		@param image The new image.
		@return The index of the image within the list. */
	size_t AddImage(FCDImage* image);

	/** Removes an image from the list.
		The initialization method indexes the images from this list.
		This function will shift all the indexes in the initialization method
		so that they continue matching the correct image.
		@param image The image to remove. Its memory is not released. */
	void RemoveImage(FCDImage* image);

	/** Retrieves the wanted dimensions of the surface.
		This parameter is optional and may contain all zeroes to indicate
		that you should read the surface dimensions from the image file(s).
		@return The wanted dimensions. */
	const FMVector3& GetSize() const { return size; }

	/** Sets the wanted dimensions of the surface.
		This parameter is optional and can contain all zeroes to indicate
		that you should read the surface dimensions from the image file(s).
		@param dimensions The wanted dimensions. */
	void SetSize(const FMVector3& dimensions) { size = dimensions; }

	/** Retrieves the viewport ratio to use when the surface is a render target.
		@return The viewport ratio. */
	float GetViewportRatio() const { return viewportRatio; }

	/** Sets the viewport ratio to use when the surface is a render target.
		@param ratio The viewport ratio. */
	void SetViewportRatio(float ratio) { viewportRatio = ratio; }

	/** Retrieves the wanted number of mip-levels.
		This parameter is optional and may be zero to indicate that you should
		retrieve the mip-levels from the image file(s) or generate a full
		mip-chain, depending on the mip-map generate flag.
		@see GetMipMapGenerate
		@return The wanted number of mip-levels. */
	uint16 GetMipLevelCount() const { return mipLevelCount; }

	/** Sets the wanted number of mip-levels.
		This parameter is optional and can be zero to indicate that you should
		retrieve the mip-levels from the image file(s) or generate a full
		mip-chain, depending on the mip-map generate flag.
		@param levelCount The wanted number of mip-levels. */
	void SetMipLevelCount(uint16 levelCount) { mipLevelCount = levelCount; }
	
	/** Retrieves whether to generate the mip-map levels on load.
		The alternative is to load the mip-map levels from the image files.
		@return Whether to generate the mip-map levels on load. */
	bool IsGenerateMipMaps() const { return generateMipmaps; }

	/** Sets whether to generate the mip-map levels of load.
		The alternative is to load the mip-map levels from the image files.
		@param _generateMipmaps Whether to generate the mip-map levels on load. */
	void SetGenerateMipMaps(bool _generateMipmaps) { generateMipmaps = _generateMipmaps; }

	/** Retrieves a specific sub-id.
		@todo I'm not too sure of the implications of the names,
			at this level of abstraction: once I'm clear why they exists, add the
			necessary interface to access/pull/push items from/to the list.
		@param index The sub-id index.
		@return The sub-id. This pointer will be NULL if the index is out-of-bounds. */
	const char* GetName(size_t index = 0) const { return (index < names.size()) ? names[index].c_str() : NULL; }

	/** Creates a full copy of the effect parameter.
		@todo The cloning does not clone the initialization method correctly.
		@return The cloned effect parameter. You will need to delete this pointer. */
	virtual FCDEffectParameter* Clone();

	/** [INTERNAL] Overwrites the target parameter with this parameter.
		This function is used during the flattening of materials.
		@param target The target parameter to overwrite. */
	virtual void Overwrite(FCDEffectParameter* target);

	/** [INTERNAL] Reads in the effect parameter from a given COLLADA XML tree node.
		@param parameterNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the parameter.*/
	virtual FUStatus LoadFromXML(xmlNode* parameterNode);

	/** [INTERNAL] Writes out the effect parameter to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the parameter.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};


/**
	[INTERNAL] The factory for COLLADA effect parameter surface initialization.

	@ingroup FCDEffect
*/
class FCOLLADA_EXPORT FCDEffectParameterSurfaceInitFactory
{
private:
	// Never instantiate: this is a static class
	FCDEffectParameterSurfaceInitFactory() {}

public:

	/** The supported initialization types. */
	enum InitType
	{
		FROM, /**< Loads a surface from one simple image file. @see FCDEffectParameterSurfaceInitFrom */
		AS_NULL, /**< No initialization. This surface may be initialized by some future effect parameter override. */
		AS_TARGET, /**< Initializes an engine-specific render target for offscreen rendering. In this case, the dimensions should be provided by the surface effect parameter. */
		CUBE, /**< Loads a cube-map from one complex image file or six simple image files. @see FCDEffectParameterSurfaceInitCube */
		VOLUME, /**< Loads a 3D images for one image file. @see FCDEffectParameterSurfaceInitVolume */
		PLANAR /**< Loads a surface from one simple image file. */
	};

	/** [INTERNAL] Creates a new effect surface initialization parameter, given a type.
		@param type The type of initialization parameter to create.*/
	static FCDEffectParameterSurfaceInit* Create(InitType type);
};


/**
	A surface initialization method.
	In COLLADA 1.4.1, this information was added to support complex surface types.
	There are six types of initialization methods, described in the InitType enumerated type.
	Expect the FROM initialization type in the large majority of cases.

	@ingroup FCDEffect
*/
class FCOLLADA_EXPORT FCDEffectParameterSurfaceInit
{
public:

	/** Constructor: builds a new surface initialization method. */
	FCDEffectParameterSurfaceInit() {}

	/** Destructor. */
	virtual ~FCDEffectParameterSurfaceInit() {}

	/** Retrieves the initialization type. You cannot modify this value.
		To change the initialization type of a surface parameter, create a new
		surface initialization structure of the correct type.
		@return The initialization type. */
	virtual FCDEffectParameterSurfaceInitFactory::InitType GetInitType() const = 0;

	/** Copies all member variables into clone.
		@param clone a valid pointer to a FCDEffectParameterSurfaceInit object*/
	void Clone(FCDEffectParameterSurfaceInit* clone);

	/** Creates a full copy of the surface initialization parameter.
		@return The surface initialization parameter. You will need to delete this pointer. */
	virtual FCDEffectParameterSurfaceInit* Clone() = 0;
};

/**
	A cube-map surface initialization method.
*/
class FCOLLADA_EXPORT FCDEffectParameterSurfaceInitCube : public FCDEffectParameterSurfaceInit
{
public:
	/** The types of cube-map initializations. */
	enum CubeType
	{
		ALL, /** Load all the mip-levels of all the cube-map faces from one image file. */
		PRIMARY, /** Load the first mip-level of all the cube-map faces from one image file. */
		FACE /** Load all the cube-map faces from separate image files. */
	};

public:
	/** Constructor: builds a new cube-map initialization method. */
	FCDEffectParameterSurfaceInitCube();

	/** Destructor. */
	virtual ~FCDEffectParameterSurfaceInitCube() {}

	/** Retrieves the initialization type. You cannot modify this value.
		To change the initialization type of a surface parameter, create a new
		surface initialization structure of the correct type.
		@return The initialization type. */
	virtual FCDEffectParameterSurfaceInitFactory::InitType GetInitType() const {return FCDEffectParameterSurfaceInitFactory::CUBE;}

	/** Creates a full copy of the surface initialization parameter.
		@return The surface initialization parameter. You will need to delete this pointer. */
	virtual FCDEffectParameterSurfaceInit* Clone();

	/** The type of cube-map initialization. */
	CubeType cubeType;
	
	/** The list of image indices.
		The images are contained within the surface effect parameter.
		This is used only for the FACE cube-map initialization type and indicates
		how to match the faces of faces of the cube-map with the images in the surface effect parameter. */
	UInt16List order;
};

/**
	A volumetric surface initialization method.
*/
class FCOLLADA_EXPORT FCDEffectParameterSurfaceInitVolume : public FCDEffectParameterSurfaceInit
{
public:
	/** The types of volumetric surfaces initialization. */
	enum VolumeType
	{
		ALL, /** Load all the mip-levels from the image file. */
		PRIMARY /** Load the first mip-level from the image file. */
	};

public:
	/** Constructor: builds a new volumetric surface initialization method. */
	FCDEffectParameterSurfaceInitVolume();

	/** Destructor. */
	virtual ~FCDEffectParameterSurfaceInitVolume() {}

	/** Retrieves the initialization type. You cannot modify this value.
		To change the initialization type of a surface parameter, create a new
		surface initialization structure of the correct type.
		@return The initialization type. */
	virtual FCDEffectParameterSurfaceInitFactory::InitType GetInitType() const {return FCDEffectParameterSurfaceInitFactory::VOLUME;}

	/** Creates a full copy of the surface initialization parameter.
		@return The surface initialization parameter. You will need to delete this pointer. */
	virtual FCDEffectParameterSurfaceInit* Clone();

	/** The type of volumetric initialization. */
	VolumeType volumeType;
};

/**
	A simple file surface initialization method.
	This is the method used for backward-compatibility.
*/
class FCOLLADA_EXPORT FCDEffectParameterSurfaceInitFrom : public FCDEffectParameterSurfaceInit
{
public:
	/** Constructor: builds a new file surface initialization method. */
	FCDEffectParameterSurfaceInitFrom() {}

	/** Destructor. */
	virtual ~FCDEffectParameterSurfaceInitFrom() {}

	/** Retrieves the initialization type. You cannot modify this value.
		To change the initialization type of a surface parameter, create a new
		surface initialization structure of the correct type.
		@return The initialization type. */
	virtual FCDEffectParameterSurfaceInitFactory::InitType GetInitType() const {return FCDEffectParameterSurfaceInitFactory::FROM;}

	/** Creates a full copy of the surface initialization parameter.
		@return The surface initialization parameter. You will need to delete this pointer. */
	virtual FCDEffectParameterSurfaceInit* Clone();

	/** The list of mip levels. */
	StringList mip;

	/** The list of matching slices. */
	StringList slice;

	/** The list of matching faces. */
	StringList face;
};

/**
	This method allows to initialize the surface at a later point.
*/
class FCOLLADA_EXPORT FCDEffectParameterSurfaceInitAsNull : public FCDEffectParameterSurfaceInit
{
public:
	/** Constructor: builds a new file surface initialization method. */
	FCDEffectParameterSurfaceInitAsNull() {}

	/** Destructor. */
	virtual ~FCDEffectParameterSurfaceInitAsNull() {}

	/** Retrieves the initialization type. You cannot modify this value.
		To change the initialization type of a surface parameter, create a new
		surface initialization structure of the correct type.
		@return The initialization type. */
	virtual FCDEffectParameterSurfaceInitFactory::InitType GetInitType() const {return FCDEffectParameterSurfaceInitFactory::AS_NULL;}

	/** Creates a full copy of the surface initialization parameter.
		@return The surface initialization parameter. You will need to delete this pointer. */
	virtual FCDEffectParameterSurfaceInit* Clone();
};

/**
	This method allows to initialize the surface as a rendering target.
*/
class FCOLLADA_EXPORT FCDEffectParameterSurfaceInitAsTarget : public FCDEffectParameterSurfaceInit
{
public:
	/** Constructor: builds a new file surface initialization method. */
	FCDEffectParameterSurfaceInitAsTarget() {};

	/** Destructor. */
	virtual ~FCDEffectParameterSurfaceInitAsTarget() {}

	/** Retrieves the initialization type. You cannot modify this value.
		To change the initialization type of a surface parameter, create a new
		surface initialization structure of the correct type.
		@return The initialization type. */
	virtual FCDEffectParameterSurfaceInitFactory::InitType GetInitType() const {return FCDEffectParameterSurfaceInitFactory::AS_TARGET;}

	/** Creates a full copy of the surface initialization parameter.
		@return The surface initialization parameter. You will need to delete this pointer. */
	virtual FCDEffectParameterSurfaceInit* Clone();
};

/**
	This method allows to initialize the surface as planar.
*/
class FCOLLADA_EXPORT FCDEffectParameterSurfaceInitPlanar : public FCDEffectParameterSurfaceInit
{
public:
	/** Constructor: builds a new file surface initialization method. */
	FCDEffectParameterSurfaceInitPlanar() {};

	/** Destructor. */
	virtual ~FCDEffectParameterSurfaceInitPlanar() {}

	/** Retrieves the initialization type. You cannot modify this value.
		To change the initialization type of a surface parameter, create a new
		surface initialization structure of the correct type.
		@return The initialization type. */
	virtual FCDEffectParameterSurfaceInitFactory::InitType GetInitType() const {return FCDEffectParameterSurfaceInitFactory::PLANAR;}

	/** Creates a full copy of the surface initialization parameter.
		@return The surface initialization parameter. You will need to delete this pointer. */
	virtual FCDEffectParameterSurfaceInit* Clone();
};



#endif // _FCD_EFFECT_PARAMETER_SURFACE_H_
