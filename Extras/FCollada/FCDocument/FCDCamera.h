/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/
/*
	Based on the FS Import classes:
	Copyright (C) 2005-2006 Feeling Software Inc
	Copyright (C) 2005-2006 Autodesk Media Entertainment
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FCDCamera.h
	This file contains the FCDCamera class.
*/

#ifndef _FCD_CAMERA_H_
#define _FCD_CAMERA_H_

#include "FCDocument/FCDTargetedEntity.h"

class FCDocument;
class FCDSceneNode;

/**
	A COLLADA camera.
	Based on the FCDTargetedEntity class to support aimed cameras.
	COLLADA defines two types of cameras: perspective and orthographic.
	Both types are fully handled by this class.

	A COLLADA perspective camera defines two of the three following parameters: horizontal field of view,
	vertical field of view and aspect ratio. The missing parameter can be calculated using the following formulae:
	aspect ratio = vertical field of view / horizontal field of view. The vertical and horizontal field
	of view are in degrees.

	A COLLADA orthographic camera defines two of the three following parameters: horizontal magnification,
	vertical magnification and aspect ratio. The missing parameter can be calculated using the following formulae:
	aspect ratio = vertical magnification / horizontal magnification. You can calculate the viewport width
	and height using the following formulas: viewport width = horizontal magnification * 2, viewport height
	= vertical magnification * 2.

	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDCamera : public FCDTargetedEntity
{
private:
	// Camera flags
	bool isPerspective;
	bool isOrthographic;
	bool hasHorizontalView;
	bool hasVerticalView;

	// Camera parameters
	float viewY;
	float viewX;
	float nearZ;
	float farZ;
	float aspectRatio;

	// Maya parameters
	bool hasAperture;
	float verticalAperture;
	float horizontalAperture;
	float lensSqueeze;

public:
	/** Constructor: do not use directly. Create new cameras using the FCDLibrary::AddEntity function.
		@param document The COLLADA document that contains this camera entity.*/
	FCDCamera(FCDocument* document);

	/** Destructor: do not release directly. Release cameras using the FCDLibrary::ReleaseEntity function.
		All cameras are released with the document that they belong to. */
	virtual ~FCDCamera();

	/** Retrieves the entity type for this class. This function is part of the FCDEntity interface.
		@return The entity type: CAMERA. */
	virtual Type GetType() const { return CAMERA; }

	/** Retrieves whether this camera is a perspective camera.
		This is the default type of camera.
		@return Whether this camera is a perspective camera.*/
	inline bool IsPerspective() const { return isPerspective; }

	/** Sets the type of this camera to perspective. */
	inline void SetPerspective() { isPerspective = true; isOrthographic = false; }

	/** Retrieves whether the perspective camera defines an horizontal field of view.
		If the camera does not define the horizontal field of view, you can calculate
		it using the following formula: horizontal field of view = vertical field of view / aspect ratio.
		@return Whether the perspective camera defines an horizontal field of view. */
	inline bool HasHorizontalFov() const { return hasHorizontalView; }

	/** Retrieves whether the perspective camera defines a vertical field of view.
		If the camera does not define the vertical field of view, you can calculate
		it using the following formula: vertical field of view = aspect ratio * horizontal field of view.
		@return Whether the perspective camera defines a vertical field of view. */
	inline bool HasVerticalFov() const { return hasVerticalView; }

	/** Retrieves the horizontal field of view. Before retrieving this value, 
		check whether the camera defines the horizontal field of view using the
		HasHorizontalFov function.
		@return The horizontal field of view, in degrees. */
	inline float& GetFovX() { return viewX; }
	inline const float& GetFovX() const { return viewX; } /**< See above. */

	/** Retrieves the vertical field of view. Before retrieving this value, 
		check whether the camera defines the vertical field of view using the
		HasVerticalFov function.
		@return The horizontal field of view, in degrees. */
	inline float& GetFovY() { return viewY; }
	inline const float& GetFovY() const { return viewY; } /**< See above. */

	/** Sets the horizontal field of view value for this camera.
		@param fovX The new horizontal field of view, in degrees. */
	void SetFovX(float fovX);

	/** Sets the vertical field of view value for this camera.
		@param fovY The new vertical field of view, in degrees. */
	void SetFovY(float fovY);

	/** Retrieves whether this camera is an orthographic camera.
		@return Whether this camera is an orthographic camera. */
	inline bool IsOrthographic() const { return isOrthographic; }

	/** Sets the type of this camera to orthographic. */
	inline void SetOrthographic() { isPerspective = false; isOrthographic = true; }

	/** Retrieves whether the orthographic camera defines an horizontal magnification.
		If the camera does not define the horizontal magnification, you can calculate
		it using the following formula: horizontal magnification = vertical magnification / aspect ratio.
		@return Whether the orthographic camera defines an horizontal magnification. */
	inline bool HasHorizontalMag() const { return hasHorizontalView; }

	/** Retrieves whether the perspective camera defines a vertical magnification.
		If the camera does not define the vertical magnification, you can calculate
		it using the following formula: vertical magnification = aspect ratio * horizontal magnification.
		@return Whether the perspective camera defines a vertical magnification. */
	inline bool HasVerticalMag() const { return hasVerticalView; }

	/** Retrieves the horizontal magnification. Before retrieving this value, 
		check whether the camera defines the horizontal magnification using the
		HasHorizontalMag function.
		@return The horizontal magnification. */
	inline float& GetMagX() { return viewX; }
	inline const float& GetMagX() const { return viewX; } /**< See above. */

	/** Retrieves the vertical magnification. Before retrieving this value, 
		check whether the camera defines the vertical magnification using the
		HasVerticalMag function.
		@return The vertical magnification. */
	inline float& GetMagY() { return viewY; }
	inline const float& GetMagY() const { return viewY; } /**< See above. */

	/** Sets the horizontal magnification for this camera.
		@param magX The new horizontal magnification, in degrees. */
	inline void SetMagX(float magX) { return SetFovX(magX); }

	/** Sets the vertical magnification value for this camera.
		@param magY The new vertical magnification, in degrees. */
	inline void SetMagY(float magY) { return SetFovY(magY); }

	/** Retrieves the near-z value for this camera.
		The near-z value represent how close the near-clip
		plane of the view frustrum is. For orthographic cameras,
		this value is solely used for depth-buffering.
		@return The near-z value for this camera. */
	inline float& GetNearZ() { return nearZ; }
	inline const float& GetNearZ() const { return nearZ; } /**< See above. */

	/** Retrieves the far-z value for this camera. The far-z value represent
		how close the far-clip plane of the view frustrum is.
		For orthographic cameras, this value is solely used for depth-buffering.
		@return The far-z value for this camera. */
	inline float& GetFarZ() { return farZ; }
	inline const float& GetFarZ() const { return farZ; } /**< See above. */

	/** Retrieves the aspect ratio for the view of this camera. Before using this value,
		check if there are only one of the horizontal and vertical view ratios.
		If there are both of the view ratios provided for the camera, you will
		need to calculate the aspect ratio using the following formula:
		aspect ratio = vertical field of view / horizontal field of view.
		@return The view aspect ratio. */
	inline float& GetAspectRatio() { return aspectRatio; }
	inline const float& GetAspectRatio() const { return aspectRatio; } /**< See above. */

	/** Sets the near-z value for this camera.
		The near-z value represent how close the near-clip
		plane of the view frustrum is. For orthographic cameras,
		this value is solely used for depth-buffering.
		@param _nearZ A valid near-z value. No check is made to verify that the
		near-z value is greather than the far-z value.*/
	inline void SetNearZ(float _nearZ) { nearZ = _nearZ; }

	/** Sets the far-z value for this camera. The far-z value represent
		how close the far-clip plane of the view frustrum is.
		For orthographic cameras, this value is solely used for depth-buffering.
		@param _farZ A valid far-z value. No check is made to verify that the
		far-z value is greather than the near-z value.*/
	inline void SetFarZ(float _farZ) { farZ = _farZ; }

	/** Sets the aspect ratio for the view of this camera.
		@param aspectRatio The new view aspect ratio. */
	void SetAspectRatio(float aspectRatio);

	/** Retrieves whether the camera provides aperture information. This information is specific
		to COLLADA documents exported from ColladaMaya.
		@return Whether the camera provides aperture information. */
	inline bool HasAperture() const { return hasAperture; }
	
	/** Retrieves the vertical aperture of the camera. This information is specific to
		COLLADA documents exported from ColladaMaya.
		@return The vertical aperture of the camera. */
	inline float& GetVerticalAperture() { return verticalAperture; }
	inline const float& GetVerticalAperture() const { return verticalAperture; } /**< See above. */

	/** Retrieves the horizontal aperture of the camera. This information is specific to
		COLLADA documents exported from ColladaMaya.
		@return The horizontal aperture of the camera. */
	inline float& GetHorizontalAperture() { return horizontalAperture; }
	inline const float& GetHorizontalAperture() const { return horizontalAperture; } /**< See above. */

	/** Retrieves the lens squeeze of the camera. This information is specific to
		COLLADA documents exported from ColladaMaya. The lens squeeze of the camera
		is a multiplier that acts directly on the horizontal aperture, following this
		formula: real horizontal aperture = given horizontal aperture * lens squeeze.
		@return The lens squeeze of the camera. */
	inline float& GetLensSqueeze() { return lensSqueeze; }
	inline const float& GetLensSqueeze() const { return lensSqueeze; } /**< See above. */

	/** Sets the vertical aperture of the camera.
		@param aperture The vertical aperture of the camera. */
	inline void SetVerticalAperture(float aperture) { verticalAperture = aperture; hasAperture = true; }

	/** Sets the horizontal aperture of the camera.
		@param aperture The horizontal aperture of the camera. */
	inline void SetHorizontalAperture(float aperture) { horizontalAperture = aperture; hasAperture = true; }

	/** Sets the lens squeeze of the camera.
		@param factor The lense squeeze of the camera. */
	inline void SetLensSqueeze(float factor) { lensSqueeze = factor; }

	/** [INTERNAL] Reads in the \<camera\> element from a given COLLADA XML tree node.
		@param cameraNode A COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the camera.*/
	FUStatus LoadFromXML(xmlNode* cameraNode);

	/** [INTERNAL] Writes out the \<camera\> element to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the geometry information.
		@return The created XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_CAMERA_H_

