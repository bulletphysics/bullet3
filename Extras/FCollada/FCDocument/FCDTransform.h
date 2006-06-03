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
	@file FCDTransform.h
	This file contains the FCDTransform class and its up-classes:
	FCDTTranslation, FCDTScale, FCDTRotation, FCDTMatrix, FCDTLookAt and FCDTSkew.
*/

#ifndef _FCD_TRANSFORM_H_
#define _FCD_TRANSFORM_H_

class FCDocument;
class FCDSceneNode;

#include "FCDocument/FCDObject.h"

/**
	A COLLADA transform.

	COLLADA supports six transformation types: translations(FCDTTranslation),
	rotations(FCDTRotation), scales(FCDTScale), matrices(FCDTMatrix),
	skews(FCDTSkew) and the 'look-at' transform(FCDTLookAt).

	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDTransform : public FCDObject
{
public:
	/** The COLLADA transform types. */
	enum Type
	{
		TRANSLATION, /**< A translation(FCDTTranslation). */
		ROTATION, /**< A rotation(FCDTRotation). */
		SCALE, /**< A non-uniform scale(FCDTScale). */
		MATRIX, /**< A matrix multiplication(FCDTMatrix). */
		LOOKAT, /**< A targeted, 'look-at' transformation(FCDTLookAt). */
		SKEW /**< A skew(FCDTSkew). */
	};

private:
	FCDSceneNode* parent;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDSceneNode::AddTransform function.
		@param document The COLLADA document that owns the transform.
		@param _parent The visual scene node that contains the transform.
			Set this pointer to NULL if this transform is not owned by a
			visual scene node. */
	FCDTransform(FCDocument* document, FCDSceneNode* _parent) : FCDObject(document, "FCDTransform") { parent = _parent; }

	/** Destructor: do not use directly.
		Instead, use the FCDSceneNode::ReleaseTransform function. */
	virtual ~FCDTransform() { parent = NULL; }

	/** Retrieves the visual scene node that contains this transformation.
		@return The parent visual scene node. This pointer will be NULL
			if the transformation is not contained by a visual scene node. */
	FCDSceneNode* GetParent() { return parent; }
	const FCDSceneNode* GetParent() const { return parent; } /**< See above. */

	/** Creates a copy of a transformation.
		@param newParent The visual scene node that will contain the clone.
		@return The cloned transformation. */
	virtual FCDTransform* Clone(FCDSceneNode* newParent) = 0;

	/** Retrieves the class type of the transformation.
		The class type should be used to up-case the transformation pointer.
		@return The class type. */
	virtual Type GetType() const = 0;

	/** Converts the transformation into a matrix.
		Useful for visual scene nodes with a weird transformation stack.
		@return A matrix equivalent of the transformation. */
	virtual FMMatrix44 ToMatrix() const = 0;

	/** Retrieves whether this transformation has an animation tied to its values.
		@return Whether the transformation is animated. */
	virtual bool IsAnimated() const = 0;

	/** Retrieves the animated element for the transformation.
		@return The animated element. This pointer will be NULL if the transformation
			is not animated. */
	virtual FCDAnimated* GetAnimated() = 0;

	/** Retrieves whether a given transformation is the exact opposite of
		this transformation. Executing two opposite transformations, one after the
		other will not give any resulting transformation. This function is useful
		to detect pivots within the transform stack.
		@param UNUSED A second transformation.
		@return Whether the two transformations are opposites. */
	virtual bool IsInverse(const FCDTransform* UNUSED(transform)) const { return false; }

	/** [INTERNAL] Reads in the transformation from a given COLLADA XML tree node.
		@param transformNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the transformation.*/
	virtual FUStatus LoadFromXML(xmlNode* transformNode) = 0;

	/** [INTERNAL] Writes out the transformation to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the transformation.
		@return The created XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const = 0;
};

/**
	A COLLADA translation.
	A translation is a simple 3D displacement.

	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDTTranslation : public FCDTransform
{
private:
	FMVector3 translation;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDSceneNode::AddTransform function with
		the TRANSLATION transformation type.
		@param document The COLLADA document that owns the translation.
		@param parent The visual scene node that contains the translation.
			Set this pointer to NULL if the translation is not owned
			by a visual scene node. */
	FCDTTranslation(FCDocument* document, FCDSceneNode* parent);
	
	/** Destructor: do not use directly.
		Instead, use the FCDSceneNode::ReleaseTransform function. */
	virtual ~FCDTTranslation();

	/** Retrieves the transformation class type for the translation.
		@return The transformation class type: TRANSLATION. */
	virtual Type GetType() const { return TRANSLATION; }

	/** Retrieves the translation 3D displacement vector.
		This displacement vector may be animated.
		@return The displacement vector. */
	inline FMVector3& GetTranslation() { return translation; }
	inline const FMVector3& GetTranslation() const { return translation; } /**< See above. */

	/** Sets the translation 3D displacement vector.
		@param _translation The displacement vector. */
	inline void SetTranslation(const FMVector3& _translation) { translation = _translation; }

	/** Sets the translation 3D displacement vector.
		@param x The x-component displacement.
		@param y The y-component displacement.
		@param z The z-component displacement. */
	inline void SetTranslation(float x, float y, float z) { translation = FMVector3(x, y, z); }

	/** Converts the translation into a matrix.
		@return A matrix equivalent of the translation. */
	virtual FMMatrix44 ToMatrix() const;

	/** Retrieves whether this translation is affected by an animation.
		@return Whether the translation is animated. */
	virtual bool IsAnimated() const;

	/** Retrieves the animated element for the translation.
		@see FCDAnimatedPoint3
		@return The animated element. This pointer will be NULL if the translation
			is not animated. */
	virtual FCDAnimated* GetAnimated();

	/** Retrieves whether a given transform is the exact opposite of
		this translation. The opposite of a translation has a displacement
		vector with all the components multiplied by -1.
		@param transform A second transformation.
		@return Whether the two transformations are opposites. */
	virtual bool IsInverse(const FCDTransform* transform) const;

	/** Creates a copy of the translation.
		@param newParent The visual scene node that will contain the clone.
		@return The cloned translation. */
	virtual FCDTransform* Clone(FCDSceneNode* newParent);

	/** [INTERNAL] Reads in the translation from a given COLLADA XML tree node.
		@param translationNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the translation.*/
	virtual FUStatus LoadFromXML(xmlNode* translationNode);

	/** [INTERNAL] Writes out the translation to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the translation.
		@return The created XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

/**
	A COLLADA non-uniform scale.
	A non-uniform scale contains three scale factors.
	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDTScale : public FCDTransform
{
private:
	FMVector3 scale;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDSceneNode::AddTransform function with
		the SCALE transformation type.
		@param document The COLLADA document that owns the non-uniform scale.
		@param parent The visual scene node that contains the non-uniform scale.
			Set this pointer to NULL if the non-uniform scale is not owned
			by a visual scene node. */
	FCDTScale(FCDocument* document, FCDSceneNode* parent);

	/** Destructor: do not use directly.
		Instead, use the FCDSceneNode::ReleaseTransform function. */
	virtual ~FCDTScale();

	/** Retrieves the transformation class type for the non-uniform scale.
		@return The class type: SCALE. */
	virtual Type GetType() const { return SCALE; }

	/** Retrieves the factors of the non-uniform scale.
		These factors may be animated.
		@return The scale factors. */
	FMVector3& GetScale() { return scale; }
	const FMVector3& GetScale() const { return scale; } /**< See above. */

	/** Sets the factors of the non-uniform scale.
		@param _scale The scale factors. */
	inline void SetScale(const FMVector3& _scale) { scale = _scale; }

	/** Sets the factors of the non-uniform scale.
		@param x The x-component scale factor.
		@param y The y-component scale factor.
		@param z The z-component scale factor. */
	inline void SetScale(float x, float y, float z) { scale = FMVector3(x, y, z); }

	/** Converts the non-uniform scale into a matrix.
		@return A matrix equivalent of the non-uniform scale. */
	virtual FMMatrix44 ToMatrix() const;

	/** Retrieves whether the factors of the non-uniform scale are animated.
		@return Whether the scale factors are animated. */
	virtual bool IsAnimated() const;

	/** Retrieves the animated element for the non-uniform scale factors.
		@see FCDAnimatedPoint3
		@return The animated element. This pointer will be NULL if the
			scale factors are not animated. */
	virtual FCDAnimated* GetAnimated();

	/** Creates a copy of the non-uniform scale.
		@param newParent The visual scene node that will contain the clone.
		@return The cloned non-uniform scale. */
	virtual FCDTransform* Clone(FCDSceneNode* newParent);

	/** [INTERNAL] Reads in the non-uniform scale from a given COLLADA XML tree node.
		@param scaleNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the transformation.*/
	virtual FUStatus LoadFromXML(xmlNode* scaleNode);

	/** [INTERNAL] Writes out the non-uniform scale to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the transformation.
		@return The created XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

/**
	A COLLADA angle-axis rotation.
	This rotation defines an axis around which the 3D points
	are rotated by a given angle.
	@todo (clock-wise/counter-clock-wise?)
	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDTRotation : public FCDTransform
{
private:
	float angle;
	FMVector3 axis;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDSceneNode::AddTransform function with
		the transformation type: ROTATION.
		@param document The COLLADA document that owns the rotation.
		@param parent The visual scene node that contains the rotation.
			Set this pointer to NULL if the rotation is not owned
			by a visual scene node. */
	FCDTRotation(FCDocument* document, FCDSceneNode* parent);
	
	/** Destructor: do not use directly.
		Instead, use the FCDSceneNode::ReleaseTransform function. */
	virtual ~FCDTRotation();

	/** Retrieves the transformation class type for the rotation.
		@return The class type: ROTATION. */
	virtual Type GetType() const { return ROTATION; }

	/** Retrieves the rotation axis.
		This 3D vector may be animated.
		@return The rotation axis. */
	inline FMVector3& GetAxis() { return axis; }
	inline const FMVector3& GetAxis() const { return axis; } /**< See above. */

	/** Sets the rotation axis.
		@param _axis The rotation axis. */
	inline void SetAxis(const FMVector3& _axis) { axis = _axis; }

	/** Sets the rotation axis.
		@param x The x-component of the rotation axis.
		@param y The y-component of the rotation axis.
		@param z The z-component of the rotation axis. */
	inline void SetAxis(float x, float y, float z) { axis = FMVector3(x, y, z); }

	/** Retrieves the rotation angle.
		This angle may be animated.
		@return The rotation angle, in degrees. */
	inline float& GetAngle() { return angle; }
	inline const float& GetAngle() const { return angle; } /**< See above. */

	/** Sets the rotation angle.
		@param a The rotation angle, in degrees. */
	inline void SetAngle(float a) { angle = a; }

	/** Sets the rotation components
		@param _axis The rotation axis.
		@param a The rotation angle, in degrees. */
	inline void SetRotation(const FMVector3& _axis, float a) { axis = _axis; angle = a; }

	/** Converts the rotation into a matrix.
		@return A matrix equivalent of the rotation. */
	virtual FMMatrix44 ToMatrix() const;

	/** Retrieves whether the axis or the angle of the rotation are animated.
		@return Whether the rotation is animated. */
	virtual bool IsAnimated() const;

	/** Retrieves the animated element for the angle-axis rotation.
		@see FCDAnimatedAngleAxis
		@return The animated element. This pointer will be NULL if the
			rotation is not animated. */
	virtual FCDAnimated* GetAnimated();

	/** Retrieves whether a given transform is the exact opposite of
		this rotation. The opposite of an angle-axis rotation has the
		same axis as this rotation but the angle is multiplied by -1.
		@param transform A second transformation.
		@return Whether the two rotation are opposites. */
	virtual bool IsInverse(const FCDTransform* transform) const;

	/** Creates a copy of the angle-axis rotation.
		@param newParent The visual scene node that will contain the clone.
		@return The cloned angle-axis rotation. */
	virtual FCDTransform* Clone(FCDSceneNode* newParent);

	/** [INTERNAL] Reads in the rotation from a given COLLADA XML tree node.
		@param rotationNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the rotation.*/
	virtual FUStatus LoadFromXML(xmlNode* rotationNode);

	/** [INTERNAL] Writes out the rotation to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the rotation.
		@return The created XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

/**
	A COLLADA matrix transformation.
	This transformation contains a matrix that should be
	multiplied to the local transformation matrix.
	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDTMatrix : public FCDTransform
{
private:
	FMMatrix44 transform;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDSceneNode::AddTransform function with
		the transformation type: MATRIX.
		@param document The COLLADA document that owns the transformation.
		@param parent The visual scene node that contains the transformation. */
	FCDTMatrix(FCDocument* document, FCDSceneNode* parent);

	/** Destructor: do not use directly.
		Instead, use the FCDSceneNode::ReleaseTransform function. */
	virtual ~FCDTMatrix();
	
	/** Retrieves the transformation class type for the transformation.
		@return The class type: MATRIX. */
	virtual Type GetType() const { return MATRIX; }

	/** Retrieves the matrix for the transformation.
		All 16 values of the matrix may be animated.
		@return The transformation matrix. */
	FMMatrix44& GetTransform() { return transform; }
	const FMMatrix44& GetTransform() const { return transform; } /**< See above. */

	/** Sets the matrix for the transformation.
		@param mx The transformation matrix. */
	inline void SetTransform(const FMMatrix44& mx) { transform = mx; }

	/** Converts the transformation into a matrix.
		For matrix transformations, that's simply the transformation matrix.
		@return The transformation matrix. */
	virtual FMMatrix44 ToMatrix() const { return transform; }

	/** Retrieves whether the transformation matrix is animated.
		@return Whether the transformation matrix is animated. */
	virtual bool IsAnimated() const;

	/** Retrieves the animated element for the transformation matrix.
		@see FCDAnimatedMatrix
		@return The animated element. This pointer will be NULL if the
			transformation matrix is not animated. */
	virtual FCDAnimated* GetAnimated();

	/** Creates a copy of the matrix transformation.
		@param newParent The visual scene node that will contain the clone.
		@return The cloned matrix transformation. */
	virtual FCDTransform* Clone(FCDSceneNode* newParent);

	/** [INTERNAL] Reads in the matrix transformation from a given COLLADA XML tree node.
		@param matrixNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the transformation.*/
	virtual FUStatus LoadFromXML(xmlNode* matrixNode);

	/** [INTERNAL] Writes out the matrix transformation to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the transformation.
		@return The created XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

/**
	A COLLADA 'look-at' transformation.
	This transformation type fully defines a position
	and an orientation with a 3D world by using three
	3D vectors: the viewer's position, the position
	that the viewer is looking at, and the up-vector
	for camera rolls. */
class FCOLLADA_EXPORT FCDTLookAt : public FCDTransform
{
private:
	FMVector3 position;
	FMVector3 target;
	FMVector3 up;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDSceneNode::AddTransform function with
		the transformation type: LOOKAT.
		@param document The COLLADA document that owns the transformation.
		@param parent The visual scene node that contains the transformation. */
	FCDTLookAt(FCDocument* document, FCDSceneNode* parent);

	/** Destructor: do not use directly.
		Instead, use the FCDSceneNode::ReleaseTransform function. */
	virtual ~FCDTLookAt();

	/** Retrieves the transformation class type for the transformation.
		@return The class type: LOOKAT. */
	virtual Type GetType() const { return LOOKAT; }

	/** Retrieves the viewer's position.
		@see FCDAnimatedPoint3
		@return The viewer's position. */
	FMVector3& GetPosition() { return position; }
	const FMVector3& GetPosition() const { return position; } /**< See above. */

	/** Sets the viewer's position.
		@param pos The viewer's position. */
	inline void SetPosition(const FMVector3& pos) { position = pos; }

	/** Sets the viewer's position.
		@param x The x-component of the position.
		@param y The y-component of the position.
		@param z The z-component of the position. */
	inline void SetPosition(float x, float y, float z) { position = FMVector3(x, y, z); }

	/** Retrieves the position that the viewer is looking at.
		@see FCDAnimatedPoint3
		@return The viewer's target. */
	FMVector3& GetTarget() { return target; }
	const FMVector3& GetTarget() const { return target; } /**< See above. */

	/** Sets the position that the viewer is looking at.
		@param _target The target position. */
	inline void SetTarget(const FMVector3& _target) { target = _target; }

	/** Sets the position that the viewer is looking at.
		@param x The x-component of the target position.
		@param y The y-component of the target position.
		@param z The z-component of the target position. */
	inline void SetTarget(float x, float y, float z) { target = FMVector3(x, y, z); }

	/** Retrieves the viewer's up-vector.
		@see FCDAnimatedPoint3
		@return The up-vector. */
	FMVector3& GetUp() { return up; }
	const FMVector3& GetUp() const { return up; } /**< See above. */

	/** Sets the viewer's up-vector.
		@param _up The up-vector. */
	inline void SetUp(const FMVector3& _up) { up = _up; }

	/** Sets the viewer's up-vector.
		@param x The x-component of the up-vector.
		@param y The y-component of the up-vector.
		@param z The z-component of the up-vector. */
	inline void SetUp(float x, float y, float z) { up = FMVector3(x, y, z); }

	/** Converts the transformation into a matrix.
		@return The transformation matrix. */
	virtual FMMatrix44 ToMatrix() const;

	/** Retrieves whether the transformation is animated.
		@return FCollada doesn't support animated 'look-at' transforms: false. */
	virtual bool IsAnimated() const;

	/** Retrieves the animated element for the transformation matrix.
		@return FCollada doesn't support animated 'look-at' transforms: NULL. */
	virtual FCDAnimated* GetAnimated();

	/** Creates a copy of the transformation.
		@param newParent The visual scene node that will contain the clone.
		@return The cloned 'look-at' transformation. */
	virtual FCDTransform* Clone(FCDSceneNode* newParent);

	/** [INTERNAL] Reads in the transformation from a given COLLADA XML tree node.
		@param lookAtNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the transformation.*/
	virtual FUStatus LoadFromXML(xmlNode* lookAtNode);

	/** [INTERNAL] Writes out the transformation to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the transformation.
		@return The created XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

/**
	A COLLADA skew.
	In COLLADA, the skew transformation follows the Renderman convention.
	A skew is defined by two axis and one angle: the axis which is rotated, the axis around
	which the rotation is done and the angle of the rotation.
	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDTSkew : public FCDTransform
{
private:
	FMVector3 rotateAxis;
	FMVector3 aroundAxis;
	float angle;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDSceneNode::AddTransform function with
		the transformation type: SKEW.
		@param document The COLLADA document that owns the skew.
		@param parent The visual scene node that contains the skew. */
	FCDTSkew(FCDocument* document, FCDSceneNode* parent);

	/** Destructor: do not use directly.
		Instead, use the FCDSceneNode::ReleaseTransform function. */
	virtual ~FCDTSkew();

	/** Retrieves the transformation class type for the transformation.
		@return The class type: SKEW. */
	virtual Type GetType() const { return SKEW; }

	/** Retrieves the axis which is rotated.
		@return The rotated axis. */
	const FMVector3& GetRotateAxis() const { return rotateAxis; }

	/** Sets the axis which is rotated.
		@param axis The rotated axis. */
	inline void SetRotateAxis(const FMVector3& axis) { rotateAxis = axis; }

	/** Retrieves the axis around which the rotation is done.
		@return The rotation axis. */
	const FMVector3& GetAroundAxis() const { return aroundAxis; }

	/** Sets the axis around which the rotation is done.
		@param axis The rotation axis. */
	inline void SetAroundAxis(const FMVector3& axis) { aroundAxis = axis; }

	/** Retrieves the rotation angle.
		@return The rotation angle. */
	const float& GetAngle() { return angle; }

	/** Sets the rotation angle.
		@param _angle The rotation angle. */
	inline void SetAngle(float _angle) { angle = _angle; }

	/** Converts the skew into a matrix.
		@return The transformation matrix. */
	virtual FMMatrix44 ToMatrix() const;

	/** Retrieves whether the transformation is animated.
		@return FCollada doesn't support animated skews: false. */
	virtual bool IsAnimated() const;

	/** Retrieves the animated element for the skew.
		@return FCollada doesn't support animated skews: NULL. */
	virtual FCDAnimated* GetAnimated();

	/** Creates a copy of the skew.
		@param newParent The visual scene node that will contain the clone.
		@return The cloned skew. */
	virtual FCDTransform* Clone(FCDSceneNode* newParent);

	/** [INTERNAL] Reads in the skew from a given COLLADA XML tree node.
		@param skewNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the skew.*/
	virtual FUStatus LoadFromXML(xmlNode* skewNode);

	/** [INTERNAL] Writes out the skew to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the skew.
		@return The created XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

/**
	[INTERNAL] A factory for COLLADA transforms.
	Creates the correct transform object for a given transform type/XML tree node.
	To create new transforms, use the FCDSceneNode::AddTransform function.
*/
class FCOLLADA_EXPORT FCDTFactory
{
private:
	FCDTFactory() {} // Static class: do not instantiate.

public:
	/** Creates a new COLLADA transform, given a transform type.
		@param document The COLLADA document that will own the new transform.
		@param parent The visual scene node that will contain the transform.
		@param type The type of transform object to create.
		@return The new COLLADA transform. This pointer will be NULL
			if the given type is invalid. */
	static FCDTransform* CreateTransform(FCDocument* document, FCDSceneNode* parent, FCDTransform::Type type);

	/** [INTERNAL] Imports a COLLADA transform, given an XML tree node.
		@param document The COLLADA document that will own the new transform.
		@param parent The visual scene node that will contain the transform.
		@param node The XML tree node.
		@return The imported COLLADA transform. This pointer will be NULL
			if the XML tree node does not describe a COLLADA transform. */
	static FCDTransform* CreateTransform(FCDocument* document, FCDSceneNode* parent, xmlNode* node);
};

#endif // _FR_TRANSFORM_H_
