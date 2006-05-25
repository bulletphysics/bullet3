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
	@file FCDAnimated.h
	This file contains the FCDAnimated class.
*/

#ifndef _FCD_ANIMATED_H_
#define _FCD_ANIMATED_H_

#include "FCDocument/FCDObject.h"

class FCDocument;
class FCDAnimated;
class FCDAnimationCurve;
class FCDAnimationChannel;
class FCDAnimationMultiCurve;

typedef vector<float*> FloatPtrList; /**< A dynamically-sized array of floating-point value pointers. */
typedef vector<FCDAnimationCurve*> FCDAnimationCurveList; /**< A dynamically-sized array of animation curves. */
typedef vector<FCDAnimationChannel*> FCDAnimationChannelList; /**< A dynamically-sized array of animation channels. */
typedef vector<FCDAnimated*> FCDAnimatedList; /**< A dynamically-sized array of animated values. */

/**
	An animated element.
	An animated element encapsulates a set of floating-point values that are
	marked as animated.

	For this purpose, an animated element holds a list of floating-point values,
	their animation curves and their COLLADA qualifiers for the generation of
	COLLADA targets. For animated list elements, an animated element holds an array index.

	There are many classes built on top of this class. They represent
	the different element types that may be animated, such as 3D points,
	colors and matrices.

	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDAnimated : public FCDObject
{
protected:
	/** The list of value pointers. */
	FloatPtrList values;

	/** The list of target qualifiers.
		There is always one qualifier for one value pointer. */
	StringList qualifiers; 

	/** The list of animation curves.
		There is always one curve for one value pointer, although
		that curve may be the NULL pointer to indicate a non-animated value. */
	FCDAnimationCurveList curves; 

	/** The array index for animated element that belong
		to a list of animated elements. This value may be -1
		to indicate that the element does not belong to a list.
		Otherwise, the index should always be unsigned. */
	int32 arrayElement;

	/** [INTERNAL] The target pointer prefix. */
	string pointer; 

public:
	/** Constructor.
		In most cases, it is preferable to create objects of the up-classes.
		@param document The COLLADA document that owns this animated element.
		@param valueCount The number of values inside the animated element. */
	FCDAnimated(FCDocument* document, size_t valueCount);

	/** Destructor. */
	virtual ~FCDAnimated();

	/** Retrieves the number of values contained within this animated element.
		@return The number of values. */
	inline size_t GetValueCount() const { return values.size(); }

	/** Retrieves the animation curve affecting the value of an animated element.
		@param index The value index.
		@return The curve affecting the value at the given index. This pointer will
			be NULL if the index is out-of-bounds or if the value is not animated. */
	inline FCDAnimationCurve* GetCurve(size_t index) { FUAssert(index < GetValueCount(), return NULL); return curves.at(index); }
	inline const FCDAnimationCurve* GetCurve(size_t index) const { FUAssert(index < GetValueCount(), return NULL); return curves.at(index); } /**< See above. */

	/** Retrieves the list of the curves affecting the values of an animated element.
		This list may contain the NULL pointer, where a value is not animated.
		@return The list of animation curves. */
	inline FCDAnimationCurveList& GetCurves() { return curves; }
	inline const FCDAnimationCurveList& GetCurves() const { return curves; } /**< See above. */

	/** Assigns a curve to a value of the animated element.
		The previously assigned curve will be deleted.
		@param index The value index.
		@param curve The new curve that will affect the value at the given index.
		@return Whether the curve was successfully assigned. Will return false if
			the index is out-of-bounds. */
	bool SetCurve(size_t index, FCDAnimationCurve* curve);

	/** Removes the curve affecting a value of the animated element.
		@param index The value index.
		@return Whether a curve was successfully removed. Will return false
			if there was no curve to release or the index is out-of-bounds. */
	bool RemoveCurve(size_t index);

	/** Retrieves the value of an animated element.
		@param index The value index.
		@return The value at the given index. This pointer will
			be NULL if the index is out-of-boudns. */
	inline float* GetValue(size_t index) { FUAssert(index < GetValueCount(), return NULL); return values.at(index); }
	inline const float* GetValue(size_t index) const { FUAssert(index < GetValueCount(), return NULL); return values.at(index); } /**< See above. */

	/** Retrieves the qualifier of the value of an animated element.
		@param index The value index.
		@return The qualifier for the value. The value returned will be an
			empty string when the index is out-of-bounds. */
	inline const string& GetQualifier(size_t index) const;

	/** Retrieves an animated value given a valid qualifier.
		@param qualifier A valid qualifier.
		@return The animated value for this qualifier. This pointer will be
			NULL if the given qualifier is not used within this animated element. */
	float* FindValue(const string& qualifier);
	const float* FindValue(const string& qualifier) const; /**< See above. */

	/** Retrieves an animation curve given a valid qualifier.
		@param qualifier A valid qualifier.
		@return The animation curve for this qualifier. This pointer will be
			NULL if the given qualifier is not used within this animated element
			or if the value for the given qualifier is not animated. */
	inline FCDAnimationCurve* FindCurve(const char* qualifier) { return GetCurve(FindQualifier(qualifier)); }
	inline FCDAnimationCurve* FindCurve(const string& qualifier) { return FindCurve(qualifier.c_str()); } /**< See above. */
	inline const FCDAnimationCurve* FindCurve(const char* qualifier) const { return GetCurve(FindQualifier(qualifier)); } /**< See above. */
	inline const FCDAnimationCurve* FindCurve(const string& qualifier) const { return FindCurve(qualifier.c_str()); } /**< See above. */

	/** Retrieves an animation curve given a value pointer.
		@param value A value pointer contained within the animated element.
		@return The animation curve for this qualifier. This pointer will be
			NULL if the value pointer is not contained by this animated element
			or if the value is not animated. */
	inline FCDAnimationCurve* FindCurve(const float* value) { return GetCurve(FindValue(value)); }
	inline const FCDAnimationCurve* FindCurve(const float* value) const { return GetCurve(FindValue(value)); } /**< See above. */

	/** Retrieves the value index for a given qualifier.
		@param qualifier A valid qualifier.
		@return The value index. This value will be -1 to indicate that the
			qualifier does not belong to this animated element. */
	size_t FindQualifier(const char* qualifier) const;
	inline size_t FindQualifier(const string& qualifier) const { return FindQualifier(qualifier.c_str()); } /**< See above. */

	/** Retrieves the value index for a given value pointer.
		@param value A value pointer contained within the animated element.
		@return The value index. This value will be -1 to indicate that the
			value pointer is not contained by this animated element. */
	size_t FindValue(const float* value) const;

	/** Retrieves the array index for an animated element.
		This value is used only for animated elements that belong
		to a list of animated elements within the COLLADA document.
		@return The array index. This value will be -1 to indicate that
			the animated element does not belong to a list. */
	inline int32 GetArrayElement() const { return arrayElement; }

	/** Sets the array index for an animated element.
		This value is used only for animated elements that belong
		to a list of animated elements within the COLLADA document.
		@param index The array index. This value should be -1 to indicate that
			the animated element does not belong to a list. */
	inline void SetArrayElement(int32 index) { arrayElement = index; }

	/** Retrieves whether this animated element has any animation curves
		affecting its values.
		@return Whether any curves affect this animated element. */
	bool HasCurve() const;

	/** Creates one multi-dimensional animation curve from this animated element.
		This function is useful is your application does not handle animations
		per-values, but instead needs one animation per-element.
		@return The multi-dimensional animation curve. */
	FCDAnimationMultiCurve* CreateMultiCurve() const;

	/** Creates one multi-dimensional animation curve from a list of animated element.
		This function is useful is your application does not handle animations
		per-values. For example, we use this function is ColladaMax for animated scale values,
		where one scale value is two rotations for the scale rotation pivot and one
		3D point for the scale factors.
		@param toMerge The list of animated elements to merge
		@return The multi-dimensional animation curve. */
	static FCDAnimationMultiCurve* CreateMultiCurve(const FCDAnimatedList& toMerge);

	/** Evaluates the animated element at a given time.
		This function directly and <b>permanently</b> modifies the values
		of the animated element according to the curves affecting them.
		@param time The evaluation time. */
	void Evaluate(float time);

	/** [INTERNAL] Clones an animated element.
		@param document The COLLADA document that owns the cloned animated element.
		@param animatedValue One animated value contained within the original animated element.
		@param newAnimatedValues The list of value pointers to be contained by the cloned animated element.
		@return The cloned animated element. */
	static FCDAnimated* Clone(FCDocument* document, const float* animatedValue, FloatPtrList& newAnimatedValues);

	/** [INTERNAL] Clones an animated element.
		@param document The COLLADA document that owns the cloned animated element.
		@return The cloned animated element. */
	FCDAnimated* Clone(FCDocument* document);

	/** [INTERNAL] Retrieves the target pointer that prefixes the
		fully-qualified target for the element.
		@return The target pointer prefix. */
	const string& GetTargetPointer() const { return pointer; }

	/** [INTERNAL] Links this animated element with a given XML tree node.
		This function is solely used within the import of a COLLADA document.
		The floating-point values held within the XML tree node will be linked
		with the list of floating-point value pointers held by the animated entity.
		@param node The XML tree node.
		@return Whether there was any linkage done. */
	bool Link(xmlNode* node);

	/** [INTERNAL] Links the animated element with the imported animation curves.
		This compares the animation channel targets with the animated element target
		and qualifiers to assign curves unto the value pointers.
		@param channels A list of animation channels with the correct target pointer.
		@return Whether any animation curves were assigned to the animation element. */
	bool ProcessChannels(FCDAnimationChannelList& channels);
};

/** A COLLADA animated single floating-point value element.
	Use this animated element class for all generic-purpose single floating-point values.
	For angles, use the FCDAnimatedAngle class.
	@ingroup FCDocument */
class FCOLLADA_EXPORT FCDAnimatedFloat : public FCDAnimated
{
private:
	// Don't build directly, use the Create function instead
	FCDAnimatedFloat(FCDocument* document, float* value, int32 arrayElement);

public:
	/** Creates a new animated element.
		@param document The COLLADA document that owns the animated element.
		@param value The value pointer for the single floating-point value.
		@param arrayElement The optional array index for animated element
			that belong to an animated element list.
		@return The new animated element. */
	static FCDAnimatedFloat* Create(FCDocument* document, float* value, int32 arrayElement=-1);

	/** [INTERNAL] Creates a new animated element.
		This function is used during the import of a COLLADA document.
		@param document The COLLADA document that owns the animated element.
		@param node The XML tree node that contains the animated values.
		@param value The value pointer for the single floating-point value.
		@param arrayElement The optional array index for animated element
			that belong to an animated element list.
		@return The new animated element. */
	static FCDAnimatedFloat* Create(FCDocument* document, xmlNode* node, float* value, int32 arrayElement=-1);

	/** [INTERNAL] Clones an animated element.
		@param document The COLLADA document that owns the cloned animated element.
		@param oldValue The single floating-point value pointer contained within the original animated element.
		@param newValue The single floating-point value pointer for the cloned animated element.
		@return The cloned animated value. */
	static FCDAnimated* Clone(FCDocument* document, const float* oldValue, float* newValue);
};

/** A COLLADA animated 3D vector element.
	@ingroup FCDocument */
class FCOLLADA_EXPORT FCDAnimatedPoint3 : public FCDAnimated
{
private:
	// Don't build directly, use the Create function instead
	FCDAnimatedPoint3(FCDocument* document, FMVector3* value, int32 arrayElement);

public:
	/** Creates a new animated element.
		@param document The COLLADA document that owns the animated element.
		@param value The value pointer for the 3D vector.
		@param arrayElement The optional array index for animated element
			that belong to an animated element list.
		@return The new animated element. */
	static FCDAnimatedPoint3* Create(FCDocument* document, FMVector3* value, int32 arrayElement=-1);

	/** [INTERNAL] Creates a new animated element.
		This function is used during the import of a COLLADA document.
		@param document The COLLADA document that owns the animated element.
		@param node The XML tree node that contains the animated values.
		@param value The value pointer for the 3D vector.
		@param arrayElement The optional array index for animated element
			that belong to an animated element list.
		@return The new animated element. */
	static FCDAnimatedPoint3* Create(FCDocument* document, xmlNode* node, FMVector3* value, int32 arrayElement=-1);

	/** [INTERNAL] Clones an animated element.
		@param document The COLLADA document that owns the cloned animated element.
		@param oldValue The 3D vector contained within the original animated element.
		@param newValue The 3D vector for the cloned animated element.
		@return The cloned animated value. */
	static FCDAnimated* Clone(FCDocument* document, const FMVector3* oldValue, FMVector3* newValue);
};

/** A COLLADA animated RGB color element.
	@ingroup FCDocument */
class FCOLLADA_EXPORT FCDAnimatedColor : public FCDAnimated
{
private:
	// Don't build directly, use the Create function instead
	FCDAnimatedColor(FCDocument* document, FMVector3* value, int32 arrayElement);

public:
	/** Creates a new animated element.
		@param document The COLLADA document that owns the animated element.
		@param value The value pointer for the RGB color.
		@param arrayElement The optional array index for animated element
			that belong to an animated element list.
		@return The new animated element. */
	static FCDAnimatedColor* Create(FCDocument* document, FMVector3* value, int32 arrayElement=-1);

	/** [INTERNAL] Creates a new animated element.
		This function is used during the import of a COLLADA document.
		@param document The COLLADA document that owns the animated element.
		@param node The XML tree node that contains the animated values.
		@param value The value pointer for the RGB color.
		@param arrayElement The optional array index for animated element
			that belong to an animated element list.
		@return The new animated element. */
	static FCDAnimatedColor* Create(FCDocument* document, xmlNode* node, FMVector3* value, int32 arrayElement=-1);

	/** [INTERNAL] Clones an animated element.
		@param document The COLLADA document that owns the cloned animated element.
		@param oldValue The RGB color contained within the original animated element.
		@param newValue The RGB color for the cloned animated element.
		@return The cloned animated value. */
	static FCDAnimated* Clone(FCDocument* document, const FMVector3* oldValue, FMVector3* newValue);
};

/** A COLLADA floating-point value that represents an angle.
	@ingroup FCDocument */
class FCOLLADA_EXPORT FCDAnimatedAngle : public FCDAnimated
{
private:
	// Don't build directly, use the Create function instead
	FCDAnimatedAngle(FCDocument* document, float* value, int32 arrayElement);

public:
	/** Creates a new animated element.
		@param document The COLLADA document that owns the animated element.
		@param value The value pointer for the angle.
		@param arrayElement The optional array index for animated element
			that belong to an animated element list.
		@return The new animated element. */
	static FCDAnimatedAngle* Create(FCDocument* document, float* value, int32 arrayElement=-1);

	/** [INTERNAL] Creates a new animated element.
		This function is used during the import of a COLLADA document.
		@param document The COLLADA document that owns the animated element.
		@param node The XML tree node that contains the animated values.
		@param value The value pointer for the angle.
		@param arrayElement The optional array index for animated element
			that belong to an animated element list.
		@return The new animated element. */
	static FCDAnimatedAngle* Create(FCDocument* document, xmlNode* node, float* value, int32 arrayElement=-1);

	/** [INTERNAL] Clones an animated element.
		@param document The COLLADA document that owns the cloned animated element.
		@param oldValue The angle value pointer contained within the original animated element.
		@param newValue The angle value pointer for the cloned animated element.
		@return The cloned animated value. */
	static FCDAnimated* Clone(FCDocument* document, const float* oldValue, float* newValue);
};

/** A COLLADA animated angle-axis.
	Used for rotations, takes in a 3D vector for the axis and
	a single floating-point value for the angle.
	@ingroup FCDocument */
class FCOLLADA_EXPORT FCDAnimatedAngleAxis : public FCDAnimated
{
private:
	// Don't build directly, use the Create function instead
	FCDAnimatedAngleAxis(FCDocument* document, FMVector3* axis, float* angle, int32 arrayElement);

public:
	/** Creates a new animated element.
		@param document The COLLADA document that owns the animated element.
		@param value The value pointer for the axis.
		@param angle The value pointer for the angle.
		@param arrayElement The optional array index for animated element
			that belong to an animated element list.
		@return The new animated element. */
	static FCDAnimatedAngleAxis* Create(FCDocument* document, FMVector3* value, float* angle, int32 arrayElement=-1);

	/** [INTERNAL] Creates a new animated element.
		This function is used during the import of a COLLADA document.
		@param document The COLLADA document that owns the animated element.
		@param node The XML tree node that contains the animated values.
		@param axis The value pointer for the axis.
		@param angle The value pointer for the angle.
		@param arrayElement The optional array index for animated element
			that belong to an animated element list.
		@return The new animated element. */
	static FCDAnimatedAngleAxis* Create(FCDocument* document, xmlNode* node, FMVector3* axis, float* angle, int32 arrayElement=-1);

	/** [INTERNAL] Clones an animated element.
		@param document The COLLADA document that owns the cloned animated element.
		@param oldAngle The angle value pointer contained within the original animated element.
		@param newAxis The axis value pointer for the cloned animated element.
		@param newAngle The angle value pointer for the cloned animated element.
		@return The cloned animated value. */
	static FCDAnimated* Clone(FCDocument* document, const float* oldAngle, FMVector3* newAxis, float* newAngle);
};

/** A COLLADA animated matrix.
	Used for animated transforms, takes in a 16 floating-point values.
	@ingroup FCDocument */
class FCOLLADA_EXPORT FCDAnimatedMatrix : public FCDAnimated
{
private:
	// Don't build directly, use the Create function instead
	FCDAnimatedMatrix(FCDocument* document, FMMatrix44* value, int32 arrayElement);

public:
	/** Creates a new animated element.
		@param document The COLLADA document that owns the animated element.
		@param value The value pointer for the matrix.
		@param arrayElement The optional array index for animated element
			that belong to an animated element list.
		@return The new animated element. */
	static FCDAnimatedMatrix* Create(FCDocument* document, FMMatrix44* value, int32 arrayElement=-1);

	/** [INTERNAL] Creates a new animated element.
		This function is used during the import of a COLLADA document.
		@param document The COLLADA document that owns the animated element.
		@param node The XML tree node that contains the animated values.
		@param value The value pointer for the matrix.
		@param arrayElement The optional array index for animated element
			that belong to an animated element list.
		@return The new animated element. */
	static FCDAnimatedMatrix* Create(FCDocument* document, xmlNode* node, FMMatrix44* value, int32 arrayElement=-1);

	/** [INTERNAL] Clones an animated element.
		@param document The COLLADA document that owns the cloned animated element.
		@param oldMx The matrix value pointer contained within the original animated element.
		@param newMx The matrix value pointer for the cloned animated element.
		@return The cloned animated value. */
	static FCDAnimated* Clone(FCDocument* document, const FMMatrix44* oldMx, FMMatrix44* newMx);
};

/** A COLLADA custom animated value.
	Used for animated extra elements. A single value is used multiple times to hold
	as many value pointers are necessary to hold the animation curves.
	@ingroup FCDocument */
class FCOLLADA_EXPORT FCDAnimatedCustom : public FCDAnimated
{
private:
	float dummy;

	// Don't build directly, use the Create function instead
	FCDAnimatedCustom(FCDocument* document);

	bool Link(xmlNode* node);

public:
	/** Creates a new animated element.
		@param document The COLLADA document that owns the animated element.
		@return The new animated element. */
	static FCDAnimatedCustom* Create(FCDocument* document);

	/** [INTERNAL] Creates a new animated element.
		This function is used during the import of a COLLADA document.
		@param document The COLLADA document that owns the animated element.
		@param node The XML tree node that contains the animated values.
		@return The new animated element. */
	static FCDAnimatedCustom* Create(FCDocument* document, xmlNode* node);

	/** Retrieves the floating-point value used for all the value pointers.
		@return The dummy floating-point value. */
	const float& GetDummy() const { return dummy; }
};

#endif // _FCD_ANIMATED_H_

