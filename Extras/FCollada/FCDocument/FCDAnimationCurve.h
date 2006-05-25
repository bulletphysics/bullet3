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
	@file FCDAnimationCurve.h
	This file contains the FCDAnimationCurve class.
*/

#ifndef _FCD_ANIMATION_CURVE_H_
#define _FCD_ANIMATION_CURVE_H_

#include "FUtils/FUDaeEnum.h"
#include "FCDocument/FCDObject.h"

class FCDAnimationClip;
class FCDAnimationChannel;

typedef vector<FCDAnimationClip*> FCDAnimationClipList; /**< A dynamically-sized array of animation clips. */
typedef float (*FCDConversionFunction)(float v); /**< A simple conversion function. */

/**
	A COLLADA single-dimensional animation curve.
	An animation curve holds the keyframes necessary
	to animate an animatable floating-point value.

	There are multiple interpolation mechanisms supported by COLLADA.
	FCollada supports the CONSTANT, LINEAR and BEZIER interpolations.

	@see FUDaeInterpolation FUDaeInfinity
	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDAnimationCurve : public FCDObject
{
private:
	FCDAnimationChannel* parent;

	// Targeting information
	int32 targetElement;
	string targetQualifier;

	// Input information
	FloatList keys, keyValues;
	FloatList inTangents, outTangents;
	FloatList inTangentWeights, outTangentWeights;
	bool isWeightedCurve;
	FUDaeInfinity::Infinity preInfinity;
	FUDaeInfinity::Infinity postInfinity;

	// Driver information
	const float* inputDriver;
	string driverPointer;

	// The interpolation values follow the FUDaeInterpolation enum (FUDaeEnum.h)
	UInt32List interpolations;

	// Animation clips that depend on this curve
	FCDAnimationClipList clips;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDAnimationChannel::AddCurve function.
		You should also attach the new curve to an animated
		element using the FCDAnimated::SetCurve function.
		@param document The COLLADA document that owns the animation curve.
		@param parent The animation channel that contains the curve. */
	FCDAnimationCurve(FCDocument* document, FCDAnimationChannel* parent);

	/** Destructor: do not release directly.
		Instead, use the FCDAnimationChannel::ReleaseCurve function. */
	virtual ~FCDAnimationCurve();

	/** Retrieves the animation channel that contains this animation curve.
		@return The parent animation channel. */
	inline FCDAnimationChannel* GetParent() { return parent; }
	inline const FCDAnimationChannel* GetParent() const { return parent; } /**< See above. */

	/** Retrieves the list of key inputs for the animation curve.
		@return The list of key inputs. */
	inline FloatList& GetKeys() { return keys; }
	inline const FloatList& GetKeys() const { return keys; } /**< See above. */

	/** Retrieves the list of key outputs for the animation curve.
		@return The list of key outputs. */
	inline FloatList& GetKeyValues() { return keyValues; }
	inline const FloatList& GetKeyValues() const { return keyValues; } /**< See above. */

	/** Retrieves the list of interpolation type for the segments of the animation curve.
		There is always one interpolation type for each key in the curve. The interpolation type
		of a segment of the curve is set at the key at which begins the segment.
		@see FUDaeInterpolation
		@return The list of interpolation types. */
	inline UInt32List& GetInterpolations() { return interpolations; }
	inline const UInt32List& GetInterpolations() const { return interpolations; } /**< See above. */

	/** Retrieves the list of key in-tangent values for the animation curve.
		This list has data only for curves that include segments with the bezier interpolation.
		@return The list of in-tangent values. */
	inline FloatList& GetInTangents() { return inTangents; }
	inline const FloatList& GetInTangents() const { return inTangents; } /**< See above. */

	/** Retrieves the list of key out-tangent values for the animation curve.
		This list has data only for curves that include segments with the bezier interpolation.
		@return The list of out-tangent values. */
	inline FloatList& GetOutTangents() { return outTangents; }
	inline const FloatList& GetOutTangents() const { return outTangents; } /**< See above. */

	/** Retrieves the list of key in-tangent weights for the animation curve.
		This list has data only for curves that are weighted
		and include segments with the bezier interpolation.
		@see IsWeightedCurve
		@return The list of in-tangent weights. */
	inline FloatList& GetInTangentWeights() { return inTangentWeights; }
	inline const FloatList& GetInTangentWeights() const { return inTangentWeights; } /**< See above. */

	/** Retrieves the list of key out-tangent weights for the animation curve.
		This list has data only for curves that are weighted
		and include segments with the bezier interpolation.
		@see IsWeightedCurve
		@return The list of out-tangent weights. */
	inline FloatList& GetOutTangentWeights() { return outTangentWeights; }
	inline const FloatList& GetOutTangentWeights() const { return outTangentWeights; } /**< See above. */

	/** Retrieves whether this curve has weighted tangents. Tangent weights
		give you access to 2D tangents by providing the length of the tangent.
		@return Whether this curve has weighted tangents. */
	inline bool IsWeightedCurve() const { return isWeightedCurve; }

	/** Sets whether this curve has weighted tangents. Tangent weights
		give you access to 2D tangents by providing the length of the tangent.
		@param _isWeightedCurve Whether this curve has weighted tangents. */
	inline void SetWeightedCurveFlag(bool _isWeightedCurve) { isWeightedCurve = _isWeightedCurve; }

	/** Retrieves the type of behavior for the curve if the input value is
		outside the input interval defined by the curve keys and less than any key input value.
		@see FUDaeInfinity
		@return The pre-infinity behavior of the curve. */
	inline FUDaeInfinity::Infinity GetPreInfinity() const { return preInfinity; }

	/** Sets the behavior of the curve if the input value is
		outside the input interval defined by the curve keys and less than any key input value.
		@see FUDaeInfinity
		@param infinity The pre-infinity behavior of the curve. */
	inline void SetPreInfinity(FUDaeInfinity::Infinity infinity) { preInfinity = infinity; }

	/** Retrieves the type of behavior for the curve if the input value is
		outside the input interval defined by the curve keys and greater than any key input value.
		@see FUDaeInfinity
		@return The post-infinity behavior of the curve. */
	inline FUDaeInfinity::Infinity GetPostInfinity() const { return postInfinity; }

	/** Sets the behavior of the curve if the input value is
		outside the input interval defined by the curve keys and greater than any key input value.
		@see FUDaeInfinity
		@param infinity The post-infinity behavior of the curve. */
	inline void SetPostInfinity(FUDaeInfinity::Infinity infinity) { postInfinity = infinity; }

	/** Retrieves the value pointer that drives this animation curve.
		@return The driver value pointer. This pointer will be NULL to indicate
			that time drives the animation curve. */
	inline const float* GetDriver() const { return inputDriver; }

	/** Sets the value pointer that drives the animation curve.
		@param driver The driver value pointer. Set this pointer to NULL
			to indicate that time drives the animation curve. */
	inline void SetDriver(const float* driver) { inputDriver = driver; }

	/** Retrieves the list of animation clips that use this animation curve.
		@return The list of animation clips. */
	inline FCDAnimationClipList& GetClips() { return clips; }
	inline const FCDAnimationClipList& GetClips() const { return clips; } /**< See above. */

	/** Readies this curve for evaluation.
		This will create the tangents and the tangent weights, if necessary. */
	void Ready();

	/** Clones the animation curve.
		@return The cloned animation curve. */
	FCDAnimationCurve* Clone();

	/** Applies a conversion function to the keys output values of the animation curve.
		@param valueConversion The conversion function to use on the key outputs.
		@param tangentConversion The conversion function to use on the key tangents. */
	void ConvertValues(FCDConversionFunction valueConversion, FCDConversionFunction tangentConversion);

	/** Applies a conversion function to the keys input values of the animation curve.
		@param timeConversion The conversion function to use on the key inputs.
		@param tangentWeightConversion The conversion function to use on the key tangent weights. */
	void ConvertInputs(FCDConversionFunction timeConversion, FCDConversionFunction tangentWeightConversion);

	/** Evaluates the animation curve.
		@param input An input value.
		@return The sampled value of the curve at the given input value. */
	float Evaluate(float input) const;

	/** [INTERNAL] Adds an animation clip to the list of animation clips that use this curve.
		@param clip An animation clip. */
	inline void RegisterAnimationClip(FCDAnimationClip* clip) { clips.push_back(clip); }

	/** [INTERNAL] Writes out the data sources necessary to import the animation curve
		to a given XML tree node.
		@param parentNode The XML tree node in which to create the data sources.
		@param baseId A COLLADA Id prefix to use when generating the source ids. */
	void WriteSourceToXML(xmlNode* parentNode, const string& baseId) const;

	/** [INTERNAL] Writes out the sampler that puts together the data sources
		and generates a sampling function.
		@param parentNode The XML tree node in which to create the sampler.
		@param baseId The COLLADA id prefix used when generating the source ids.
			This prefix is also used to generate the sampler COLLADA id.
		@return The created XML tree node. */
	xmlNode* WriteSamplerToXML(xmlNode* parentNode, const string& baseId) const;

	/** [INTERNAL] Writes out the animation channel that attaches the sampling function
		to the animatable value.
		@param parentNode The XML tree node in which to create the sampler.
		@param baseId The COLLADA Id prefix used when generating the source ids
			and the sampler id.
		@param targetPointer The target pointer prefix for the targeted animated element.
		@return The created XML tree node. */
	xmlNode* WriteChannelToXML(xmlNode* parentNode, const string& baseId, const char* targetPointer) const;

	/** [INTERNAL] Retrieves the target element suffix for the curve.
		This will be -1 if the animated element does not belong to an
		animated element list.
		@return The target element suffix. */
	inline int32 GetTargetElement() const { return targetElement; }

	/** [INTERNAL] Retrieves the target qualifier for the curve.
		This will be the empty string if that the curve affects
		a one-dimensional animated element.
		@return The target qualifier. */
	inline const string& GetTargetQualifier() const { return targetQualifier; }

	/** [INTERNAL] Sets the target element suffix for the curve.
		@param e The target element suffix. Set to value to -1
			if the animated element does not belong to an animated element list. */
	inline void SetTargetElement(int32 e) { targetElement = e; }

	/** [INTERNAL] Sets the target qualifier for the curve.
		@param q The target qualifier. You may sets this string to the empty string
			only if that the curve affects a one-dimensional animated element. */
	inline void SetTargetQualifier(const string& q) { targetQualifier = q; }

	/** [INTERNAL] Retrieves the target pointer prefix of the driver.
		@return The driver's target pointer prefix. */
	inline const string& GetDriverPointer() const { return driverPointer; }

	/** [INTERNAL] Sets the target pointer prefix of the driver.
		@param p The driver's target pointer prefix. Set this string to the
			empty string if the input of the animation curve is the time. */
	inline void SetDriverPointer(const string& p) { driverPointer = p; }
};

#endif // _FCD_ANIMATION_CURVE_H_
