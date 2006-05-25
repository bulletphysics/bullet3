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

#include "StdAfx.h"
#include "FCDocument/FCDAnimationCurve.h"
#include "FCDocument/FCDAnimationMultiCurve.h"
#include "FUtils/FUDaeEnum.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeWriter;

#define SMALL_DELTA	0.001f

FCDAnimationMultiCurve::FCDAnimationMultiCurve(FCDocument* document, uint32 _dimension) : FCDObject(document, "FCDAnimationMultiCurve")
{
	dimension = _dimension;
	if (dimension == 0) dimension = 1;
	
	// Prepare the target information
	targetElement = -1;
	targetQualifiers = new string[dimension];

	// Allocate the key values and tangents to the wanted dimension
	keyValues = new FloatList[dimension];
	inTangents = new FloatList[dimension];
	outTangents = new FloatList[dimension];
}

FCDAnimationMultiCurve::~FCDAnimationMultiCurve()
{
	SAFE_DELETE_ARRAY(targetQualifiers);
	SAFE_DELETE_ARRAY(keyValues);
	SAFE_DELETE_ARRAY(inTangents);
	SAFE_DELETE_ARRAY(outTangents);
}

// Samples all the curves for a given input
void FCDAnimationMultiCurve::Evaluate(float input, float* output) const
{
	// Single key curves imply a constant value
	if (keys.size() == 1)
	{
		for (uint32 i = 0; i < dimension; ++i) output[i] = keyValues[i].front();
		return;
	}

	// Find the current interval
	uint32 index = 0;
	FloatList::const_iterator it;
	for (it = keys.begin(); it != keys.end(); ++it, ++index)
	{
		if ((*it) > input) break;
	}

	if (it == keys.end())
	{
		// We're sampling after the curve, return the last values
		for (uint32 i = 0; i < dimension; ++i) output[i] = keyValues[i].back();
		return;
	}
	else if (it == keys.begin())
	{
		// We're sampling before the curve, return the first values
		for (uint32 i = 0; i < dimension; ++i) output[i] = keyValues[i].front();
		return;
	}

	// Get the keys and values for this interval
	float endKey = *it;
	float startKey = *(it - 1);

	// Interpolate the outputs.
	// Similar code is found in FCDAnimationCurve.cpp. If you update this, update the other one too.
	uint32 interpolation = interpolations.empty() ? ((uint32) FUDaeInterpolation::DEFAULT) : interpolations[index];
	switch (FUDaeInterpolation::Interpolation(interpolation))
	{
	case FUDaeInterpolation::LINEAR: {
		for (uint32 i = 0; i < dimension; ++i)
		{
			float startValue = keyValues[i][index - 1];
			float endValue = keyValues[i][index];
			output[i] = (input - startKey) / (endKey - startKey) * (endValue - startValue) + startValue; 
		}
		break; }

	case FUDaeInterpolation::BEZIER: {
		for (uint32 i = 0; i < dimension; ++i)
		{
			float startValue = keyValues[i][index - 1];
			float endValue = keyValues[i][index];

			float t = (input - startKey) / (endKey - startKey);
			float bValue = startValue + outTangents[i][index - 1];
			float cValue = endValue - inTangents[i][index];
			float ti = 1.0f - t;

			output[i] = startValue * ti * ti * ti + 3.0f * bValue * ti * ti * t + 3.0f * cValue * ti * t * t + endValue * t * t * t;
		}
		break; }

	case FUDaeInterpolation::UNKNOWN:
	case FUDaeInterpolation::STEP:
	default: {
		for (uint32 i = 0; i < dimension; ++i)
		{
			output[i] = keyValues[i][index - 1];
		}
		break; }
	}
}

FCDAnimationMultiCurve* FCDAnimationMultiCurve::MergeCurves(const vector<FCDAnimationCurve*>& _toMerge, const FloatList& defaultValues)
{
	vector<const FCDAnimationCurve*> toMerge(_toMerge.size());
	for (vector<FCDAnimationCurve*>::const_iterator itC = _toMerge.begin(); itC != _toMerge.end(); ++itC)
	{
		toMerge.push_back(*itC);
	}
	return MergeCurves(toMerge, defaultValues);
}

// Non-standard constructor used to merge together animation curves
FCDAnimationMultiCurve* FCDAnimationMultiCurve::MergeCurves(const vector<const FCDAnimationCurve*>& toMerge, const FloatList& defaultValues)
{
	size_t dimension = toMerge.size();
	if (dimension == 0) return NULL;

	// Look for the document pointer
	FCDocument* document = NULL;
	int32 targetElement = -1;
	for (size_t i = 0; i < dimension; ++i)
	{
		if (toMerge[i] != NULL)
		{
			document = toMerge[i]->GetDocument();
			targetElement = toMerge[i]->GetTargetElement();
		}
	}
	if (document == NULL) return NULL;

	// Allocate the output multiCurve.
	FCDAnimationMultiCurve* multiCurve = new FCDAnimationMultiCurve(document, (uint32) dimension);
	multiCurve->targetElement = targetElement;

	// Grab all the animation curve data element right away, to spare me some typing.
	FloatList& keys = multiCurve->GetKeys();
	FloatList* values = multiCurve->GetKeyValues();
	FloatList* inTangents = multiCurve->GetInTangents();
	FloatList* outTangents = multiCurve->GetOutTangents();
	UInt32List& interpolations = multiCurve->GetInterpolations();

	// Calculate the merged input keys
	for (size_t i = 0; i < dimension; ++i)
	{
		const FCDAnimationCurve* curve = toMerge[i];
		if (curve == NULL) continue;

		const FloatList& curveKeys = curve->GetKeys();

		// Merge each curve's keys, which should already be sorted, into the multi-curve's
		size_t multiCurveKeyCount = keys.size(), m = 0;
		size_t curveKeyCount = curveKeys.size(), c = 0;
		while (m < multiCurveKeyCount && c < curveKeyCount)
		{
			if (IsEquivalent(keys[m], curveKeys[c])) { ++c; ++m; }
			else if (keys[m] < curveKeys[c]) { ++m; }
			else { keys.insert(keys.begin() + m, curveKeys[c++]); }
		}
		if (c < curveKeyCount) keys.insert(keys.end(), curveKeys.begin() + c, curveKeys.end());
	}
	size_t keyCount = keys.size();

	// Start with the unknown interpolation everywhere
	interpolations.resize(keyCount);
	for (UInt32List::iterator it = interpolations.begin(); it != interpolations.end(); ++it)
	{
		(*it) = (uint32) FUDaeInterpolation::UNKNOWN;
	}

	// Merge the curves one by one into the multi-curve
	for (size_t i = 0; i < dimension; ++i)
	{
		// Pre-allocate the value and tangent data arrays
		values[i].resize(keyCount);
		inTangents[i].resize(keyCount);
		outTangents[i].resize(keyCount);

		const FCDAnimationCurve* curve = toMerge[i];
		if (curve == NULL)
		{
			// No curve, set the default value on all the keys
			float defaultValue = (i < defaultValues.size()) ? defaultValues[i] : 0.0f;
			for (size_t k = 0; k < keyCount; ++k)
			{
				values[i][k] = defaultValue;
				inTangents[i][k] = 0.0f;
				outTangents[i][k] = 0.0f;
			}
			continue;
		}

		multiCurve->targetQualifiers[i] = curve->GetTargetQualifier();

		// Does the curve have per-key interpolations?
		bool hasDefaultInterpolation = curve->GetInterpolations().empty();

		// Merge in this curve's values, sampling when the multi-curve's key is not present in the curve.
		const FloatList& curveKeys = curve->GetKeys();
		size_t curveKeyCount = curveKeys.size();
		bool sampleNextInTangent = false;
		for (size_t k = 0, c = 0; k < keyCount; ++k)
		{
			uint32 interpolation;
			if (c >= curveKeyCount || !IsEquivalent(keys[k], curveKeys[c]))
			{
				// Sample the curve
				float value = values[i][k] = curve->Evaluate(keys[k]);

				// Calculate the in-tangent and the previous key's out-tangent
				float span = ((k > 0) ? (keys[k] - keys[k - 1]) : (keys[k + 1] - keys[k])) / 3.0f;
				inTangents[i][k] = value - curve->Evaluate(keys[k] - SMALL_DELTA) / SMALL_DELTA * span;
				if (k > 0) outTangents[i][k-1] = curve->Evaluate(keys[k - 1] + SMALL_DELTA) / SMALL_DELTA * span - values[i][k-1];

				// Calculate the out-tangent and force the sampling of the next key's in-tangent
				span = (c < curveKeyCount - 1) ? (keys[k + 1] - keys[k]) / 3.0f : span;
				outTangents[i][k] = curve->Evaluate(keys[k] + SMALL_DELTA) / SMALL_DELTA * span - value;
				interpolation = FUDaeInterpolation::BEZIER;
				sampleNextInTangent = true;
			}
			else
			{
				// Keys match, grab the values directly
				values[i][k] = curve->GetKeyValues()[c];
				outTangents[i][k] = curve->GetOutTangents()[c];
				interpolation = hasDefaultInterpolation ? ((uint32) FUDaeInterpolation::DEFAULT) : curve->GetInterpolations()[c];

				// Sampling the previous key would require that we sample the inTangent
				if (!sampleNextInTangent) inTangents[i][k] = curve->GetInTangents()[c];
				else
				{
					float span = (keys[k] - keys[k - 1]) / 3.0f;
					inTangents[i][k] = values[i][k] - curve->Evaluate(keys[k] - SMALL_DELTA) / SMALL_DELTA * span;
				}
				++c;
			}

			// Merge the interpolation values, where bezier wins whenever interpolation values differ
			uint32& oldInterpolation = interpolations[k];
			if (oldInterpolation == FUDaeInterpolation::UNKNOWN) oldInterpolation = interpolation;
			else if (oldInterpolation != interpolation) oldInterpolation = FUDaeInterpolation::BEZIER;
		}
	}

	// Reset any unknown interpolation left
	for (UInt32List::iterator it = interpolations.begin(); it != interpolations.end(); ++it)
	{
		if ((*it) == (uint32) FUDaeInterpolation::UNKNOWN) (*it) = FUDaeInterpolation::DEFAULT;
	}

	return multiCurve;
}

// Collapse this multi-dimensional curve into a one-dimensional curve, given a collapsing function
FCDAnimationCurve* FCDAnimationMultiCurve::Collapse(FCDCollapsingFunction collapse) const
{
	size_t keyCount = keys.size();
	if (keyCount == 0 || dimension == 0) return NULL;
	if (collapse == NULL) collapse = Average;

	// Create the output one-dimensional curve and retrieve its data list
	FCDAnimationCurve* out = new FCDAnimationCurve(GetDocument(), NULL);
	out->SetTargetElement(targetElement);
	FloatList& outKeys = out->GetKeys();
	FloatList& outKeyValues = out->GetKeyValues();
	FloatList& outInTangents = out->GetInTangents();
	FloatList& outOutTangents = out->GetOutTangents();
	UInt32List& outInterpolations = out->GetInterpolations();

	// Pre-allocate the output arrays
	outKeys.resize(keyCount);
	outKeyValues.resize(keyCount);
	outInTangents.resize(keyCount);
	outOutTangents.resize(keyCount);
	outInterpolations.resize(keyCount);

	// Copy the key data over, collapsing the values
	float* buffer = new float[dimension];
	for (size_t i = 0; i < keyCount; ++i)
	{
		outKeys[i] = keys[i];
		outInterpolations[i] = interpolations[i];

		// Collapse the values and the tangents
#		define COLLAPSE(outArray, inArray) \
		for (uint32 j = 0; j < dimension; ++j) buffer[j] = inArray[j][i]; \
		outArray[i] = (*collapse)(buffer, dimension)

		COLLAPSE(outKeyValues, keyValues);
		COLLAPSE(outInTangents, inTangents);
		COLLAPSE(outOutTangents, outTangents);
#		undef COLLAPSE
	}
	SAFE_DELETE_ARRAY(buffer);

	return out;
}

// Write out the specific animation elements to the COLLADA xml tree node
void FCDAnimationMultiCurve::WriteSourceToXML(xmlNode* parentNode, const string& baseId)
{
	if (keys.empty() || dimension == 0 || keyValues[0].empty()) return;

	// Generate the list of the parameters
	typedef const char* pchar;
	pchar* parameters = new pchar[dimension];
	for (size_t i = 0; i < dimension; ++i)
	{
		parameters[i] = targetQualifiers[i].c_str();
		if (*(parameters[i]) == '.') ++(parameters[i]);
	}

	// Export the key times
	AddSourceFloat(parentNode, baseId + "-input", keys, "TIME");

	// Interlace the key values and tangents for the export
	size_t valueCount = keyValues[0].size();
	FloatList sourceData; sourceData.reserve(dimension * valueCount);
	for (size_t v = 0; v < valueCount; ++v) for (uint32 n = 0; n < dimension; ++n) sourceData.push_back(keyValues[n].at(v));
	AddSourceFloat(parentNode, baseId + "-output", sourceData, dimension, parameters);
	
	sourceData.clear();
	for (size_t v = 0; v < valueCount; ++v) for (uint32 n = 0; n < dimension; ++n) sourceData.push_back(inTangents[n].at(v));
	AddSourceFloat(parentNode, baseId + "-intangents", sourceData, dimension, parameters);

	sourceData.clear();
	for (size_t v = 0; v < valueCount; ++v) for (uint32 n = 0; n < dimension; ++n) sourceData.push_back(outTangents[n].at(v));
	AddSourceFloat(parentNode, baseId + "-outtangents", sourceData, dimension, parameters);

	// Weights not yet supported on multi-curve

	AddSourceInterpolation(parentNode, baseId + "-interpolations", *(FUDaeInterpolationList*)&interpolations);
}

xmlNode* FCDAnimationMultiCurve::WriteSamplerToXML(xmlNode* parentNode, const string& baseId)
{
	xmlNode* samplerNode = AddChild(parentNode, DAE_SAMPLER_ELEMENT);
	AddAttribute(samplerNode, DAE_ID_ATTRIBUTE, baseId + "-sampler");

	// Add the sampler inputs
	AddInput(samplerNode, baseId + "-input", DAE_INPUT_ANIMATION_INPUT);
	AddInput(samplerNode, baseId + "-output", DAE_OUTPUT_ANIMATION_INPUT);
	AddInput(samplerNode, baseId + "-intangents", DAE_INTANGENT_ANIMATION_INPUT);
	AddInput(samplerNode, baseId + "-outtangents", DAE_OUTTANGENT_ANIMATION_INPUT);
	AddInput(samplerNode, baseId + "-interpolations", DAE_INTERPOLATION_ANIMATION_INPUT);
	return samplerNode;	
}

xmlNode* FCDAnimationMultiCurve::WriteChannelToXML(xmlNode* parentNode, const string& baseId, const string& pointer)
{
	xmlNode* channelNode = AddChild(parentNode, DAE_CHANNEL_ELEMENT);
	AddAttribute(channelNode, DAE_SOURCE_ATTRIBUTE, baseId + "-sampler");

	// Generate and export the full target [no qualifiers]
	globalSBuilder.set(pointer);
	if (targetElement >= 0)
	{
		globalSBuilder.append('('); globalSBuilder.append(targetElement); globalSBuilder.append(')');
	}
	AddAttribute(channelNode, DAE_TARGET_ATTRIBUTE, globalSBuilder);
	return channelNode;
}
