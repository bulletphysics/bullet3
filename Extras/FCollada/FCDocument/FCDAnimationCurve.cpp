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
#include "FCDocument/FCDAnimationClip.h"
#include "FUtils/FUDaeEnum.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeWriter;

FCDAnimationCurve::FCDAnimationCurve(FCDocument* document, FCDAnimationChannel* _parent)
 :	FCDObject(document, "FCDAnimationCurve"),
	parent(_parent),
	targetElement(-1),
	preInfinity(FUDaeInfinity::CONSTANT),
	postInfinity(FUDaeInfinity::CONSTANT),
	inputDriver(NULL)
{
}

FCDAnimationCurve::~FCDAnimationCurve()
{
	inputDriver = NULL;
	parent = NULL;
	clips.clear();
}

FCDAnimationCurve* FCDAnimationCurve::Clone()
{
	FCDAnimationCurve* clone = new FCDAnimationCurve(GetDocument(), parent);

	clone->SetTargetElement(targetElement);
	string q;
	q.assign(targetQualifier);
	clone->SetTargetQualifier(q);
	
	clone->keys = keys;
	clone->keyValues = keyValues;
	clone->inTangents = inTangents;
	clone->outTangents = outTangents;
	clone->inTangentWeights = inTangentWeights;
	clone->outTangentWeights = outTangentWeights;
	clone->isWeightedCurve = isWeightedCurve;
	clone->preInfinity = preInfinity;
	clone->postInfinity = postInfinity;

	clone->inputDriver = inputDriver;
	
	clone->SetDriverPointer(driverPointer);

	clone->interpolations = interpolations;

	// Animation clips that depend on this curve
	for(FCDAnimationClipList::iterator it = clips.begin(); it != clips.end(); ++it)
	{
		clone->clips.push_back((*it)->Clone());
	}
	return clone;
}

// Prepare a curve for evaluation
void FCDAnimationCurve::Ready()
{
	if (keys.empty()) return;

	if (inTangents.empty() || outTangents.empty())
	{
		// Calculate the bezier tangents
		inTangents.resize(keys.size(), 0.0f);
		outTangents.resize(keys.size(), 0.0f);

		if (keys.size() > 1)
		{
			for (size_t i = 0; i < keys.size(); ++i)
			{
				float previousKeySpan = (i > 0) ? keys[i] - keys[i - 1] : keys[i + 1] - keys[i];
				float nextKeySpan = (i < keys.size() - 1) ? keys[i + 1] - keys[i] : previousKeySpan;
				float currentKeyValue = keyValues[i];
				float previousKeyValue = (i > 0) ? keyValues[i - 1] : currentKeyValue;
				float nextKeyValue = (i < keys.size() - 1) ? keyValues[i + 1] : currentKeyValue;
				float slope = (nextKeyValue - previousKeyValue) / (nextKeySpan + previousKeySpan);
				inTangents[i] = previousKeySpan / 3.0f * slope;
				outTangents[i] = nextKeySpan / 3.0f * slope;
			}
		}
	}

	if (interpolations.empty())
	{
		// Fill in the array with the default interpolation type
		interpolations.resize(keys.size(), FUDaeInterpolation::DEFAULT);
	}

	isWeightedCurve = !inTangentWeights.empty() && !outTangentWeights.empty();
}

// Main workhorse for the animation system:
// Evaluates the curve for a given input
float FCDAnimationCurve::Evaluate(float input) const
{
	if (keys.size() == 1) return keyValues.front();

	float outputStart = keyValues.front();
	float outputEnd = keyValues.back();
	float inputStart = keys.front();
	float inputEnd = keys.back();
	float inputSpan = inputEnd - inputStart;

	// Account for pre-infinity mode
	float outputOffset = 0.0f;
	if (input <= inputStart)
	{
		switch (preInfinity)
		{
		case FUDaeInfinity::CONSTANT: return outputStart;
		case FUDaeInfinity::LINEAR: return outputStart + (input - inputStart) * (keyValues[1] - outputStart) / (keys[1] - inputStart);
		case FUDaeInfinity::CYCLE: { float cycleCount = ceilf((inputStart - input) / inputSpan); input += cycleCount * inputSpan; break; }
		case FUDaeInfinity::CYCLE_RELATIVE: { float cycleCount = ceilf((inputStart - input) / inputSpan); input += cycleCount * inputSpan; outputOffset -= cycleCount * (outputEnd - outputStart); break; }
		case FUDaeInfinity::OSCILLATE: { float cycleCount = ceilf((inputStart - input) / (2.0f * inputSpan)); input += cycleCount * 2.0f * inputSpan; input = inputEnd - fabsf(input - inputEnd); break; }
		case FUDaeInfinity::UNKNOWN: default: return outputStart;
		}
	}

	// Account for post-infinity mode
	else if (input >= inputEnd)
	{
		switch (postInfinity)
		{
		case FUDaeInfinity::CONSTANT: return outputEnd;
		case FUDaeInfinity::LINEAR: return outputEnd + (input - inputEnd) * (keyValues[keys.size() - 2] - outputEnd) / (keys[keys.size() - 2] - inputEnd);
		case FUDaeInfinity::CYCLE: { float cycleCount = ceilf((input - inputEnd) / inputSpan); input -= cycleCount * inputSpan; break; }
		case FUDaeInfinity::CYCLE_RELATIVE: { float cycleCount = ceilf((input - inputEnd) / inputSpan); input -= cycleCount * inputSpan; outputOffset += cycleCount * (outputEnd - outputStart); break; }
		case FUDaeInfinity::OSCILLATE: { float cycleCount = ceilf((input - inputEnd) / (2.0f * inputSpan)); input -= cycleCount * 2.0f * inputSpan; input = inputStart + fabsf(input - inputStart); break; }
		case FUDaeInfinity::UNKNOWN: default: return outputEnd;
		}
	}

	// Find the current interval
	uint32 index = 0;
	FloatList::const_iterator it;
	for (it = keys.begin(); it != keys.end(); ++it, ++index)
	{
		if ((*it) > input) break;
	}

	// Get the keys and values for this interval
	float endKey = *it;
	float startKey = *(it - 1);
	float endValue = keyValues[index];
	float startValue = keyValues[index - 1];
	float output;

	// Interpolate the output.
	// Similar code is found in FCDAnimationMultiCurve.cpp. If you update this, update the other one too.
	uint32 interpolation = interpolations.empty() ? ((uint32) FUDaeInterpolation::DEFAULT) : interpolations[index];
	switch (FUDaeInterpolation::Interpolation(interpolation))
	{
	case FUDaeInterpolation::LINEAR:
		output = (input - startKey) / (endKey - startKey) * (endValue - startValue) + startValue;
		break;

	case FUDaeInterpolation::BEZIER: {
		float t = (input - startKey) / (endKey - startKey);
		float bValue = startValue + outTangents[index - 1];
		float cValue = endValue - inTangents[index];
		float ti = 1.0f - t;
		output = startValue * ti * ti * ti + 3.0f * bValue * ti * ti * t + 3.0f * cValue * ti * t * t + endValue * t * t * t;
		break; }

	case FUDaeInterpolation::STEP:
	case FUDaeInterpolation::UNKNOWN:
	default:
		output = startValue;
		break;
	}

	return outputOffset + output;
}

// Apply a conversion function on the key values and tangents
void FCDAnimationCurve::ConvertValues(FCDConversionFunction valueConversion, FCDConversionFunction tangentConversion)
{
	size_t keyCount = keys.size();
	if (valueConversion != NULL)
	{
		for (size_t k = 0; k < keyCount; k++)
		{
			keyValues[k] = (*valueConversion)(keyValues[k]);
		}
	}
	if (tangentConversion != NULL)
	{
		for (size_t k = 0; k < keyCount; k++)
		{
			inTangents[k] = (*tangentConversion)(inTangents[k]);
			outTangents[k] = (*tangentConversion)(outTangents[k]);
		}
	}
}

// Apply a conversion function on the key times and tangent weights
void FCDAnimationCurve::ConvertInputs(FCDConversionFunction timeConversion, FCDConversionFunction tangentWeightConversion)
{
	size_t keyCount = keys.size();
	if (timeConversion != NULL)
	{
		for (size_t k = 0; k < keyCount; k++)
		{
			keys[k] = (*timeConversion)(keys[k]);
		}
	}
	if (tangentWeightConversion != NULL)
	{
		for (size_t k = 0; k < keyCount; k++)
		{
			inTangentWeights[k] = (*tangentWeightConversion)(inTangentWeights[k]);
			outTangentWeights[k] = (*tangentWeightConversion)(outTangentWeights[k]);
		}
	}
}

// Write out the specific animation elements to the COLLADA xml tree node
void FCDAnimationCurve::WriteSourceToXML(xmlNode* parentNode, const string& baseId) const
{
	const char* parameter = targetQualifier.c_str();
	if (*parameter == '.') ++parameter;

	xmlNode* sourceNode = AddSourceFloat(parentNode, baseId + "-input", keys, "TIME");
	AddSourceFloat(parentNode, baseId + "-output", keyValues, parameter);
	AddSourceFloat(parentNode, baseId + "-intangents", inTangents, parameter);
	AddSourceFloat(parentNode, baseId + "-outtangents", outTangents, parameter);
	if (isWeightedCurve && !inTangentWeights.empty())
	{
		AddSourceFloat(parentNode, baseId + "-intangents_weights", inTangents, parameter);
		AddSourceFloat(parentNode, baseId + "-outtangents_weights", outTangents, parameter);
	}
	AddSourceInterpolation(parentNode, baseId + "-interpolations", *(FUDaeInterpolationList*)&interpolations);

	// Export the infinity parameters
	xmlNode* mayaTechnique = AddTechniqueChild(sourceNode, DAEMAYA_MAYA_PROFILE);
	string infinityType = FUDaeInfinity::ToString(preInfinity);
	AddChild(mayaTechnique, DAEMAYA_PREINFINITY_PARAMETER, infinityType);
	infinityType = FUDaeInfinity::ToString(postInfinity);
	AddChild(mayaTechnique, DAEMAYA_POSTINFINITY_PARAMETER, infinityType);
}

xmlNode* FCDAnimationCurve::WriteSamplerToXML(xmlNode* parentNode, const string& baseId) const
{
	xmlNode* samplerNode = AddChild(parentNode, DAE_SAMPLER_ELEMENT);
	AddAttribute(samplerNode, DAE_ID_ATTRIBUTE, baseId + "-sampler");

	// Add the sampler inputs
	AddInput(samplerNode, baseId + "-input", DAE_INPUT_ANIMATION_INPUT);
	AddInput(samplerNode, baseId + "-output", DAE_OUTPUT_ANIMATION_INPUT);
	AddInput(samplerNode, baseId + "-intangents", DAE_INTANGENT_ANIMATION_INPUT);
	AddInput(samplerNode, baseId + "-outtangents", DAE_OUTTANGENT_ANIMATION_INPUT);
	if (isWeightedCurve && !inTangentWeights.empty())
	{
		AddInput(samplerNode, baseId + "-intangents_weights", DAEMAYA_INTANGENTWEIGHT_ANIMATION_INPUT);
		AddInput(samplerNode, baseId + "-outtangents_weights", DAEMAYA_OUTTANGENTWEIGHT_ANIMATION_INPUT);
	}
	AddInput(samplerNode, baseId + "-interpolations", DAE_INTERPOLATION_ANIMATION_INPUT);

	// Add the driver input
	if (inputDriver != NULL)
	{
		AddInput(samplerNode, driverPointer, DAEMAYA_DRIVER_INPUT);
	}

	return samplerNode;	
}

xmlNode* FCDAnimationCurve::WriteChannelToXML(xmlNode* parentNode, const string& baseId, const char* targetPointer) const
{
	xmlNode* channelNode = AddChild(parentNode, DAE_CHANNEL_ELEMENT);
	AddAttribute(channelNode, DAE_SOURCE_ATTRIBUTE, baseId + "-sampler");

	// Generate and export the channel target
	globalSBuilder.set(targetPointer);
	if (targetElement >= 0)
	{
		globalSBuilder.append('('); globalSBuilder.append(targetElement); globalSBuilder.append(')');
	}
	globalSBuilder.append(targetQualifier);
	AddAttribute(channelNode, DAE_TARGET_ATTRIBUTE, globalSBuilder);
	return channelNode;
}
