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
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDAnimated.h"
#include "FCDocument/FCDAnimation.h"
#include "FCDocument/FCDAnimationChannel.h"
#include "FCDocument/FCDAnimationCurve.h"
#include "FCDocument/FCDAnimationMultiCurve.h"
#include "FUtils/FUDaeEnum.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
#include "FUtils/FUStringConversion.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDAnimationChannel::FCDAnimationChannel(FCDocument* document, FCDAnimation* _parent) : FCDObject(document, "FCDAnimationChannel")
{
	parent = _parent;
}

FCDAnimationChannel::~FCDAnimationChannel()
{
	CLEAR_POINTER_VECTOR(curves);
	parent = NULL;
}

FCDAnimationCurve* FCDAnimationChannel::AddCurve()
{
	FCDAnimationCurve* curve = new FCDAnimationCurve(GetDocument(), this);
	curves.push_back(curve);
	return curve;
}

void FCDAnimationChannel::ReleaseCurve(FCDAnimationCurve* curve)
{
	FCDAnimationCurveList::iterator itC = std::find(curves.begin(), curves.end(), curve);
	if (itC != curves.end())
	{
        // TODO: IMPLEMENT THIS. NEED RTTI and memory management. In other words, I need time!!! :(.
		// delete *itC;
		curves.erase(itC);
	}
}

// Consider this animated as the curve's driver
bool FCDAnimationChannel::LinkDriver(FCDAnimated* animated)
{
	bool driver = !driverPointer.empty();
	driver = driver && animated->GetTargetPointer() == driverPointer;
	if (driver && driverQualifier >= 0 && (uint32) driverQualifier < animated->GetValueCount())
	{
		// Retrieve the value pointer for the driver
		for (FCDAnimationCurveList::iterator itC = curves.begin(); itC != curves.end(); ++itC)
		{
			(*itC)->SetDriver(animated->GetValue((uint32) driverQualifier));
		}
	}
	return driver;
}
FUStatus FCDAnimationChannel::CheckDriver()
{
	FUStatus status;
	if (!driverPointer.empty() && !curves.empty() && curves.front()->GetDriver() == NULL)
	{
		status.Fail(FS("Unable to find animation curve driver: ") + TO_FSTRING(driverPointer) + FS(" for animation: ") + TO_FSTRING(parent->GetDaeId()));
	}
	return status;
}

// Load a Collada animation channel from the XML document
FUStatus FCDAnimationChannel::LoadFromXML(xmlNode* channelNode)
{
	FUStatus status;

	// Read the channel-specific ID
	string daeId = ReadNodeId(channelNode);
	string samplerId = ReadNodeSource(channelNode);
	ReadNodeTargetProperty(channelNode, targetPointer, targetQualifier);

	xmlNode* samplerNode = parent->FindChildById(samplerId);
	if (samplerNode == NULL || !IsEquivalent(samplerNode->name, DAE_SAMPLER_ELEMENT))
	{
		return status.Fail(FS("Unable to find sampler node for channel node: ") + TO_FSTRING(daeId), channelNode->line);
	}

	// Find and process the sources
	xmlNode* inputSource = NULL,* outputSource = NULL,* inTangentSource = NULL,* outTangentSource = NULL;
	xmlNode* outTangentWeightSource = NULL,* inTangentWeightSource = NULL,* interpolationSource = NULL;
	xmlNodeList samplerInputNodes;
	FindChildrenByType(samplerNode, DAE_INPUT_ELEMENT, samplerInputNodes);
	for (size_t i = 0; i < samplerInputNodes.size(); ++i) // Don't use iterator here because we are possibly appending source nodes in the loop
	{
		xmlNode* inputNode = samplerInputNodes[i];
		string sourceId = ReadNodeSource(inputNode);
		xmlNode* sourceNode = parent->FindChildById(sourceId);
		string sourceSemantic = ReadNodeSemantic(inputNode);

		if (sourceSemantic == DAE_INPUT_ANIMATION_INPUT) inputSource = sourceNode;
		else if (sourceSemantic == DAE_OUTPUT_ANIMATION_INPUT) outputSource = sourceNode;
		else if (sourceSemantic == DAE_INTANGENT_ANIMATION_INPUT) inTangentSource = sourceNode;
		else if (sourceSemantic == DAE_OUTTANGENT_ANIMATION_INPUT) outTangentSource = sourceNode;
		else if (sourceSemantic == DAEMAYA_INTANGENTWEIGHT_ANIMATION_INPUT) inTangentWeightSource = sourceNode;
		else if (sourceSemantic == DAEMAYA_OUTTANGENTWEIGHT_ANIMATION_INPUT) outTangentWeightSource = sourceNode;
		else if (sourceSemantic == DAE_INTERPOLATION_ANIMATION_INPUT) interpolationSource = sourceNode;
	}
	if (inputSource == NULL || outputSource == NULL)
	{
		return status.Fail(FS("Missing INPUT or OUTPUT sources in animation channel: ") + TO_FSTRING(parent->GetDaeId()), samplerNode->line);
	}

	// Calculate the number of curves that in contained by this channel
	xmlNode* outputAccessor = FindTechniqueAccessor(outputSource);
	string accessorStrideString = ReadNodeProperty(outputAccessor, DAE_STRIDE_ATTRIBUTE);
	uint32 curveCount = FUStringConversion::ToUInt32(accessorStrideString);
	if (curveCount == 0) curveCount = 1;

	// Create the animation curves
	curves.reserve(curveCount);
	for (uint32 i = 0; i < curveCount; ++i) AddCurve();

	// Read in the animation curves
	// The input keys are shared by all the curves
    ReadSource(inputSource, curves.front()->GetKeys());
	for (uint32 i = 1; i < curveCount; ++i) curves[i]->GetKeys() = curves.front()->GetKeys();

	// Read in the interleaved outputs and tangents as floats
	#define READ_SOURCE_INTERLEAVED(sourceNode, curveArrayPtr) \
	if (sourceNode != NULL) { \
		vector<FloatList*> arrays(curveCount); \
		for (uint32 i = 0; i < curveCount; ++i) { \
			arrays[i] = &(curves[i]->curveArrayPtr()); } \
		ReadSourceInterleaved(sourceNode, arrays); \
	}

	READ_SOURCE_INTERLEAVED(outputSource, GetKeyValues)
	READ_SOURCE_INTERLEAVED(inTangentSource, GetInTangents)
	READ_SOURCE_INTERLEAVED(outTangentSource, GetOutTangents)
	READ_SOURCE_INTERLEAVED(inTangentWeightSource, GetInTangentWeights)
	READ_SOURCE_INTERLEAVED(outTangentWeightSource, GetOutTangentWeights)
	#undef READ_SOURCE_INTERLEAVED

	// Read in the interleaved interpolation values, parsing the tokens directly
	if (interpolationSource != NULL)
	{
		vector<UInt32List*> arrays(curveCount);
		for (uint32 i = 0; i < curveCount; ++i) arrays[i] = &(curves[i]->GetInterpolations());
		ReadSourceInterpolationInterleaved(interpolationSource, arrays);
	}

	// Read in the pre/post-infinity type
	xmlNodeList mayaParameterNodes; StringList mayaParameterNames;
	xmlNode* mayaTechnique = FindTechnique(inputSource, DAEMAYA_MAYA_PROFILE);
	FindParameters(mayaTechnique, mayaParameterNames, mayaParameterNodes);
	size_t parameterCount = mayaParameterNodes.size();
	for (size_t i = 0; i < parameterCount; ++i)
	{
		xmlNode* parameterNode = mayaParameterNodes[i];
		const string& paramName = mayaParameterNames[i];
		const char* content = ReadNodeContentDirect(parameterNode);

		if (paramName == DAEMAYA_PREINFINITY_PARAMETER || paramName == DAEMAYA_PREINFINITY_PARAMETER1_3)
		{
			for (FCDAnimationCurveList::iterator itC = curves.begin(); itC != curves.end(); ++itC)
			{
				(*itC)->SetPreInfinity(FUDaeInfinity::FromString(content));
			}
		}
		else if (paramName == DAEMAYA_POSTINFINITY_PARAMETER || paramName == DAEMAYA_POSTINFINITY_PARAMETER1_3)
		{
			for (FCDAnimationCurveList::iterator itC = curves.begin(); itC != curves.end(); ++itC)
			{
				(*itC)->SetPostInfinity(FUDaeInfinity::FromString(content));
			}
		}
		else
		{
			// Look for driven-key input target
			if (paramName == DAE_INPUT_ELEMENT)
			{
				string semantic = ReadNodeSemantic(parameterNode);
				if (semantic == DAEMAYA_DRIVER_INPUT)
				{
					string fullDriverTarget = ReadNodeSource(parameterNode);
					const char* driverTarget = FUDaeParser::SkipPound(fullDriverTarget);
					if (driverTarget != NULL)
					{
						string driverQualifierValue;
						FUDaeParser::SplitTarget(driverTarget, driverPointer, driverQualifierValue);
						driverQualifier = FUDaeParser::ReadTargetMatrixElement(driverQualifierValue);
					}
				}
			}
		}
	}

	// Ready the curves for usage/evaluation.
	for (uint32 i = 0; i < curveCount; ++i) curves[i]->Ready();

	return status;
}

// Write out the animation curves for an animation channel to a COLLADA document
void FCDAnimationChannel::WriteToXML(xmlNode* parentNode) const
{
	string baseId = CleanId(targetPointer);

	// Check for curve merging
	uint32 realCurveCount = 0;
	FCDAnimationCurve* masterCurve = NULL;
	bool mergeCurves = true;
	for (FCDAnimationCurveList::const_iterator itC = curves.begin(); itC != curves.end(); ++itC)
	{
		FCDAnimationCurve* curve = (*itC);
		if ((*itC) != NULL)
		{
			++realCurveCount;
			if (masterCurve == NULL)
			{
				masterCurve = curve;
				if (masterCurve->GetDriver() != NULL) break;
			}
			else
			{
				// Check preliminary information, before verifying the individual keys: key count, infinity types and such..
				const FloatList& masterKeys = masterCurve->GetKeys();
				const FloatList& curveKeys = curve->GetKeys();
				size_t keyCount = masterKeys.size();
				mergeCurves &= curveKeys.size() == keyCount && curve->GetPostInfinity() == masterCurve->GetPostInfinity() && curve->GetPreInfinity() == masterCurve->GetPreInfinity();
				for (size_t k = 0; k < keyCount && mergeCurves; ++k)
				{
					mergeCurves = curveKeys[k] == masterKeys[k];
				}
			}
		}
	}

	if (mergeCurves && realCurveCount > 1)
	{
		// HACK: Will need to merge the channel and animated classes.
		FloatList defaultValues(curves.size(), 0.0f);

		// Merge and export the curves
		FCDAnimationMultiCurve* multiCurve = FCDAnimationMultiCurve::MergeCurves(curves, defaultValues);
		multiCurve->WriteSourceToXML(parentNode, baseId);
		multiCurve->WriteSamplerToXML(parentNode, baseId);
		multiCurve->WriteChannelToXML(parentNode, baseId, targetPointer);
		SAFE_DELETE(multiCurve);
	}
	else
	{
		// Interlace the curve's sources, samplers and channels
		// Generate new ids for each of the curve's data sources, to avoid collision in special cases
		size_t curveCount = curves.size();
		StringList ids; ids.resize(curves.size());
		FUSStringBuilder curveId;
		for (size_t c = 0; c < curveCount; ++c)
		{
			if (curves[c] != NULL)
			{
				// Generate a valid id for this curve
				curveId.set(baseId);
				if (curves[c]->GetTargetElement() >= 0)
				{
					curveId.append('_'); curveId.append(curves[c]->GetTargetElement()); curveId.append('_');
				}
				curveId.append(curves[c]->GetTargetQualifier());
				ids[c] = CleanId(curveId.ToCharPtr());

				// Write out the curve's sources
				curves[c]->WriteSourceToXML(parentNode, ids[c]);
			}
		}
		for (size_t c = 0; c < curveCount; ++c)
		{
			if (curves[c] != NULL) curves[c]->WriteSamplerToXML(parentNode, ids[c]);
		}
		for (size_t c = 0; c < curveCount; ++c)
		{
			if (curves[c] != NULL) curves[c]->WriteChannelToXML(parentNode, ids[c], targetPointer.c_str());
		}
	}
}
