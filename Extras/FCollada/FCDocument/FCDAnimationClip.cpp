/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDAnimation.h"
#include "FCDocument/FCDAnimationChannel.h"
#include "FCDocument/FCDAnimationClip.h"
#include "FCDocument/FCDAnimationCurve.h"
#include "FCDocument/FCDAnimated.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDAnimationClip::FCDAnimationClip(FCDocument* document) : FCDEntity(document, "AnimationClip")
{
	start = end = 0.0f;
}

FCDAnimationClip::~FCDAnimationClip()
{
	curves.clear();
}

FCDAnimationClip* FCDAnimationClip::Clone()
{
	FCDAnimationClip* clone = new FCDAnimationClip(GetDocument());
	FCDEntity::Clone(clone);

	for(FCDAnimationCurveList::iterator it = curves.begin(); it != curves.end(); ++it)
	{
		curves.push_back((*it)->Clone());
	}
	
	clone->start = start;
	clone->end = end;
	return clone;
}


FUStatus FCDAnimationClip::LoadFromXML(xmlNode* clipNode)
{
	FUStatus status = FCDEntity::LoadFromXML(clipNode);
	if (!status) return status;
	if (!IsEquivalent(clipNode->name, DAE_ANIMCLIP_ELEMENT))
	{
		return status.Warning(FS("Unknown element in animation clip library."), clipNode->line);
	}

	// Read in and verify the clip's time/input bounds
	start = FUStringConversion::ToFloat(ReadNodeProperty(clipNode, DAE_START_ATTRIBUTE));
	end = FUStringConversion::ToFloat(ReadNodeProperty(clipNode, DAE_END_ATTRIBUTE));
	if (end - start < FLT_TOLERANCE)
	{
		status.Warning(FS("Invalid start/end pair for animation clip: ") + TO_FSTRING(GetDaeId()), clipNode->line);
	}

	// Read in the <input> elements and segment the corresponding animation curves
	xmlNodeList inputNodes;
	FindChildrenByType(clipNode, DAE_INSTANCE_ANIMATION_ELEMENT, inputNodes);
	for (xmlNodeList::iterator itI = inputNodes.begin(); itI != inputNodes.end(); ++itI)
	{
		xmlNode* inputNode = (*itI);

		// Retrieve the animation for this input
		FUUri animationId = ReadNodeUrl(inputNode);
		if (animationId.suffix.empty() || !animationId.prefix.empty())
		{
			return status.Fail(FS("Invalid animation instantiation for animation clip: ") + TO_FSTRING(GetDaeId()), inputNode->line);
		}
		FCDAnimation* animation = GetDocument()->FindAnimation(animationId.suffix);
		if (animation == NULL) continue;

		// Retrieve all the curves created under this animation node
		FCDAnimationCurveList animationCurves;
		animation->GetCurves(animationCurves);
		if (animationCurves.empty())
		{
			status.Warning(FS("No curves instantiated by animation '") + TO_FSTRING(animationId.suffix) + FS("' for animation clip: ") + TO_FSTRING(GetDaeId()), inputNode->line);
		}
        
		for (FCDAnimationCurveList::iterator itC = animationCurves.begin(); itC != animationCurves.end(); ++itC)
		{
			// Keep only newly listed curves
			FCDAnimationCurve* curve = *itC;
			FCDAnimationCurveList::iterator itF = std::find(curves.begin(), curves.end(), curve);
			if (itF == curves.end()) continue; 

			curve->RegisterAnimationClip(this);
			curves.push_back(curve);
		}
	}

	// Check for an empty clip
	if (curves.empty())
	{
		status.Warning(FS("Empty animation clip :") + TO_FSTRING(GetDaeId()), clipNode->line);
	}

	return status;
}

// Write out the COLLADA animations to the document
xmlNode* FCDAnimationClip::WriteToXML(xmlNode* parentNode) const
{
	// Create the <clip> element and write out its start/end information.
	xmlNode* clipNode = FCDEntity::WriteToEntityXML(parentNode, DAE_ANIMCLIP_ELEMENT);
	AddAttribute(clipNode, DAE_START_ATTRIBUTE, start);
	AddAttribute(clipNode, DAE_END_ATTRIBUTE, end);

	// Build a list of the animations to instantiate
	// from the list of curves for this clip
	typedef vector<const FCDAnimation*> FCDAnimationConstList;
	FCDAnimationConstList animations;
	for (FCDAnimationCurveList::const_iterator itC = curves.begin(); itC != curves.end(); ++itC)
	{
		const FCDAnimationChannel* channel = (*itC)->GetParent();
		if (channel == NULL) continue;
		const FCDAnimation* animation = channel->GetParent();
		if (std::find(animations.begin(), animations.end(), animation) == animations.end())
		{
			animations.push_back(animation);
		}
	}

	// Instantiate all the animations
	for (FCDAnimationConstList::iterator itA = animations.begin(); itA != animations.end(); ++itA)
	{
		xmlNode* instanceNode = AddChild(clipNode, DAE_INSTANCE_ANIMATION_ELEMENT);
		AddAttribute(instanceNode, DAE_URL_ATTRIBUTE, string("#") + (*itA)->GetDaeId());
	}

	FCDEntity::WriteToExtraXML(clipNode);
	return clipNode;
}

