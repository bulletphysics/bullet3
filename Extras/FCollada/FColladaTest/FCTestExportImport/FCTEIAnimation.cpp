/*
	Copyright (C) 2006 Feeling Software Inc.
	Available only to licensees.
	Distribution of this file or its content is strictly prohibited.
*/

#include "StdAfx.h"
#include "FCDocument/FCDAnimated.h"
#include "FCDocument/FCDAnimation.h"
#include "FCDocument/FCDAnimationChannel.h"
#include "FCDocument/FCDAnimationCurve.h"
#include "FCDocument/FCDLight.h"
#include "FCTestExportImport.h"

static string animId1 = "GrossAnimation";
static string animId2 = "GrossAnimation";
static string animatedLightId;
static const fstring animSubTreeNote = FS("TestingSTNote");

namespace FCTestExportImport
{
	void FillAnimationLibrary(FCDAnimationLibrary* library)
	{
		// Create two more tree within the animation library
		FailIf(library == NULL);
		size_t startAnimCount = library->GetEntityCount();
		FCDAnimation* animTree1 = library->AddEntity();
		FCDAnimation* animTree2 = library->AddEntity();
		PassIf(library->GetEntityCount() == startAnimCount + 2);

		// Retrieve the ids of the created entities.
		animTree2->SetDaeId(animId2);
		animTree1->SetDaeId(animId1);
		FailIf(animId1.empty());
		FailIf(animId2.empty());
		FailIf(animId1 == animId2);

		// Add to the first animation tree some sub-trees.
		FCDAnimation* animSubTree1 = animTree1->AddChild();
		FCDAnimation* animSubTree2 = animTree1->AddChild();
		animSubTree1->SetNote(animSubTreeNote);
		animSubTree2->SetNote(animSubTreeNote);
		FailIf(animTree1->GetChildCount() != 2);

		// Animate some selected parameters
		FillAnimationLight(library->GetDocument(), animSubTree2);
	}

	void CheckAnimationLibrary(FCDAnimationLibrary* library)
	{
		FailIf(library == NULL);

		// Retrieve the animation trees using the saved ids.
		FCDAnimation* animTree1 = library->FindDaeId(animId1);
		FCDAnimation* animTree2 = library->FindDaeId(animId2);
		FailIf(animTree1 == NULL);
		FailIf(animTree2 == NULL);

		// Verify that the first animation tree has the correct sub-trees.
		// Retrieve the animation sub-tree which contains our channels.
		FCDAnimation* ourChannels = NULL;
		FailIf(animTree1->GetChildCount() != 2);
		for (size_t i = 0; i < 2; ++i)
		{
			FCDAnimation* subTree = animTree1->GetChild(i);
			FailIf(subTree == NULL);
			PassIf(subTree->GetNote() == animSubTreeNote);
			if (subTree->GetChannelCount() > 0)
			{
				PassIf(ourChannels == NULL);
				ourChannels = subTree;
			}
		}

		PassIf(ourChannels != NULL);
		CheckAnimationLight(library->GetDocument(), ourChannels);
	}

	void FillAnimationLight(FCDocument* document, FCDAnimation* animationTree)
	{
		// Retrieve a light entity and add an animation to its color
		FCDLightLibrary* lightLibrary = document->GetLightLibrary();
		PassIf(lightLibrary != NULL);
		PassIf(lightLibrary->GetEntityCount() > 0);
		FCDLight* light1 = lightLibrary->GetEntity(0);
		animatedLightId = light1->GetDaeId();

		// Create the animated object for the color
		PassIf(document->FindAnimatedValue(&light1->GetColor().x) == NULL);
		FCDAnimatedColor* animated = FCDAnimatedColor::Create(document, &light1->GetColor());
		FailIf(animated == NULL);

		// Create a channel for the animation curves
		FCDAnimationChannel* channel = animationTree->AddChannel();
		FailIf(channel == NULL);

		FCDAnimationCurve* curve = channel->AddCurve();
		animated->SetCurve(0, curve);
	}

	void CheckAnimationLight(FCDocument* document, FCDAnimation* UNUSED(animationTree))
	{
		// Retrieve the light whose color is animated.
		FCDLightLibrary* lightLibrary = document->GetLightLibrary();
		PassIf(lightLibrary != NULL);
		PassIf(lightLibrary->GetEntityCount() > 0);
		FCDLight* light1 = lightLibrary->FindDaeId(animatedLightId);
		PassIf(light1 != NULL);
	}
};