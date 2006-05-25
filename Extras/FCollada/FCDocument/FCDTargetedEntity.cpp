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
#include "FCDocument/FCDTargetedEntity.h"
#include "FCDocument/FCDSceneNode.h"

FCDTargetedEntity::FCDTargetedEntity(FCDocument* document, const char* className) : FCDEntity(document, className)
{
	targetNode = NULL;
}

FCDTargetedEntity::~FCDTargetedEntity()
{
	targetNode = NULL;
}

// Sets a new target
void FCDTargetedEntity::SetTargetNode(FCDSceneNode* target)
{
	if (targetNode != NULL)
	{
		targetNode->DecrementTargetCount();
	}

	targetNode = target;

	if (targetNode != NULL)
	{
		targetNode->IncrementTargetCount();
	}
}

// Link this camera with its target, if there is one
FUStatus FCDTargetedEntity::LinkTarget(FCDSceneNode* sceneRoot)
{
	FUStatus status;
	if (targetId.empty()) return status;

	if (targetId[0] == '#') targetId = targetId.substr(1);

	if (sceneRoot == NULL)
	{
		return status.Fail(FS("Invalid parameter in call to FCDTargetEntity::LinkTarget."));
	}
	FCDSceneNode* target = (FCDSceneNode*) sceneRoot->FindDaeId(targetId);
	if (target == NULL)
	{
		return status.Fail(FS("Unable to find target scene node for object: ") + TO_FSTRING(GetDaeId()));
	}
	SetTargetNode(target);

	return status;
}
