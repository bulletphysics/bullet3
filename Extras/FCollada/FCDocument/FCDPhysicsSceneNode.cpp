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
#include "FCDocument/FCDPhysicsSceneNode.h"
#include "FCDocument/FCDPhysicsModelInstance.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
#include "FUtils/FUFileManager.h"
#include "FUtils/FUStringConversion.h"
#include "FCDocument/FCDExtra.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDPhysicsSceneNode::FCDPhysicsSceneNode(FCDocument* document) : FCDEntity(document, "PhysicsSceneNode")
{
	//FIXME: no default values are specified in the 1.4 spec!
	gravity.x = 0.f; gravity.y = 0.f; gravity.z = -9.8f;
	timestep = 1.f;
}

FCDPhysicsSceneNode::~FCDPhysicsSceneNode()
{
	CLEAR_POINTER_VECTOR(instances);
}

// Parse a <scene> or a <node> node from a COLLADA document
FUStatus FCDPhysicsSceneNode::LoadFromXML(xmlNode* sceneNode)
{
	FUStatus status = FCDEntity::LoadFromXML(sceneNode);
	if (!status) return status;

	if(IsEquivalent(sceneNode->name, DAE_PHYSICS_SCENE_ELEMENT))
	{
		for (xmlNode* child = sceneNode->children; child != NULL; child = child->next)
		{
			if (child->type != XML_ELEMENT_NODE) continue;

			// Look for instantiation elements
			if (IsEquivalent(child->name, DAE_INSTANCE_PHYSICS_MODEL_ELEMENT)) 
			{
				FUUri url = ReadNodeUrl(child); 
				if (url.prefix.empty()) 
				{
					FCDPhysicsModel* entity = GetDocument()->FindPhysicsModel(url.suffix);
					if (entity != NULL) 
					{
						FCDPhysicsModelInstance* instance = new FCDPhysicsModelInstance(GetDocument(), (FCDEntity*)entity);
						instances.push_back(instance);
						status.AppendStatus(instance->LoadFromXML(child));
						continue; 
					}
				}
				status.Warning(FS("Unable to retrieve FCDPhysicsModel instance for scene node: ") + TO_FSTRING(GetDaeId()), child->line); 
			}
			else if(IsEquivalent(child->name, DAE_TECHNIQUE_COMMON_ELEMENT))
			{
				xmlNode* gravityNode = FindChildByType(child, DAE_GRAVITY_ATTRIBUTE);
				if(gravityNode)
				{
					const char* gravityVal = ReadNodeContentDirect(gravityNode);
					gravity.x = FUStringConversion::ToFloat(&gravityVal);
					gravity.y = FUStringConversion::ToFloat(&gravityVal);
					gravity.z = FUStringConversion::ToFloat(&gravityVal);
				}
				xmlNode* timestepNode = FindChildByType(child, DAE_TIME_STEP_ATTRIBUTE);
				if(timestepNode)
				{
					timestep = FUStringConversion::ToFloat(ReadNodeContentDirect(timestepNode));
				}
			}
			else if (IsEquivalent(child->name, DAE_EXTRA_ELEMENT))
			{
				// The extra information is loaded by the FCDEntity class.
			}
		}
	}

	return status;
}

// Write out a <physics_scene> element to a COLLADA xml document
xmlNode* FCDPhysicsSceneNode::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* physicsSceneNode = WriteToEntityXML(parentNode, DAE_PHYSICS_SCENE_ELEMENT);
	if (physicsSceneNode == NULL) return physicsSceneNode;
	WriteToNodeXML(physicsSceneNode);
	return physicsSceneNode;
}

// Write out a <physics_scene>  element to a COLLADA xml document
void FCDPhysicsSceneNode::WriteToNodeXML(xmlNode* node) const
{
	// Write out the instantiation
	for (FCDPhysicsModelInstanceList::const_iterator itI = instances.begin(); itI != instances.end(); ++itI)
	{
		FCDEntityInstance* instance = (*itI);
		instance->WriteToXML(node);
	}

	// Write out the extra information
	FCDEntity::WriteToExtraXML(node);
}
