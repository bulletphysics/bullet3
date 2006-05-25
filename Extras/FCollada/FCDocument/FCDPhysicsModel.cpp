/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDPhysicsModel.h"
#include "FCDocument/FCDPhysicsRigidBody.h"
#include "FCDocument/FCDPhysicsRigidConstraint.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDAnimated.h"
#include "FUtils/FUStringConversion.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDPhysicsModel::FCDPhysicsModel(FCDocument* document) : FCDEntity(document, "PhysicsModel")
{
}

FCDPhysicsModel::~FCDPhysicsModel()
{
	CLEAR_POINTER_VECTOR(instances);
	CLEAR_POINTER_VECTOR(rigidBodies);
	CLEAR_POINTER_VECTOR(rigidConstraints);
}

// Create a copy of this physicsModel, with the vertices overwritten
FCDPhysicsModel* FCDPhysicsModel::Clone(/*FloatList& newPositions, uint32 newPositionsStride, FloatList& newNormals, uint32 newNormalsStride*/)
{
	FCDPhysicsModel* clone = new FCDPhysicsModel(GetDocument());
	return clone;
}

FCDPhysicsRigidBody* FCDPhysicsModel::FindRigidBodyFromSid(const string& sid)
{
	for(FCDPhysicsRigidBodyList::iterator it = rigidBodies.begin(); it!= rigidBodies.end(); ++it)
	{
		if((*it)->GetSid()==sid) return (*it);
	}
	return NULL;
}

FCDPhysicsRigidConstraint* FCDPhysicsModel::FindRigidConstraintFromSid(const string& sid)
{
	for(FCDPhysicsRigidConstraintList::iterator it = rigidConstraints.begin(); it!= rigidConstraints.end(); ++it)
	{
		if((*it)->GetSid()==sid) return (*it);
	}
	return NULL;
}


// Load from a XML node the given physicsModel
FUStatus FCDPhysicsModel::LoadFromXML(xmlNode* physicsModelNode)
{
	FUStatus status = FCDEntity::LoadFromXML(physicsModelNode);
	if (!status) return status;
	if (!IsEquivalent(physicsModelNode->name, DAE_PHYSICS_MODEL_ELEMENT))
	{
		return status.Warning(FS("PhysicsModel library contains unknown element."), physicsModelNode->line);
	}

	// Read in the first valid child element found
	for (xmlNode* child = physicsModelNode->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;

		if(IsEquivalent(child->name, DAE_RIGID_BODY_ELEMENT))
		{
			FCDPhysicsRigidBody* rigidBody = new FCDPhysicsRigidBody(GetDocument());
			status.AppendStatus(rigidBody->LoadFromXML(child));
			rigidBodies.push_back(rigidBody);

		}
		else if(IsEquivalent(child->name, DAE_RIGID_CONSTRAINT_ELEMENT))
		{
			FCDPhysicsRigidConstraint* rigidConstraint = new FCDPhysicsRigidConstraint(GetDocument(), this);
			status.AppendStatus(rigidConstraint->LoadFromXML(child));
			rigidConstraints.push_back(rigidConstraint);
		}
		else if(IsEquivalent(child->name, DAE_INSTANCE_PHYSICS_MODEL_ELEMENT))
		{
			//FIXME: the instantiated physicsModel might not have been parsed yet
			FUUri url = ReadNodeUrl(child);
			if (url.prefix.empty()) 
			{ 
				FCDEntity* entity = GetDocument()->FindPhysicsModel(url.suffix);
				if (entity != NULL) 
				{
					FCDEntityInstance* instance = new FCDEntityInstance(GetDocument(), entity);
					instances.push_back(instance);
					status.AppendStatus(instance->LoadFromXML(child));
				}
				else
				{
					status.Warning(FS("Unable to retrieve instance for scene node: ") + TO_FSTRING(GetDaeId()), child->line);
				}
			}
		}
		else if(IsEquivalent(child->name, DAE_ASSET_ELEMENT))
		{
		}
		else if(IsEquivalent(child->name, DAE_EXTRA_ELEMENT))
		{
		}
	}

	return status;
}

// Write out the <physicsModel> node
xmlNode* FCDPhysicsModel::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* physicsModelNode = WriteToEntityXML(parentNode, DAE_PHYSICS_MODEL_ELEMENT);
	for(FCDEntityInstanceList::const_iterator it = instances.begin(); it != instances.end(); ++it)
	{
		(*it)->WriteToXML(physicsModelNode);
	}
	for(FCDPhysicsRigidBodyList::const_iterator it = rigidBodies.begin(); it != rigidBodies.end(); ++it)
	{
		(*it)->WriteToXML(physicsModelNode);
	}
	for(FCDPhysicsRigidConstraintList::const_iterator it = rigidConstraints.begin(); it != rigidConstraints.end(); ++it)
	{
		(*it)->WriteToXML(physicsModelNode);
	}

	//TODO: Add asset and extra

	FCDEntity::WriteToExtraXML(physicsModelNode);
	return physicsModelNode;
}
