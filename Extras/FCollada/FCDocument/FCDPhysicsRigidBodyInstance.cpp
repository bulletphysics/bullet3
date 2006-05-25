/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDEntity.h"
#include "FCDocument/FCDPhysicsMaterial.h"
#include "FCDocument/FCDPhysicsModel.h"
#include "FCDocument/FCDPhysicsModelInstance.h"
#include "FCDocument/FCDPhysicsRigidBody.h"
#include "FCDocument/FCDPhysicsRigidBodyInstance.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDPhysicsRigidBodyInstance::FCDPhysicsRigidBodyInstance(FCDocument* document, FCDEntity* _parent) : FCDEntityInstance(document, NULL)
{
	if(_parent->GetType() == FCDEntity::PHYSICS_MODEL)
		parent = (FCDPhysicsModel*)_parent;

	rigidBody = NULL;
	physicsMaterial = NULL;
	ownsPhysicsMaterial = false;
	SetClassName("FCDPhysicsRigidBodyInstance");
}

FCDPhysicsRigidBodyInstance::~FCDPhysicsRigidBodyInstance()
{
	parent = NULL;
	rigidBody = NULL;
	CLEAR_POINTER_VECTOR(parameters);

	if(ownsPhysicsMaterial)
	{
		SAFE_DELETE(physicsMaterial);
	}
	else
	{
		physicsMaterial = NULL;
	}
}

void FCDPhysicsRigidBodyInstance::AddParameter(FCDPhysicsParameterGeneric* parameter)
{
	parameters.push_back(parameter);
}


// Load the geometry instance from the COLLADA document
FUStatus FCDPhysicsRigidBodyInstance::LoadFromXML(xmlNode* instanceNode)
{
	FUStatus status = FCDEntityInstance::LoadFromXML(instanceNode);
	if (!status) return status;

	// Check for the expected instantiation node type
	if (!IsEquivalent(instanceNode->name, DAE_INSTANCE_RIGID_BODY_ELEMENT))
	{
		return status.Fail(FS("Unknown element for instantiation of entity: ") + TO_FSTRING(entity->GetDaeId()), instanceNode->line);
	}
	if (parent == NULL)
	{
		return status.Fail(FS("No Physics Model parent for rigid body instantiation"), instanceNode->line);
	}

	
	string targetNodeId = ReadNodeProperty(instanceNode, DAE_TARGET_ATTRIBUTE);
	targetNode = GetDocument()->FindSceneNode(SkipPound(targetNodeId));
	if(!targetNode)
	{
		return status.Fail(FS("Couldn't find target node for instantiation of rigid body"), instanceNode->line);
	}
	string physicsRigidBodySid = ReadNodeProperty(instanceNode, DAE_BODY_ATTRIBUTE);
	entity = rigidBody = parent->FindRigidBodyFromSid(physicsRigidBodySid);
	if(!rigidBody)
	{
		return status.Fail(FS("Couldn't find rigid body for instantiation"), instanceNode->line);
	}


	//Read in the same children as rigid_body + velocity and angular_velocity
	xmlNode* techniqueNode = FindChildByType(instanceNode, DAE_TECHNIQUE_COMMON_ELEMENT);
	xmlNode* param = FindChildByType(techniqueNode, DAE_DYNAMIC_ELEMENT);
	if(param)
	{
		FCDPhysicsParameter<bool>* p = new FCDPhysicsParameter<bool>(GetDocument(), DAE_DYNAMIC_ELEMENT);
		p->SetValue(FUStringConversion::ToBoolean(ReadNodeContentDirect(param)));
		AddParameter(p);
	}

	xmlNode* massFrame;
	massFrame = FindChildByType(techniqueNode, DAE_MASS_FRAME_ELEMENT);
	if(massFrame)
	{
        param = FindChildByType(massFrame, DAE_TRANSLATE_ELEMENT);
		if(param)
		{
			FCDPhysicsParameter<FMVector3>* p = new FCDPhysicsParameter<FMVector3>(GetDocument(), DAE_TRANSLATE_ELEMENT);
			p->SetValue(FUStringConversion::ToPoint(ReadNodeContentDirect(param)));
			AddParameter(p);
		}
		param = FindChildByType(massFrame, DAE_ROTATE_ELEMENT);
		if(param)
		{
			FCDPhysicsParameter<FMVector3>* p = new FCDPhysicsParameter<FMVector3>(GetDocument(), DAE_ROTATE_ELEMENT);
			p->SetValue(FUStringConversion::ToPoint(ReadNodeContentDirect(param)));
			AddParameter(p);
		}
	}
	param = FindChildByType(techniqueNode, DAE_INERTIA_ELEMENT);
	if(param) 
	{
		FCDPhysicsParameter<FMVector3>* p = new FCDPhysicsParameter<FMVector3>(GetDocument(), DAE_INERTIA_ELEMENT);
		p->SetValue(FUStringConversion::ToPoint(ReadNodeContentDirect(param)));
		AddParameter(p);
	}

	param = FindChildByType(techniqueNode, DAE_MASS_ELEMENT);
	if(param)
	{
		FCDPhysicsParameter<float>* p = new FCDPhysicsParameter<float>(GetDocument(), DAE_MASS_ELEMENT);
		p->SetValue(FUStringConversion::ToFloat(ReadNodeContentDirect(param)));
		AddParameter(p);
	}

	param = FindChildByType(techniqueNode, DAE_PHYSICS_MATERIAL_ELEMENT);
	if(param) 
	{
		if(physicsMaterial && ownsPhysicsMaterial)
			SAFE_DELETE(physicsMaterial);
		physicsMaterial = new FCDPhysicsMaterial(GetDocument());
		physicsMaterial->LoadFromXML(param);
		ownsPhysicsMaterial = true;
	}
	else
	{
		param = FindChildByType(techniqueNode, DAE_INSTANCE_PHYSICS_MATERIAL_ELEMENT);
		if(param)
		{
			physicsMaterial = GetDocument()->FindPhysicsMaterial(ReadNodeId(param));
			ownsPhysicsMaterial = false;
		}
	}
	
	param = FindChildByType(techniqueNode, DAE_VELOCITY_ELEMENT);
	if(param)
	{
		FCDPhysicsParameter<FMVector3>* p = new FCDPhysicsParameter<FMVector3>(GetDocument(), DAE_VELOCITY_ELEMENT);
		p->SetValue(FUStringConversion::ToPoint(ReadNodeContentDirect(param)));
		AddParameter(p);
	}
	param = FindChildByType(techniqueNode, DAE_ANGULAR_VELOCITY_ELEMENT);
	if(param)
	{
		FCDPhysicsParameter<FMVector3>* p = new FCDPhysicsParameter<FMVector3>(GetDocument(), DAE_ANGULAR_VELOCITY_ELEMENT);
		p->SetValue(FUStringConversion::ToPoint(ReadNodeContentDirect(param)));
		AddParameter(p);
	}

	return status;
}


FCDPhysicsRigidBody* FCDPhysicsRigidBodyInstance::FlattenRigidBody()
{
	FCDPhysicsRigidBody* clone = rigidBody->Clone();
	clone->Flatten();

	for (FCDPhysicsParameterList::iterator itP = parameters.begin(); itP != parameters.end(); ++itP)
	{
		FCDPhysicsParameterGeneric* param = clone->FindParameterByReference((*itP)->GetReference());
		if(param)
		{
			(*itP)->Overwrite(param);
		}
		else
		{
			clone->CopyParameter(*itP);
		}
	}

	if(physicsMaterial)
		clone->SetPhysicsMaterial(physicsMaterial);

	return clone;
}

// Write out the instantiation information to the xml node tree
xmlNode* FCDPhysicsRigidBodyInstance::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* instanceNode = FCDEntityInstance::WriteToXML(parentNode);

	//TODO

	return instanceNode;
}
