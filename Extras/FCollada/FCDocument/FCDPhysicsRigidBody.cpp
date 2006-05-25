/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDPhysicsRigidBody.h"
#include "FCDocument/FCDPhysicsMaterial.h"
#include "FCDocument/FCDocument.h"
#include "FUtils/FUStringConversion.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDPhysicsRigidBody::FCDPhysicsRigidBody(FCDocument* document) : FCDEntity(document, "RigidBody")
{
	physicsMaterial = NULL;
	SetMaterialOwnership(false);
}

FCDPhysicsRigidBody::~FCDPhysicsRigidBody()
{
	CLEAR_POINTER_VECTOR(parameters);
	CLEAR_POINTER_VECTOR(physicsShape);

	if(ownsPhysicsMaterial)
	{
		SAFE_DELETE(physicsMaterial);
	}
	else
	{
		physicsMaterial = NULL;
	}
}

// Create a copy of this physicsRigidBody
FCDPhysicsRigidBody* FCDPhysicsRigidBody::Clone()
{
	FCDPhysicsRigidBody* clone = new FCDPhysicsRigidBody(GetDocument());
	FCDEntity::Clone(clone);

	clone->SetParameters(parameters);
	clone->SetPhysicsShapes(physicsShape);
	clone->SetPhysicsMaterial(physicsMaterial);

	return clone;
}

void FCDPhysicsRigidBody::SetParameters(FCDPhysicsParameterList& params)
{
	for (size_t i = 0; i < params.size(); ++i)
	{
		CopyParameter(params[i]);
	}
}


void FCDPhysicsRigidBody::CopyParameter(FCDPhysicsParameterGeneric* parameter)
{
	FCDPhysicsParameterGeneric* p = parameter->Clone();
	parameters.push_back(p);
}

void FCDPhysicsRigidBody::AddParameter(FCDPhysicsParameterGeneric* parameter)
{
	parameters.push_back(parameter);
}

void FCDPhysicsRigidBody::SetPhysicsMaterial(FCDPhysicsMaterial* _physicsMaterial)
{
	if(physicsMaterial && ownsPhysicsMaterial)
		SAFE_DELETE(physicsMaterial);

	physicsMaterial = _physicsMaterial;
	SetMaterialOwnership(false);
}

void FCDPhysicsRigidBody::SetPhysicsShapes(FCDPhysicsShapeList& _physicsShape)
{
	physicsShape.clear();

	for(FCDPhysicsShapeList::iterator it = _physicsShape.begin(); it != _physicsShape.end(); it++)
		physicsShape.push_back((*it)->Clone());
}


// Load from a XML node the given physicsRigidBody
//FIXME: Default values not assigned if child elements not found
FUStatus FCDPhysicsRigidBody::LoadFromXML(xmlNode* physicsRigidBodyNode)
{
	FUStatus status = FCDEntity::LoadFromXML(physicsRigidBodyNode);
	if (!status) return status;
	if (!IsEquivalent(physicsRigidBodyNode->name, DAE_RIGID_BODY_ELEMENT)) 
	{
		return status.Warning(FS("PhysicsRigidBody library contains unknown element."), physicsRigidBodyNode->line);
	}

	sid = FUDaeParser::ReadNodeSid(physicsRigidBodyNode);

	xmlNode* techniqueNode = FindChildByType(physicsRigidBodyNode, DAE_TECHNIQUE_COMMON_ELEMENT);
	if (techniqueNode != NULL)
	{
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

		xmlNodeList shapeNodes;
		FindChildrenByType(techniqueNode, DAE_SHAPE_ELEMENT, shapeNodes);
		for(xmlNodeList::iterator itS = shapeNodes.begin(); itS != shapeNodes.end(); ++itS)
		{
			FCDPhysicsShape* shape = new FCDPhysicsShape(GetDocument());
			status.AppendStatus(shape->LoadFromXML(*itS));
			physicsShape.push_back(shape);
		}

		param = FindChildByType(techniqueNode, DAE_PHYSICS_MATERIAL_ELEMENT);
		if(param) 
		{
			if(physicsMaterial && ownsPhysicsMaterial)
				SAFE_DELETE(physicsMaterial);
			physicsMaterial = new FCDPhysicsMaterial(GetDocument());
			physicsMaterial->LoadFromXML(param);
			SetMaterialOwnership(true);
		}
		else
		{
			param = FindChildByType(techniqueNode, DAE_INSTANCE_PHYSICS_MATERIAL_ELEMENT);
			if (param == NULL)
			{
				return status.Fail(FS("Error: No physics material defined in rigid body."), techniqueNode->line);
			}
			FUUri url = ReadNodeUrl(param);
			if (url.prefix.empty())
			{
				physicsMaterial = GetDocument()->FindPhysicsMaterial(url.suffix);
				if(!physicsMaterial)
				{
					return status.Fail(FS("Error: Instantiated physics material in rigid body was not found."), techniqueNode->line);
				}
			}
		}
	}

	return status;
}



// Flatten the rigid body: remove all the modifier parameters from the parameter list, permanently modifying their base parameter
void FCDPhysicsRigidBody::Flatten()
{
	vector<FCDPhysicsParameterList::iterator> toDelete;
	for (FCDPhysicsParameterList::iterator itP = parameters.begin(); itP != parameters.end(); ++itP)
	{
		FCDPhysicsParameterList generators;
		if ((*itP)->IsModifier())
		{
			// Overwrite the generators
			FCDPhysicsParameterGeneric* generator = FindParameterByReference((*itP)->GetReference());
			if(generator)
			{
				(*itP)->Overwrite(generator);
				toDelete.push_back(itP);
			}
			else
			{
				(*itP)->SetGenerator(true);
			}

		}
	}

	while(!toDelete.empty())
	{
		parameters.erase(*toDelete.begin());
		SAFE_DELETE(**(toDelete.begin()));
		toDelete.erase(toDelete.begin());
	}

}


FCDPhysicsParameterGeneric* FCDPhysicsRigidBody::FindParameterByReference(const string& reference)
{
	for (FCDPhysicsParameterList::iterator it = parameters.begin(); it != parameters.end(); ++it)
	{
		if ((*it)->GetReference() == reference) return (*it);
	}
	return NULL;
}


// Write out the <physicsRigidBody> node
xmlNode* FCDPhysicsRigidBody::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* physicsRigidBodyNode = WriteToEntityXML(parentNode, DAE_RIGID_BODY_ELEMENT);
	xmlNode* baseNode = AddChild(physicsRigidBodyNode, DAE_TECHNIQUE_COMMON_ELEMENT);

	if(physicsMaterial)
		physicsMaterial->WriteToXML(baseNode);

	for(FCDPhysicsShapeList::const_iterator it = physicsShape.begin(); it != physicsShape.end(); ++it)
	{
		(*it)->WriteToXML(baseNode);
	}
	for(FCDPhysicsParameterList::const_iterator it = parameters.begin(); it != parameters.end(); ++it)
	{
		(*it)->WriteToXML(baseNode);
	}

	FCDEntity::WriteToExtraXML(physicsRigidBodyNode);
	return physicsRigidBodyNode;
}
