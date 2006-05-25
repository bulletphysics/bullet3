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
#include "FCDocument/FCDController.h"
#include "FCDocument/FCDGeometry.h"
#include "FCDocument/FCDGeometryMesh.h"
#include "FCDocument/FCDGeometryPolygons.h"
#include "FCDocument/FCDGeometrySource.h"
#include "FCDocument/FCDGeometrySpline.h"
#include "FCDocument/FCDMorphController.h"
#include "FCDocument/FCDSceneNode.h"
#include "FUtils/FUStringConversion.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDMorphController::FCDMorphController(FCDocument* document, FCDController* _parent) : FCDObject(document, "FCDMorphController")
{
	parent = _parent;
	baseTarget = NULL;
}

FCDMorphController::~FCDMorphController()
{
	baseTarget = NULL;
	parent = NULL;

	CLEAR_POINTER_VECTOR(morphTargets);
}

// Changes the base target of the morpher
void FCDMorphController::SetBaseTarget(FCDEntity* entity)
{
	baseTarget = NULL;

	// Retrieve the actual base entity, as you can chain controllers.
	FCDEntity* baseEntity = entity;
	while (baseEntity != NULL && baseEntity->GetType() == FCDEntity::CONTROLLER)
	{
		baseEntity = ((FCDController*) baseEntity)->GetBaseTarget();
	}
	if (baseEntity != NULL && baseEntity->GetType() == FCDEntity::GEOMETRY)
	{
		baseTarget = entity;

		// Remove the old morph targets which are not similar, anymore, to the new base entity.
		for (size_t i = 0; i < morphTargets.size();)
		{
			if (IsSimilar(morphTargets[i]->GetGeometry()))
			{
				++i;
			}
			else
			{
				ReleaseTarget(morphTargets[i]);
			}
		}
	}
	else
	{
		// The new base target is not valid.
		CLEAR_POINTER_VECTOR(morphTargets);
	}
}

// Adds a new morph target.
FCDMorphTarget* FCDMorphController::AddTarget(FCDGeometry* geometry, float weight)
{
	FCDMorphTarget* target = NULL;
	if (IsSimilar(geometry))
	{
		target = new FCDMorphTarget(GetDocument(), this);
		target->SetGeometry(geometry);
		target->SetWeight(weight);
		morphTargets.push_back(target);
	}
	return target;
}

// Releases a morph target used in this morpher.
void FCDMorphController::ReleaseTarget(FCDMorphTarget* target)
{
	FCDMorphTargetList::iterator it = std::find(morphTargets.begin(), morphTargets.end(), target);
	if (it != morphTargets.end())
	{
		delete *it;
		morphTargets.erase(it);
	}
}

// Retrieves whether a given entity is similar to the base target.
bool FCDMorphController::IsSimilar(FCDEntity* entity)
{
	bool similar = false;
	if (entity != NULL && baseTarget != NULL)
	{
		size_t vertexCount = 0;
		bool isMesh = false;
		bool isSpline = false;

		// Find the number of vertices in the base target
		FCDEntity* baseEntity = baseTarget;
		while (baseEntity != NULL && baseEntity->GetType() == FCDEntity::CONTROLLER)
		{
			baseEntity = ((FCDController*) baseEntity)->GetBaseTarget();
		}
		if (baseEntity != NULL && baseEntity->GetType() == FCDEntity::GEOMETRY)
		{
			FCDGeometry* g = (FCDGeometry*) baseEntity;
			if (g->IsMesh())
			{
				isMesh = true;
				FCDGeometryMesh* m = g->GetMesh();
				FCDGeometrySource* positions = m->GetPositionSource();
				if (positions != NULL)
				{
					vertexCount = positions->GetSourceData().size() / positions->GetSourceStride();
				}
			}

			if (g->IsSpline())
			{
				isSpline = true;
				FCDGeometrySpline* s = g->GetSpline();
				vertexCount = s->GetCVCount();
			}
		}


		// Find the number of vertices in the given entity
		baseEntity = entity;
		while (baseEntity != NULL && baseEntity->GetType() == FCDEntity::CONTROLLER)
		{
			baseEntity = ((FCDController*) baseEntity)->GetBaseTarget();
		}
		if (baseEntity != NULL && baseEntity->GetType() == FCDEntity::GEOMETRY)
		{
			FCDGeometry* g = (FCDGeometry*) baseEntity;
			if (g->IsMesh() && isMesh)
			{
				FCDGeometryMesh* m = g->GetMesh();
				FCDGeometrySource* positions = m->GetPositionSource();
				if (positions != NULL)
				{
					similar = (vertexCount == positions->GetSourceData().size() / positions->GetSourceStride());
				}
			}

			if (g->IsSpline() && isSpline)
			{
				FCDGeometrySpline* s = g->GetSpline();
				similar = (vertexCount == s->GetCVCount());
			}
		}
	}

	return similar;
}

// Load this controller from a Collada <controller> node
FUStatus FCDMorphController::LoadFromXML(xmlNode* morphNode)
{
	FUStatus status;
	if (!IsEquivalent(morphNode->name, DAE_CONTROLLER_MORPH_ELEMENT))
	{
		return status.Warning(FS("Unexpected node in controller library: ") + TO_FSTRING((const char*) morphNode->name), morphNode->line);
	}

	// Parse in the morph method
	string methodValue = ReadNodeProperty(morphNode, DAE_METHOD_ATTRIBUTE);
	method = FUDaeMorphMethod::FromString(methodValue);
	if (method == FUDaeMorphMethod::UNKNOWN)
	{
		status.Warning(FS("Unknown processing method from morph controller: ") + TO_FSTRING(parent->GetDaeId()), morphNode->line);
	}

	// Find the base geometry
	string baseTargetId = ReadNodeSource(morphNode);
	baseTarget = GetDocument()->FindGeometry(baseTargetId);
	if (baseTarget == NULL) GetDocument()->FindController(baseTargetId);
	if (baseTarget == NULL)
	{
		return status.Warning(FS("Cannot find base target for morph controller: ") + TO_FSTRING(parent->GetDaeId()), morphNode->line);
	}

	// Find the <targets> element and process its inputs
	xmlNode* targetsNode = FindChildByType(morphNode, DAE_TARGETS_ELEMENT);
	if (targetsNode == NULL)
	{
		return status.Fail(FS("Cannot find necessary <targets> element for morph controller: ") + TO_FSTRING(parent->GetDaeId()), morphNode->line);
	}
	xmlNodeList inputNodes;
	FindChildrenByType(targetsNode, DAE_INPUT_ELEMENT, inputNodes);

	// Find the TARGET and WEIGHT input necessary sources
	xmlNode* targetSourceNode = NULL,* weightSourceNode = NULL;
	for (xmlNodeList::iterator it = inputNodes.begin(); it != inputNodes.end(); ++it)
	{
		xmlNode* inputNode = (*it);
		string semantic = ReadNodeSemantic(inputNode);
		string sourceId = ReadNodeSource(inputNode);
		if (semantic == DAE_WEIGHT_MORPH_INPUT || semantic == DAE_WEIGHT_MORPH_INPUT_DEPRECATED)
		{
			weightSourceNode = FindChildById(morphNode, sourceId);
		}
		else if (semantic == DAE_TARGET_MORPH_INPUT || semantic == DAE_TARGET_MORPH_INPUT_DEPRECATED)
		{
			targetSourceNode = FindChildById(morphNode, sourceId);
		}
		else
		{
			status.Warning(FS("Unknown morph targets input type in morph controller: ") + TO_FSTRING(parent->GetDaeId()), inputNode->line);
		}
	}
	if (targetSourceNode == NULL)
	{
		return status.Fail(FS("Cannot find TARGET source for morph controller: ") + TO_FSTRING(parent->GetDaeId()), targetsNode->line);
	}
	if (weightSourceNode == NULL)
	{
		return status.Fail(FS("Cannot find WEIGHT source for morph controller: ") + TO_FSTRING(parent->GetDaeId()), targetsNode->line);
	}

	// Read in the sources
	StringList morphTargetIds;
	ReadSource(targetSourceNode, morphTargetIds);
	FloatList weights;
	ReadSource(weightSourceNode, weights);
	size_t targetCount = morphTargetIds.size();
	if (weights.size() != targetCount)
	{
		return status.Fail(FS("TARGET and WEIGHT sources should be the same size for morph controller: ") + TO_FSTRING(parent->GetDaeId()), targetSourceNode->line);
	}

	// Find the target geometries and build the morph targets
	morphTargets.reserve(targetCount);
	for (int32 i = 0; i < (int32) targetCount; ++i)
	{
		FCDGeometry* targetGeometry = GetDocument()->FindGeometry(morphTargetIds[i]);
		if (targetGeometry == NULL)
		{
			status.Warning(FS("Unable to find target geometry, '") + TO_FSTRING(morphTargetIds[i]) + FS("' for morph controller: ") + TO_FSTRING(parent->GetDaeId()), morphNode->line);
		}
		FCDMorphTarget* morphTarget = AddTarget(targetGeometry, weights[i]);

		// Record the morphing weight as animatable
		FCDAnimatedFloat::Create(GetDocument(), weightSourceNode, &morphTarget->GetWeight(), i);
	}

	return status;
}

// Write out this controller to a COLLADA xml node tree
xmlNode* FCDMorphController::WriteToXML(xmlNode* parentNode) const
{
	size_t targetCount = GetTargetCount();

	// Create the <morph> node and set its attributes
	xmlNode* morphNode = AddChild(parentNode, DAE_CONTROLLER_MORPH_ELEMENT);
	AddAttribute(morphNode, DAE_METHOD_ATTRIBUTE, FUDaeMorphMethod::ToString(method));
	if (baseTarget != NULL)
	{
		AddAttribute(morphNode, DAE_SOURCE_ATTRIBUTE, string("#") + baseTarget->GetDaeId());
	}

	// Gather up the morph target ids and the morphing weights
	StringList targetIds; targetIds.reserve(targetCount);
	FloatList weights; weights.reserve(targetCount);
	for (FCDMorphTargetList::const_iterator it = morphTargets.begin(); it != morphTargets.end(); ++it)
	{
		FCDMorphTarget* t = (*it);
		targetIds.push_back(t->GetGeometry() != NULL ? t->GetGeometry()->GetDaeId() : DAEERR_UNKNOWN_IDREF);
		weights.push_back(t->GetWeight());
	}

	// Export the target id source
	FUSStringBuilder targetSourceId(parent->GetDaeId()); targetSourceId.append("-targets");
	AddSourceIDRef(morphNode, targetSourceId.ToCharPtr(), targetIds, DAE_TARGET_MORPH_INPUT);

	// Export the weight source
	FUSStringBuilder weightSourceId(parent->GetDaeId()); weightSourceId.append("-morph_weights");
	xmlNode* weightSourceNode = AddSourceFloat(morphNode, weightSourceId.ToCharPtr(), weights, DAE_WEIGHT_MORPH_INPUT);

	// Export the <targets> elements
	xmlNode* targetsNode = AddChild(morphNode, DAE_TARGETS_ELEMENT);
	AddInput(targetsNode, targetSourceId.ToCharPtr(), DAE_TARGET_MORPH_INPUT);
	AddInput(targetsNode, weightSourceId.ToCharPtr(), DAE_WEIGHT_MORPH_INPUT);

	// Record the morphing weight animations
	for (int32 i = 0; i < (int32) targetCount; ++i)
	{
		FCDMorphTarget* t = morphTargets[i];
		GetDocument()->WriteAnimatedValueToXML(&t->GetWeight(), weightSourceNode, "morphing_weights", i);
	}

	return morphNode;
}

FUStatus FCDMorphController::Link()
{
	return FUStatus(1);
}

// Morph Target Class Implementation
FCDMorphTarget::FCDMorphTarget(FCDocument* document, FCDMorphController* _parent) : FCDObject(document, "FCDMorphTarget")
{
	parent = _parent;
	geometry = NULL;
	weight = 0.0f;
}

FCDMorphTarget::~FCDMorphTarget()
{
	parent = NULL;
	geometry = NULL;
	weight = 0.0f;
}

void FCDMorphTarget::SetGeometry(FCDGeometry* _geometry)
{
	// Let go of the old geometry
	geometry = NULL;

	// Check if this geometry is similar to the controller base target
	if (GetParent()->IsSimilar(_geometry))
	{
		geometry = _geometry;
	}
}

FCDAnimated* FCDMorphTarget::GetAnimatedWeight()
{
	return GetDocument()->FindAnimatedValue(&weight);
}
const FCDAnimated* FCDMorphTarget::GetAnimatedWeight() const
{
	return GetDocument()->FindAnimatedValue(&weight);
}

bool FCDMorphTarget::IsAnimated() const
{
	return GetDocument()->IsValueAnimated(&weight);
}
