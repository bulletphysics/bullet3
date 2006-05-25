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
#include "FCDocument/FCDAnimationCurve.h"
#include "FCDocument/FCDCamera.h"
#include "FCDocument/FCDController.h"
#include "FCDocument/FCDEntityInstance.h"
#include "FCDocument/FCDExternalReference.h"
#include "FCDocument/FCDExtra.h"
#include "FCDocument/FCDGeometry.h"
#include "FCDocument/FCDGeometryInstance.h"
#include "FCDocument/FCDLight.h"
#include "FCDocument/FCDPhysicsModelInstance.h"
#include "FCDocument/FCDPhysicsRigidBodyInstance.h"
#include "FCDocument/FCDSceneNode.h"
#include "FCDocument/FCDTransform.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
#include "FUtils/FUFileManager.h"
#include "FUtils/FUStringConversion.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDSceneNode::FCDSceneNode(FCDocument* document) : FCDEntity(document, "VisualSceneNode")
{
	isJoint = false;
	targetCount = 0;
	visibility = 1.0f;
}

FCDSceneNode::~FCDSceneNode()
{
	CLEAR_POINTER_VECTOR(instances);
	CLEAR_POINTER_VECTOR(transforms);
	parents.clear();

	// Delete the children, be watchful for the instantiated node
	for (FCDSceneNodeList::iterator it = children.begin(); it != children.end(); ++it)
	{
		FCDSceneNode* child = (*it);
		for (FCDSceneNodeList::iterator itParent = child->parents.begin(); itParent != child->parents.end(); ++itParent)
		{
			if ((*itParent) == this) 
			{
				child->parents.erase(itParent);
				break;
			}
		}

		if (child->parents.empty()) delete child;
	}
	children.clear();
}

// Add this scene node to the list of children scene node
bool FCDSceneNode::AddChildNode(FCDSceneNode* sceneNode)
{
	if (this == sceneNode || sceneNode == NULL)
	{
		return false;
	}

	// Verify that we don't already contain this child node.
	FCDSceneNodeList::iterator it = std::find(children.begin(), children.end(), sceneNode);
	if (it != children.end()) return false;

	// Verify that this node is not one of the parents in the full hierarchically.
	FCDSceneNodeList queue = parents;
	while (!queue.empty())
	{
		FCDSceneNode* parent = queue.back();
		queue.pop_back();
		if (parent == sceneNode) return false;
		queue.insert(queue.end(), parent->parents.begin(), parent->parents.end());
	}

	children.push_back(sceneNode);
	sceneNode->parents.push_back(this);
	return true;
}

FCDSceneNode* FCDSceneNode::AddChildNode()
{
	FCDSceneNode* node = new FCDSceneNode(GetDocument());
	AddChildNode(node);
	return node;
}

FCDTransform* FCDSceneNode::AddTransform(FCDTransform::Type type, size_t index)
{
	FCDTransform* transform = FCDTFactory::CreateTransform(GetDocument(), this, type);
	if (transform != NULL)
	{
		if (index > transforms.size()) transforms.push_back(transform);
		else transforms.insert(transforms.begin() + index, transform);
	}
	return transform;
}

// Traverse the scene graph, searching for a node with the given COLLADA id
FCDEntity* FCDSceneNode::FindDaeId(const string& daeId)
{
	if (GetDaeId() == daeId) return this;
	
	for (FCDSceneNodeList::iterator it = children.begin(); it != children.end(); ++it)
	{
		FCDEntity* found = (*it)->FindDaeId(daeId);
		if (found != NULL) return found;
	}
	return NULL;
}

// Calculate the transform matrix for a given scene node
FMMatrix44 FCDSceneNode::ToMatrix() const
{
	FMMatrix44 localTransform = FMMatrix44::Identity;
	for (FCDTransformList::const_iterator it = transforms.begin(); it != transforms.end(); ++it)
	{
		localTransform = localTransform * (*it)->ToMatrix();
	}
	return localTransform;
}

void FCDSceneNode::GenerateSampledMatrixAnimation(FloatList& sampleKeys, FMMatrix44List& sampleValues)
{
	FCDAnimatedList animateds;
	
	// Collect all the animation curves
	for (FCDTransformList::iterator it = transforms.begin(); it != transforms.end(); ++it)
	{
		FCDAnimated* animated = (*it)->GetAnimated();
		if (animated != NULL && animated->HasCurve()) animateds.push_back(animated);
	}
	if (animateds.empty()) return;

	// Make a list of the ordered key times to sample
	for (FCDAnimatedList::iterator it = animateds.begin(); it != animateds.end(); ++it)
	{
		const FCDAnimationCurveList& curves = (*it)->GetCurves();
		for (FCDAnimationCurveList::const_iterator curveIt = curves.begin(); curveIt != curves.end(); ++curveIt)
		{
			if ((*curveIt) == NULL) continue;

			const FloatList& curveKeys = (*curveIt)->GetKeys();
			size_t sampleKeyCount = sampleKeys.size();
			size_t curveKeyCount = curveKeys.size();
			
			// Merge this curve's keys in with the sample keys
			// This assumes both key lists are in increasing order
			size_t s = 0, c = 0;
			while (s < sampleKeyCount && c < curveKeyCount)
			{
				float sampleKey = sampleKeys[s], curveKey = curveKeys[c];
				if (IsEquivalent(sampleKey, curveKey)) { ++s; ++c; }
				else if (sampleKey < curveKey) { ++s; }
				else
				{
					// Add this curve key to the sampling key list
					sampleKeys.insert(sampleKeys.begin() + (s++), curveKeys[c++]);
					sampleKeyCount++;
				}
			}

			// Add all the left-over curve keys to the sampling key list
			if (c < curveKeyCount) sampleKeys.insert(sampleKeys.end(), curveKeys.begin() + c, curveKeys.end());
		}
	}
	size_t sampleKeyCount = sampleKeys.size();
	if (sampleKeyCount == 0) return;

	// Pre-allocate the value array;
	sampleValues.reserve(sampleKeyCount);
	
	// Sample the scene node transform
	for (size_t i = 0; i < sampleKeyCount; ++i)
	{
		float sampleTime = sampleKeys[i];
		for (FCDAnimatedList::iterator it = animateds.begin(); it != animateds.end(); ++it)
		{
			// Sample each animated, which changes the transform values directly
			(*it)->Evaluate(sampleTime);
		}

		// Retrieve the new transform matrix for the COLLADA scene node
		sampleValues.push_back(ToMatrix());
	}
}

// Parse a <scene> or a <node> node from a COLLADA document
FUStatus FCDSceneNode::LoadFromXML(xmlNode* sceneNode)
{
	FUStatus status = FCDEntity::LoadFromXML(sceneNode);
	if (!status) return status;
	if (!IsEquivalent(sceneNode->name, DAE_VSCENE_ELEMENT) && !IsEquivalent(sceneNode->name, DAE_NODE_ELEMENT)
		&& !IsEquivalent(sceneNode->name, DAE_SCENE_ELEMENT))
	{
		// The <scene> element is accepted here only as COLLADA 1.3 backward compatibility
		return status.Fail(FS("Unknown scene node element with id: ") + TO_FSTRING(GetDaeId()), sceneNode->line);
	}

	// look up the visual scene to extrace bind info
	if (IsEquivalent(sceneNode->name, DAE_VSCENE_ELEMENT)) BindMaterial(sceneNode);

	// Read in the <node> element's type
	string nodeType = ReadNodeProperty(sceneNode, DAE_TYPE_ATTRIBUTE);
	if (nodeType == DAE_JOINT_NODE_TYPE) SetJointFlag(true);
	else if (nodeType.length() == 0 || nodeType == DAE_NODE_NODE_TYPE) {} // No special consideration
	else
	{
		status.Warning(FS("Unknown node type for scene's <node> element: ") + TO_FSTRING(GetDaeId()), sceneNode->line);
	}

	// The scene node has ordered elements, so process them directly and in order.
	for (xmlNode* child = sceneNode->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;

		if (IsEquivalent(child->name, DAE_NODE_ELEMENT))
		{
			// Load the child scene node
			FCDSceneNode* node = AddChildNode();
			status = node->LoadFromXML(child);
			if (!status) break;
		}

		// Look for instantiation elements
#		define INSTANTIATE(name, instanceType) { \
			FUUri url = ReadNodeUrl(child); \
			if (url.prefix.empty()) { \
				FCD##name* entity = GetDocument()->Find##name(url.suffix); \
				if (entity != NULL) { \
					instanceType* instance = new instanceType(GetDocument(), entity); \
					instances.push_back(instance); \
					status.AppendStatus(instance->LoadFromXML(child)); \
					continue; } } \
				status.Warning(fstring(FC("Unable to retrieve '") FC(#name) FC("' instance for scene node: ")) + TO_FSTRING(GetDaeId()), child->line); }

		else if (IsEquivalent(child->name, DAE_INSTANCE_CAMERA_ELEMENT)) { INSTANTIATE(Camera, FCDEntityInstance); }
		else if (IsEquivalent(child->name, DAE_INSTANCE_CONTROLLER_ELEMENT)) { INSTANTIATE(Controller, FCDGeometryInstance); }
		else if (IsEquivalent(child->name, DAE_INSTANCE_GEOMETRY_ELEMENT)) { INSTANTIATE(Geometry, FCDGeometryInstance); }
		else if (IsEquivalent(child->name, DAE_INSTANCE_LIGHT_ELEMENT)) { INSTANTIATE(Light, FCDEntityInstance); }
		else if (IsEquivalent(child->name, DAE_INSTANCE_NODE_ELEMENT))
		{
			FUUri url = ReadNodeUrl(child);
			if (url.prefix.empty())
			{
				FCDSceneNode* node = GetDocument()->FindSceneNode(url.suffix);
				if (node != NULL)
				{
					if (!AddChildNode(node))
					{
						status.Warning(FS("A cycle was found in the visual scene at node: ") + TO_FSTRING(GetDaeId()), child->line);
					}
				}
				else
				{
					status.Warning(FS("Unable to retrieve node instance for scene node: ") + TO_FSTRING(GetDaeId()), child->line);
				}
			}
		}
#		undef INSTANTIATE

		else if (IsEquivalent(child->name, DAE_INSTANCE_ELEMENT))
		{
			// COLLADA 1.3 backward compatibility: Weakly-typed instantiation
			// Might be a geometry, controller, camera or light.

			FUUri url = ReadNodeUrl(child);
			if (url.prefix.empty())
			{
#				define INSTANTIATE(name, instanceType) { \
					FCD##name* entity = GetDocument()->Find##name(url.suffix); \
					if (entity != NULL) { \
						instanceType* instance = new instanceType(GetDocument(), entity); \
						instances.push_back(instance); \
						instance->LoadFromXML(child); \
						continue; } }

				INSTANTIATE(Geometry, FCDGeometryInstance);
				INSTANTIATE(Controller, FCDGeometryInstance);
				INSTANTIATE(Camera, FCDEntityInstance);
				INSTANTIATE(Light, FCDEntityInstance);
#				undef INSTANTIATE

				FCDSceneNode* node = GetDocument()->FindSceneNode(url.suffix);
				if (node != NULL)
				{
					if (!AddChildNode(node))
					{
						status.Warning(FS("A cycle was found in the visual scene at node: ") + TO_FSTRING(GetDaeId()), child->line);
					}
				}
				else
				{
					status.Warning(FS("Unable to retrieve node instance for scene node: ") + TO_FSTRING(GetDaeId()), child->line);
				}

				status.Warning(FS("Unable to retrieve weakly-typed instance for scene node: ") + TO_FSTRING(GetDaeId()), child->line);
			}
			else
			{
				GetDocument()->GetFileManager()->GetFilePath(url.prefix);
				instances.push_back(new FCDExternalReference(GetDocument(), url));
			}
		}
		else if (IsEquivalent(child->name, DAE_EXTRA_ELEMENT)) {}
		else if (IsEquivalent(child->name, DAE_ASSET_ELEMENT)) {}
		else
		{
			FCDTransform* transform = FCDTFactory::CreateTransform(GetDocument(), this, child);
			if (transform != NULL)
			{
				transforms.push_back(transform);
				status.AppendStatus(transform->LoadFromXML(child));
			}
			else
			{
				status.Warning(FS("Unknown element or bad transform in scene node: ") + TO_FSTRING(GetDaeId()), child->line);
			}
		}
	}

	status.AppendStatus(LoadFromExtra());
	return status;
}

FUStatus FCDSceneNode::LoadFromExtra()
{
	FUStatus status;

	FCDENodeList parameterNodes;
	StringList parameterNames;

	// Retrieve the extra information from the base entity class
	FCDExtra* extra = GetExtra();

	// Read the Maya-specific technique
	FCDETechnique* mayaTechnique = extra->FindTechnique(DAEMAYA_MAYA_PROFILE);
	if (mayaTechnique == NULL) return status;

	mayaTechnique->FindParameters(parameterNodes, parameterNames);
	size_t parameterCount = parameterNodes.size();
	for (size_t i = 0; i < parameterCount; ++i)
	{
		FCDENode* parameterNode = parameterNodes[i];
		const string& parameterName = parameterNames[i];
		FCDEAttribute* parameterType = parameterNode->FindAttribute(DAE_TYPE_ATTRIBUTE);
		if (parameterName == DAEMAYA_STARTTIME_PARAMETER || parameterName == DAEMAYA_STARTTIME_PARAMETER1_3)
		{
			GetDocument()->SetStartTime(FUStringConversion::ToFloat(parameterNode->GetContent()));
			parameterNode->Release();
		}
		else if (parameterName == DAEMAYA_ENDTIME_PARAMETER || parameterName == DAEMAYA_ENDTIME_PARAMETER1_3)
		{
			GetDocument()->SetEndTime(FUStringConversion::ToFloat(parameterNode->GetContent()));
			parameterNode->Release();
		}
		else if (parameterName == DAEMAYA_VISIBILITY_PARAMETER || parameterName == DAEMAYA_VISIBILITY_PARAMETER1_3)
		{
			visibility = FUStringConversion::ToBoolean(parameterNode->GetContent()) ? 1.0f : 0.0f;
			FCDAnimatedCustom* animated = parameterNode->GetAnimated();
			if (animated != NULL)
			{
				FCDAnimatedFloat::Clone(GetDocument(), &animated->GetDummy(), &visibility);
			}
			parameterNode->Release();
		}
		else if (parameterName == DAEMAYA_LAYER_PARAMETER || (parameterType != NULL && FUStringConversion::ToString(parameterType->value) == DAEMAYA_LAYER_PARAMETER))
		{
			FCDEAttribute* nameAttribute = parameterNode->FindAttribute(DAE_NAME_ATTRIBUTE);
			if (nameAttribute == NULL) continue;

			// Create a new layer object list
			FCDLayerList& layers = GetDocument()->GetLayers();
			FCDLayer* layer = new FCDLayer(); layers.push_back(layer);

			// Parse in the layer
			layer->name = FUStringConversion::ToString(nameAttribute->value);
			FUStringConversion::ToStringList(parameterNode->GetContent(), layer->objects);
			parameterNode->Release();
		}
	}

	return status;
}

// bind material info
void FCDSceneNode::BindMaterial(xmlNode* node)
{
	xmlNodeList nodelist;
	FindChildrenByType(node, DAE_NODE_ELEMENT, nodelist);
	if(nodelist.size() == 0) return;
	for(xmlNodeList::iterator itN = nodelist.begin(); itN != nodelist.end(); ++itN)
	{
		BindMaterial(*itN);
		
		xmlNode* geometry = FindChildByType(*itN, DAE_INSTANCE_GEOMETRY_ELEMENT);
		if(geometry == NULL) continue;
		
		xmlNode* bindmatNode = FindChildByType(geometry, DAE_BINDMATERIAL_ELEMENT);
		xmlNode* techniqueNode = FindChildByType(bindmatNode, DAE_TECHNIQUE_COMMON_ELEMENT);
		xmlNode* materialNode = FindChildByType(techniqueNode, DAE_INSTANCE_MATERIAL_ELEMENT);
		string shadername = ReadNodeProperty(materialNode, DAE_SYMBOL_ATTRIBUTE);
		
		xmlNodeList bindNodes;
		FindChildrenByType( materialNode, DAE_BIND_ELEMENT, bindNodes);
		for (xmlNodeList::iterator itB = bindNodes.begin(); itB != bindNodes.end(); ++itB)
		{
			string semantic = ReadNodeSemantic(*itB);
			string target = ReadNodeProperty(*itB, DAE_TARGET_ATTRIBUTE);
			
			GetPostProcessCmds().push_back(shadername);
			GetPostProcessCmds().push_back(target);
			GetPostProcessCmds().push_back(semantic);
		}
	}
}


// Write out a <visual_scene> element to a COLLADA xml document
xmlNode* FCDSceneNode::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* visualSceneNode = WriteToEntityXML(parentNode, DAE_VSCENE_ELEMENT);
	if (visualSceneNode != NULL)
	{
		WriteToNodeXML(visualSceneNode, true);
	}
	return visualSceneNode;
}

// Write out a <visual_scene> or a <node> element to a COLLADA xml document
void FCDSceneNode::WriteToNodeXML(xmlNode* node, bool isVisualScene) const
{
	// Write out the extra attributes
	if (!isVisualScene)
	{
		if (isJoint)
		{
			AddAttribute(node, DAE_SID_ATTRIBUTE, GetDaeId());
			AddAttribute(node, DAE_TYPE_ATTRIBUTE, DAE_JOINT_NODE_TYPE);
		}
		else
		{
			AddAttribute(node, DAE_TYPE_ATTRIBUTE, DAE_NODE_NODE_TYPE);
		}
	}

	// Write out the transforms
	for (FCDTransformList::const_iterator itT = transforms.begin(); itT != transforms.end(); ++itT)
	{
		FCDTransform* transform = (*itT);
		transform->WriteToXML(node);
	}

	// Write out the instantiation
	for (FCDEntityInstanceList::const_iterator itI = instances.begin(); itI != instances.end(); ++itI)
	{
		FCDEntityInstance* instance = (*itI);
		instance->WriteToXML(node);
	}

	// Write out the child scene graph nodes as <node> elements
	if (!isVisualScene || !children.empty())
	{
		for (FCDSceneNodeList::const_iterator itC = children.begin(); itC != children.end(); ++itC)
		{
			FCDSceneNode* child = (*itC);
			xmlNode* nodeNode = child->WriteToEntityXML(node, DAE_NODE_ELEMENT);
			if (nodeNode != NULL) child->WriteToNodeXML(nodeNode, false);
		}
	}
	else
	{
		// In COLLADA 1.4, the visual scene must contain at least one <node>.
		UNUSED(xmlNode* dummyNodeNode =) FUXmlWriter::AddChild(node, DAE_NODE_ELEMENT);
	}

	// Write out the extra information
	FCDEntity::WriteToExtraXML(node);
}
