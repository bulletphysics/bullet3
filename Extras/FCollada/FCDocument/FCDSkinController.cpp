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
#include "FCDocument/FCDController.h"
#include "FCDocument/FCDGeometry.h"
#include "FCDocument/FCDGeometryMesh.h"
#include "FCDocument/FCDGeometryPolygons.h"
#include "FCDocument/FCDGeometrySource.h"
#include "FCDocument/FCDGeometrySpline.h"
#include "FCDocument/FCDSceneNode.h"
#include "FCDocument/FCDSkinController.h"
#include "FUtils/FUStringConversion.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDSkinController::FCDSkinController(FCDocument* document, FCDController* _parent) : FCDObject(document, "FCDSkinController")
{
	parent = _parent;
	target = NULL;
	ownsTarget = false;
	bindShapeTransform = FMMatrix44::Identity;
}

FCDSkinController::~FCDSkinController()
{
	if (ownsTarget) { SAFE_DELETE(target); }
	else target = NULL;
	parent = NULL;
}

void FCDSkinController::SetTarget(FCDEntity* _target)
{
	target = NULL;
	weightedMatches.clear();

	// Retrieve the actual base entity, as you can chain controllers.
	FCDEntity* baseEntity = _target;
	while (baseEntity != NULL && baseEntity->GetType() == FCDEntity::CONTROLLER)
	{
		baseEntity = ((FCDController*) baseEntity)->GetBaseTarget();
	}

	if (baseEntity == NULL || baseEntity->GetType() != FCDEntity::GEOMETRY)
	{
		// The new target is no good!
		return;
	}

	target = _target;
	FCDGeometry* geometry = (FCDGeometry*) baseEntity;

	// Retrieve the new vertex count
	size_t vertexCount = 0;
	if (geometry->IsMesh())
	{
		FCDGeometryMesh* mesh = geometry->GetMesh();
		FCDGeometrySource* positionSource = mesh->GetPositionSource();
		if (positionSource != NULL)
		{
			vertexCount = positionSource->GetSourceData().size() / positionSource->GetSourceStride();
		}
	}
	else if (geometry->IsSpline())
	{
		FCDGeometrySpline* spline = geometry->GetSpline();
		vertexCount = spline->GetCVCount();
	}

	// Modify the list of influences to match the new target's vertex count.
	weightedMatches.resize(vertexCount);
}

// Adds a joint and its bind-pose to the list of joint influencing the skin.
void FCDSkinController::AddJoint(FCDSceneNode* joint, const FMMatrix44& bindPose)
{
	FCDJointMatrixPair pair;
	pair.joint = joint;
	pair.invertedBindPose = bindPose.Inverted();
	joints.push_back(pair);

	// Also set the scene node's joint flag.
	joint->SetJointFlag(true);
}

// Removes a joint from the list of joints influencing the skin.
void FCDSkinController::RemoveJoint(FCDSceneNode* joint)
{
	// Find the joint within the joint list.
	size_t index = 0;
	for (index = 0; index < joints.size(); ++index)
	{
		if (joints[index].joint == joint) break;
	}
	if (index == joints.size()) return;

	// Look through the per-vertex influences and remove the influences
	// that use this joint. Also, shift down the influences that use
	// the joints that appear after this one in the list.
	for (FCDWeightedMatches::iterator itM = weightedMatches.begin(); itM != weightedMatches.end(); ++itM)
	{
		FCDJointWeightPairList& pairs = (*itM);
		for (FCDJointWeightPairList::iterator itP = pairs.begin(); itP != pairs.end();)
		{
			FCDJointWeightPair& p = (*itP);
			if (p.jointIndex == index)
			{
				pairs.erase(itP);
			}
			else if (p.jointIndex > index)
			{
				--p.jointIndex;
				++itP;
			}
			else
			{
				++itP;
			}
		}
	}
}

// Look for the information on a given joint
FCDJointMatrixPair* FCDSkinController::FindJoint(FCDSceneNode* joint)
{
	if (joint == NULL) return NULL;
	for (FCDJointList::iterator itJ = joints.begin(); itJ != joints.end(); ++itJ)
	{
		if ((*itJ).joint == joint) return &(*itJ);
	}
	return NULL;
}

const FCDJointMatrixPair* FCDSkinController::FindJoint(const FCDSceneNode* joint) const
{
	if (joint == NULL) return NULL;
	for (FCDJointList::const_iterator itJ = joints.begin(); itJ != joints.end(); ++itJ)
	{
		if ((*itJ).joint == joint) return &(*itJ);
	}
	return NULL;
}

// Reduce the number of joints influencing each vertex to a maximum count
void FCDSkinController::ReduceInfluences(uint32 maxInfluenceCount, float minimumWeight)
{
	// Pre-cache an empty weight list to the reduced count
	FCDJointWeightPairList reducedWeights;
	reducedWeights.reserve(maxInfluenceCount + 1);

	for (FCDWeightedMatches::iterator itM = weightedMatches.begin(); itM != weightedMatches.end(); ++itM)
	{
		FCDJointWeightPairList& weights = (*itM);
		size_t oldWeightCount = weights.size();

		// Reduce the weights, keeping only the more important ones using a sorting algorithm.
		// Also, calculate the current total of the weights, to re-normalize the reduced weights
		float oldTotal = 0.0f;
		reducedWeights.clear();
		for (FCDJointWeightPairList::iterator itW = weights.begin(); itW != weights.end(); ++itW)
		{
			FCDJointWeightPairList::iterator itRW = reducedWeights.begin();
			if ((*itW).weight >= minimumWeight)
			{
			for (; itRW != reducedWeights.end() && (*itRW).weight < (*itW).weight; ++itRW) {}
			if (itRW != reducedWeights.end() || reducedWeights.size() <= maxInfluenceCount)
			{
				reducedWeights.insert(itRW, (*itW));
				if (reducedWeights.size() > maxInfluenceCount) reducedWeights.pop_back();
			}
			}
			oldTotal += (*itW).weight;
		}

		if (oldWeightCount > reducedWeights.size())
		{
			// Replace the old weights and re-normalize to their old total
			weights = reducedWeights;
			float newTotal = 0.0f;
			for (FCDJointWeightPairList::iterator itW = weights.begin(); itW != weights.end(); ++itW) newTotal += (*itW).weight;
			float renormalizingFactor = oldTotal / newTotal;
			for (FCDJointWeightPairList::iterator itW = weights.begin(); itW != weights.end(); ++itW) (*itW).weight *= renormalizingFactor;
		}
	}
}

// Load this controller from a COLLADA <controller> node
FUStatus FCDSkinController::LoadFromXML(xmlNode* skinNode)
{
	FUStatus status;
	if (!IsEquivalent(skinNode->name, DAE_CONTROLLER_SKIN_ELEMENT))
	{
		return status.Warning(FS("Unexpected node in controller library: ") + TO_FSTRING((const char*) skinNode->name), skinNode->line);
	}

	// Get the <skin> element and process the inner <vertices> element
	xmlNode* verticesNode = FindChildByType(skinNode, DAE_VERTICES_ELEMENT);
	bool isCollada1_3 = verticesNode != NULL;

	// Read in the <bind_shape_matrix> element
	xmlNode* bindShapeTransformNode = FindChildByType(skinNode, DAE_BINDSHAPEMX_SKIN_PARAMETER);
	if (bindShapeTransformNode == NULL) bindShapeTransform = FMMatrix44::Identity;
	else
	{
		const char* content = ReadNodeContentDirect(bindShapeTransformNode);
		FUStringConversion::ToMatrix(&content, bindShapeTransform, GetDocument()->GetLengthUnitConversion());
	}

	// Find the target geometry
	string targetId = (isCollada1_3) ? parent->GetTargetId() : ReadNodeProperty(skinNode, DAE_SOURCE_ATTRIBUTE);
	target = GetDocument()->FindGeometry(targetId);
	if (target == NULL) target = GetDocument()->FindController(targetId);
	if (target == NULL)
	{
		return status.Warning(FS("Target not found for controller: ") + TO_FSTRING(parent->GetDaeId()), skinNode->line);
	}

	// Retrieve the <joints> and <combiner> elements
	xmlNode* jointsNode = NULL,* combinerNode = NULL,* containerNode = NULL;
	if (isCollada1_3)
	{
		// COLLADA 1.3 backward compatibility: read in the bind-shape positions/normals
		xmlNodeList vertexInputNodes;
		FindChildrenByType(verticesNode, DAE_INPUT_ELEMENT, vertexInputNodes);
		xmlNode* bindShapePositionSourceNode = NULL,* jointWeightSourceNode = NULL,* bindShapeNormalSourceNode = NULL;
		for (xmlNodeList::iterator it = vertexInputNodes.begin(); it != vertexInputNodes.end(); ++it)
		{
			string semantic = ReadNodeSemantic(*it);
			string sourceId = ReadNodeSource(*it);
			if (semantic == DAE_BINDPOS_SKIN_INPUT) bindShapePositionSourceNode = FindChildById(skinNode, sourceId);
			else if (semantic == DAE_BINDNORMAL_SKIN_INPUT) bindShapeNormalSourceNode = FindChildById(skinNode, sourceId);
			else if (semantic == DAE_JOINTWEIGHT_SKIN_INPUT) jointWeightSourceNode = FindChildById(skinNode, sourceId);
			else
			{
				status.Warning(FS("Unknown vertex input in skin controller: ") + TO_FSTRING(parent->GetDaeId()), (*it)->line);
			}
		}
		if (jointWeightSourceNode == NULL)
		{
			return status.Fail(FS("Cannot find 'JOINTS_AND_WEIGHTS' input in <vertices> element for controller: ") + TO_FSTRING(parent->GetDaeId()), jointWeightSourceNode->line);
		}
		if (bindShapePositionSourceNode == NULL)
		{
			return status.Fail(FS("Cannot find 'BIND_SHAPE_POSITION' input in <vertices> element for controller: ") + TO_FSTRING(parent->GetDaeId()), jointWeightSourceNode->line);
		}

		// Read in the bindshape positions/normals and override the original mesh's positions/normals
		FloatList bindShapePositions, bindShapeNormals;
		uint32 positionStride = ReadSource(bindShapePositionSourceNode, bindShapePositions);
		uint32 normalsStride = ReadSource(bindShapeNormalSourceNode, bindShapeNormals);
		if (target->GetType() != FCDEntity::GEOMETRY)
		{
			return status.Fail(FS("COLLADA 1.3 only supports geometric bind shapes for skin controller: ") + TO_FSTRING(parent->GetDaeId()), skinNode->line);
		}
		target = ((FCDGeometry*)target)->Clone(bindShapePositions, positionStride, bindShapeNormals, normalsStride);
		ownsTarget = true;

		// Retrieve the <joints> element and the <combiner> element
		xmlNode* jointWeightTechniqueNode = FindTechnique(jointWeightSourceNode, DAE_COMMON_PROFILE);
		jointsNode = FindChildByType(jointWeightTechniqueNode, DAE_JOINTS_ELEMENT);
		combinerNode = FindChildByType(jointWeightTechniqueNode, DAE_COMBINER_ELEMENT);
		containerNode = jointWeightSourceNode;
	}
	else
	{
		// COLLADA 1.4: use the target geometry directly as the bind-shape.

		// Retrieve the <joints> element and the <vertex_weights> element
		jointsNode = FindChildByType(skinNode, DAE_JOINTS_ELEMENT);
		combinerNode = FindChildByType(skinNode, DAE_WEIGHTS_ELEMENT);
	}

	// Verify that we have the necessary data structures: bind-shape, <joints> elements, <combiner> element
	if (target == NULL)
	{
		return status.Warning(FS("Unable to clone/find the target geometry for controller: ") + TO_FSTRING(parent->GetDaeId()), skinNode->line);
	}
	if (jointsNode == NULL)
	{
		return status.Fail(FS("No <joints> element found in the skin weight source for controller: ") + TO_FSTRING(parent->GetDaeId()), skinNode->line);
	}
	if (combinerNode == NULL)
	{
		return status.Fail(FS("No <combiner> node found in the skin weight source for controller: ") + TO_FSTRING(parent->GetDaeId()), skinNode->line);
	}

	// Gather the inputs for the <joints> element and the <combiner> element
	xmlNode* firstCombinerValueNode = NULL;
	xmlNodeList skinningInputNodes;
	FindChildrenByType(jointsNode, DAE_INPUT_ELEMENT, skinningInputNodes);
	uint32 combinerValueCount = ReadNodeCount(combinerNode);
	for (xmlNode* child = combinerNode->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;
		if (IsEquivalent(child->name, DAE_INPUT_ELEMENT)) skinningInputNodes.push_back(child);
		else if (IsEquivalent(child->name, DAE_VERTEX_ELEMENT) || IsEquivalent(child->name, DAE_VERTEXCOUNT_ELEMENT))
		{ 
			firstCombinerValueNode = child;
			break;
		}
	}
	if (firstCombinerValueNode == NULL)
	{
		return status.Fail(FS("Unable to find <combiner> element's value nodes for controller: ") + TO_FSTRING(parent->GetDaeId()), combinerNode->line);
	}

	// Process these inputs
	FloatList weights;
	FMMatrix44List invertedBindPoses;
	int32 jointIdx = 0, weightIdx = 1;
	for (xmlNodeList::iterator it = skinningInputNodes.begin(); it != skinningInputNodes.end(); ++it)
	{
		string semantic = ReadNodeSemantic(*it);
		string sourceId = ReadNodeSource(*it);

		const char* arrayContent = NULL;
		uint32 count = 0;
		xmlNode* sourceNode = NULL;
		if (isCollada1_3)
		{
			// COLLADA 1.3 backward compatibility: retrieve the <accessor> node and its <array>
			xmlNode* techniqueNode = FindTechnique(containerNode, DAE_COMMON_PROFILE);
			xmlNode* accessorNode = FindChildById(techniqueNode, sourceId);
			string arrayId = ReadNodeSource(accessorNode);
			xmlNode* arrayNode = FindChildById(containerNode, arrayId);
			arrayContent = ReadNodeContentDirect(arrayNode);
			count = ReadNodeCount(accessorNode);
		}
		else
		{
			// Find the source node for this source id
			sourceNode = FindChildById(skinNode, sourceId);
		}

		if (semantic == DAE_JOINT_SKIN_INPUT)
		{
			string idx = ReadNodeProperty(*it, DAE_OFFSET_ATTRIBUTE);
			if (idx.empty()) idx = ReadNodeProperty(*it, DAE_IDX_ATTRIBUTE); // COLLADA 1.3 Backward-compatibility
			if (!idx.empty()) jointIdx = FUStringConversion::ToInt32(idx);
			if (!jointIds.empty()) continue;

			// Read in joint ids <source> element
			if (isCollada1_3)
			{
				// COLLADA 1.3 backward compatibility: read in the joint Ids directly
				jointIds.resize(count);
				FUStringConversion::ToStringList(arrayContent, jointIds);
			}
			else ReadSource(sourceNode, jointIds);
		}
		else if (semantic == DAE_BINDMATRIX_SKIN_INPUT)
		{
			if (!invertedBindPoses.empty())
			{
				return status.Fail(FS("No inverted bind matrix input in controller: ") + TO_FSTRING(parent->GetDaeId()), (*it)->line);
			}

			// Read in the bind-pose matrices <source> element
			if (isCollada1_3)
			{
				// COLLADA 1.3 backward compatibility: read in the bind-pose matrices directly
				invertedBindPoses.resize(count);
				FUStringConversion::ToMatrixList(arrayContent, invertedBindPoses, GetDocument()->GetLengthUnitConversion());
			}
			else ReadSource(sourceNode, invertedBindPoses, GetDocument()->GetLengthUnitConversion());
		}
		else if (semantic == DAE_WEIGHT_SKIN_INPUT)
		{
			string idx = ReadNodeProperty(*it, DAE_OFFSET_ATTRIBUTE);
			if (idx.empty()) idx = ReadNodeProperty(*it, DAE_IDX_ATTRIBUTE); // COLLADA 1.3 Backward-compatibility
			if (!idx.empty()) weightIdx = FUStringConversion::ToInt32(idx);

			// Read in the weights <source> element
			if (isCollada1_3)
			{
				// COLLADA 1.3 backward compatibility: read in the weights directly
				weights.resize(count);
				FUStringConversion::ToFloatList(arrayContent, weights);
			}
			else ReadSource(sourceNode, weights);
		}
	}

	// Parse the <vcount> and the <v> elements
	UInt32List combinerVertexCounts; combinerVertexCounts.reserve(combinerValueCount);
	Int32List combinerVertexIndices; combinerVertexIndices.reserve(combinerValueCount * 5);
	if (!isCollada1_3)
	{
		// The <vcount> and the <v> elements are ordered. Read the <vcount> element first.
		if (!IsEquivalent(firstCombinerValueNode->name, DAE_VERTEXCOUNT_ELEMENT))
		{
			return status.Fail(FS("Expecting <vcount> element in combiner for controller: ") + TO_FSTRING(parent->GetDaeId()), firstCombinerValueNode->line);
		}
		const char* content = ReadNodeContentDirect(firstCombinerValueNode);
		FUStringConversion::ToUInt32List(content, combinerVertexCounts);

		// Read the <v> element second.
		xmlNode* vNode = firstCombinerValueNode->next;
		while (vNode != NULL && vNode->type != XML_ELEMENT_NODE) vNode = vNode->next;
		if (vNode == NULL || !IsEquivalent(vNode->name, DAE_VERTEX_ELEMENT))
		{
			return status.Fail(FS("Expecting <v> element after <vcount> element in combiner for controller: ") + TO_FSTRING(parent->GetDaeId()), vNode->line);
		}
		content = ReadNodeContentDirect(vNode);
		FUStringConversion::ToInt32List(content, combinerVertexIndices);
	}
	else
	{
		// COLLADA 1.3 backward compatibility: Read in the many <v> elements,
		// creating the vertex counts array along the way
		Int32List indices; indices.reserve(32);
		for (xmlNode* valueNode = firstCombinerValueNode; valueNode != NULL; valueNode = valueNode->next)
		{
			if (valueNode->type != XML_ELEMENT_NODE) continue;

			indices.clear();
			const char* valueNodeContent = ReadNodeContentDirect(valueNode);
			FUStringConversion::ToInt32List(valueNodeContent, indices);
			size_t indexCount = indices.size() / 2;
			combinerVertexCounts.push_back((uint32) indexCount);
			combinerVertexIndices.insert(combinerVertexIndices.end(), indices.begin(), indices.end());
		}
	}
	size_t combinerVertexIndexCount = combinerVertexIndices.size();

	// Validate the inputs
	if (jointIds.size() != invertedBindPoses.size())
	{
		return status.Fail(FS("Joint count and bind pose matrix count aren't equal for controller: ") + TO_FSTRING(parent->GetDaeId()), skinNode->line);
	}
	if (combinerVertexCounts.size() != combinerValueCount)
	{
		return status.Fail(FS("The <vcount> element list should contains the number of values determined by the <vertex_weights>'s 'count' attribute: ") + TO_FSTRING(parent->GetDaeId()), skinNode->line);
	}

	// Setup the joint-weight-vertex matches
	weightedMatches.resize(combinerValueCount);
	size_t jointCount = jointIds.size(), weightCount = weights.size(), offset = 0;
	for (size_t j = 0; j < combinerValueCount; ++j)
	{
		FCDJointWeightPairList& pairList = weightedMatches[j];
		uint32 localValueCount = combinerVertexCounts[j];
		pairList.resize(localValueCount);
		for (size_t i = 0; i < localValueCount && offset < combinerVertexIndexCount - 1; ++i)
		{
			pairList[i].jointIndex = combinerVertexIndices[offset + jointIdx];
			if (pairList[i].jointIndex >= jointCount)
			{
				status.Warning(FS("Joint index out of bounds in combiner for controller: ") + TO_FSTRING(parent->GetDaeId()));
				pairList[i].jointIndex = 0;
			}
			uint32 weightIndex = combinerVertexIndices[offset + weightIdx];
			if (weightIndex >= weightCount)
			{
				status.Warning(FS("Weight index out of bounds in combiner for controller: ") + TO_FSTRING(parent->GetDaeId()));
				weightIndex = 0;
			}
			pairList[i].weight = weights[weightIndex];
			offset += 2;
		}
	}

	// Normalize the weights, per-vertex, to 1 (or 0)
	// This step is still being debated as necessary or not, for COLLADA 1.4.
	for (FCDWeightedMatches::iterator it = weightedMatches.begin(); it != weightedMatches.end(); ++it)
	{
		FCDJointWeightPairList& pair = (*it);
		float weightSum = 0.0f;
		for (FCDJointWeightPairList::iterator itP = pair.begin(); itP != pair.end(); ++itP)
		{
			weightSum += (*itP).weight;
		}

		if (IsEquivalent(weightSum, 0.0f) || IsEquivalent(weightSum, 1.0f)) continue;

		float invWeightSum = 1.0f / weightSum;
		for (FCDJointWeightPairList::iterator itP = pair.begin(); itP != pair.end(); ++itP)
		{
			(*itP).weight *= invWeightSum;
		}
	}

	// Setup the bind poses.
	// Joints are linked later, as they are loaded last: with the scene graph
	joints.resize(jointCount);
	for (uint32 i = 0; i < jointCount; ++i)
	{
		FCDJointMatrixPair& joint = joints[i];
		joint.invertedBindPose = invertedBindPoses[i];
		joint.joint = NULL;
	}

	return status;
}

// Write out this controller to a COLLADA xml node tree
xmlNode* FCDSkinController::WriteToXML(xmlNode* parentNode) const
{
	// Create the <skin> element
	xmlNode* skinNode = AddChild(parentNode, DAE_CONTROLLER_SKIN_ELEMENT);
	if (target != NULL) AddAttribute(skinNode, DAE_SOURCE_ATTRIBUTE, string("#") + target->GetDaeId());

	// Create the <bind_shape_matrix> element
	string bindShapeMatrixString = FUStringConversion::ToString(bindShapeTransform);
	AddChild(skinNode, DAE_BINDSHAPEMX_SKIN_PARAMETER, bindShapeMatrixString);

	// Create the joint source
	FUSStringBuilder jointSourceId(parent->GetDaeId()); jointSourceId += "-joints";
	StringList jointIds;
	for (FCDJointList::const_iterator itJ = joints.begin(); itJ != joints.end(); ++itJ)
	{
		if ((*itJ).joint != NULL) jointIds.push_back((*itJ).joint->GetDaeId());
		else jointIds.push_back(DAEERR_UNKNOWN_INPUT);
	}
	AddSourceIDRef(skinNode, jointSourceId.ToCharPtr(), jointIds, DAE_JOINT_SKIN_INPUT);

	// Create the joint bind matrix source
	FUSStringBuilder jointBindSourceId(parent->GetDaeId()); jointBindSourceId += "-bind_poses";
	FMMatrix44List jointBindPoses;
	for (FCDJointList::const_iterator itJ = joints.begin(); itJ != joints.end(); ++itJ)
	{
		jointBindPoses.push_back((*itJ).invertedBindPose);
	}
	AddSourceMatrix(skinNode, jointBindSourceId.ToCharPtr(), jointBindPoses);

	// Create the weight source
	FloatList weights;
	weights.push_back(1.0f);
	for (FCDWeightedMatches::const_iterator itW = weightedMatches.begin(); itW != weightedMatches.end(); ++itW)
	{
		const FCDJointWeightPairList& pairs = (*itW);
		for (FCDJointWeightPairList::const_iterator itP = pairs.begin(); itP != pairs.end(); ++itP)
		{
			float w = (*itP).weight;
			if (!IsEquivalent(w, 1.0f)) weights.push_back(w);
		}
	}
	FUSStringBuilder weightSourceId(parent->GetDaeId()); weightSourceId += "-weights";
	AddSourceFloat(skinNode, weightSourceId.ToCharPtr(), weights, DAE_WEIGHT_SKIN_INPUT);

	// Create the <joints> element
	xmlNode* jointsNode = AddChild(skinNode, DAE_JOINTS_ELEMENT);
	AddInput(jointsNode, jointSourceId.ToCharPtr(), DAE_JOINT_SKIN_INPUT);
	AddInput(jointsNode, jointBindSourceId.ToCharPtr(), DAE_BINDMATRIX_SKIN_INPUT);

	// Create the <vertex_weights> element
	xmlNode* matchesNode = AddChild(skinNode, DAE_WEIGHTS_ELEMENT);
	AddInput(matchesNode, jointSourceId.ToCharPtr(), DAE_JOINT_SKIN_INPUT, 0);
	AddInput(matchesNode, weightSourceId.ToCharPtr(), DAE_WEIGHT_SKIN_INPUT, 1);
	AddAttribute(matchesNode, DAE_COUNT_ATTRIBUTE, weightedMatches.size());

	// Generate the vertex count and match value strings and export the <v> and <vcount> elements
	FUSStringBuilder vertexCounts; vertexCounts.reserve(1024);
	FUSStringBuilder vertexMatches; vertexMatches.reserve(1024);
	uint32 weightOffset = 1;
	for (FCDWeightedMatches::const_iterator itW = weightedMatches.begin(); itW != weightedMatches.end(); ++itW)
	{
		const FCDJointWeightPairList& pairs = (*itW);
		vertexCounts.append((uint32) pairs.size()); vertexCounts.append(' ');
		for (FCDJointWeightPairList::const_iterator itP = pairs.begin(); itP != pairs.end(); ++itP)
		{
			vertexMatches.append((*itP).jointIndex); vertexMatches.append(' ');
			if (!IsEquivalent((*itP).weight, 1.0f)) vertexMatches.append(weightOffset++);
			else vertexMatches.append('0');
			vertexMatches.append(' ');
		}
	}
	if (!vertexMatches.empty()) vertexMatches.pop_back();
	AddChild(matchesNode, DAE_VERTEXCOUNT_ELEMENT, vertexCounts);
	AddChild(matchesNode, DAE_VERTEX_ELEMENT, vertexMatches);
	return skinNode;
}

// Done after the scene graph is loaded, link this controller to its joints
FUStatus FCDSkinController::Link()
{
	FUStatus status;

	// Look for each joint, by COLLADA id, within the scene graph
	size_t jointCount = joints.size();
	if (jointCount != jointIds.size())
	{
		return status.Fail(FS("Parsing programming error in controller: ") + TO_FSTRING(parent->GetDaeId()));
	}
	for (size_t i = 0; i < jointCount; ++i)
	{
		FCDJointMatrixPair& joint = joints[i];
		joint.joint = GetDocument()->FindSceneNode(jointIds[i]);
		if (joint.joint != NULL)
		{
			joint.joint->SetJointFlag(true);
		}
		else
		{
			status.Warning(FS("Unknown joint '") + TO_FSTRING(jointIds[i]) + FS("' referenced in controller: ") + TO_FSTRING(parent->GetDaeId()));
		}
	}
	jointIds.clear();

	return status;
}
