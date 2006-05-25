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

/**
	@file FCDSceneNode.h
	This file contains the FCDSceneNode class.
*/

#ifndef _FCD_SCENE_NODE_
#define _FCD_SCENE_NODE_

#include "FCDocument/FCDEntity.h"
#include "FCDocument/FCDTransform.h" /** @todo Remove this include by moving the FCDTransform::Type enum to FUDaeEnum.h. */

class FCDocument;
class FCDEntityInstance;
class FCDSceneNode;
class FCDTransform;
class FCDExtra;
class FCDTRotation;
class FCDTTranslation;
class FCDTScale;
class FCDTSkew;
class FCDTLookAt;
class FCDTMatrix;

typedef vector<FCDSceneNode*> FCDSceneNodeList; /**< A dynamically-sized array of scene nodes. */
typedef vector<FCDEntityInstance*> FCDEntityInstanceList; /**< A dynamically-sized array of entitiy instances. */
typedef vector<FCDTransform*> FCDTransformList; /**< A dynamically-sized array of transforms. */

/**
	A COLLADA visual scene node.
	This class is also used to represent COLLADA visual scene entities.

	A visual scene node contains child scene nodes to make a tree.
	A visual scene node may appear multiple times within the scene graph,
	but checks are made to verify that there are no cycles within the graph.

	A visual scene node also contained an ordered list of transformations
	and a list of entity instances.

	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDSceneNode : public FCDEntity
{
private:
	FCDSceneNodeList parents;
	FCDSceneNodeList children;
	FCDTransformList transforms;
	FCDEntityInstanceList instances;
	FCDExtra* extra;

	// Visibility parameter. Should be a boolean, but is animated
	float visibility; // Maya-specific

	// The number of entities that target this node
	uint32 targetCount;

	// Whether this node is a joint.
	bool isJoint;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDSceneNode::AddChild function for child
		visual scene nodes or the FCDLibrary::AddEntity function
		for visual scenes.
		@param document The COLLADA document that owns the scene node. */
	FCDSceneNode(FCDocument* document);

	/** Destructor: do not use directly.
		Instead, use the FCDSceneNode::ReleaseChild function for visual
		scene nodes or the FCDLibrary::ReleaseEntity function for visual scenes. */
	virtual ~FCDSceneNode();

	/** Retrieves the type of the entity class.
		@return The type of entity class: SCENE_NODE. */
	inline virtual Type GetType() const { return SCENE_NODE; }

	/** Retrieves the number of parent nodes for this visual scene node.
		@return The number of parents. */
	inline size_t GetParentCount() const { return parents.size(); };

	/** Retrieves a specific parent of the visual scene node.
		@param index The index of the parent.
		@return The parent visual scene node. This pointer will be NULL if
			the scene node has no parents or if the index is out-of-bounds. */
	inline FCDSceneNode* GetParent(size_t index = 0) { FUAssert(index == 0 || index < parents.size(), return NULL); return (!parents.empty()) ? parents.at(index) : NULL; }
	inline const FCDSceneNode* GetParent(size_t index = 0) const { FUAssert(index == 0 || index < parents.size(), return NULL); return (!parents.empty()) ? parents.at(index) : NULL; } /**< See above. */

	/** Retrieves the number of child nodes for this visual scene node.
		@return The number of children. */
	inline size_t GetChildrenCount() const { return children.size(); };

	/** Retrieves a specific child of the visual scene node.
		@param index The index of the child.
		@return The child scene node. This pointer will be NULL if the
			index is out-of-bounds. */
	inline FCDSceneNode* GetChild(size_t index) { FUAssert(index < children.size(), return NULL); return children.at(index); }
	inline const FCDSceneNode* GetChild(size_t index) const { FUAssert(index < children.size(), return NULL); return children.at(index); } /**< See above. */

	/** Retrieves the list of children of the visual scene node.
		@return The list of child scene nodes. */
	inline FCDSceneNodeList& GetChildren() { return children; }
	inline const FCDSceneNodeList& GetChildren() const { return children; } /**< See above. */

	/** Creates a new child scene node.
		@return The new child scene node. */
	FCDSceneNode* AddChildNode();

	/** Attaches a existing scene node to this visual scene node.
		This function will fail if attaching the given scene node
		to this visual scene node creates a cycle within the scene graph.
		@param sceneNode The scene node to attach.
		@return Whether the given scene node was attached to this scene node. */
	bool AddChildNode(FCDSceneNode* sceneNode);

	/** Retrieves the number of entity instances at this node of the scene graph.
		@return The number of entity instances. */
	inline size_t GetInstanceCount() const { return instances.size(); };

	/** Retrieves a specific entity instance.
		@param index The index of the instance.
		@return The entity instance at the given index. This pointer will be
			NULL if the index is out-of-bounds. */
	inline FCDEntityInstance* GetInstance(size_t index) { FUAssert(index < instances.size(), return NULL); return instances.at(index); }
	inline const FCDEntityInstance* GetInstance(size_t index) const { FUAssert(index < instances.size(), return NULL); return instances.at(index); } /**< See above. */

	/** Retrieves the list of entity instances at this node of the scene graph.
		@return The list of entity instances. */
	inline FCDEntityInstanceList& GetInstances() { return instances; }
	inline const FCDEntityInstanceList& GetInstances() const { return instances; } /**< See above. */

	/** Retrieves the number of transforms for this node of the scene graph.
		@return The number of transforms. */
	inline size_t GetTransformCount() const { return transforms.size(); };

	/** Retrieves a specific transform.
		@param index The index of the transform.
		@return The transform at the given index. This pointer will be NULL
			if the index is out-of-bounds. */
	inline FCDTransform* GetTransform(size_t index) { FUAssert(index < transforms.size(), return NULL); return transforms.at(index); }
	inline const FCDTransform* GetTransform(size_t index) const { FUAssert(index < transforms.size(), return NULL); return transforms.at(index); } /**< See above. */

	/** Retrieves the list of transforms for this node of the scene graph.
		@return The list of transforms. */
	inline FCDTransformList& GetTransforms() { return transforms; }
	inline const FCDTransformList& GetTransforms() const { return transforms; } /**< See above. */

	/** Creates a new transform for this visual scene node.
		The transforms are processed in order and COLLADA is column-major.
		For row-major matrix stacks, such as DirectX, this implies that the
		transformations will be processed in reverse order.
		By default, a transform is added at the end of the list.
		@param type The type of transform to create.
		@param index The index at which to insert the transform. Set this value to -1
			to indicate that you want this transform at the end of the stack.
		@return The created transform. */
	FCDTransform* AddTransform(FCDTransform::Type type, size_t index = (size_t)-1);

	/** Retrieves the visual scene node with the given id.
		This function looks through the whole tree of visual scene nodes
		for the wanted COLLADA id.
		@param daeId The COLLADA id to look for.
		@return The visual scene node which has the given COLLADA id. This pointer
			will be NULL if no visual scene node can be found with the given COLLADA id. */
	virtual FCDEntity* FindDaeId(const string& daeId);

	/** Retrieves whether the visual scene node is visible.
		A hidden visual scene node will not be rendered but will
		still affect the world. This parameter is a floating-point value
		because it is animated. It should be intepreted as a Boolean value.
		@return Whether the scene node is visible. */
	inline float& GetVisibility() { return visibility; }
	inline const float& GetVisibility() const { return visibility; } /**< See above. */

	/** Sets the visibility of the visual scene node.
		A hidden visual scene node will not be rendered but will
		still affect the world.
		@param isVisible Whether the visual scene node is visible. */
	inline void SetVisibility(bool isVisible) { visibility = isVisible; }

	/** Retrieves whether this visual scene node is the target of an entity.
		@return Whether this is an entity target. */
	inline bool IsTarget() const { return targetCount > 0; }

	/** Retrieves whether this visual scene node is a joint.
		Joints are called bones in 3dsMax. A joint is a scene node that is used in skinning.
		@return Whether this node is a joint. */
	bool IsJoint() const { return isJoint; }

	/** Sets whether a visual scene node is a joint.
		Joints are called bones in 3dsMax. A joint is a scene node that is used in skinning.
		@param _isJoint Whether this node is a joint. */
	void SetJointFlag(bool _isJoint) { isJoint = _isJoint; }

	/** Retrieves the local transform for this visual scene node.
		This function does not handle or apply animations.
		@return The local transform. */
	FMMatrix44 ToMatrix() const;

	/** Generates a list of local transform samples for this visual scene node.
		This function will <b>permanently</b> modify the transforms of this visual scene node.
		@param keys A list of key inputs that will be filled in with the sample times.
		@param values A list of matrices that will be filled in with the sampled local transforms. */
	void GenerateSampledMatrixAnimation(FloatList& keys, FMMatrix44List& values);

	/** [INTERNAL] Increments the number of entities target this node.
		To set targets, use the FCDTargetedEntity::SetTarget function. */
	inline void IncrementTargetCount() { ++targetCount; }

	/** [INTERNAL] Decrements the number of entities target this node.
		To set targets, use the FCDTargetedEntity::SetTarget function. */
	inline void DecrementTargetCount() { if (targetCount > 0) --targetCount; }

	/** [INTERNAL] Reads in the visual scene node from a given COLLADA XML tree node.
		@param sceneNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the node.*/
	virtual FUStatus LoadFromXML(xmlNode* sceneNode);

	/** [INTERNAL] Writes out the visual scene node to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the node.
		@return The created XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;

private:
	FUStatus LoadFromExtra();
	void BindMaterial(xmlNode* node);
	void WriteToNodeXML(xmlNode* node, bool isVisualScene) const;
};

#endif // _FCD_SCENE_NODE_
