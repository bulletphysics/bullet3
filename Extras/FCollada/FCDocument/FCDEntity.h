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
	@file FCDEntity.h
	This file contains the FCDEntity class.
*/

#ifndef _FCD_ENTITY_H_
#define _FCD_ENTITY_H_

#include "FCDocument/FCDObject.h"

class FCDocument;
class FCDExtra;

/**
	A COLLADA entity.

	A COLLADA entity is an object contained within a COLLADA library.
	As such, it is based on the FCDObjectWithId class so that it
	can be accessed by other entities, such as the scene graph.

	The entity adds to the FCDObjectWithId class: a name,
	an extra tree and an optional note, as well as a way
	to identity the type of the entity, in order to up-cast it
	to its correct class.

	@ingroup FCDocument
*/

class FCOLLADA_EXPORT FCDEntity : public FCDObjectWithId
{
public:
	/** The types of entity classes.
		Each type corresponds directly to one class that contains the
		FCDEntity class as a parent, so you can up-cast FCDEntity pointers. */
	enum Type
	{
		ENTITY, /**< A generic entity (FCDEntity). Should never be used. */
		ANIMATION, /**< An animation (FCDAnimation). */
		ANIMATION_CLIP, /**< An animation clip (FCDAnimationClip). */
		CAMERA, /**< A camera (FCDCamera). */
		LIGHT, /**< A light (FCDLight). */
		IMAGE, /**< An image (FCDImage). */
		TEXTURE, /**< A texture (FCDTexture). Used for COLLADA 1.3 backward compatibility only! */
		MATERIAL, /**< A visual material definition (FCDMaterial). */
		EFFECT, /**< An effect definition (FCDEffect). */
		GEOMETRY, /**< A geometric object (FCDGeometry). Includes splines and meshes. */
		CONTROLLER, /**< A geometric controller (FCDController). Includes skins and morphers. */
		SCENE_NODE, /**< A visual scene node (FCDSceneNode). */
		PHYSICS_RIGID_CONSTRAINT, /**< A physics rigid constraint (FCDPhysicsRigidConstraint). */
		PHYSICS_MATERIAL, /**< A physics material definiton (FCDPhysicsMaterial). */
		PHYSICS_RIGID_BODY, /**< A physics rigid body (FCDPhysicsRigidBody). */
		PHYSICS_SHAPE, /**< A physics shape (FCDPhysicsShape). */
		PHYSICS_ANALYTICAL_GEOMETRY, /**< A physics analytical geometric object (FCDPhysicsAnalyticalGeometry). */
		PHYSICS_MODEL, /**< A physics model (FCDPhysicsModel). */
		PHYSICS_SCENE_NODE /**< A physics scene node (FCDPhysicsSceneNode). */
	};

private:
	fstring name;

	// Extra information for the entity.
	FCDExtra* extra;

	// Maya and Max both support custom strings for objects.
	fstring note;

	// Deprecated ColladaMaya post-processing information.
	StringList postCmds;

public:
	/** Constructor: do not use directly.
		Instead, create objects of the up-classes.
		@param document The COLLADA document that owns the entity.
		@param baseId The prefix COLLADA id to be used if no COLLADA id is provided. */
	FCDEntity(FCDocument* document, const char* baseId = "GenericEntity");

	/** Destructor: do not use directly.
		Instead, release objects through their libraries or their parent entities. */
	virtual ~FCDEntity();

	/** Retrieves the entity class type for an entity.
		You can use the entity class type of up-cast an entity pointer
		to the correct up-class.
		This function should be overwritten by all up-classes.
		@return The entity class type. */
	virtual Type GetType() const { return ENTITY; }

	/** Retrieves the name of the entity.
		This value has no direct use in COLLADA but is useful
		to track the user-friendly name of an entity.
		@return The name. */
	const fstring& GetName() const { return name; }

	/** Sets the name of the entity.
		This value has no direct use in COLLADA but is useful
		to track the user-friendly name of an entity.
		@param _name The name. */
	void SetName(const fstring& _name);

	/** Retrieves the extra information tree for this entity.
		The prefered way to save extra information in FCollada is at
		the entity level. Use this extra information tree to store
		any information you want exported and imported back.
		@return The extra information tree. */
	FCDExtra* GetExtra() { return extra; }
	const FCDExtra* GetExtra() const { return extra; } /**< See above. */

	/** Retrieves whether the entity has a user-defined note.
		This value is a simpler way, than the extra tree, to store
		user-defined information that does not belong in COLLADA.
		@return Whether the entity has an user-defined note. */
	bool HasNote() const { return !note.empty(); }

	/** Retrieves the user-defined note for this entity.
		This value is a simpler way, than the extra tree, to store
		user-defined information that does not belong in COLLADA.
		@return The user-defined note. */
	const fstring& GetNote() const { return note; }

	/** Sets the user-defined note for this entity.
		This value is a simpler way, than the extra tree, to store
		user-defined information that does not belong in COLLADA.
		@param _note The user-defined note. */
	void SetNote(const fstring& _note) { note = _note; }

	/** Retrieves the child entity that has the given COLLADA id.
		This function is only useful for entities that are hierarchical:
		visual/physics scene nodes and animations.
		@param daeId A COLLADA id.
		@return The child entity with the given id. This pointer will be NULL
			if no child entity matches the given id. */
	virtual FCDEntity* FindDaeId(const string& daeId);

	/** [INTERNAL] Reads in the entity from a given COLLADA XML tree node.
		This function should be overwritten by all up-classes.
		@param entityNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the entity.*/
	virtual FUStatus LoadFromXML(xmlNode* entityNode);

	/** [INTERNAL] Writes out the entity to the given COLLADA XML tree node.
		This function should be overwritten by all up-classes.
		@param parentNode The COLLADA XML parent node in which to insert the entity.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;

	/** @deprecated Retrieves the like of post-processing commands.
		Used only in ColladaMaya and should be taken out.
		@return The list of post-processing commands. */
	StringList& GetPostProcessCmds() { return postCmds; }

protected:
	/** [INTERNAL] Writes out the top entity XML node for the entity.
		This function should be used by all up-classes within the
		WriteToXML overwritting function to create the top XML node,
		as it will write out the name and COLLADA id of the entity.
		@param parentNode The COLLADA XML parent node in which to insert the entity.
		@param nodeName The COLLADA XML node name for the top entity XML node.
		@return The created element XML tree node. */
	xmlNode* WriteToEntityXML(xmlNode* parentNode, const char* nodeName) const;

	/** [INTERNAL] Writes out the extra information for the entity.
		This function should be used by all up-classes within the
		WriteToXML overwritting function, at the very end, to write
		the user-defined note and the extra tree to the COLLADA document.
		@param entityNode The created element XML tree node returned
			by the WriteToEntityXML function. */
	void WriteToExtraXML(xmlNode* entityNode) const;

	/** [INTERNAL] Copies the entity information into a cloned entity.
		This function should be used by all up-classes when cloning an entity
		to copy the COLLADA id and the other entity-level information into a clone.
		@param clone The cloned entity. */
	void Clone(FCDEntity* clone);
};

#endif // _FCD_ENTITY_H_

