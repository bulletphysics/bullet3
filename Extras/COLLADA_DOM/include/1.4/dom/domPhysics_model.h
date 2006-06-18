/*
 * Copyright 2006 Sony Computer Entertainment Inc.
 *
 * Licensed under the SCEA Shared Source License, Version 1.0 (the "License"); you may not use this 
 * file except in compliance with the License. You may obtain a copy of the License at:
 * http://research.scea.com/scea_shared_source_license.html
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License 
 * is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or 
 * implied. See the License for the specific language governing permissions and limitations under the 
 * License. 
 */
#ifndef __domPhysics_model_h__
#define __domPhysics_model_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domAsset.h>
#include <dom/domRigid_body.h>
#include <dom/domRigid_constraint.h>
#include <dom/domInstance_physics_model.h>
#include <dom/domExtra.h>

/**
 * This element allows for building complex combinations of rigid-bodies and
 * constraints that  may be instantiated multiple times.
 */
class domPhysics_model : public daeElement
{
protected:  // Attributes
/**
 *  The id attribute is a text string containing the unique identifier of
 * this element.  This value must be unique within the instance document.
 * Optional attribute. 
 */
	xsID attrId;
/**
 *  The name attribute is the text string name of this element. Optional attribute.
 */
	xsNCName attrName;

protected:  // Elements
/**
 *  The physics_model element may contain an asset element.  @see domAsset
 */
	domAssetRef elemAsset;
/**
 *  The physics_model may define any number of rigid_body elements.  @see
 * domRigid_body
 */
	domRigid_body_Array elemRigid_body_array;
/**
 *  The physics_model may define any number of rigid_constraint elements.
 * @see domRigid_constraint
 */
	domRigid_constraint_Array elemRigid_constraint_array;
/**
 *  The physics_model may instance any number of other physics_model elements.
 * @see domInstance_physics_model
 */
	domInstance_physics_model_Array elemInstance_physics_model_array;
/**
 *  The extra element may appear any number of times.  @see domExtra
 */
	domExtra_Array elemExtra_array;

public:	//Accessors and Mutators
	/**
	 * Gets the id attribute.
	 * @return Returns a xsID of the id attribute.
	 */
	xsID getId() const { return attrId; }
	/**
	 * Sets the id attribute.
	 * @param atId The new value for the id attribute.
	 */
	void setId( xsID atId ) { attrId = atId; }

	/**
	 * Gets the name attribute.
	 * @return Returns a xsNCName of the name attribute.
	 */
	xsNCName getName() const { return attrName; }
	/**
	 * Sets the name attribute.
	 * @param atName The new value for the name attribute.
	 */
	void setName( xsNCName atName ) { attrName = atName; }

	/**
	 * Gets the asset element.
	 * @return a daeSmartRef to the asset element.
	 */
	const domAssetRef getAsset() const { return elemAsset; }
	/**
	 * Gets the rigid_body element array.
	 * @return Returns a reference to the array of rigid_body elements.
	 */
	domRigid_body_Array &getRigid_body_array() { return elemRigid_body_array; }
	/**
	 * Gets the rigid_body element array.
	 * @return Returns a constant reference to the array of rigid_body elements.
	 */
	const domRigid_body_Array &getRigid_body_array() const { return elemRigid_body_array; }
	/**
	 * Gets the rigid_constraint element array.
	 * @return Returns a reference to the array of rigid_constraint elements.
	 */
	domRigid_constraint_Array &getRigid_constraint_array() { return elemRigid_constraint_array; }
	/**
	 * Gets the rigid_constraint element array.
	 * @return Returns a constant reference to the array of rigid_constraint elements.
	 */
	const domRigid_constraint_Array &getRigid_constraint_array() const { return elemRigid_constraint_array; }
	/**
	 * Gets the instance_physics_model element array.
	 * @return Returns a reference to the array of instance_physics_model elements.
	 */
	domInstance_physics_model_Array &getInstance_physics_model_array() { return elemInstance_physics_model_array; }
	/**
	 * Gets the instance_physics_model element array.
	 * @return Returns a constant reference to the array of instance_physics_model elements.
	 */
	const domInstance_physics_model_Array &getInstance_physics_model_array() const { return elemInstance_physics_model_array; }
	/**
	 * Gets the extra element array.
	 * @return Returns a reference to the array of extra elements.
	 */
	domExtra_Array &getExtra_array() { return elemExtra_array; }
	/**
	 * Gets the extra element array.
	 * @return Returns a constant reference to the array of extra elements.
	 */
	const domExtra_Array &getExtra_array() const { return elemExtra_array; }
protected:
	/**
	 * Constructor
	 */
	domPhysics_model() : attrId(), attrName(), elemAsset(), elemRigid_body_array(), elemRigid_constraint_array(), elemInstance_physics_model_array(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domPhysics_model() {}
	/**
	 * Copy Constructor
	 */
	domPhysics_model( const domPhysics_model &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domPhysics_model &operator=( const domPhysics_model &cpy ) { (void)cpy; return *this; }

public: // STATIC METHODS
	/**
	 * Creates an instance of this class and returns a daeElementRef referencing it.
	 * @param bytes The size allocated for this instance.
	 * @return a daeElementRef referencing an instance of this object.
	 */
	static daeElementRef create(daeInt bytes);
	/**
	 * Creates a daeMetaElement object that describes this element in the meta object reflection framework.
	 * If a daeMetaElement already exists it will return that instead of creating a new one. 
	 * @return A daeMetaElement describing this COLLADA element.
	 */
	static daeMetaElement* registerElement();

public: // STATIC MEMBERS
	/**
	 * The daeMetaElement that describes this element in the meta object reflection framework.
	 */
	static daeMetaElement* _Meta;
};


#endif
