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
#ifndef __domInstance_rigid_constraint_h__
#define __domInstance_rigid_constraint_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domExtra.h>

/**
 * This element allows instancing a rigid_constraint within an instance_physics_model.
 */
class domInstance_rigid_constraint : public daeElement
{
protected:  // Attribute
/**
 *  The constraint attribute indicates which rigid_constraing to instantiate.
 * Required attribute. 
 */
	xsNCName attrConstraint;

protected:  // Element
/**
 *  The extra element may appear any number of times.  @see domExtra
 */
	domExtra_Array elemExtra_array;

public:	//Accessors and Mutators
	/**
	 * Gets the constraint attribute.
	 * @return Returns a xsNCName of the constraint attribute.
	 */
	xsNCName getConstraint() const { return attrConstraint; }
	/**
	 * Sets the constraint attribute.
	 * @param atConstraint The new value for the constraint attribute.
	 */
	void setConstraint( xsNCName atConstraint ) { attrConstraint = atConstraint; }

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
	domInstance_rigid_constraint() : attrConstraint(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domInstance_rigid_constraint() {}
	/**
	 * Copy Constructor
	 */
	domInstance_rigid_constraint( const domInstance_rigid_constraint &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domInstance_rigid_constraint &operator=( const domInstance_rigid_constraint &cpy ) { (void)cpy; return *this; }

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
