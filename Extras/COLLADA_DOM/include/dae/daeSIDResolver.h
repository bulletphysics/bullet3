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

#ifndef __DAE_SIDRESOLVER_H__
#define __DAE_SIDRESOLVER_H__

#include <dae/daeTypes.h>
#include <dae/daeElement.h>

/**
 * The daeSIDResolver class is designed to resolve sid references within a COLLADA document.
 * The rules for sid resolution are set forth by the Addressing Syntax section in Chapter 3 of the
 * COLLADA specification which can be found at https://www.khronos.org/collada .
 * This resolver always attempts to resolve to the daeElement which is referenced. If the element contains 
 * a daeDoubleArray (domFloatArray) value, the resolver will set the pointer to that array. The
 * resolver will also do this if the sid target points to a <source> element which has a <float_array> as
 * a child. If the sid target specifies a value, i.e. blah.X or blah(6), the resolver will attempt to
 * get a pointer to that specific value. The resolver only attempts to resolve to that level for values which
 * are defined in the COMMON profile glossary of the COLLADA specification, or values reference with the (#)
 * syntax. You can check the return value from getState() to see which level of resolution is possible.
 */
class daeSIDResolver
{
public:
	/**
	 * An enum describing the status of the SID resolution process.
	 */
	enum ResolveState{
		/** No target specified */
		target_empty,
		/** target specified but not resolved */
		target_loaded,
		/** Resolution failed because target was not found */
		sid_failed_not_found,
		/** Resolution successful to the Element level */
		sid_success_element,
		/** Resolution successful to the Double Array level */
		sid_success_array,
		/** Resolution successful to the Double level */
		sid_success_double
	};

	/**
	 * Constructor.
	 * @param container The element which contains the target that you want to resolve.
	 * @param target The target string which needs to be resolved.
	 * @param platform The platform name of the technique to use. A NULL value indicates the common platform.
	 */
	daeSIDResolver( daeElement *container, daeString target, daeString platform = NULL );

	/**
	 * Destructor.
	 */
	~daeSIDResolver();

	/**
	 * Gets the target string.
	 * @return Returns the target string of this SID resolver.
	 */
	daeString getTarget() const			{ return target; }
	/**
	 * Sets the target string.
	 * @param t The new target string for this resolver.
	 */
	void setTarget( daeString t );

	/**
	 * Gets the name of the profile to use when resolving.
	 * @return Returns the name of the profile or NULL for the common profile.
	 */
	daeString getProfile()	const		{ return profile; }
	/**
	 * Sets the profile to use when resolving.
	 * @param p The profile name of the technique to use. A NULL value indicates the common profile.
	 */
	void setProfile( daeString p );

	/**
	 * Gets a pointer to the @c daeElement that contains the target to resolve.
	 * @return Returns the pointer to the containing daeElmement.
	 */
	daeElement* getContainer() const	{ return container; }
	/**
	 * Sets the pointer to the @c daeElement that contains the target to resolve.
	 * @param element Pointer to the containing @c daeElmement.
	 */
	void setContainer(daeElement* element);

	/**
	 * Gets the resolution state.
	 * @return Returns the current state of SID resolution.
	 */
	ResolveState getState() const		{ return state; }

	/**
	 * Gets the element that this SID resolves to.
	 * @return Returns the element that the URI resolves to.
	 */
	daeElementRef getElement();

	/**
	 * Gets the value array of the element that the SID resolves to.
	 * @return Returns a pointer to the value array that the SID resolves to
	 * @note The daeSIDResolver can only resolve to this level for daeDoubleArray values.
	 */
	daeDoubleArray *getDoubleArray();

	/**
	 * Gets a pointer to the particle this target resolved to.
	 * @return Returns a pointer to a double value which is the fully resolved target.
	 * @note The daeSIDResolver can only resolve to this level for domDouble values and only if the
	 * final symbolic name is from the COMMON profile or a cardinal value is specified.
	 * @note The daeSIDResolver assumes the value is a 4x4 matrix if there are 2 cardinal values specified.
	 */
	daeDouble *getDouble();

private:
	
	void resolve();
	/**
	 * Recursive function which will find an element with specified sid in the subtree of el.
	 */
	daeElement *findSID( daeElement *el, daeString sid );

private:	
	
	daeString		target;
	daeString		profile;	
	daeElement		*container;
	ResolveState	state;

	daeElement		*element;
	daeDoubleArray	*doubleArray;
	daeDouble		*doublePtr;

};

#endif

