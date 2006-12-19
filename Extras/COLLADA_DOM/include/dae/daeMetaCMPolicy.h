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

#ifndef __DAE_META_CM_POLICY_H__
#define __DAE_META_CM_POLICY_H__

#include <dae/daeTypes.h>
#include <dae/daeElement.h>
//class daeElement;
class daeMetaElement;

/**
 * The daeMetaCMPolicy class is the base class for the content model policy classes which are used to
 * describe the availability and ordering of an element's children.
 */
class daeMetaCMPolicy
{
public:
	/**
	 * Places an element into the parent element based on this content model policy object.
	 * @param parent The parent element for which the child element will be placed.
	 * @param child The new child element.
	 * @param ordinal A reference to a daeUInt which holds the ordinal return value for a placed child. Used
	 * to maintain proper ording of child elements.
	 * @param offset The offset to used when attempting to place this element. Affects comparison against
	 * minOccurs and maxOccurs.
	 * @param before The element that the child should appear before. Optional.
	 * @param after The element that the child should appear after. Optional.
	 * @return Returns The child element that was placed within this content model object or any of its 
	 * children. NULL if placement failed.
	 */
	virtual daeElement *placeElement( daeElement *parent, daeElement *child, daeUInt &ordinal, daeInt offset = 0, daeElement* before = NULL, daeElement *after = NULL ) = 0;
	/**
	 * Removes an element from the parent based on this content model object.
	 * @param parent The parent element for which child you want to remove.
	 * @param child The child that will be removed from the parent.
	 * @return Returns true if the child was successfully removed from this content model object or any of
	 * its children. False otherwise.
	 */
	virtual daeBool removeElement(daeElement* parent, daeElement* child ) = 0;
	/**
	 * Gets the daeMetaElement of an acceptable child of this content model object.
	 * @param elementName The name of the element whos metaElement information you are interested in.
	 * @return Returns a pointer to a daeMetaElement class that describes the element interested in. 
	 * Returns NULL if the element is not valid in this content model.
	 */
	virtual daeMetaElement *findChild( daeString elementName ) = 0;
	/**
	 * Populates an array with the children of parent based on this content model object.
	 * @param parent The parent element whos children you want.
	 * @param array The array where you the children will be appended to.
	 */
	virtual void getChildren( daeElement* parent, daeElementRefArray &array ) = 0;

	/**
	 * Adds a child to this content model object.
	 * @param p The child content model policy object.
	 */
	void appendChild( daeMetaCMPolicy *p ) { _children.append( p ); }

	/**
	 * Gets the parent of this content model policy object.
	 * @return Returns a pointer to the parent node.
	 */
	daeMetaCMPolicy *getParent() { return _parent; }

	/**
	 * Sets the maximum ordinal value of this policy objects children. Used to keep proper ording for
	 * cm objects that may appear multiple times.
	 * @param ord The maximum ordinal value for this content model object.
	 */
	void setMaxOrdinal( daeUInt ord ) { _maxOrdinal = ord; }

protected:
	/**
	 * Constructor.
	 * @param container The daeMetaElement that this policy object belongs to.
	 * @param parent The daeMetaCMPolicy parent of this policy object.
	 * @param odinal The ordinal value offset of this specific policy object. Used for maintaining the 
	 * correct order of child elements.
	 * @param minO The minimum number of times this CMPolicy object must appear. This value comes from the COLLADA schema.
	 * @param maxO The maximum number of times this CMPolicy object may appear. This value comes from the COLLADA schema.
	 */
	daeMetaCMPolicy( daeMetaElement *container ,daeMetaCMPolicy *parent, daeUInt ordinal, 
					 daeInt minO, daeInt maxO ) : _container( container ), _parent( parent ), _minOccurs( minO ), 
					 _maxOccurs( maxO ), _maxOrdinal( 0 ), _ordinalOffset( ordinal ) {}

	/**
	 * Destructor.
	 */
public: // sthomas
	virtual ~daeMetaCMPolicy();
protected: // sthomas

	daeMetaElement * _container;
	daeMetaCMPolicy * _parent;
	daeTArray<daeMetaCMPolicy*> _children;

	/** Minimum number of times this meta element can occur. */
	daeInt _minOccurs;
	/** Maximum number of times this meta element can occur. -1 for unbounded */
	daeInt _maxOccurs;

	daeUInt _maxOrdinal;
	daeUInt _ordinalOffset;

};

#endif

