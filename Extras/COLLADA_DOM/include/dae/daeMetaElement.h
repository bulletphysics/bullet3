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

#ifndef __DAE_META_ELEMENT_H__
#define __DAE_META_ELEMENT_H__

#include <dae/daeTypes.h>
#include <dae/daeArrayTypes.h>
#include <dae/daeElement.h>
#include <dae/daeMetaAttribute.h>

typedef daeElementRef (*daeElementConstructFunctionPtr)(daeInt bytes);
class daeMetaElement;
typedef daeSmartRef<daeMetaElement> daeMetaElementRef;
typedef daeTArray<daeMetaElementRef> daeMetaElementRefArray;

/**
 * Each instance of the @c daeMetaElement class describes a C++ COLLADA dom
 * element type.
 * @par
 * The meta information in @c daeMetaElement is a combination of the information
 * required to create and maintain C++ object instances and
 * the information necessary to parse and construct a hierarchy of COLLADA
 * elements.
 * @par
 * @c daeMetaElement objects also act as factories for C++ COLLADA dom classes where
 * each @c daeElement is capable of creating an instance of the class it describes.
 * Further, each @c daeMetaElement contains references to other @c daeMetaElements
 * for potential XML children elements.  This enables this system to easily
 * create @c daeElements of the appropriate type while navigating through XML
 * recursive parse.
 * @par
 * See @c daeElement for information about the functionality that every @c daeElement implements.
 */
class daeMetaElement : public daeElement
{
protected:
	daeStringRef					_name;

	daeElementConstructFunctionPtr	_createFunc;
	daeInt							_minOccurs;
	daeInt							_maxOccurs;
	daeStringRef					_ref;
	daeBool							_isSequence;
	daeBool							_isChoice;
	daeBool							_needsResolve;
	daeInt							_elementSize;
	
	daeMetaElementAttributeArray		_metaElements;
	daeMetaAttributeRefArray			_metaAttributes;
	daeMetaAttributeRef					_metaValue;
	daeMetaElementArrayAttribute*		_metaContents;
	daeMetaElement *					_metaIntegration;
	daeMetaAttributeRef					_metaID;

	daeMetaElement*						_parent;

	daeMetaElement**					_staticPointerAddress;

	daeMetaAttributePtrArray			_resolvers;

	daeBool								_isTrackableForQueries;
	daeBool								_usesStringContents;

	daeBool							_isTransparent;
	daeBool							_isAbstract;
	daeBool							_allowsAny;
	
	static daeMetaElementRefArray _metas;

	daeStringArray						_otherChildren;
	daeStringArray						_otherChildrenTypes;
	daeTArray<daeMetaElementAttribute*> _otherChildrenContainer;
	

public:
	/**
	 * Constructor
	 */
	daeMetaElement();

	/**
	 * Destructor
	 */
	~daeMetaElement();

public: // public accessors
	/**
	 * Gets the number of possible children of elements of this type that don't actually
	 * belong to this type.
	 * @return Returns the number of other possible children.
	 */
	size_t getPossibleChildrenCount() { return _otherChildren.getCount(); }
	/**
	 * Gets the name of the possible child specified.
	 * @param index Index into the _otherChildren array.
	 * @return Returns the name of the possible child specified.
	 */
	daeString getPossibleChildName(daeInt index) { return _otherChildren.get(index); }
	/**
	 * Gets the containing element for the possible child specified.
	 * @param index Index into the _otherChildrenContainer array.
	 * @return Returns the containing element for the possible child specified.
	 */
	daeMetaElementAttribute* getPossibleChildContainer(daeInt index) { return _otherChildrenContainer.get(index); }
	/**
	 * Gets the type of the possible child specified.
	 * @param index Index into the _otherChildren array.
	 * @return Returns a string of the type of the possible child specified.
	 */
	daeString getPossibleChildType(daeInt index) { return _otherChildrenTypes.get(index); }

	/**
	 * Determines if elements of this type can be placed in the object model.
	 * @return Returns true if this element type is abstract, false otherwise.
	 */
	daeBool getIsAbstract() { return _isAbstract; }
	/**
	 * Determines if elements of this type should have an element tag printed when saving.
	 * @return Returns true if this element type should not have a tag, false otherwise.
	 */
	daeBool getIsTransparent() { return _isTransparent; }
	/**
	 * Sets if elements of this type are abstract.
	 * @param abstract True if this type is abstract.
	 */
	void setIsAbstract( daeBool abstract ) { _isAbstract = abstract; }
	/**
	 * Sets whether or not elements of this type should have an element tag printed when saving.
	 * @param transparent True if this type is transparent.
	 */
	void setIsTransparent( daeBool transparent ) { _isTransparent = transparent; }

	/**
	 * Determines if elements of this type should be tracked
	 * for daeDatabase queries.
	 * @return Returns true if this element type should be tracked
	 */
	daeBool getIsTrackableForQueries() { return _isTrackableForQueries; }

	/**
	 * Gets whether elements of this type have "string" based
	 * contents; this is necessary to change the parsing mode for strings.
	 * @return Returns true if this element type has string contents, false if not.
	 */
	daeBool getUsesStringContents() { return _usesStringContents; }

	/**
	 * Sets whether elements of this type should be tracked
	 * for @c daeDatabase queries.
	 * @param trackable Indicates whether this element should be tracked.  
	 * A value of true indicates this element type should be tracked and be available for
	 * database queries.
	 */
	void setIsTrackableForQueries(daeBool trackable) {
		_isTrackableForQueries = trackable; }
		
	/**
	 * Determines if elements of this type allow for any element as a child.
	 * @return Returns true if this element can have any child element, false otherwise.
	 */
	daeBool getAllowsAny() { return _allowsAny; }
	/**
	 * Sets if elements of this type allow for any element as a child.
	 * @param allows True if this element allows for any child element, false otherwise.
	 */
	void setAllowsAny( daeBool allows ) { _allowsAny = allows; }

	/**
	 * Gets the @c daeMetaElement for the corresponding integration object
	 * associated with this COLLADA element (if any).
	 * @return Returns the @c daeMetaElement for the integration object; this can
	 * be used as a factory.
	 */
	daeMetaElement* getMetaIntegration() { return _metaIntegration; }

	/**
	 * Sets the @c daeMetaElement for the corresponding integration object
	 * associated with this COLLADA element (if any).
	 * @param mI @c daeMetaElement for the integration object; this is
	 * used as a factory to automatically create this integration object
	 * whenever an instance of this element is created.
	 */
	void setMetaIntegration(daeMetaElement* mI) { _metaIntegration = mI; }

	/**
	 * Gets the @c daeMetaAttribute for the non-element contents of a @c daeElement.
	 * This corresponds to a @c daeMetaFloatAttribute, @c daeMetaFloatArrayAttribute,
	 * et cetera.
	 * @return Returns the @c daeMetaAttribute pointer for the non-element contents of
	 * this element type.
	 */
	daeMetaAttribute* getValueAttribute() { return _metaValue; }

	/**
	 * Gets the @c daeMetaAttribute for the ID attribute of a @c daeElement.
	 * @return Returns the ID @c daeMetaAttribute, or NULL if the element type
	 * does not have an ID attribute.
	 */
	daeMetaAttribute* getIDAttribute() { return _metaID; }

	/**
	 * Gets the @c daeMetaElement associated with a child element of a given
	 * element type.
	 * @param elementName Name of the element to find as a child of @c this.
	 * @return Returns the @c daeMetaElement describing the potential child element, or
	 * NULL if no such child type exists in the context of this element.
	 */
	daeMetaElement* findChild(daeString elementName);

	/**
	 * Gets the container of this element type as defined by the COLLADA's XML
	 * schema.  This parent type controls where this element
	 * can be directly inlined inside of another element.
	 * Although an element can be referred to in multiple places, it is only
	 * included in one; thus a single parent.
	 * @return Returns the parent @c daeMetaElement.
	 */
	daeMetaElement* getParent() { return _parent; }

	/**
	 * Gets the name of this element type.
	 * @return Returns the name of this element type.
	 */
	daeStringRef getName() { return _name; } 

	/**
	 * Sets the name of this element type.
	 * @param s String name	to set.
	 */
	void setName(daeString s) { _name = s; }

	/**
	 * Gets the array of element attributes associated with this element type.
	 * @return  Returns the array of potential child elements in the XML COLLADA
	 * hierarchy.
	 */
	daeMetaElementAttributeArray& getMetaElements() {
		return _metaElements; }

	/**
	 * Gets the array of attributes that represent URI fields that need
	 * to be "resolved" after the database is completely read in.
	 * @return Returns the array of @c daeMetaAttribute*'s with all of the relevant
	 * attributes.
	 */
	daeMetaAttributePtrArray& getMetaResolvers() {
		return _resolvers; }

	/**
	 * Gets the array of all known attributes on this element type.
	 * This includes all meta attributes except those describing child
	 * elements. It does include the value element.
	 * @return Returns the array of @c daeMetaAttributeRefs.
	 */
	daeMetaAttributeRefArray& getMetaAttributes() {
		return _metaAttributes; }

	/**
	 * Gets the array of element attributes associated with this element type.
	 * @returns Returns the array of potential child elements in the XML COLLADA
	 * hierarchy.
	 */
	daeMetaElementAttributeArray& getMetaElementArray() {
		return _metaElements; }

	/**
	 * Gets the attribute which has a name as provided by the <tt><i>s</i></tt> parameter. 
	 * @param s String containing the  desired attribute's name.
	 * @return Returns the corresponding @c daeMetaAttribute, or NULL if none found.
	 */
	daeMetaAttribute* getMetaAttribute(daeString s);

	/**
	 * Sets the size in bytes of each instance of this element type.
	 * Used for factory element creation.
	 * @param size Number of bytes for each C++ element instance.
	 */
	void setElementSize(daeInt size) {_elementSize = size;}

	/**
	 * Gets the size in bytes of each instance of this element type.
	 * Used for factory element creation.
	 * @return Returns the number of bytes for each C++ element instance.
	 */
	daeInt getElementSize() { return _elementSize;}
	
public: 
	/**
	 * Resisters with the reflective object system that the dom class described by this @c daeMetaElement
	 * contains a <tt><i>_contents</i></tt> array. This method is @em only for @c daeMetaElement contstuction, and
	 * should only be called by the system as it sets up the Reflective Object System.
	 * @param offset Byte offset for the contents field in the C++
	 * element class.
	 */
	void addContents(daeInt offset);

	/**
	 * Gets the attribute associated with the contents meta information.
	 * @see @c addContents()
	 * @return Returns the @c daeMetaElementArrayAttribute.
	 */
	daeMetaElementArrayAttribute* getContents() { return _metaContents; }

	/**
	 * Appends another element type to be a potential child
	 * element of this element type.
	 * @param metaElement @c daeMetaElement of the potential child element.
	 * @param offset Byte offset where the corresponding C++ field lives
	 * in each c++ class instance for this element type.
	 * @param name The name for this attribute if the type is complex, if none is
	 * specified, the name of the @c daeMetaElement will be used.
	 */
	void appendElement(daeMetaElement* metaElement, daeInt offset, daeString name=NULL);

	/**
	 * Appends the potential child element
	 * as a list of potential child elements rather than as a singleton.
	 * @param metaElement @c daeMetaElement of the potential child element.
	 * @param offset Byte offset where the corresponding C++ field lives
	 * in each C++ class instance for this element type.  In this case the
	 * C++ field will be an array of elements rather than merely a pointer to
	 * one.
	 * @param name The name for this attribute if the type is complex, if none is
	 * specified, the name of the metaElement will be used.
	 * @note This function is the same as @c appendElement(), except that it appends the potential child element
	 * as a list of potential child elements rather than as a singleton.
	 */
	void appendArrayElement(daeMetaElement* metaElement, daeInt offset, daeString name=NULL);

	/**
	 * Appends a @c daeMetaAttribute that represents a field corresponding to an
	 * XML attribute to the C++ version of this element type.
	 * @param attr Attribute to append to this element types list
	 * of potential attributes.
	 */
	void appendAttribute(daeMetaAttribute* attr);

	/**
	 * Appends a possible child and maps the name to the actual container.
	 * @param name The name of the child element.
	 * @param cont Pointer to the @c daeMetaElementAttribute which contains the element.
	 * @param type The type name of the possible child.
	 */
	void appendPossibleChild( daeString name, daeMetaElementAttribute* cont, daeString type = NULL );

	/**
	 * Sets the address where the static pointer lives for this element type's
	 * @c daeMetaElement.  For instance, <tt> daeNode::_Meta </tt> will point to its 
	 * corresponding @c daeMetaElement.
	 * If the @c daeMetaElement is deleted independently, this pointer is automatically set to NULL.
	 * @param addr Address of the storage for the pointer to the @c daeMetaElement.
	 */
	void setStaticPointerAddress(daeMetaElement** addr) {
		_staticPointerAddress = addr; }

	
	/**
	 * Gets the address where the static pointer lives for this element type's
	 * @c daeMetaElement.  For instance, <tt> daeNode::_Meta </tt> will point to its 
	 * corresponding @c daeMetaElement.
	 * If the @c daeMetaElement is deleted independently, this pointer is automatically set to NULL.
	 * @return Returns the address of the storage for the pointer to the @c daeMetaElement.
	 */
	daeMetaElement** getStaticPointerAddress() { return _staticPointerAddress;}

	/**
	 * Registers the function that can construct a C++ instance
	 * of this class.  Necessary for the factory system such that C++
	 * can still call @c new and the @c vptr will still be initialized even when
	 * constructed via the factory system.
	 * @param func Pointer to a function that does object construction.
	 */
	void registerConstructor(daeElementConstructFunctionPtr func) {
		_createFunc = func; }

	/**
	 * Determines if this element contains attributes
	 * of type @c daeURI which need to be resolved after they are read
	 * or setup.
	 * @return Returns true if this element type requires resolving, false if not.
	 */
	daeBool needsResolve() { return _needsResolve; }

	/**
	 * Validates this class to be used by the runtime c++ object model
	 * including factory creation.
	 */
	void validate();
	/**
	 * Places a child element into the <tt><i>parent</i></tt> element where the
	 * calling object is the @c daeMetaElement for the parent element.
	 * @param parent Element to act as the container.
	 * @param child Child element to place in the parent.
	 * @return Returns true if the operation was successful, false otherwise.
	 */
	daeBool place(daeElementRef parent, daeElementRef child);

	/**
	 * Invokes the factory element creation routine set by @c registerConstructor() 
	 * to return a C++ COLLADA Object Model instance of this element type.
	 * @return Returns a created @c daeElement of appropriate type via the
	 * object creation function and the <tt> daeElement::setup() </tt> function.
	 */
	daeElementRef create();

	/**
	 * Looks through the list of potential child elements
	 * for this element type finding the corresponding element type; if a corresponding element type 
	 * is found, use that type as a factory and return an instance of that
	 * child type.  Typically @c place() is called after @c create(childelementname)
	 * @param childElementTypeName Type name to create.
	 * @return Returns the created element if the type was found as a potential child element.
	 */
	daeElementRef create(daeString childElementTypeName);

	/**
	 * Gets the meta information for a given subelement
	 * @param s Name of the child element type to look up.
	 * @return Returns the meta information for a given subelement.
	 */
	daeMetaElement* getChildMetaElement(daeString s);

	/**
	 * Gets the meta information for a given subelement
	 * @param s Name of the child element type to look up.
	 * @return Returns the meta information for a given subelement.
	 */
	daeMetaElementAttribute* getChildMetaElementAttribute(daeString s);

public:
	/**
	 * Unused
	 */
	static daeMetaElement* _Schema;
public:
	/**
	 * Empty no-op function.
	 */
	static void initializeSchemaMeta();
	
	/**
	 * Releases all of the meta information contained in @c daeMetaElements.
	 */
	static void releaseMetas();
};
#endif //__DAE_META_ELEMENT_H__





