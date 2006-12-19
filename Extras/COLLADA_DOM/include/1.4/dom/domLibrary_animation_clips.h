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
#ifndef __domLibrary_animation_clips_h__
#define __domLibrary_animation_clips_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domAsset.h>
#include <dom/domAnimation_clip.h>
#include <dom/domExtra.h>

/**
 * The library_animation_clips element declares a module of animation_clip
 * elements.
 */
class domLibrary_animation_clips : public daeElement
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::LIBRARY_ANIMATION_CLIPS; }
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
 *  The library_animation_clips element may contain an asset element.  @see
 * domAsset
 */
	domAssetRef elemAsset;
/**
 *  There must be at least one animation_clip element.  @see domAnimation_clip
 */
	domAnimation_clip_Array elemAnimation_clip_array;
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
	void setId( xsID atId ) { *(daeStringRef*)&attrId = atId;
	 _validAttributeArray[0] = true; }

	/**
	 * Gets the name attribute.
	 * @return Returns a xsNCName of the name attribute.
	 */
	xsNCName getName() const { return attrName; }
	/**
	 * Sets the name attribute.
	 * @param atName The new value for the name attribute.
	 */
	void setName( xsNCName atName ) { *(daeStringRef*)&attrName = atName;
	 _validAttributeArray[1] = true; }

	/**
	 * Gets the asset element.
	 * @return a daeSmartRef to the asset element.
	 */
	const domAssetRef getAsset() const { return elemAsset; }
	/**
	 * Gets the animation_clip element array.
	 * @return Returns a reference to the array of animation_clip elements.
	 */
	domAnimation_clip_Array &getAnimation_clip_array() { return elemAnimation_clip_array; }
	/**
	 * Gets the animation_clip element array.
	 * @return Returns a constant reference to the array of animation_clip elements.
	 */
	const domAnimation_clip_Array &getAnimation_clip_array() const { return elemAnimation_clip_array; }
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
	domLibrary_animation_clips() : attrId(), attrName(), elemAsset(), elemAnimation_clip_array(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domLibrary_animation_clips() {}
	/**
	 * Copy Constructor
	 */
	domLibrary_animation_clips( const domLibrary_animation_clips &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domLibrary_animation_clips &operator=( const domLibrary_animation_clips &cpy ) { (void)cpy; return *this; }

public: // STATIC METHODS
	/**
	 * Creates an instance of this class and returns a daeElementRef referencing it.
	 * @param bytes The size allocated for this instance.
	 * @return a daeElementRef referencing an instance of this object.
	 */
	static DLLSPEC daeElementRef create(daeInt bytes);
	/**
	 * Creates a daeMetaElement object that describes this element in the meta object reflection framework.
	 * If a daeMetaElement already exists it will return that instead of creating a new one. 
	 * @return A daeMetaElement describing this COLLADA element.
	 */
	static DLLSPEC daeMetaElement* registerElement();

public: // STATIC MEMBERS
	/**
	 * The daeMetaElement that describes this element in the meta object reflection framework.
	 */
	static DLLSPEC daeMetaElement* _Meta;
};


#endif
