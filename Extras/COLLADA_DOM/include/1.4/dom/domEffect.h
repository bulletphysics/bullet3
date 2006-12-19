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
#ifndef __domEffect_h__
#define __domEffect_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domAsset.h>
#include <dom/domImage.h>
#include <dom/domFx_profile_abstract.h>
#include <dom/domExtra.h>
#include <dom/domFx_annotate_common.h>
#include <dom/domFx_newparam_common.h>

/**
 * A self contained description of a shader effect.
 */
class domEffect : public daeElement
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::EFFECT; }
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
 *  The effect element may contain an asset element.  @see domAsset
 */
	domAssetRef elemAsset;
/**
 * The annotate element allows you to specify an annotation on this effect.
 * @see domAnnotate
 */
	domFx_annotate_common_Array elemAnnotate_array;
/**
 *  The image element allows you to create image resources which can be shared
 * by multipe profiles.  @see domImage
 */
	domImage_Array elemImage_array;
/**
 * The newparam element allows you to create new effect parameters which can
 * be shared by multipe profiles. @see domNewparam
 */
	domFx_newparam_common_Array elemNewparam_array;
/**
 *  This is the substituion group hook which allows you to swap in other COLLADA
 * FX profiles.  @see domFx_profile_abstract
 */
	domFx_profile_abstract_Array elemFx_profile_abstract_array;
/**
 *  The extra element may appear any number of times.  @see domExtra
 */
	domExtra_Array elemExtra_array;
	/**
	 * Used to preserve order in elements that do not specify strict sequencing of sub-elements.
	 */
	daeElementRefArray _contents;
	/**
	 * Used to preserve order in elements that have a complex content model.
	 */
	daeUIntArray       _contentsOrder;


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
	 * Gets the annotate element array.
	 * @return Returns a reference to the array of annotate elements.
	 */
	domFx_annotate_common_Array &getAnnotate_array() { return elemAnnotate_array; }
	/**
	 * Gets the annotate element array.
	 * @return Returns a constant reference to the array of annotate elements.
	 */
	const domFx_annotate_common_Array &getAnnotate_array() const { return elemAnnotate_array; }
	/**
	 * Gets the image element array.
	 * @return Returns a reference to the array of image elements.
	 */
	domImage_Array &getImage_array() { return elemImage_array; }
	/**
	 * Gets the image element array.
	 * @return Returns a constant reference to the array of image elements.
	 */
	const domImage_Array &getImage_array() const { return elemImage_array; }
	/**
	 * Gets the newparam element array.
	 * @return Returns a reference to the array of newparam elements.
	 */
	domFx_newparam_common_Array &getNewparam_array() { return elemNewparam_array; }
	/**
	 * Gets the newparam element array.
	 * @return Returns a constant reference to the array of newparam elements.
	 */
	const domFx_newparam_common_Array &getNewparam_array() const { return elemNewparam_array; }
	/**
	 * Gets the fx_profile_abstract element array.
	 * @return Returns a reference to the array of fx_profile_abstract elements.
	 */
	domFx_profile_abstract_Array &getFx_profile_abstract_array() { return elemFx_profile_abstract_array; }
	/**
	 * Gets the fx_profile_abstract element array.
	 * @return Returns a constant reference to the array of fx_profile_abstract elements.
	 */
	const domFx_profile_abstract_Array &getFx_profile_abstract_array() const { return elemFx_profile_abstract_array; }
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
	/**
	 * Gets the _contents array.
	 * @return Returns a reference to the _contents element array.
	 */
	daeElementRefArray &getContents() { return _contents; }
	/**
	 * Gets the _contents array.
	 * @return Returns a constant reference to the _contents element array.
	 */
	const daeElementRefArray &getContents() const { return _contents; }

protected:
	/**
	 * Constructor
	 */
	domEffect() : attrId(), attrName(), elemAsset(), elemAnnotate_array(), elemImage_array(), elemNewparam_array(), elemFx_profile_abstract_array(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domEffect() {}
	/**
	 * Copy Constructor
	 */
	domEffect( const domEffect &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domEffect &operator=( const domEffect &cpy ) { (void)cpy; return *this; }

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
