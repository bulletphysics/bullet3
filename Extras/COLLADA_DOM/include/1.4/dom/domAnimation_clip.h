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
#ifndef __domAnimation_clip_h__
#define __domAnimation_clip_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domAsset.h>
#include <dom/domExtra.h>
#include <dom/domInstanceWithExtra.h>

/**
 * The animation_clip element defines a section of the animation curves to
 * be used together as  an animation clip.
 */
class domAnimation_clip : public daeElement
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
/**
 *  The start attribute is the time in seconds of the beginning of the clip.
 * This time is  the same as that used in the key-frame data and is used to
 * determine which set of  key-frames will be included in the clip.  The start
 * time does not specify when the clip  will be played.  If the time falls
 * between two keyframes of a referenced animation, an  interpolated value
 * should be used.  The default value is 0.0.  Optional attribute. 
 */
	xsDouble attrStart;
/**
 *  The end attribute is the time in seconds of the end of the clip.  This
 * is used in the  same way as the start time.  If end is not specified, the
 * value is taken to be the end  time of the longest animation.  Optional
 * attribute. 
 */
	xsDouble attrEnd;

protected:  // Elements
/**
 *  The animation_clip element may contain an asset element.  @see domAsset
 */
	domAssetRef elemAsset;
/**
 * The animation_clip must instance at least one animation element. @see domInstance_animation
 */
	domInstanceWithExtra_Array elemInstance_animation_array;
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
	 * Gets the start attribute.
	 * @return Returns a xsDouble of the start attribute.
	 */
	xsDouble getStart() const { return attrStart; }
	/**
	 * Sets the start attribute.
	 * @param atStart The new value for the start attribute.
	 */
	void setStart( xsDouble atStart ) { attrStart = atStart;
	 _validAttributeArray[2] = true; }

	/**
	 * Gets the end attribute.
	 * @return Returns a xsDouble of the end attribute.
	 */
	xsDouble getEnd() const { return attrEnd; }
	/**
	 * Sets the end attribute.
	 * @param atEnd The new value for the end attribute.
	 */
	void setEnd( xsDouble atEnd ) { attrEnd = atEnd;
	 _validAttributeArray[3] = true; }

	/**
	 * Gets the asset element.
	 * @return a daeSmartRef to the asset element.
	 */
	const domAssetRef getAsset() const { return elemAsset; }
	/**
	 * Gets the instance_animation element array.
	 * @return Returns a reference to the array of instance_animation elements.
	 */
	domInstanceWithExtra_Array &getInstance_animation_array() { return elemInstance_animation_array; }
	/**
	 * Gets the instance_animation element array.
	 * @return Returns a constant reference to the array of instance_animation elements.
	 */
	const domInstanceWithExtra_Array &getInstance_animation_array() const { return elemInstance_animation_array; }
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
	domAnimation_clip() : attrId(), attrName(), attrStart(), attrEnd(), elemAsset(), elemInstance_animation_array(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domAnimation_clip() {}
	/**
	 * Copy Constructor
	 */
	domAnimation_clip( const domAnimation_clip &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domAnimation_clip &operator=( const domAnimation_clip &cpy ) { (void)cpy; return *this; }

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
