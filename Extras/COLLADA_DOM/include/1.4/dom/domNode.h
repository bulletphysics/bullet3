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
#ifndef __domNode_h__
#define __domNode_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domAsset.h>
#include <dom/domLookat.h>
#include <dom/domMatrix.h>
#include <dom/domRotate.h>
#include <dom/domScale.h>
#include <dom/domSkew.h>
#include <dom/domTranslate.h>
#include <dom/domInstance_camera.h>
#include <dom/domInstance_controller.h>
#include <dom/domInstance_geometry.h>
#include <dom/domInstance_light.h>
#include <dom/domInstance_node.h>
#include <dom/domNode.h>
#include <dom/domExtra.h>

/**
 * Nodes embody the hierarchical relationship of elements in the scene.
 */
class domNode : public daeElement
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
 *  The sid attribute is a text string value containing the sub-identifier
 * of this element.  This value must be unique within the scope of the parent
 * element. Optional attribute. 
 */
	xsNCName attrSid;
/**
 *  The type attribute indicates the type of the node element. The default
 * value is “NODE”.  Optional attribute. 
 */
	domNodeType attrType;
/**
 *  The layer attribute indicates the names of the layers to which this node
 * belongs.  For example,  a value of “foreground glowing” indicates that
 * this node belongs to both the ‘foreground’ layer  and the ‘glowing’
 * layer.  The default value is empty, indicating that the node doesn’t
 * belong to  any layer.  Optional attribute. 
 */
	domListOfNames attrLayer;

protected:  // Elements
/**
 *  The node element may contain an asset element.  @see domAsset
 */
	domAssetRef elemAsset;
/**
 *  The node element may contain any number of lookat elements.  @see domLookat
 */
	domLookat_Array elemLookat_array;
/**
 *  The node element may contain any number of matrix elements.  @see domMatrix
 */
	domMatrix_Array elemMatrix_array;
/**
 *  The node element may contain any number of rotate elements.  @see domRotate
 */
	domRotate_Array elemRotate_array;
/**
 *  The node element may contain any number of scale elements.  @see domScale
 */
	domScale_Array elemScale_array;
/**
 *  The node element may contain any number of skew elements.  @see domSkew
 */
	domSkew_Array elemSkew_array;
/**
 *  The node element may contain any number of translate elements.  @see domTranslate
 */
	domTranslate_Array elemTranslate_array;
/**
 *  The node element may instance any number of camera objects.  @see domInstance_camera
 */
	domInstance_camera_Array elemInstance_camera_array;
/**
 *  The node element may instance any number of controller objects.  @see
 * domInstance_controller
 */
	domInstance_controller_Array elemInstance_controller_array;
/**
 *  The node element may instance any number of geometry objects.  @see domInstance_geometry
 */
	domInstance_geometry_Array elemInstance_geometry_array;
/**
 *  The node element may instance any number of light objects.  @see domInstance_light
 */
	domInstance_light_Array elemInstance_light_array;
/**
 *  The node element may instance any number of node elements or hierarchies
 * objects.  @see domInstance_node
 */
	domInstance_node_Array elemInstance_node_array;
/**
 *  The node element may be hierarchical and be the parent of any number of
 * other node elements.  @see domNode
 */
	domNode_Array elemNode_array;
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
	 * Gets the sid attribute.
	 * @return Returns a xsNCName of the sid attribute.
	 */
	xsNCName getSid() const { return attrSid; }
	/**
	 * Sets the sid attribute.
	 * @param atSid The new value for the sid attribute.
	 */
	void setSid( xsNCName atSid ) { *(daeStringRef*)&attrSid = atSid;
	 _validAttributeArray[2] = true; }

	/**
	 * Gets the type attribute.
	 * @return Returns a domNodeType of the type attribute.
	 */
	domNodeType getType() const { return attrType; }
	/**
	 * Sets the type attribute.
	 * @param atType The new value for the type attribute.
	 */
	void setType( domNodeType atType ) { attrType = atType;
	 _validAttributeArray[3] = true; }

	/**
	 * Gets the layer array attribute.
	 * @return Returns a domListOfNames reference of the layer array attribute.
	 */
	domListOfNames &getLayer() { return attrLayer; }
	/**
	 * Gets the layer array attribute.
	 * @return Returns a constant domListOfNames reference of the layer array attribute.
	 */
	const domListOfNames &getLayer() const { return attrLayer; }
	/**
	 * Sets the layer array attribute.
	 * @param atLayer The new value for the layer array attribute.
	 */
	void setLayer( const domListOfNames &atLayer ) { attrLayer = atLayer;
	 _validAttributeArray[4] = true; }

	/**
	 * Gets the asset element.
	 * @return a daeSmartRef to the asset element.
	 */
	const domAssetRef getAsset() const { return elemAsset; }
	/**
	 * Gets the lookat element array.
	 * @return Returns a reference to the array of lookat elements.
	 */
	domLookat_Array &getLookat_array() { return elemLookat_array; }
	/**
	 * Gets the lookat element array.
	 * @return Returns a constant reference to the array of lookat elements.
	 */
	const domLookat_Array &getLookat_array() const { return elemLookat_array; }
	/**
	 * Gets the matrix element array.
	 * @return Returns a reference to the array of matrix elements.
	 */
	domMatrix_Array &getMatrix_array() { return elemMatrix_array; }
	/**
	 * Gets the matrix element array.
	 * @return Returns a constant reference to the array of matrix elements.
	 */
	const domMatrix_Array &getMatrix_array() const { return elemMatrix_array; }
	/**
	 * Gets the rotate element array.
	 * @return Returns a reference to the array of rotate elements.
	 */
	domRotate_Array &getRotate_array() { return elemRotate_array; }
	/**
	 * Gets the rotate element array.
	 * @return Returns a constant reference to the array of rotate elements.
	 */
	const domRotate_Array &getRotate_array() const { return elemRotate_array; }
	/**
	 * Gets the scale element array.
	 * @return Returns a reference to the array of scale elements.
	 */
	domScale_Array &getScale_array() { return elemScale_array; }
	/**
	 * Gets the scale element array.
	 * @return Returns a constant reference to the array of scale elements.
	 */
	const domScale_Array &getScale_array() const { return elemScale_array; }
	/**
	 * Gets the skew element array.
	 * @return Returns a reference to the array of skew elements.
	 */
	domSkew_Array &getSkew_array() { return elemSkew_array; }
	/**
	 * Gets the skew element array.
	 * @return Returns a constant reference to the array of skew elements.
	 */
	const domSkew_Array &getSkew_array() const { return elemSkew_array; }
	/**
	 * Gets the translate element array.
	 * @return Returns a reference to the array of translate elements.
	 */
	domTranslate_Array &getTranslate_array() { return elemTranslate_array; }
	/**
	 * Gets the translate element array.
	 * @return Returns a constant reference to the array of translate elements.
	 */
	const domTranslate_Array &getTranslate_array() const { return elemTranslate_array; }
	/**
	 * Gets the instance_camera element array.
	 * @return Returns a reference to the array of instance_camera elements.
	 */
	domInstance_camera_Array &getInstance_camera_array() { return elemInstance_camera_array; }
	/**
	 * Gets the instance_camera element array.
	 * @return Returns a constant reference to the array of instance_camera elements.
	 */
	const domInstance_camera_Array &getInstance_camera_array() const { return elemInstance_camera_array; }
	/**
	 * Gets the instance_controller element array.
	 * @return Returns a reference to the array of instance_controller elements.
	 */
	domInstance_controller_Array &getInstance_controller_array() { return elemInstance_controller_array; }
	/**
	 * Gets the instance_controller element array.
	 * @return Returns a constant reference to the array of instance_controller elements.
	 */
	const domInstance_controller_Array &getInstance_controller_array() const { return elemInstance_controller_array; }
	/**
	 * Gets the instance_geometry element array.
	 * @return Returns a reference to the array of instance_geometry elements.
	 */
	domInstance_geometry_Array &getInstance_geometry_array() { return elemInstance_geometry_array; }
	/**
	 * Gets the instance_geometry element array.
	 * @return Returns a constant reference to the array of instance_geometry elements.
	 */
	const domInstance_geometry_Array &getInstance_geometry_array() const { return elemInstance_geometry_array; }
	/**
	 * Gets the instance_light element array.
	 * @return Returns a reference to the array of instance_light elements.
	 */
	domInstance_light_Array &getInstance_light_array() { return elemInstance_light_array; }
	/**
	 * Gets the instance_light element array.
	 * @return Returns a constant reference to the array of instance_light elements.
	 */
	const domInstance_light_Array &getInstance_light_array() const { return elemInstance_light_array; }
	/**
	 * Gets the instance_node element array.
	 * @return Returns a reference to the array of instance_node elements.
	 */
	domInstance_node_Array &getInstance_node_array() { return elemInstance_node_array; }
	/**
	 * Gets the instance_node element array.
	 * @return Returns a constant reference to the array of instance_node elements.
	 */
	const domInstance_node_Array &getInstance_node_array() const { return elemInstance_node_array; }
	/**
	 * Gets the node element array.
	 * @return Returns a reference to the array of node elements.
	 */
	domNode_Array &getNode_array() { return elemNode_array; }
	/**
	 * Gets the node element array.
	 * @return Returns a constant reference to the array of node elements.
	 */
	const domNode_Array &getNode_array() const { return elemNode_array; }
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
	domNode() : attrId(), attrName(), attrSid(), attrType(), attrLayer(), elemAsset(), elemLookat_array(), elemMatrix_array(), elemRotate_array(), elemScale_array(), elemSkew_array(), elemTranslate_array(), elemInstance_camera_array(), elemInstance_controller_array(), elemInstance_geometry_array(), elemInstance_light_array(), elemInstance_node_array(), elemNode_array(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domNode() {}
	/**
	 * Copy Constructor
	 */
	domNode( const domNode &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domNode &operator=( const domNode &cpy ) { (void)cpy; return *this; }

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
