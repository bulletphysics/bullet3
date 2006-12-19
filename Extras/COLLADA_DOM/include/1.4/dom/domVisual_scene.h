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
#ifndef __domVisual_scene_h__
#define __domVisual_scene_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domAsset.h>
#include <dom/domNode.h>
#include <dom/domExtra.h>
#include <dom/domInstance_effect.h>

/**
 * The visual_scene element declares the base of the visual_scene hierarchy
 * or scene graph. The  scene contains elements that comprise much of the
 * visual and transformational information  content as created by the authoring
 * tools.
 */
class domVisual_scene : public daeElement
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::VISUAL_SCENE; }
public:
	class domEvaluate_scene;

	typedef daeSmartRef<domEvaluate_scene> domEvaluate_sceneRef;
	typedef daeTArray<domEvaluate_sceneRef> domEvaluate_scene_Array;

/**
 * The evaluate_scene element declares information specifying a specific way
 * to evaluate this  visual_scene. There may be any number of evaluate_scene
 * elements.
 */
	class domEvaluate_scene : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::EVALUATE_SCENE; }
	public:
		class domRender;

		typedef daeSmartRef<domRender> domRenderRef;
		typedef daeTArray<domRenderRef> domRender_Array;

/**
 * The render element describes one effect pass to evaluate the scene. There
 * must be at least one render element.
 */
		class domRender : public daeElement
		{
		public:
			COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::RENDER; }
		public:
			class domLayer;

			typedef daeSmartRef<domLayer> domLayerRef;
			typedef daeTArray<domLayerRef> domLayer_Array;

/**
 * The layer element specifies which layer to render in this compositing step
 * while evaluating the scene. You may specify any number of layers.
 */
			class domLayer : public daeElement
			{
			public:
				COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::LAYER; }

			protected:  // Value
				/**
				 * The xsNCName value of the text data of this element. 
				 */
				xsNCName _value;

			public:	//Accessors and Mutators
				/**
				 * Gets the value of this element.
				 * @return Returns a xsNCName of the value.
				 */
				xsNCName getValue() const { return _value; }
				/**
				 * Sets the _value of this element.
				 * @param val The new value for this element.
				 */
				void setValue( xsNCName val ) { *(daeStringRef*)&_value = val; }

			protected:
				/**
				 * Constructor
				 */
				domLayer() : _value() {}
				/**
				 * Destructor
				 */
				virtual ~domLayer() {}
				/**
				 * Copy Constructor
				 */
				domLayer( const domLayer &cpy ) : daeElement() { (void)cpy; }
				/**
				 * Overloaded assignment operator
				 */
				virtual domLayer &operator=( const domLayer &cpy ) { (void)cpy; return *this; }

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


		protected:  // Attribute
/**
 *  The camera_node attribute refers to a node that contains a camera describing
 * the viewpoint to  render this compositing step from. 
 */
			xsAnyURI attrCamera_node;

		protected:  // Elements
/**
 * The layer element specifies which layer to render in this compositing step
 * while evaluating the scene. You may specify any number of layers. @see
 * domLayer
 */
			domLayer_Array elemLayer_array;
/**
 *  The instance_effect element specifies which effect to render in this compositing
 * step  while evaluating the scene.  @see domInstance_effect
 */
			domInstance_effectRef elemInstance_effect;

		public:	//Accessors and Mutators
			/**
			 * Gets the camera_node attribute.
			 * @return Returns a xsAnyURI reference of the camera_node attribute.
			 */
			xsAnyURI &getCamera_node() { return attrCamera_node; }
			/**
			 * Gets the camera_node attribute.
			 * @return Returns a constant xsAnyURI reference of the camera_node attribute.
			 */
			const xsAnyURI &getCamera_node() const { return attrCamera_node; }
			/**
			 * Sets the camera_node attribute.
			 * @param atCamera_node The new value for the camera_node attribute.
			 */
			void setCamera_node( const xsAnyURI &atCamera_node ) { attrCamera_node = atCamera_node;		
	 _validAttributeArray[0] = true; }

			/**
			 * Gets the layer element array.
			 * @return Returns a reference to the array of layer elements.
			 */
			domLayer_Array &getLayer_array() { return elemLayer_array; }
			/**
			 * Gets the layer element array.
			 * @return Returns a constant reference to the array of layer elements.
			 */
			const domLayer_Array &getLayer_array() const { return elemLayer_array; }
			/**
			 * Gets the instance_effect element.
			 * @return a daeSmartRef to the instance_effect element.
			 */
			const domInstance_effectRef getInstance_effect() const { return elemInstance_effect; }
		protected:
			/**
			 * Constructor
			 */
			domRender() : attrCamera_node(), elemLayer_array(), elemInstance_effect() {}
			/**
			 * Destructor
			 */
			virtual ~domRender() {}
			/**
			 * Copy Constructor
			 */
			domRender( const domRender &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domRender &operator=( const domRender &cpy ) { (void)cpy; return *this; }

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


	protected:  // Attribute
/**
 *  The name attribute is the text string name of this element. Optional attribute.
 */
		xsNCName attrName;

	protected:  // Element
/**
 * The render element describes one effect pass to evaluate the scene. There
 * must be at least one render element. @see domRender
 */
		domRender_Array elemRender_array;

	public:	//Accessors and Mutators
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
	 _validAttributeArray[0] = true; }

		/**
		 * Gets the render element array.
		 * @return Returns a reference to the array of render elements.
		 */
		domRender_Array &getRender_array() { return elemRender_array; }
		/**
		 * Gets the render element array.
		 * @return Returns a constant reference to the array of render elements.
		 */
		const domRender_Array &getRender_array() const { return elemRender_array; }
	protected:
		/**
		 * Constructor
		 */
		domEvaluate_scene() : attrName(), elemRender_array() {}
		/**
		 * Destructor
		 */
		virtual ~domEvaluate_scene() {}
		/**
		 * Copy Constructor
		 */
		domEvaluate_scene( const domEvaluate_scene &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domEvaluate_scene &operator=( const domEvaluate_scene &cpy ) { (void)cpy; return *this; }

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


protected:  // Attributes
/**
 *  The id attribute is a text string containing the unique identifier of
 * this element. This  value must be unique within the instance document.
 * Optional attribute. 
 */
	xsID attrId;
/**
 *  The name attribute is the text string name of this element. Optional attribute.
 */
	xsNCName attrName;

protected:  // Elements
/**
 *  The visual_scene element may contain an asset element.  @see domAsset
 */
	domAssetRef elemAsset;
/**
 *  The visual_scene element must have at least one node element.  @see domNode
 */
	domNode_Array elemNode_array;
/**
 * The evaluate_scene element declares information specifying a specific way
 * to evaluate this  visual_scene. There may be any number of evaluate_scene
 * elements. @see domEvaluate_scene
 */
	domEvaluate_scene_Array elemEvaluate_scene_array;
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
	 * Gets the evaluate_scene element array.
	 * @return Returns a reference to the array of evaluate_scene elements.
	 */
	domEvaluate_scene_Array &getEvaluate_scene_array() { return elemEvaluate_scene_array; }
	/**
	 * Gets the evaluate_scene element array.
	 * @return Returns a constant reference to the array of evaluate_scene elements.
	 */
	const domEvaluate_scene_Array &getEvaluate_scene_array() const { return elemEvaluate_scene_array; }
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
	domVisual_scene() : attrId(), attrName(), elemAsset(), elemNode_array(), elemEvaluate_scene_array(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domVisual_scene() {}
	/**
	 * Copy Constructor
	 */
	domVisual_scene( const domVisual_scene &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domVisual_scene &operator=( const domVisual_scene &cpy ) { (void)cpy; return *this; }

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
