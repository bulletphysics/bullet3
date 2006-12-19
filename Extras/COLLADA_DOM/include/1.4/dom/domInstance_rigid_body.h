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
#ifndef __domInstance_rigid_body_h__
#define __domInstance_rigid_body_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domTechnique.h>
#include <dom/domExtra.h>
#include <dom/domInstance_physics_material.h>
#include <dom/domPhysics_material.h>
#include <dom/domTargetableFloat.h>
#include <dom/domTranslate.h>
#include <dom/domRotate.h>
#include <dom/domTargetableFloat3.h>
#include <dom/domInstance_geometry.h>
#include <dom/domPlane.h>
#include <dom/domBox.h>
#include <dom/domSphere.h>
#include <dom/domCylinder.h>
#include <dom/domTapered_cylinder.h>
#include <dom/domCapsule.h>
#include <dom/domTapered_capsule.h>

/**
 * This element allows instancing a rigid_body within an instance_physics_model.
 */
class domInstance_rigid_body : public daeElement
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INSTANCE_RIGID_BODY; }
public:
	class domTechnique_common;

	typedef daeSmartRef<domTechnique_common> domTechnique_commonRef;
	typedef daeTArray<domTechnique_commonRef> domTechnique_common_Array;

/**
 * The technique_common element specifies the instance_rigid_body information
 * for the common  profile which all COLLADA implementations need to support.
 */
	class domTechnique_common : public daeElement
	{
	public:
		COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::TECHNIQUE_COMMON; }
	public:
		class domAngular_velocity;

		typedef daeSmartRef<domAngular_velocity> domAngular_velocityRef;
		typedef daeTArray<domAngular_velocityRef> domAngular_velocity_Array;

/**
 * Specifies the initial angular velocity of the rigid_body instance in degrees
 * per second  around each axis, in the form of an X-Y-Z Euler rotation.
 */
		class domAngular_velocity : public daeElement
		{
		public:
			COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::ANGULAR_VELOCITY; }

		protected:  // Value
			/**
			 * The domFloat3 value of the text data of this element. 
			 */
			domFloat3 _value;

		public:	//Accessors and Mutators
			/**
			 * Gets the _value array.
			 * @return Returns a domFloat3 reference of the _value array.
			 */
			domFloat3 &getValue() { return _value; }
			/**
			 * Gets the _value array.
			 * @return Returns a constant domFloat3 reference of the _value array.
			 */
			const domFloat3 &getValue() const { return _value; }
			/**
			 * Sets the _value array.
			 * @param val The new value for the _value array.
			 */
			void setValue( const domFloat3 &val ) { _value = val; }

		protected:
			/**
			 * Constructor
			 */
			domAngular_velocity() : _value() {}
			/**
			 * Destructor
			 */
			virtual ~domAngular_velocity() {}
			/**
			 * Copy Constructor
			 */
			domAngular_velocity( const domAngular_velocity &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domAngular_velocity &operator=( const domAngular_velocity &cpy ) { (void)cpy; return *this; }

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

		class domVelocity;

		typedef daeSmartRef<domVelocity> domVelocityRef;
		typedef daeTArray<domVelocityRef> domVelocity_Array;

/**
 * Specifies the initial linear velocity of the rigid_body instance.
 */
		class domVelocity : public daeElement
		{
		public:
			COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::VELOCITY; }

		protected:  // Value
			/**
			 * The domFloat3 value of the text data of this element. 
			 */
			domFloat3 _value;

		public:	//Accessors and Mutators
			/**
			 * Gets the _value array.
			 * @return Returns a domFloat3 reference of the _value array.
			 */
			domFloat3 &getValue() { return _value; }
			/**
			 * Gets the _value array.
			 * @return Returns a constant domFloat3 reference of the _value array.
			 */
			const domFloat3 &getValue() const { return _value; }
			/**
			 * Sets the _value array.
			 * @param val The new value for the _value array.
			 */
			void setValue( const domFloat3 &val ) { _value = val; }

		protected:
			/**
			 * Constructor
			 */
			domVelocity() : _value() {}
			/**
			 * Destructor
			 */
			virtual ~domVelocity() {}
			/**
			 * Copy Constructor
			 */
			domVelocity( const domVelocity &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domVelocity &operator=( const domVelocity &cpy ) { (void)cpy; return *this; }

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

		class domDynamic;

		typedef daeSmartRef<domDynamic> domDynamicRef;
		typedef daeTArray<domDynamicRef> domDynamic_Array;

		class domDynamic : public daeElement
		{
		public:
			COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::DYNAMIC; }
		protected:  // Attribute
/**
 *  The sid attribute is a text string value containing the sub-identifier
 * of this element.  This value must be unique within the scope of the parent
 * element. Optional attribute. 
 */
			xsNCName attrSid;

		protected:  // Value
			/**
			 * The domBool value of the text data of this element. 
			 */
			domBool _value;

		public:	//Accessors and Mutators
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
	 _validAttributeArray[0] = true; }

			/**
			 * Gets the value of this element.
			 * @return a domBool of the value.
			 */
			domBool getValue() const { return _value; }
			/**
			 * Sets the _value of this element.
			 * @param val The new value for this element.
			 */
			void setValue( domBool val ) { _value = val; }

		protected:
			/**
			 * Constructor
			 */
			domDynamic() : attrSid(), _value() {}
			/**
			 * Destructor
			 */
			virtual ~domDynamic() {}
			/**
			 * Copy Constructor
			 */
			domDynamic( const domDynamic &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domDynamic &operator=( const domDynamic &cpy ) { (void)cpy; return *this; }

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

		class domMass_frame;

		typedef daeSmartRef<domMass_frame> domMass_frameRef;
		typedef daeTArray<domMass_frameRef> domMass_frame_Array;

		class domMass_frame : public daeElement
		{
		public:
			COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::MASS_FRAME; }

		protected:  // Elements
			domTranslate_Array elemTranslate_array;
			domRotate_Array elemRotate_array;
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
			domMass_frame() : elemTranslate_array(), elemRotate_array() {}
			/**
			 * Destructor
			 */
			virtual ~domMass_frame() {}
			/**
			 * Copy Constructor
			 */
			domMass_frame( const domMass_frame &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domMass_frame &operator=( const domMass_frame &cpy ) { (void)cpy; return *this; }

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

		class domShape;

		typedef daeSmartRef<domShape> domShapeRef;
		typedef daeTArray<domShapeRef> domShape_Array;

		class domShape : public daeElement
		{
		public:
			COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::SHAPE; }
		public:
			class domHollow;

			typedef daeSmartRef<domHollow> domHollowRef;
			typedef daeTArray<domHollowRef> domHollow_Array;

			class domHollow : public daeElement
			{
			public:
				COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::HOLLOW; }
			protected:  // Attribute
/**
 *  The sid attribute is a text string value containing the sub-identifier
 * of this element. This value must be unique within the scope of the parent
 * element. Optional attribute. 
 */
				xsNCName attrSid;

			protected:  // Value
				/**
				 * The domBool value of the text data of this element. 
				 */
				domBool _value;

			public:	//Accessors and Mutators
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
	 _validAttributeArray[0] = true; }

				/**
				 * Gets the value of this element.
				 * @return a domBool of the value.
				 */
				domBool getValue() const { return _value; }
				/**
				 * Sets the _value of this element.
				 * @param val The new value for this element.
				 */
				void setValue( domBool val ) { _value = val; }

			protected:
				/**
				 * Constructor
				 */
				domHollow() : attrSid(), _value() {}
				/**
				 * Destructor
				 */
				virtual ~domHollow() {}
				/**
				 * Copy Constructor
				 */
				domHollow( const domHollow &cpy ) : daeElement() { (void)cpy; }
				/**
				 * Overloaded assignment operator
				 */
				virtual domHollow &operator=( const domHollow &cpy ) { (void)cpy; return *this; }

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



		protected:  // Elements
			domHollowRef elemHollow;
			domTargetableFloatRef elemMass;
			domTargetableFloatRef elemDensity;
			domInstance_physics_materialRef elemInstance_physics_material;
			domPhysics_materialRef elemPhysics_material;
			domInstance_geometryRef elemInstance_geometry;
			domPlaneRef elemPlane;
			domBoxRef elemBox;
			domSphereRef elemSphere;
			domCylinderRef elemCylinder;
			domTapered_cylinderRef elemTapered_cylinder;
			domCapsuleRef elemCapsule;
			domTapered_capsuleRef elemTapered_capsule;
			domTranslate_Array elemTranslate_array;
			domRotate_Array elemRotate_array;
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
			 * Gets the hollow element.
			 * @return a daeSmartRef to the hollow element.
			 */
			const domHollowRef getHollow() const { return elemHollow; }
			/**
			 * Gets the mass element.
			 * @return a daeSmartRef to the mass element.
			 */
			const domTargetableFloatRef getMass() const { return elemMass; }
			/**
			 * Gets the density element.
			 * @return a daeSmartRef to the density element.
			 */
			const domTargetableFloatRef getDensity() const { return elemDensity; }
			/**
			 * Gets the instance_physics_material element.
			 * @return a daeSmartRef to the instance_physics_material element.
			 */
			const domInstance_physics_materialRef getInstance_physics_material() const { return elemInstance_physics_material; }
			/**
			 * Gets the physics_material element.
			 * @return a daeSmartRef to the physics_material element.
			 */
			const domPhysics_materialRef getPhysics_material() const { return elemPhysics_material; }
			/**
			 * Gets the instance_geometry element.
			 * @return a daeSmartRef to the instance_geometry element.
			 */
			const domInstance_geometryRef getInstance_geometry() const { return elemInstance_geometry; }
			/**
			 * Gets the plane element.
			 * @return a daeSmartRef to the plane element.
			 */
			const domPlaneRef getPlane() const { return elemPlane; }
			/**
			 * Gets the box element.
			 * @return a daeSmartRef to the box element.
			 */
			const domBoxRef getBox() const { return elemBox; }
			/**
			 * Gets the sphere element.
			 * @return a daeSmartRef to the sphere element.
			 */
			const domSphereRef getSphere() const { return elemSphere; }
			/**
			 * Gets the cylinder element.
			 * @return a daeSmartRef to the cylinder element.
			 */
			const domCylinderRef getCylinder() const { return elemCylinder; }
			/**
			 * Gets the tapered_cylinder element.
			 * @return a daeSmartRef to the tapered_cylinder element.
			 */
			const domTapered_cylinderRef getTapered_cylinder() const { return elemTapered_cylinder; }
			/**
			 * Gets the capsule element.
			 * @return a daeSmartRef to the capsule element.
			 */
			const domCapsuleRef getCapsule() const { return elemCapsule; }
			/**
			 * Gets the tapered_capsule element.
			 * @return a daeSmartRef to the tapered_capsule element.
			 */
			const domTapered_capsuleRef getTapered_capsule() const { return elemTapered_capsule; }
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
			domShape() : elemHollow(), elemMass(), elemDensity(), elemInstance_physics_material(), elemPhysics_material(), elemInstance_geometry(), elemPlane(), elemBox(), elemSphere(), elemCylinder(), elemTapered_cylinder(), elemCapsule(), elemTapered_capsule(), elemTranslate_array(), elemRotate_array(), elemExtra_array() {}
			/**
			 * Destructor
			 */
			virtual ~domShape() {}
			/**
			 * Copy Constructor
			 */
			domShape( const domShape &cpy ) : daeElement() { (void)cpy; }
			/**
			 * Overloaded assignment operator
			 */
			virtual domShape &operator=( const domShape &cpy ) { (void)cpy; return *this; }

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



	protected:  // Elements
/**
 * Specifies the initial angular velocity of the rigid_body instance in degrees
 * per second  around each axis, in the form of an X-Y-Z Euler rotation. @see
 * domAngular_velocity
 */
		domAngular_velocityRef elemAngular_velocity;
/**
 * Specifies the initial linear velocity of the rigid_body instance. @see
 * domVelocity
 */
		domVelocityRef elemVelocity;
		domDynamicRef elemDynamic;
		domTargetableFloatRef elemMass;
		domMass_frameRef elemMass_frame;
		domTargetableFloat3Ref elemInertia;
		domInstance_physics_materialRef elemInstance_physics_material;
		domPhysics_materialRef elemPhysics_material;
		domShape_Array elemShape_array;
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
		 * Gets the angular_velocity element.
		 * @return a daeSmartRef to the angular_velocity element.
		 */
		const domAngular_velocityRef getAngular_velocity() const { return elemAngular_velocity; }
		/**
		 * Gets the velocity element.
		 * @return a daeSmartRef to the velocity element.
		 */
		const domVelocityRef getVelocity() const { return elemVelocity; }
		/**
		 * Gets the dynamic element.
		 * @return a daeSmartRef to the dynamic element.
		 */
		const domDynamicRef getDynamic() const { return elemDynamic; }
		/**
		 * Gets the mass element.
		 * @return a daeSmartRef to the mass element.
		 */
		const domTargetableFloatRef getMass() const { return elemMass; }
		/**
		 * Gets the mass_frame element.
		 * @return a daeSmartRef to the mass_frame element.
		 */
		const domMass_frameRef getMass_frame() const { return elemMass_frame; }
		/**
		 * Gets the inertia element.
		 * @return a daeSmartRef to the inertia element.
		 */
		const domTargetableFloat3Ref getInertia() const { return elemInertia; }
		/**
		 * Gets the instance_physics_material element.
		 * @return a daeSmartRef to the instance_physics_material element.
		 */
		const domInstance_physics_materialRef getInstance_physics_material() const { return elemInstance_physics_material; }
		/**
		 * Gets the physics_material element.
		 * @return a daeSmartRef to the physics_material element.
		 */
		const domPhysics_materialRef getPhysics_material() const { return elemPhysics_material; }
		/**
		 * Gets the shape element array.
		 * @return Returns a reference to the array of shape elements.
		 */
		domShape_Array &getShape_array() { return elemShape_array; }
		/**
		 * Gets the shape element array.
		 * @return Returns a constant reference to the array of shape elements.
		 */
		const domShape_Array &getShape_array() const { return elemShape_array; }
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
		domTechnique_common() : elemAngular_velocity(), elemVelocity(), elemDynamic(), elemMass(), elemMass_frame(), elemInertia(), elemInstance_physics_material(), elemPhysics_material(), elemShape_array() {}
		/**
		 * Destructor
		 */
		virtual ~domTechnique_common() {}
		/**
		 * Copy Constructor
		 */
		domTechnique_common( const domTechnique_common &cpy ) : daeElement() { (void)cpy; }
		/**
		 * Overloaded assignment operator
		 */
		virtual domTechnique_common &operator=( const domTechnique_common &cpy ) { (void)cpy; return *this; }

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
 *  The body attribute indicates which rigid_body to instantiate. Required
 * attribute. 
 */
	xsNCName attrBody;
/**
 *  The sid attribute is a text string value containing the sub-identifier
 * of this element. This  value must be unique within the scope of the parent
 * element. Optional attribute. 
 */
	xsNCName attrSid;
/**
 *  The name attribute is the text string name of this element. Optional attribute.
 */
	xsNCName attrName;
/**
 *  The target attribute indicates which node is influenced by this rigid_body
 * instance.  Required attribute 
 */
	xsAnyURI attrTarget;

protected:  // Elements
/**
 * The technique_common element specifies the instance_rigid_body information
 * for the common  profile which all COLLADA implementations need to support.
 * @see domTechnique_common
 */
	domTechnique_commonRef elemTechnique_common;
/**
 *  This element may contain any number of non-common profile techniques.
 * @see domTechnique
 */
	domTechnique_Array elemTechnique_array;
/**
 *  The extra element may appear any number of times.  @see domExtra
 */
	domExtra_Array elemExtra_array;

public:	//Accessors and Mutators
	/**
	 * Gets the body attribute.
	 * @return Returns a xsNCName of the body attribute.
	 */
	xsNCName getBody() const { return attrBody; }
	/**
	 * Sets the body attribute.
	 * @param atBody The new value for the body attribute.
	 */
	void setBody( xsNCName atBody ) { *(daeStringRef*)&attrBody = atBody;
	 _validAttributeArray[0] = true; }

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
	 _validAttributeArray[1] = true; }

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
	 _validAttributeArray[2] = true; }

	/**
	 * Gets the target attribute.
	 * @return Returns a xsAnyURI reference of the target attribute.
	 */
	xsAnyURI &getTarget() { return attrTarget; }
	/**
	 * Gets the target attribute.
	 * @return Returns a constant xsAnyURI reference of the target attribute.
	 */
	const xsAnyURI &getTarget() const { return attrTarget; }
	/**
	 * Sets the target attribute.
	 * @param atTarget The new value for the target attribute.
	 */
	void setTarget( const xsAnyURI &atTarget ) { attrTarget = atTarget;
	 _validAttributeArray[3] = true; }

	/**
	 * Gets the technique_common element.
	 * @return a daeSmartRef to the technique_common element.
	 */
	const domTechnique_commonRef getTechnique_common() const { return elemTechnique_common; }
	/**
	 * Gets the technique element array.
	 * @return Returns a reference to the array of technique elements.
	 */
	domTechnique_Array &getTechnique_array() { return elemTechnique_array; }
	/**
	 * Gets the technique element array.
	 * @return Returns a constant reference to the array of technique elements.
	 */
	const domTechnique_Array &getTechnique_array() const { return elemTechnique_array; }
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
	domInstance_rigid_body() : attrBody(), attrSid(), attrName(), attrTarget(), elemTechnique_common(), elemTechnique_array(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domInstance_rigid_body() {}
	/**
	 * Copy Constructor
	 */
	domInstance_rigid_body( const domInstance_rigid_body &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domInstance_rigid_body &operator=( const domInstance_rigid_body &cpy ) { (void)cpy; return *this; }

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
