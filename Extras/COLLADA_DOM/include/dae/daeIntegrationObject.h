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

#ifndef __DAE_INTEGRATION_OBJECT_H__
#define __DAE_INTEGRATION_OBJECT_H__

#include <dae/daeElement.h>

class daeIntegrationObject;
typedef daeSmartRef<daeIntegrationObject> daeIntegrationObjectRef;
/**
 * The @c daeIntegrationObject class provides methods to translate COLLADA
 * objects to and from application objects.
 */
class daeIntegrationObject : public daeElement
{
public:
	/**
	 *  Constructor.
	 */
	DLLSPEC daeIntegrationObject() { _element = NULL; _object = NULL; _from_state = int_uninitialized; _to_state = int_uninitialized; }
	/**
	 *  Destructor.
	 */
	virtual DLLSPEC ~daeIntegrationObject() {}

public:
	/** A pointer to the element associated with this integration object. */
	daeElement	*_element;
	/** A pointer at which to store the user object associated with this element. */
	void*			_object;
	/** An enum describing the state of the conversion from COLLADA. */
	IntegrationState _from_state;
	/** An enum describing the state of the conversion to COLLADA. */
	IntegrationState _to_state;

public:
	/** 
	 * Sets the element associated with this integration object.
	 * @param element A daeSmartRef to the element for this integration object.
	 */
	void			setElement(daeElementRef element) { _element = element; }
	/**
	 * Gets the element associated with this integration object.
	 * @return The element associated with this integration object.
	 */
	daeElementRef	getElement() { return _element; }
	/**
	 * Sets the user object associated with this integration object.
	 * @param object A void * to the user object to be associated with this integration object.
	 */
	void			setObject(void* object) { _object = object; }
	/**
	 * Gets the user object associated with this integration object.
	 * @return The user object associated with this integration object.
	 */
	void*			getObject() { return _object; }
public: // Do not implement these by default
	virtual daeElementRef				lookupElement(daeString s) { (void)s; return NULL;}
	virtual daeIntegrationObjectRef		lookup(daeString s) { (void)s; return NULL;}
protected:
	/**
	 * Defines the code to create the application-specific data structure associated with the DOM class 
	 * for this integration template. This method sets up the integration object for the DOM class.
	 * @param element A daeSmartRef to the element to convert into the user's structure.
	 */
	virtual DLLSPEC void createFrom(daeElementRef element) = 0;
	/**
	 * Defines the code to convert the COLLADA Object Model data structure into your application-specific 
	 * data structure.
	 */
	virtual DLLSPEC void fromCOLLADA() = 0;
	/**
	 * Defines any postprocessing code that must execute after the basic conversion.
	 */
	virtual DLLSPEC void fromCOLLADAPostProcess() = 0;
	/**
	 * Defines code to create the COLLADA Object Model data structure associated with the DOM class for 
	 * this template.
	 * @param userData A pointer to the application-specific data structure to convert to the DOM structure.
	 */
	virtual DLLSPEC void createTo(void *userData) = 0;
	/**
	 * Defines the code to convert your application's data structures back into COLLADA Object Model data 
	 * structures.
	 */
	virtual DLLSPEC void toCOLLADA() = 0;
	/**
	 * Defines any postprocessing code that must execute after the basic conversion.
	 */
	virtual DLLSPEC void toCOLLADAPostProcess() = 0;

public:
	/**
	 * Defines the code to create the application-specific data structure associated with the DOM class 
	 * for this integration template. This method sets up the integration object for the DOM class. This
	 * method checks and updates the conversion state stored in _from_state and converts only if 
	 * necessary.
	 * @param element A daeSmartRef to the element to convert into the user's structure.
	 */
	void createFromChecked(daeElementRef element) {
		if ( _from_state >= int_created ) {
			return;
		}
		createFrom(element);
		_from_state = int_created;
	};
	/**
	 * Defines the code to convert the COLLADA Object Model data structure into your application-specific 
	 * data structure. This method checks and updates the conversion state stored in _from_state and
	 * converts only if necessary.
	 */
	void fromCOLLADAChecked() {
		if ( _from_state >= int_converted ) {
			return;
		}
		fromCOLLADA();
		_from_state = int_converted;
	};
	/**
	 * Defines any postprocessing code that must execute after the basic conversion. This method 
	 * checks and updates the conversion state stored in _from_state and converts only if necessary.
	 */
	void fromCOLLADAPostProcessChecked()  {
		if ( _from_state >= int_finished) {
			return;
		}
		fromCOLLADAPostProcess();
		_from_state = int_finished;
	};
	/**
	 * Defines code to create the COLLADA Object Model data structure associated with the DOM class for 
	 * this template. This method checks and updates the conversion state stored in _to_state and 
	 * converts only if necessary.
	 * @param userData A pointer to the application-specific data structure to convert to the DOM structure.
	 */
	void createToChecked(void *userData) {
		if ( _to_state >= int_created ) {
			return;
		}
		createTo(userData);
		_to_state = int_created;
	};
	/**
	 * Defines the code to convert your application's data structures back into COLLADA Object Model data 
	 * structures. This method checks and updates the conversion state stored in _to_state and  
	 * converts only if necessary.
	 */
	void toCOLLADAChecked() {
		if ( _to_state >= int_converted ) {
			return;
		}
		toCOLLADA();
		_to_state = int_converted;
	};
	/**
	 * Defines any postprocessing code that must execute after the basic conversion. This method 
	 * checks and updates the conversion state stored in _to_state and converts only if necessary.
	 */
	void toCOLLADAPostProcessChecked()  {
		if ( _to_state >= int_finished) {
			return;
		}
		toCOLLADAPostProcess();
		_to_state = int_finished;
	};

};
#endif
