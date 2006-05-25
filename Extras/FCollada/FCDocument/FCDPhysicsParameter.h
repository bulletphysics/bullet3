/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#ifndef _FCD_PHYSICS_PARAMETER_H_
#define _FCD_PHYSICS_PARAMETER_H_

#include "FCDocument/FCDPhysicsParameterGeneric.h"

class FCDocument;

template <class T>

class FCOLLADA_EXPORT FCDPhysicsParameter: public FCDPhysicsParameterGeneric
{
public:
	FCDPhysicsParameter(FCDocument* document, const string& ref);
	virtual ~FCDPhysicsParameter();

	// Clone
	virtual FCDPhysicsParameterGeneric* Clone();

	void SetValue(T val);
	void SetValue(T* val);

	T* GetValue() const {return value;}

	// Flattening: overwrite the target parameter with this parameter
	virtual void Overwrite(FCDPhysicsParameterGeneric* target);

	// Parse in this ColladaFX parameter from the xml node tree
//	virtual FUStatus LoadFromXML(xmlNode* parameterNode);

	// Write out this ColladaFX parameter to the xml node tree
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;

protected:
	T* value;

};

#include "FCDPhysicsParameter.hpp"

#endif // _FCD_PHYSICS_PARAMETER_H_
