/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#ifndef _FCD_PHYSICS_PARAMETER_GENERIC_H_
#define _FCD_PHYSICS_PARAMETER_GENERIC_H_

#include "FCDocument/FCDObject.h"

class FCDocument;

class FCOLLADA_EXPORT FCDPhysicsParameterGeneric: public FCDObject
{
public:
    FCDPhysicsParameterGeneric(FCDocument* document, const string& ref);
	virtual ~FCDPhysicsParameterGeneric();

	virtual FCDPhysicsParameterGeneric* Clone()=0;
	bool IsGenerator() const { return isGenerator; }
	bool IsModifier() const { return !isGenerator; }

	const string& GetReference() const {return reference;};
	void SetReference(const string& ref) {reference = ref;};
	void SetGenerator(bool val) {isGenerator = val;}

	virtual void Overwrite(FCDPhysicsParameterGeneric* target) = 0;

	// Parse in this ColladaFX parameter from the xml node tree
//	virtual FUStatus LoadFromXML(xmlNode* parameterNode) = 0;

	// Write out this ColladaFX parameter to the xml node tree
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const = 0;

protected:
	bool isGenerator; // whether this effect parameter structure generates a new value or modifies an existing value (is <newparam>?)
	string reference;

};

#endif // _FCD_PHYSICS_PARAMETER_GENERIC_H_
