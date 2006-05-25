/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FCDEffectParameterFactory.h
	This file contains the FCDEffectParameterFactory class.
*/

#ifndef _FCD_EFFECT_PARAMETER_FACTORY_H_
#define _FCD_EFFECT_PARAMETER_FACTORY_H_

class FCDocument;

/**
	[INTERNAL] The factory for COLLADA effect parameters.

	Takes in a COLLADA XML tree and returns a new
	parameter that represent it, if one is possible.

	@ingroup FCDEffect
*/
class FCOLLADA_EXPORT FCDEffectParameterFactory
{
private:
	// Never instantiate: this is a static class
	FCDEffectParameterFactory() {}

public:
	/** [INTERNAL] Creates a new effect parameter, given a type.
		To create new effect parameters, use the FCDEffectParameterList::AddParameter function.
		@param document The COLLADA document that will own the effect parameter.
		@param type The type of effect to create.
			This value should reflect the FCDEffectParameter::Type enum. */
	static FCDEffectParameter* Create(FCDocument* document, uint32 type);

	/** [INTERNAL] Generates the effect parameter object for the given XML node tree.
		@param document The COLLADA document that will own the effect parameter.
		@param parameterNode The COLLADA XML tree node.
		@param status An optional return status.
		@return The new effect parameter. This pointer will be NULL if no parameter can be generated
			from the given COLLADA XML tree node. */
	static FCDEffectParameter* LoadFromXML(FCDocument* document, xmlNode* parameterNode, FUStatus* status);
};

#endif // _FCD_EFFECT_PARAMETER_FACTORY_H_
