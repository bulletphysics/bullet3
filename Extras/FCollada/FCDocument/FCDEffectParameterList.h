/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FCDEffectParameterList.h
	This file contains the FCDEffectParameterList class.
*/

#ifndef _FCD_EFFECT_PARAMETER_LIST_H_
#define _FCD_EFFECT_PARAMETER_LIST_H_

class FCDEffectParameter;

/**
	A searchable list of COLLADA effect parameters.

	This class is based on the STL vector class and adds some
	useful search methods: by reference and by semantic.

	@ingroup FCDEffect
*/
class FCOLLADA_EXPORT FCDEffectParameterList : public vector<FCDEffectParameter*>, public FCDObject
{
private:
	bool ownParameters;

public:
	/** Constructor.
		All the objects that need a parameter list will create it when necessary.
		You may also create new lists for the retrieval of parameters during a search.
		@param document The COLLADA document that owns this parameter list. This pointer
			can remain NULL unless you expect to create new parameters within this list.
		@param ownParameters Whether this list should release the contained parameters
			during its destruction. */
	FCDEffectParameterList(FCDocument* document = NULL, bool ownParameters = false);

	/** Destructor. */
	virtual ~FCDEffectParameterList();

	/** Creates a new parameters within this list.
		@param type The effect parameter type.
		@return The new effect parameter. This pointer will be NULL if this list does not own its parameters. */
	FCDEffectParameter* AddParameter(uint32 type);

	/** Releases a parameter contained within this list.
		The memory used by this parameter will be released only if this list owns the parameters.
		@param parameter The effect parameter to release. */
	void ReleaseParameter(FCDEffectParameter* parameter);

	/** Retrieves the first effect parameter with the given reference.
		For effect parameter generators, the sub-id is used instead of the reference.
		@param reference A reference to match.
		@return The effect parameter that matches the reference. This pointer will be NULL,
			if no parameter matches the reference. */
	FCDEffectParameter* FindReference(const char* reference);
	const FCDEffectParameter* FindReference(const char* reference) const; /**< See above. */
	inline FCDEffectParameter* FindReference(const string& reference) { return FindReference(reference.c_str()); } /**< See above. */
	inline const FCDEffectParameter* FindReference(const string& reference) const { return FindReference(reference.c_str()); } /**< See above. */

	/** Retrieves the first effect parameter with the given semantic.
		@param semantic A semantic to match.
		@return The effect parameter that matches the semantic. This pointer will be NULL
			if no parameter matches the semantic. */
	FCDEffectParameter* FindSemantic(const char* semantic);
	const FCDEffectParameter* FindSemantic(const char* semantic) const; /**< See above. */
	inline FCDEffectParameter* FindSemantic(const string& semantic) { return FindReference(semantic.c_str()); } /**< See above. */
	inline const FCDEffectParameter* FindSemantic(const string& semantic) const { return FindReference(semantic.c_str()); } /**< See above. */

	/** Retrieves a subset of this parameter list.
		All the effects that match the given reference will be added to the given list.
		For effect parameter generators, the sub-id is used instead of the reference.
		@param reference A reference to match.
		@param list The effect parameter list to fill in with the matched parameters.
			This list is not clear. */
	void FindReference(const char* reference, FCDEffectParameterList& list);
	inline void FindReference(const string& reference, FCDEffectParameterList& list) { return FindReference(reference.c_str(), list); } /**< See above. */

	/** Retrieves a subset of this parameter list.
		All the effects that match the given semantic will be added to the given list.
		@param semantic A semantic to match.
		@param list The effect parameter list to fill in with the matched parameters.
			This list is not clear. */
	void FindSemantic(const char* semantic, FCDEffectParameterList& list);
	inline void FindSemantic(const string& semantic, FCDEffectParameterList& list) { return FindReference(semantic.c_str(), list); } /**< See above. */

	/** Creates a full copy of the list of parameters and its content.
		@return The cloned list. You will need to delete this pointer.*/
	FCDEffectParameterList* Clone() const;
};

#endif // _FCD_EFFECT_PARAMETER_LIST_H_
