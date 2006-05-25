/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/
/*
	Based on the FS Import classes:
	Copyright (C) 2005-2006 Feeling Software Inc
	Copyright (C) 2005-2006 Autodesk Media Entertainment
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FCDAnimation.h
	This file contains the FCDAnimation class.
*/

#ifndef _FCD_ANIMATION_H_
#define _FCD_ANIMATION_H_

#include "FUtils/FUXmlNodeIdPair.h"
#include "FCDocument/FCDEntity.h"

class FCDocument;
class FCDAnimated;
class FCDAnimation;
class FCDAnimationChannel;
class FCDAnimationCurve;

typedef vector<FCDAnimation*> FCDAnimationList; /**< A dynamically-sized array of animation entities. */
typedef vector<FCDAnimationChannel*> FCDAnimationChannelList; /**< A dynamically-sized array of animation channels. */
typedef vector<FCDAnimationCurve*> FCDAnimationCurveList; /**< A dynamically-sized array of animation curves. */

/**
	A COLLADA animation entity.
	An animation entity contains a list of child animation entities,
	in order to form a tree of animation entities.
	It also hold a list of animation channels, which hold the information
	to generate animation curves.

	In other words, the animation entity is a structural class
	used to group animation channels hierarchically.

	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDAnimation : public FCDEntity
{
private:
	FCDAnimationChannelList channels;
	FUXmlNodeIdPairList childNodes;
	FCDAnimationList children;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDLibrary::AddEntity function
		or the AddChild function, depending on the
		hierarchical level of the animation entity.
		@param document The COLLADA document that owns the animation entity. */
	FCDAnimation(FCDocument* document);

	/** Destructor: do not use directly.
		Instead, use the FCDLibrary::ReleaseEntity function
		or the ReleaseChild function, depending on the
		hierarchical level of the animation entity.*/
	virtual ~FCDAnimation();

	/** Retrieves the entity class type.
		This function is a part of the FCDEntity interface.
		@return The entity class type: ANIMATION. */
	virtual Type GetType() const { return ANIMATION; }
    
	/** Retrieves the entity with the given COLLADA id.
		This function will look through the local sub-tree of animations
		for the given COLLADA id.
		@param daeId A COLLADA id.
		@return The animation entity that matches the COLLADA id. This pointer
			will be NULL if there are no animation entities that matches the COLLADA id. */
	virtual FCDEntity* FindDaeId(const string& daeId);

	/** Retrieves the number of animation entity sub-trees contained
		by this animation entity tree.
		@return The number of animation entity sub-trees. */
	inline size_t GetChildCount() const { return children.size(); }

	/** Retrieves an animation entity sub-tree contained by this
		animation entity tree.
		@param index The index of the sub-tree.
		@return The animation entity sub-tree at the given index. This pointer will
			be NULL if the index is out-of-bounds. */
	inline FCDAnimation* GetChild(size_t index) { FUAssert(index < GetChildCount(), return NULL); return children.at(index); }
	inline const FCDAnimation* GetChild(size_t index) const { FUAssert(index < GetChildCount(), return NULL); return children.at(index); } /**< See above. */

	/** Creates a new animation entity sub-tree contained within this animation entity tree.
		@return The new animation sub-tree. */
	inline FCDAnimation* AddChild();

	/** Releases an animation entity sub-tree contained by this animation entity tree.
		@param animation The animation entity the release. */
	inline void ReleaseChild(FCDAnimation* animation);

	/** Retrieves the animation channels that target the given COLLADA target pointer.
		@param pointer A COLLADA target pointer.
		@param targetChannels A list of animation channels to fill in.
			This list is not cleared. */
	void FindAnimationChannels(const string& pointer, FCDAnimationChannelList& targetChannels);

	/** Retrieves the number of animation channels at this level within the animation tree.
		@return The number of animation channels. */
	size_t GetChannelCount() const { return channels.size(); }

	/** Retrieves an animation channel contained by this animation entity.
		@param index The index of the channel.
		@return The channel at the given index. This pointer will be NULL
			if the index is out-of-bounds. */
	FCDAnimationChannel* GetChannel(size_t index) { FUAssert(index < GetChannelCount(), return NULL); return channels.at(index); }
	const FCDAnimationChannel* GetChannel(size_t index) const { FUAssert(index < GetChannelCount(), return NULL); return channels.at(index); } /**< See above. */

	/** Adds a new animation channel to this animation entity.
		@return The new animation channel. */
	FCDAnimationChannel* AddChannel();

	/** Releases an animation channel contained within this animation entity.
		@param channel The animation channel to release. */
	void ReleaseChannel(FCDAnimationChannel* channel);

	/** Retrieves all the curves created in the subtree of this animation element.
		@param curves A list of animation curves to fill in.
			This list is not cleared. */
	void GetCurves(FCDAnimationCurveList& curves);

	/** [INTERNAL] Links the animation sub-tree with the other entities within the document.
		This function is used at the end of the import of a document to verify that all the
		necessary drivers were found.
		@return The status of the linkage. */
	FUStatus Link();

	/** [INTERNAL] Reads in the animation entity from a given COLLADA XML tree node.
		@param animationNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the animation. */
	virtual FUStatus LoadFromXML(xmlNode* animationNode);

	/** [INTERNAL] Writes out the \<animation\> element to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the animation tree.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;

	/** [INTERNAL] Retrieves the child source or sampler.
		This function should only be used by the FCDAnimationChannel class
		during the import of a COLLADA document.
		@param id The COLLADA id of a sampler or a source.
		@return The XML node tree for the sampler or the source. This pointer
			will be NULL if there are no child nodes for the given id. */
	xmlNode* FindChildById(const string& id);

	/** [INTERNAL] Links a possible driver with the animation curves contained
		within the subtree of this animation element.
		This function is used during the import of a COLLADA document.
		@param animated The driver animated value.
		@return Whether any linkage was done. */
	bool LinkDriver(FCDAnimated* animated);
};

#endif // _FCD_ANIMATION_H_
