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
	@file FCDAnimationChannel.h
	This file contains the FCDAnimationChannel class.
*/

#ifndef _FCD_ANIMATION_CHANNEL_H_
#define _FCD_ANIMATION_CHANNEL_H_

#include "FCDocument/FCDObject.h"

class FCDocument;
class FCDAnimated;
class FCDAnimation;
class FCDAnimationCurve;

/** A dynamically-sized array of animation curves. */
typedef vector<FCDAnimationCurve*> FCDAnimationCurveList;

/**
	A COLLADA animation channel.
	Each animation channel holds the animation curves for one animatable element,
	such as a single floating-point value, a 3D vector or a matrix.

	@see FCDAnimated
	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDAnimationChannel : public FCDObject
{
private:
	FCDAnimation* parent;

	// Channel target
	string targetPointer;
	string targetQualifier;

	// Maya-specific: the driver for this/these curves
	string driverPointer;
	int32 driverQualifier;

	FCDAnimationCurveList curves;

public:
	/** Constructor: do not use directly.
		Instead, call the FCDAnimation::AddChannel function.
		@param document The COLLADA document that owns the animation channel.
		@param parent The animation sub-tree that contains the animation channel. */
	FCDAnimationChannel(FCDocument* document, FCDAnimation* parent);

	/** Destructor: do not use directly.
		Instead, call the FCDAnimation::ReleaseChannel function. */
	virtual ~FCDAnimationChannel();
	
	/** Retrieves the animation sub-tree that contains the animation channel.
		@return The parent animation sub-tree. */
	FCDAnimation* GetParent() { return parent; }
	const FCDAnimation* GetParent() const { return parent; } /**< See above. */

	/** Retrieves the list of animation curves contained within the channel.
		@return The list of animation curves. */
	const FCDAnimationCurveList& GetCurves() const { return curves; }

	/** Retrieves the number of animation curves contained within the channel.
		@return The number of animation curves. */
	size_t GetCurveCount() const { return curves.size(); }

	/** Retrieves an animation curve contained within the channel.
		@param index The index of the animation curve.
		@return The animation curve at the given index. This pointer will be NULL
			if the index is out-of-bounds. */
	FCDAnimationCurve* GetCurve(size_t index) { FUAssert(index < GetCurveCount(), return NULL); return curves.at(index); }
	const FCDAnimationCurve* GetCurve(size_t index) const { FUAssert(index < GetCurveCount(), return NULL); return curves.at(index); } /**< See above. */

	/** Adds a new animation curve to this animation channel.
		@return The new animation curve. */
	FCDAnimationCurve* AddCurve();

	/** Releases an animation curve contained within this channel.
		@todo This function is not yet implemented, as it requires
		a lot more memory management than FCollada currently does.
		@param curve The animation curve to release. */
	void ReleaseCurve(FCDAnimationCurve* curve);

	/** [INTERNAL] Retrieves the target pointer prefix for this animation channel.
		This function is used during the import of a COLLADA document to match the
		target pointer prefixes with the animated elements.
		@return The target pointer prefix. */
	const string& GetTargetPointer() const { return targetPointer; }

	/** [INTERNAL] Retrieves the target qualifier for this animation channel.
		This function is used during the import of a COLLADA document.
		Where there is a target qualifier, there should be only one curve contained by the channel.
		@return The target qualifier. This value may be the empty string if the channel
		targets all the values targeted by the target pointer prefix. */
	const string& GetTargetQualifier() const { return targetQualifier; }

	/** [INTERNAL] Enforces the tarrget pointer prefix for the animation channel.
		This function is used during the export of a COLLADA document.
		@param p The new target pointer prefix. */
	void SetTargetPointer(const string& p) { targetPointer = p; }

	/** [INTERNAL] Considers the given animated element as the driver for this animation channel.
		@param animated An animated element. 
		@return Whether the animated element is in fact the driver for the animation channel. */
	bool LinkDriver(FCDAnimated* animated);

	/** [INTERNAL] Verifies that if a driver is used by this channel, then it was found during
		the import of the animated elements.
		@return The status of the verification. */
	FUStatus CheckDriver();

	/** [INTERNAL] Reads in the animation channel from a given COLLADA XML tree node.
		@param channelNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the animation channel. */
	FUStatus LoadFromXML(xmlNode* channelNode);

	/** [INTERNAL] Writes out the animation channel to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the animation channel.
		@return The created element XML tree node. */
	void WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_ANIMATION_CHANNEL_H_
