/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#ifndef _FCD_ANIMATION_CLIP_H_
#define _FCD_ANIMATION_CLIP_H_

class FCDocument;
class FCDAnimationCurve;

typedef vector<FCDAnimationCurve*> FCDAnimationCurveList;

#include "FCDocument/FCDEntity.h"
#include "FCDocument/FCDObject.h"

class FCOLLADA_EXPORT FCDAnimationClip : public FCDEntity
{
private:
	FCDAnimationCurveList curves;
	float start, end;

public:
	FCDAnimationClip(FCDocument* document);
	virtual ~FCDAnimationClip();

	FCDAnimationClip* Clone();

	// FCDEntity overrides
	virtual Type GetType() const { return ANIMATION_CLIP; }

	// Accessors
	FCDAnimationCurveList& GetClipCurves() { return curves; }
	const FCDAnimationCurveList& GetClipCurves() const { return curves; }
	float GetStart() const { return start; }
	float GetEnd() const { return end; }

	// Load a Collada animation node from the XML document
	virtual FUStatus LoadFromXML(xmlNode* clipNode);

	// Write out the COLLADA animations to the document
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_ANIMATION_CLIP_H_

