/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#ifndef _FCD_ASSET_H_
#define _FCD_ASSET_H_

#include "FCDocument/FCDObject.h"
#include "FUtils/FUDateTime.h"

class FCDAssetContributor;

typedef vector<FCDAssetContributor*> FCDAssetContributorList;

class FCOLLADA_EXPORT FCDAsset : public FCDObject
{
	FCDAssetContributorList contributors;
	FUDateTime creationDateTime;
	FUDateTime modifiedDateTime;
	fstring keywords;
	fstring revision;
	fstring subject;
	fstring title;
	FMVector3 upAxis;

	// <unit>
	fstring unitName;
	float unitConversionFactor;

public:
	FCDAsset(FCDocument* document);
	virtual ~FCDAsset();

	// Direct contributor list access
	inline FCDAssetContributorList& GetContributors() { return contributors; }
	inline const FCDAssetContributorList& GetContributors() const { return contributors; }
	inline size_t GetContributorCount() const { return contributors.size(); }
	inline FCDAssetContributor* GetContributor(size_t index) { return index < contributors.size() ? contributors[index] : NULL; }
	inline const FCDAssetContributor* GetContributor(size_t index) const { return index < contributors.size() ? contributors[index] : NULL; }
	FCDAssetContributor* AddContributor();

	// Direct accessors
	inline const FUDateTime& GetCreationDateTime() const { return creationDateTime; }
	inline const FUDateTime& GetModifiedDateTime() const { return modifiedDateTime; }
	inline const fstring& GetKeywords() const { return keywords; }
	inline const fstring& GetRevision() const { return revision; }
	inline const fstring& GetSubject() const { return subject; }
	inline const fstring& GetTitle() const { return title; }
	inline const FMVector3& GetUpAxis() const { return upAxis; }
	inline const fstring& GetUnitName() const { return unitName; }
	inline float GetUnitConversionFactor() const { return unitConversionFactor; }

	// Direct mutators
	inline void SetKeywords(const fstring& _keywords) { keywords = _keywords; }
	inline void SetRevision(const fstring& _revision) { revision = _revision; }
	inline void SetSubject(const fstring& _subject) { subject = _subject; }
	inline void SetTitle(const fstring& _title) { title = _title; }
	inline void SetUpAxis(const FMVector3& _upAxis) { upAxis = _upAxis; }
	inline void SetUnitName(const fstring& _unitName) { unitName = _unitName; }
	inline void SetUnitConversionFactor(float factor) { unitConversionFactor = factor; }

	// Read in the <asset> element from a COLLADA xml document
	FUStatus LoadFromXML(xmlNode* assetNode);

	// Write out the <asset> element to a COLLADA xml node tree
	// Calling this function will update the 'last modified' timestamp.
	xmlNode* WriteToXML(xmlNode* parentNode) const;
};

// Encapsulates the <asset><contributor> element
class FCOLLADA_EXPORT FCDAssetContributor : public FCDObject
{
private:
	fstring author;
	fstring authoringTool;
	fstring comments;
	fstring copyright;
	fstring sourceData;

public:
	FCDAssetContributor(FCDocument* document);
	virtual ~FCDAssetContributor();

	// Direct accessors
	inline const fstring& GetAuthor() const { return author; }
	inline const fstring& GetAuthoringTool() const { return authoringTool; }
	inline const fstring& GetComments() const { return comments; }
	inline const fstring& GetCopyright() const { return copyright; }
	inline const fstring& GetSourceData() const { return sourceData; }

	// Direct mutators
	inline void SetAuthor(const fstring& _author) { author = _author; }
	inline void SetAuthoringTool(const fstring& _authoringTool) { authoringTool = _authoringTool; }
	inline void SetComments(const fstring& _comments) { comments = _comments; }
	inline void SetCopyright(const fstring& _copyright) { copyright = _copyright; }
	inline void SetSourceData(const fstring& _sourceData) { sourceData = _sourceData; }

	// Returns whether this contributor element contain any valid data
	bool IsEmpty() const;

	// Read in the <asset><contributor> element from a COLLADA xml document
	FUStatus LoadFromXML(xmlNode* contributorNode, bool isPreCollada1_4=false);

	// Write out the <asset><contributor> element to a COLLADA xml node tree
	xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_ASSET_H_
