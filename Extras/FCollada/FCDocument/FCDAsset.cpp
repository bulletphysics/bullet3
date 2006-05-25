/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FUtils/FUDateTime.h"
#include "FCDocument/FCDAsset.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDAsset::FCDAsset(FCDocument* document) : FCDObject(document, "FCDAsset")
{
	unitConversionFactor = 1.0f;
	unitName = FC("meter");
	upAxis = FMVector3::YAxis;
	creationDateTime = modifiedDateTime = FUDateTime::GetNow();
}

FCDAsset::~FCDAsset()
{
	while(!contributors.empty())
	{
		SAFE_DELETE(contributors.back());
		contributors.pop_back();
	}
}

// Insert a new contributor within the list
FCDAssetContributor* FCDAsset::AddContributor()
{
	FCDAssetContributor* contributor = new FCDAssetContributor(GetDocument());
	contributors.push_back(contributor);
	return contributor;
}

// Read in the <asset> element from a COLLADA xml document
FUStatus FCDAsset::LoadFromXML(xmlNode* assetNode)
{
	FUStatus status;
	bool isPreCollada1_4 = false;
	for (xmlNode* child = assetNode->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;

		const char* content = ReadNodeContentDirect(child);
		if (IsEquivalent(child->name, DAE_CONTRIBUTOR_ASSET_ELEMENT))
		{
			FCDAssetContributor* contributor = new FCDAssetContributor(GetDocument());
			contributors.push_back(contributor);
			status.AppendStatus(contributor->LoadFromXML(child, false));
		}
		else if (IsEquivalent(child->name, DAE_CREATED_ASSET_PARAMETER))
		{
			FUStringConversion::ToDateTime(content, creationDateTime);
		}
		else if (IsEquivalent(child->name, DAE_KEYWORDS_ASSET_PARAMETER))
		{
			keywords = TO_FSTRING(content);
		}
		else if (IsEquivalent(child->name, DAE_MODIFIED_ASSET_PARAMETER))
		{
			FUStringConversion::ToDateTime(content, modifiedDateTime ); 
		}
		else if (IsEquivalent(child->name, DAE_REVISION_ASSET_PARAMETER))
		{
			revision = TO_FSTRING(content);
		}
		else if (IsEquivalent(child->name, DAE_SUBJECT_ASSET_PARAMETER))
		{
			subject = TO_FSTRING(content);
		}
		else if (IsEquivalent(child->name, DAE_TITLE_ASSET_PARAMETER))
		{
			title = TO_FSTRING(content);
		}
		else if (IsEquivalent(child->name, DAE_UNITS_ASSET_PARAMETER))
		{
			unitName = TO_FSTRING(ReadNodeName(child));
			unitConversionFactor = FUStringConversion::ToFloat(ReadNodeProperty(child, DAE_METERS_ATTRIBUTE));
			if (unitName.empty()) unitName = FC("UNKNOWN");
			if (IsEquivalent(unitConversionFactor, 0.0f) || unitConversionFactor < 0.0f) unitConversionFactor = 1.0f;
		}
		else if (IsEquivalent(child->name, DAE_UPAXIS_ASSET_PARAMETER))
		{
			if (IsEquivalent(content, DAE_X_UP)) upAxis = FMVector3::XAxis;
			else if (IsEquivalent(content, DAE_Y_UP)) upAxis = FMVector3::YAxis;
			else if (IsEquivalent(content, DAE_Z_UP)) upAxis = FMVector3::ZAxis;
		}
		else if (IsEquivalent(child->name, DAE_AUTHOR_ASSET_PARAMETER) || IsEquivalent(child->name, DAE_AUTHORINGTOOL_ASSET_PARAMETER)
			|| IsEquivalent(child->name, DAE_COMMENTS_ASSET_PARAMETER) || IsEquivalent(child->name, DAE_SOURCEDATA_ASSET_PARAMETER)
			|| IsEquivalent(child->name, DAE_COPYRIGHT_ASSET_PARAMETER))
		{
			isPreCollada1_4 = true;
		}
		else
		{
			status.Warning(FS("Unknown <asset> child element: ") + TO_FSTRING((const char*) child->name), child->line);
		}
	}

	// COLLADA 1.3 Backward Compatibility: Look for the contributor information within the <asset> element
	if (isPreCollada1_4)
	{
		FCDAssetContributor* contributor = new FCDAssetContributor(GetDocument());
		contributor->LoadFromXML(assetNode, true);
		if (!contributor->IsEmpty()) contributors.push_back(contributor);
		else SAFE_DELETE(contributor);
	}

	return status;
}

// Write out the <asset> element to a COLLADA xml node tree
xmlNode* FCDAsset::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* assetNode = AddChild(parentNode, DAE_ASSET_ELEMENT);

	// Update the 'last modified time'
	FCDAsset* hackedAsset = const_cast<FCDAsset*>(this);
	hackedAsset->modifiedDateTime = FUDateTime::GetNow();

	// Write out the contributors first.
	for (FCDAssetContributorList::const_iterator itC = contributors.begin(); itC != contributors.end(); ++itC)
	{
		(*itC)->WriteToXML(assetNode);
	}

	// Write out the parameters, one by one and in the correct order.
	AddChild(assetNode, DAE_CREATED_ASSET_PARAMETER, FUStringConversion::ToString(creationDateTime));
	if (!keywords.empty()) AddChild(assetNode, DAE_KEYWORDS_ASSET_PARAMETER, keywords);
	AddChild(assetNode, DAE_MODIFIED_ASSET_PARAMETER, FUStringConversion::ToString(modifiedDateTime));
	if (!revision.empty()) AddChild(assetNode, DAE_REVISION_ASSET_PARAMETER, revision);
	if (!subject.empty()) AddChild(assetNode, DAE_SUBJECT_ASSET_PARAMETER, subject);
	if (!title.empty()) AddChild(assetNode, DAE_TITLE_ASSET_PARAMETER, title);

	// Finally: <unit> and <up_axis>
	xmlNode* unitNode = AddChild(assetNode, DAE_UNITS_ASSET_PARAMETER);
	AddAttribute(unitNode, DAE_METERS_ATTRIBUTE, unitConversionFactor);
	AddAttribute(unitNode, DAE_NAME_ATTRIBUTE, unitName);
	AddChild(assetNode, DAE_UPAXIS_ASSET_PARAMETER, FUStringConversion::ToString(subject));
	return assetNode;
}

FCDAssetContributor::FCDAssetContributor(FCDocument* document) : FCDObject(document, "FCDAssetContributor") {}
FCDAssetContributor::~FCDAssetContributor() {}

// Returns whether this contributor element contain any valid data
bool FCDAssetContributor::IsEmpty() const
{
	return author.empty() && authoringTool.empty() && comments.empty() && copyright.empty() && sourceData.empty();
}

// Read in the <asset><contributor> element from a COLLADA xml document
FUStatus FCDAssetContributor::LoadFromXML(xmlNode* contributorNode, bool isPreCollada1_4)
{
	FUStatus status;
	for (xmlNode* child = contributorNode->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;

		const char* content = ReadNodeContentDirect(child);
		if (IsEquivalent(child->name, DAE_AUTHOR_ASSET_PARAMETER))
		{
			author = TO_FSTRING(content);
		}
		else if (IsEquivalent(child->name, DAE_AUTHORINGTOOL_ASSET_PARAMETER))
		{
			authoringTool = TO_FSTRING(content);
		}
		else if (IsEquivalent(child->name, DAE_COMMENTS_ASSET_PARAMETER))
		{
			comments = TO_FSTRING(content);
		}
		else if (IsEquivalent(child->name, DAE_COPYRIGHT_ASSET_PARAMETER))
		{
			copyright = TO_FSTRING(content);
		}
		else if (IsEquivalent(child->name, DAE_SOURCEDATA_ASSET_PARAMETER))
		{
			sourceData = TO_FSTRING(content);
		}
		else if (!isPreCollada1_4)
		{
			status.Warning(FS("Unknown <asset><contributor> child element: ") + TO_FSTRING((const char*) child->name), child->line);
		}
	}
	return status;
}

// Write out the <asset><contributor> element to a COLLADA xml node tree
xmlNode* FCDAssetContributor::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* contributorNode = NULL;
	if (!IsEmpty())
	{
		contributorNode = AddChild(parentNode, DAE_CONTRIBUTOR_ASSET_ELEMENT);
		if (!author.empty()) AddChild(contributorNode, DAE_AUTHOR_ASSET_PARAMETER, author);
		if (!authoringTool.empty()) AddChild(contributorNode, DAE_AUTHORINGTOOL_ASSET_PARAMETER, authoringTool);
		if (!comments.empty()) AddChild(contributorNode, DAE_COMMENTS_ASSET_PARAMETER, comments);
		if (!copyright.empty()) AddChild(contributorNode, DAE_COPYRIGHT_ASSET_PARAMETER, copyright);
		if (!sourceData.empty()) AddChild(contributorNode, DAE_SOURCEDATA_ASSET_PARAMETER, sourceData);
	}
	return contributorNode;
}
