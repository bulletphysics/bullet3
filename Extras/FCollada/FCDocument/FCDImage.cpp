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

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDImage.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
#include "FUtils/FUFileManager.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDImage::FCDImage(FCDocument* document) : FCDEntity(document, "Image")
{
	width = 0;
	height = 0;
	depth = 0;
}

FCDImage::~FCDImage()
{
}

// Read in the image information from the COLLADA document
FUStatus FCDImage::LoadFromXML(xmlNode* imageNode)
{
	FUStatus status = FCDEntity::LoadFromXML(imageNode);
	if (!status) return status;
	if (!IsEquivalent(imageNode->name, DAE_IMAGE_ELEMENT))
	{
		return status.Warning(FS("Image library contains unknown element."), imageNode->line);
	}

	if(HasNodeProperty(imageNode, DAE_WIDTH_ELEMENT))
		width = FUStringConversion::ToUInt32(ReadNodeProperty(imageNode, DAE_WIDTH_ELEMENT));
	if(HasNodeProperty(imageNode, DAE_HEIGHT_ELEMENT))
		height = FUStringConversion::ToUInt32(ReadNodeProperty(imageNode, DAE_HEIGHT_ELEMENT));
	if(HasNodeProperty(imageNode, DAE_DEPTH_ELEMENT))
		depth = FUStringConversion::ToUInt32(ReadNodeProperty(imageNode, DAE_DEPTH_ELEMENT));

	// Read in the image's filename, within the <init_from> element: binary images are not supported.
	xmlNode* filenameSourceNode = FindChildByType(imageNode, DAE_INITFROM_ELEMENT);
	filename = TO_FSTRING(ReadNodeContentDirect(filenameSourceNode));

	// COLLADA 1.3 backward-compatibility
	if (filename.empty()) filename = TO_FSTRING(ReadNodeSource(imageNode));

	// Convert the filename to something the OS can use
	filename = GetDocument()->GetFileManager()->GetFilePath(filename);
	if (filename.empty())
	{
		return status.Fail(FS("Invalid filename for image: ") + TO_FSTRING(GetDaeId()), imageNode->line);
	}

	return status;
}

// Write out the image information to the COLLADA xml tree node
xmlNode* FCDImage::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* imageNode = WriteToEntityXML(parentNode, DAE_IMAGE_ELEMENT);
	if (!filename.empty())
	{
		fstring url = GetDocument()->GetFileManager()->GetFileURL(filename, true);
		AddChild(imageNode, DAE_INITFROM_ELEMENT, url);
	}

	if (width > 0) AddAttribute(imageNode, DAE_WIDTH_ELEMENT, width);
	if (height > 0) AddAttribute(imageNode, DAE_HEIGHT_ELEMENT, height);
	if (depth > 0) AddAttribute(imageNode, DAE_DEPTH_ELEMENT, depth);

	WriteToExtraXML(imageNode);
	return imageNode;
}
