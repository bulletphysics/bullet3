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

#ifndef _FU_XML_NODE_ID_PAIR_H_
#define _FU_XML_NODE_ID_PAIR_H_

#ifdef HAS_LIBXML

class FUXmlNodeIdPair
{
public:
	FUCrc32::crc32 id;
	xmlNode* node;
};

typedef vector<FUXmlNodeIdPair> FUXmlNodeIdPairList;

#endif // HAS_LIBXML

#endif // _FU_XML_NODE_ID_PAIR_H_

