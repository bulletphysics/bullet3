/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FCDGeometrySource.h
	This file contains the FCDGeometrySource class.
*/

#ifndef _FCD_GEOMETRY_SOURCE_H_
#define _FCD_GEOMETRY_SOURCE_H_


#include "FCDocument/FCDObject.h"
#include "FUtils/FUDaeEnum.h"

class FCDAnimated;

/** A dynamically-sized array of FCDAnimated objects. */
typedef vector<FCDAnimated*> FCDAnimatedList;

/**
	A COLLADA data source for geometric meshes.

	A COLLADA data source for geometric meshes contains a list of floating-point values and the information
	to parse these floating-point values into meaningful content: the stride of the list and the type of data
	that the floating-point values represent. When the floating-point values are split according to the stride,
	you get multiple elemental values of the given type. A data source may also have a user-generated name to
	identify the data within. The name is optional and is used to keep around the user-friendly name for texture coordinate
	sets or color sets.

	The values of the COLLADA data source may be animated individually, or together: as an element.

	@ingroup FCDGeometry
*/
class FCOLLADA_EXPORT FCDGeometrySource : public FCDObjectWithId
{
private:
	fstring name;
	FloatList sourceData;
	uint32 sourceStride;
	xmlNode* sourceNode;
	FUDaeGeometryInput::Semantic sourceType;

	// The animated values held here are contained within the document.
	FCDAnimatedList animatedValues;

public:
	/** Constructor: do not use directly.
		Use FCDGeometryMesh::AddSource or FCDGeometryMesh::AddValueSource instead.
		@param document The COLLADA document which owns the data source. */
	FCDGeometrySource(FCDocument* document);

	/** Destructor: do not use directly.
		The geometric mesh which contains the data source will release it. */
	virtual ~FCDGeometrySource();

	/** Retrieves the name of the data source. The name is optional and is used to
		keep around a user-friendly name for texture coordinate sets or color sets.
		@return The name of the data source. */
	const fstring& GetName() const { return name; }

	/** Retrieves the pure data of the data source. This is a dynamically-sized array of
		floating-point values that contains all the data of the source.
		@return The pure data of the data source. */
	FloatList& GetSourceData() { return sourceData; }
	const FloatList& GetSourceData() const { return sourceData; } /**< See above. */

	/** Retrieves the stride of the data within the source.
		There is no guarantee that the number of data values within the source is a multiple of the stride,
		yet you should always verify that the stride is at least the wanted dimension. For example, there is
		no guarantee that your vertex position data source has a stride of 3. 3dsMax is known to always
		export 3D texture coordinate positions.
		@return The stride of the data. */
	uint32 GetSourceStride() const { return sourceStride; }

	/** @deprecated Retrieves the COLLADA id for the source.
		Use the class parent's GetDaeId function instead.
		@return The COLLADA id. */
	const string& GetSourceId() const { return GetDaeId(); }

	/** Retrieves the list of animated values for the data of the source.
		@return The list of animated values. */
	FCDAnimatedList& GetAnimatedValues() { return animatedValues; }
	const FCDAnimatedList& GetAnimatedValues() const { return animatedValues; } /**< See above. */

	/** @deprecated [INTERNAL] Retrieves the XML tree node that represent this source.
		This is used when computing the list of animated values.
		@todo Take the XML tree node out of this class.
		@return The XML tree node. This pointer is invalid if accessed after the document is
			fully parsed. */
	xmlNode* GetSourceNode() { return sourceNode; } // Should be taken out of this class

	/** Retrieves the type of data contained within the source.
		Common values for the type of data are POSITION, NORMAL, COLOR and TEXCOORD.
		Please see FUDaeGeometryInput for more information.
		@see FUDaeGeometryInput.
		@return The type of data contained within the source. */
	FUDaeGeometryInput::Semantic GetSourceType() const { return sourceType; }

	/** Sets the user-friendly name of the data source. The name is optional and is used to
		keep around a user-friendly name for texture coordinate sets or color sets.
		@param _name The user-friendly name of the data source. */		
	void SetName(const fstring& _name) { name = _name; }

	/** Overwrites the data contained within the data source.
		@param _sourceData The new data for this source.
		@param _sourceStride The stride for the new data.
		@param offset The offset at which to start retrieving the new data.
			This argument defaults at 0 to indicate that the data copy should start from the beginning.
		@param count The number of data entries to copy into the data source.
			This argument defaults at 0 to indicate that the data copy should include everything. */
	void SetSourceData(const FloatList& _sourceData, uint32 _sourceStride, size_t offset=0, size_t count=0);

	/** [INTERNAL] Sets the XML tree node associated with the data source.
		@todo Take the XML tree node out of this class.
		@param _sourceNode A XML tree node. */
	void SetSourceNode(xmlNode* _sourceNode) { sourceNode = _sourceNode; }

	/** Sets the type of data contained within this data source.
		@param type The new type of data for this data source. */
	void SetSourceType(FUDaeGeometryInput::Semantic type);

	/** [INTERNAL] Clones this data source. You will need to release this pointer manually.
		@return An identical copy of the data source. */
	FCDGeometrySource* Clone() const;

	/** [INTERNAL] Reads in the \<source\> element from a given COLLADA XML tree node.
		@param sourceNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the data source.*/
	FUStatus LoadFromXML(xmlNode* sourceNode);

	/** [INTERNAL] Writes out the \<source\> element to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the data source.
		@return The created \<source\> element XML tree node. */
	xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_GEOMETRY_SOURCE_H_
