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
	@file FCDGeometryPolygons.h
	This file defines the FCDGeometryPolygons and the FCDGeometryPolygonsInput classes.
*/

#ifndef _FCD_GEOMETRY_POLYGONS_H_
#define _FCD_GEOMETRY_POLYGONS_H_

#include "FCDocument/FCDObject.h"
#include "FUtils/FUDaeEnum.h"

class FCDocument;
class FRMeshPolygons;
class FCDMaterial;
class FCDGeometryMesh;
class FCDGeometrySource;

/**
	An input data source for one mesh polygons set.
	This structure knows the type of input data in the data source
	as well as the set and offset for the data. It also contains a
	pointer to the mesh data source.

	Many polygon set inputs may have the same offset (or 'idx') when multiple
	data sources are compressed together within the COLLADA document.
	In this case, one and only one of the polygon set input will have
	the 'ownsIdx' flag set. A polygon set input with this flag set will
	contain valid indices. To find the indices of any polygon set input,
	it is recommended that you use the FCDGeometryPolygons::FindIndicesForIdx function.

	@ingroup FCDGeometry
*/
class FCDGeometryPolygonsInput
{
public:
	/** Constructor. */
    FCDGeometryPolygonsInput();

	/** Determines the type of data to input. */
	FUDaeGeometryInput::Semantic semantic; 

	/** Offset within the COLLADA document for this input.
		All the inputs with the same offset value use the same indices within the COLLADA document. */
	uint32 idx; 

	/** [INTERNAL] Offset owner flags. One and only one polygon set input will have this flag set for each offset value.
		A polygon set input with this flag set will contain valid indices within the 'indices' member variable.
		You should not set or access this flag directly. Instead, use the FCDGeometryPolygons::FindIndicesForIdx function. */
	bool ownsIdx;

	/** [INTERNAL] Tessellation indices. Use these indices to generate a list of unique vertices and generate your vertex buffers.
		You should not set or access the indices directly. Instead, use the FCDGeometryPolygons::FindIndicesForIdx function. */
	UInt32List indices;

	/** Data source. This is the data source into which the indices are indexing. You need to take the data source
		stride into consideration when unindexing the data. */
	FCDGeometrySource* source;

	/** Input set. Used to group together the texture coordinates with the texture tangents and binormals.
		ColladaMax: this value should also represent the map channel index or texture coordinates
		and vertex color channels. */
	int32 set;
};

/** A dynamically-sized array of FCDGeometryPolygonsInput objects. */
typedef vector<FCDGeometryPolygonsInput*> FCDGeometryPolygonsInputList;

/**
	A mesh polygon set.
	Each polygon set contains a list of inputs and the tessellation information
	to make polygons out of the data and indices of the input. FCollada
	supports triangle lists as well as polygon lists and lists of polygons with holes.
	This implies that each face has an undeterminate number of vertices.
	The tessellation information creates polygons, but may also creates holes within the polygons.

	@ingroup FCDGeometry
*/
class FCOLLADA_EXPORT FCDGeometryPolygons : public FCDObject
{
private:
	FCDGeometryPolygonsInputList inputs;
	FCDGeometryPolygonsInputList idxOwners;
	UInt32List faceVertexCounts;
	FCDGeometryMesh* parent;
	size_t faceVertexCount;
	UInt32List holeFaces;

	// Buffered statistics
	size_t faceOffset;
	size_t faceVertexOffset;
	size_t holeOffset;

	// Material for this set of polygons
	fstring materialSemantic;

public:
	/** Constructor: do not use directly. Instead, use the FCDGeometryMesh::AddPolygons function
		to create new polygon sets.
		@param document The COLLADA document which owns this polygon set.
		@param parent The geometric mesh which contains this polygon set.*/
	FCDGeometryPolygons(FCDocument* document, FCDGeometryMesh* parent);

	/** Destructor: do not use directly.
		The mesh which contains this polygon set will handle its release. */
	virtual ~FCDGeometryPolygons();

	/** Retrieves the list of face-vertex counts. Each face within the polygon set
		has one or more entry within this list, depending on the number of holes within that face.
		Each face-vertex count indicates the number of ordered indices
		within the polygon set inputs that are used to generate a face or its holes.
		To find out if a face-vertex count represents a face or its holes, check
		the hole-faces list retrieved using the GetHoleFaces function.
		Indirectly, the face-vertex count indicates the degree of the polygon.
		@see GetHoleFaces @see GetHoleCount
		@return The list of face-vertex counts.*/
	inline const UInt32List& GetFaceVertexCounts() const { return faceVertexCounts; }

	/** Retrieves the number of holes within the faces of the polygon set.
		@return The number of holes within the faces of the polygon set. */
	inline size_t GetHoleCount() const { return holeFaces.size(); }

	/** Retrieves the number of faces within the polygon set.
		@return The number of faces within the polygon set. */
	inline size_t GetFaceCount() const { return faceVertexCounts.size() - GetHoleCount(); }

	/** Retrieves the number of faces which appear before this polygon set within the geometric mesh.
		This value is useful when traversing all the faces of a geometric mesh.
		@return The number of faces in previous polygon sets. */
	inline size_t GetFaceOffset() const { return faceOffset; }

	/** Retrieves the total number of face-vertex pairs within the polygon set.
		This value is the total of all the values within the face-vertex count list.
		Do remember that the list of face-vertex pairs includes holes.
		@return The total number of face-vertex pairs within the polygon set. */
	inline size_t GetFaceVertexCount() const { return faceVertexCount; }

	/** Retrieves the number of face-vertex pairs for a given face.
		This value includes face-vertex pairs that create the polygon and its holes.
		@param index A face index.
		@return The number of face-vertex pairs for a given face. */
	size_t GetFaceVertexCount(size_t index) const;

	/** Retrieves the total number of face-vertex pairs which appear
		before this polygon set within the geometric mesh.
		This value is useful when traversing all the face-vertex pairs of a geometric mesh.
		@return The number of face-vertex pairs in previous polygon sets. */
	inline size_t GetFaceVertexOffset() const { return faceVertexOffset; }

	/** Retrieves the number of holes which appear before this polygon set.
		This value is useful when traversing all the face-vertex pairs of a geometric mesh. */
	inline size_t GetHoleOffset() const { return holeOffset; }

	/** Retrieves the number of face-vertex pairs which appear
		before a given face within the polygon set.
		This value is useful when doing per-vertex mesh operations within the polygon set.
		@param index The index of the face.
		@return The number of face-vertex pairs before the given face, within the polygon set. */
	size_t GetFaceVertexOffset(size_t index) const;

	/** [INTERNAL] Sets the number of faces in previous polygon sets.
		Used by the FCDGeometryMesh::Recalculate function.
		@param offset The number of faces in previous polygon sets. */
	inline void SetFaceOffset(size_t offset) { faceOffset = offset; }

	/** [INTERNAL] Sets the number of face-vertex pairs in previous polygon sets.
		Used by the FCDGeometryMesh::Recalculate function.
		@param offset The number of face-vertex pairs in previous polygon sets. */
	inline void SetFaceVertexOffset(size_t offset) { faceVertexOffset = offset; }

	/** [INTERNAL] Sets the number of holes in previous polygon sets.
		Used by the FCDGeometryMesh::Recalculate function.
		@param offset The number of holes in previous polygon sets. */
	inline void SetHoleOffset(size_t offset) { holeOffset = offset; }

	/** Creates a new face.
		Enough indices to fill the face will be added to the polygon set inputs: you will
		want to overwrite those, as they will all be set to zero.
		@param degree The degree of the polygon. This number implies the number of indices
			that will be expected, in order, within each of the input index lists. */
	void AddFace(uint32 degree);

	/** Removes a face
		@param index The index of the face to remove. All the indices associated
			with this face will also be removed. */
	void RemoveFace(size_t index);

	/** Retrieves the list of polygon set inputs.
		@see FCDGeometryPolygonsInput
		@return The list of polygon set inputs. */
	inline FCDGeometryPolygonsInputList& GetInputs() { return inputs; }
	inline const FCDGeometryPolygonsInputList& GetInputs() const { return inputs; } /**< See above. */

	/** Retrieves the number of polygon set inputs.
		@return The number of polygon set inputs. */
	inline size_t GetInputCount() const { return inputs.size(); }

	/** Retrieves a specific polygon set input.
		@param index The index of the polygon set input. This index should
			not be greater than or equal to the number of polygon set inputs.
		@return The specific polygon set input. This pointer will be NULL if the index is out-of-bounds. */
	inline FCDGeometryPolygonsInput* GetInput(size_t index) { FUAssert(index < GetInputCount(), return NULL); return inputs.at(index); }
	inline const FCDGeometryPolygonsInput* GetInput(size_t index) const { FUAssert(index < GetInputCount(), return NULL); return inputs.at(index); } /**< See above. */

	/** Creates a new polygon set input.
		@param source The data source for the polygon set input.
		@param offset The tessellation indices offset for the polygon set input.
			If this value is new to the list of polygon inputs, you will need to fill in the indices.
			Please use the FindIndices function to verify that the offset is new and that indices need
			to be provided. The offset of zero is reserved for per-vertex data sources.
		@return The new polygon set input. */
	FCDGeometryPolygonsInput* AddInput(FCDGeometrySource* source, uint32 offset);

	/** Deletes a polygon set input.
		This function releases the memory held by the polygon set input as well as moves 
		the indices to another polygon set input with the same offset, if the offset is re-used.
		@param input The polygon set input to delete. */
	void ReleaseInput(FCDGeometryPolygonsInput* input);

	/** Retrieves the list of entries within the face-vertex count list
		that are considered holes. COLLADA does not support holes within holes,
		so each entry within this list implies a hole within the previous face.
		@see GetFaceVertexCounts
		@return The list of hole entries within the face-vertex counts. */
	inline const UInt32List& GetHoleFaces() const { return holeFaces; }

	/** Retrieves the number of holes within faces of the polygon set that appear
		before the given face index. This value is useful when trying to access
		a specific face of a mesh, as holes and faces appear together within the 
		face-vertex degree list.
		@param index A face index.
		@return The number of holes within the polygon set that appear
			before the given face index. */
	size_t GetHoleCountBefore(size_t index) const;

	/** Retrieves the number of holes within a given face.
		@param index A face index.
		@return The number of holes within the given face. */
	size_t GetHoleCount(size_t index) const;

	/** Retrieves the first polygon set input found that has the given data type.
		@param semantic A type of geometry data.
		@return The polygon set input. This pointer will be NULL if 
			no polygon set input matches the data type. */
	FCDGeometryPolygonsInput* FindInput(FUDaeGeometryInput::Semantic semantic);
	const FCDGeometryPolygonsInput* FindInput(FUDaeGeometryInput::Semantic semantic) const; /**< See above. */

	/** Retrieves the polygon set input that points towards a given data source.
		@param source A geometry data source.
		@return The polygon set input. This pointer will be NULL if
			no polygon set input matches the data source. */
	FCDGeometryPolygonsInput* FindInput(FCDGeometrySource* source);
	const FCDGeometryPolygonsInput* FindInput(const FCDGeometrySource* source) const; /**< See above. */

	/** [INTERNAL] Retrieves the polygon set input that points towards a given data source.
		@param sourceId The COLLADA id of a geometry data source.
		@return The polygon set input. This pointer will be NULL if
			no polygon set input matches the COLLADA id. */
	FCDGeometryPolygonsInput* FindInput(const string& sourceId);

	/** Retrieves all the polygon set inputs that have the given data type.
		@param semantic A type of geometry data.
		@param inputs A list of polygon set inputs to fill in. This list is not emptied by the function
			and may remain untouched, if no polygon set input matches the given data type. */
	void FindInputs(FUDaeGeometryInput::Semantic semantic, FCDGeometryPolygonsInputList& inputs);

	/** Retrieves the tessellation indices for a given polygon set input offset.
		@deprecated Instead, use the FindIndices function.
		@param idx A polygon set input offset.
		@return The tessellation indices corresponding to the offset. This pointer
			will be NULL if there are no polygon set input which uses the given offset. */
	UInt32List* FindIndicesForIdx(uint32 idx);
	const UInt32List* FindIndicesForIdx(uint32 idx) const; /**< See above. */

	/** Retrieves the first tessellation index list for a given data source.
		@param source A data source.
		@return The first tessellation index list corresponding to the data source. This pointer
			will be NULL if the data source is not used within this polygon set. */
	UInt32List* FindIndices(FCDGeometrySource* source);
	const UInt32List* FindIndices(const FCDGeometrySource* source) const; /**< See above. */

	/** Retrieves the tessellation indices for a given polygon set input.
		@param input A given polygon set input.
		@return The tessellation indices corresponding to the polygon set input. This pointer
			will be NULL if the polygon set input is not used within this polygon set. */
	UInt32List* FindIndices(FCDGeometryPolygonsInput* input);
	const UInt32List* FindIndices(const FCDGeometryPolygonsInput* input) const; /**< See above. */

	/** Retrieves the symbolic name for the material used on this polygon set.
		Match this symbolic name within a FCDGeometryInstance to get the
		correct material instance.
		@return A symbolic material name. */
	inline const fstring& GetMaterialSemantic() const { return materialSemantic; }

	/** Triangulates the polygon set.
		A simple fanning techique is currently used: holes will not be triangulated correctly. */
	void Triangulate();

	/** [INTERNAL] Recalculates the buffered offset and count values for this polygon set. */
	void Recalculate();

	/** [INTERNAL] Reads in the polygon set element from a given COLLADA XML tree node.
		COLLADA has multiple polygon set elements. The most common ones are \<triangles\> and \<polylist\>.
		@param polygonNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the polygon set.*/
	FUStatus LoadFromXML(xmlNode* polygonNode);

	/** [INTERNAL] Writes out the correct polygon set element to the given COLLADA XML tree node.
		COLLADA has multiple polygon set elements. The most common ones are \<triangles\> and \<polylist\>.
		@param parentNode The COLLADA XML parent node in which to insert the geometric mesh.
		@return The created XML tree node. */
	xmlNode* WriteToXML(xmlNode* parentNode) const;

	/** [INTERNAL] Creates a copy of this mesh.
		You should use the FCDGeometry::Clone function instead of this function.
		You will need to release the cloned entity.
		@param cloneParent The geometric mesh which will contain the cloned polygon set.
		@return An identical copy of the polygon set. */
	FCDGeometryPolygons* Clone(FCDGeometryMesh* cloneParent);

private:
	// Performs operations needed before tessellation
	bool InitTessellation(xmlNode* itNode, 
		uint32* localFaceVertexCount, UInt32List& allIndices, 
		const char* content, xmlNode*& holeNode, uint32 idxCount, 
		bool* failed);
};

#endif // _FCD_GEOMETRY_POLYGONS_H_
