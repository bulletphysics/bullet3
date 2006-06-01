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
	@file FCDGeometryMesh.h
	This file contains the FCDGeometryMesh class.
*/

#ifndef _FCD_GEOMETRY_MESH_H_
#define _FCD_GEOMETRY_MESH_H_

#include "FCDocument/FCDObject.h"
#include "FUtils/FUDaeEnum.h"

class FCDocument;
class FCDGeometry;
class FCDGeometryPolygons;
class FCDGeometrySource;

/** A dynamically-sized array of FCDGeometrySource objects. */
typedef vector<FCDGeometrySource*> FCDGeometrySourceList;
/** A dynamically-sized array of FCDGeometryPolygons objects. */
typedef vector<FCDGeometryPolygons*> FCDGeometryPolygonsList;

/**
	A COLLADA geometric mesh.
	A COLLADA geometric mesh is a list of vertices tied together in polygons.
	A set of per-vertex data is used to determine the vertices of the mesh.
	This data usually includes a single list: of vertex positions, but it may also
	contain per-vertex colors, per-vertex normals or per-vertex texture coordinates.
	The other data sources declare per-vertex-face data.

	The faces of a mesh may be split across different groups, as they may have
	different materials assigned to them. The FCDGeometryPolygons objects contains one such group
	of faces.

	@ingroup FCDGeometry
*/
class FCOLLADA_EXPORT FCDGeometryMesh : public FCDObject
{
private:
	FCDGeometry* parent;
	FCDGeometrySourceList sources;
	FCDGeometryPolygonsList polygons;

	FCDGeometrySourceList vertexSources;
	size_t faceCount, holeCount;
	size_t faceVertexCount;
	bool isDoubleSided; // Maya-specific, defaults to true

public:
	/** Contructor: do not use directly. Use FCDGeometry::AddMesh instead.
		@param document The COLLADA document which owns this mesh.
		@param parent The geometry entity which contains this mesh. */
	FCDGeometryMesh(FCDocument* document, FCDGeometry* parent);

	/** Destructor: do not use directly. All geometric meshes are released with the geometry that they belong to. */
	virtual ~FCDGeometryMesh();

	/** Retrieves the number of faces within the geometric mesh.
		@return The number of faces within the geometric mesh. */
	inline size_t GetFaceCount() const { return faceCount; }

	/** Retrieves the number of holes within the faces of the geometric mesh.
		As one face may contain multiple holes, this value may be larger than the number of faces.
		@return The number of holes within the faces of the geometric mesh. */
	inline size_t GetHoleCount() const { return holeCount; }

	/** Retrieves the total number of per-face vertices in the mesh.
		This function makes no assumption about the uniqueness of the per-face vertices.
		@return The total number of per-face vertices in the mesh. */
	inline size_t GetFaceVertexCount() const { return faceVertexCount; }

	/** Retrieves whether the mesh should be treated as double-sided.
		This flag does not belong to COLLADA but is exported at the geometric-level by ColladaMaya.
		@return Whether the mesh is double-sided. */
	inline bool IsDoubleSided() const { return isDoubleSided; }

	/** Retrieves the COLLADA id of the mesh.
		This is a shortcut to the parent geometry's COLLADA id.
		@return The COLLADA id of the mesh. */
	const string& GetDaeId() const;

	/** Retrieves the number of independent polygon groups.
		Each polygon group is represented within a FCDGeometryPolygons object.
		An independent polygon group is usually created to assign a different material to different parts of the mesh
		or to assign partial texture coordinates and texture tangents to different parts of the mesh.
		@return The number of independent polygon groups. */
	inline size_t GetPolygonsCount() const { return polygons.size(); }

	/** Retrieves a specific polygon group.
		Each polygon group is represented within a FCDGeometryPolygons object.
		An independent polygon group is usually created to assign a different material to different parts of the mesh
		or to assign partial texture coordinates and texture tangents to different parts of the mesh.
		@param index The index of the polygon group. This index should be less than the number
			of independent polygon groups returned by the GetPolygonsCount function.
		@return The polygon group. This pointer will be NULL if the index is out-of-bounds. */
	inline FCDGeometryPolygons* GetPolygons(size_t index) { FUAssert(index < GetPolygonsCount(), return NULL); return polygons.at(index); }
	inline const FCDGeometryPolygons* GetPolygons(size_t index) const { FUAssert(index < GetPolygonsCount(), return NULL); return polygons.at(index); } /**< See above. */

	/** Creates a new polygon group.
		Each polygon group is represented within a FCDGeometryPolygons object.
		The new polygon group will be assigned all the existing per-vertex data sources.
		No material will be assigned to the new polygon group.
		@return The new polygon group. This pointer should never be NULL. */
	FCDGeometryPolygons* AddPolygons();

	/** [INTERNAL] Retrieves the list of per-vertex data sources.
		There should usually be one per-vertex data source that contains positions.
		All the sources within this list are also present within the data source list.
		@return The list of per-vertex data sources. */
	inline FCDGeometrySourceList& GetVertexSources() { return vertexSources; }
	inline const FCDGeometrySourceList& GetVertexSources() const { return vertexSources; } /**< See above. */

	/** Retrieves the number of per-vertex data sources.
		This number should always be lesser or equal to the number of data sources, as each per-vertex
		data source is also included within the list of data sources.
		@return The number of per-vertex data sources. */
	inline size_t GetVertexSourceCount() const { return vertexSources.size(); }

	/** Retrieves a specific per-vertex data source.
		All the per-vertex data sources are also included in the list of data sources.
		@param index The index of the per-vertex data source. This index should be less than the number of
			per-vertex data sources returns by the GetVertexSourceCount function.
		@return The per-vertex data source. This pointer will be NULL if the index is out-of-bounds. */
	inline FCDGeometrySource* GetVertexSource(size_t index) { FUAssert(index < GetVertexSourceCount(), return NULL); return vertexSources.at(index); }
	inline const FCDGeometrySource* GetVertexSource(size_t index) const { FUAssert(index < GetVertexSourceCount(), return NULL); return vertexSources.at(index); } /**< See above. */

	/** Creates a new per-vertex data source for this geometric mesh.
		The per-vertex data source will be added to both the per-vertex data source list and the data source list.
		The new per-vertex data source will automatically be added to all the existing polygon groups.
		@return The new per-vertex data source. This pointer should never be NULL. */
	FCDGeometrySource* AddVertexSource();

	/** [INTERNAL] Retrieves the data source that matches the given COLLADA id.
		@param id A valid COLLADA id.
		@return The data source. This pointer will be NULL if no matching data source was found. */		
	FCDGeometrySource* FindSourceById(const string& id);
	const FCDGeometrySource* FindSourceById(const string& id) const; /**< See above. */

	/** Retrieves the per-vertex data that specifically contains the vertex positions.
		If there are more than one per-vertex data source that contains vertex positions, the first one is returned.
		@return A per-vertex data source that contains vertex positions. This pointer will be NULL
			in the unlikely possibility that there are no per-vertex data source that contains vertex positions. */
	FCDGeometrySource* GetPositionSource();
	const FCDGeometrySource* GetPositionSource() const; /**< See above. */

	/** Retrieves the number of data sources contained within this geometric mesh.
		@return The number of data sources within the mesh. */
	inline size_t GetSourceCount() const { return sources.size(); }

	/** Retrieves a specific data source.
		@param index The index of the data source. This index should be less than the number of
			data sources returns by the GetSourceCount function.
		@return The data source. This pointer will be NULL if the index is out-of-bounds. */
	inline FCDGeometrySource* GetSource(size_t index) { FUAssert(index < GetSourceCount(), return NULL); return sources.at(index); }
	inline const FCDGeometrySource* GetSource(size_t index) const { FUAssert(index < GetSourceCount(), return NULL); return sources.at(index); } /**< See above. */

	/** Creates a new data source for this geometric mesh.
		The new data source will not be added to any of the existing polygon groups.
		@return The new per-vertex data source. This pointer should never be NULL. */
	FCDGeometrySource* AddSource();

	/** Triangulates the mesh.
		A simple fanning techique is currently used: holes will not be triangulated correctly. */
	void Triangulate();

	/** [INTERNAL] Forces the recalculation of the hole count, vertex count, face-vertex counts and their offsets.
		Since the counts and offsets are buffered at the geometric mesh object level, this function allows the polygon
		groups to force the recalculation of the buffered values, when they are modified. */
	void Recalculate();
	
	/** [INTERNAL] Creates a copy of this mesh. You may use the FCDGeometry::Clone function instead of this function.
		Creates a full copy of the geometry, with the vertices overwritten
		by the given data: this is used when importing COLLADA 1.3 skin controllers.
		You will need to release the cloned entity.
		@see FCDGeometry::Clone.
		@param newPositions The list of vertex position that will
			overwrite the current mesh vertex positions. This list may be empty.
		@param newPositionsStride The stride, in bytes, of the newPositions list.
			For an empty newPositions list, this value is discarded.
		@param newNormals The list of vertex normals that will overwrite
			the current mesh vertex normals. This list may be empty.
		@param newNormalsStride The stride, in bytes, of the newNormals list.
			For an empty newNormals list, this value is discarded.
		@return The cloned geometry entity. You will need to release this pointer. */
	FCDGeometryMesh* Clone(FloatList& newPositions, uint32 newPositionsStride, FloatList& newNormals, uint32 newNormalsStride);

	/** [INTERNAL] Reads in the \<mesh\> element from a given COLLADA XML tree node.
		@param meshNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the geometric mesh.*/
	FUStatus LoadFromXML(xmlNode* meshNode);

	/** [INTERNAL] Writes out the \<mesh\> element to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the geometric mesh.
		@return The created \<mesh\> element XML tree node. */
	xmlNode* WriteToXML(xmlNode* parentNode) const;

	bool	m_convex;
};

#endif // _FCD_GEOMETRY_MESH_H_
