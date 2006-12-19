/*
 * Copyright 2006 Sony Computer Entertainment Inc.
 *
 * Licensed under the SCEA Shared Source License, Version 1.0 (the "License"); you may not use this 
 * file except in compliance with the License. You may obtain a copy of the License at:
 * http://research.scea.com/scea_shared_source_license.html
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License 
 * is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or 
 * implied. See the License for the specific language governing permissions and limitations under the 
 * License. 
 */
#ifndef __domConvex_mesh_h__
#define __domConvex_mesh_h__

#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domSource.h>
#include <dom/domVertices.h>
#include <dom/domLines.h>
#include <dom/domLinestrips.h>
#include <dom/domPolygons.h>
#include <dom/domPolylist.h>
#include <dom/domTriangles.h>
#include <dom/domTrifans.h>
#include <dom/domTristrips.h>
#include <dom/domExtra.h>

/**
 * The definition of the convex_mesh element is identical to the mesh element
 * with the exception that  instead of a complete description (source, vertices,
 * polygons etc.), it may simply point to another  geometry to derive its
 * shape. The latter case means that the convex hull of that geometry should
 * be computed and is indicated by the optional “convex_hull_of” attribute.
 */
class domConvex_mesh : public daeElement
{
public:
	COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::CONVEX_MESH; }
protected:  // Attribute
/**
 *  The convex_hull_of attribute is a URI string of geometry to compute the
 * convex hull of.  Optional attribute. 
 */
	xsAnyURI attrConvex_hull_of;

protected:  // Elements
	domSource_Array elemSource_array;
	domVerticesRef elemVertices;
	domLines_Array elemLines_array;
	domLinestrips_Array elemLinestrips_array;
	domPolygons_Array elemPolygons_array;
	domPolylist_Array elemPolylist_array;
	domTriangles_Array elemTriangles_array;
	domTrifans_Array elemTrifans_array;
	domTristrips_Array elemTristrips_array;
/**
 *  The extra element may appear any number of times.  @see domExtra
 */
	domExtra_Array elemExtra_array;
	/**
	 * Used to preserve order in elements that do not specify strict sequencing of sub-elements.
	 */
	daeElementRefArray _contents;
	/**
	 * Used to preserve order in elements that have a complex content model.
	 */
	daeUIntArray       _contentsOrder;


public:	//Accessors and Mutators
	/**
	 * Gets the convex_hull_of attribute.
	 * @return Returns a xsAnyURI reference of the convex_hull_of attribute.
	 */
	xsAnyURI &getConvex_hull_of() { return attrConvex_hull_of; }
	/**
	 * Gets the convex_hull_of attribute.
	 * @return Returns a constant xsAnyURI reference of the convex_hull_of attribute.
	 */
	const xsAnyURI &getConvex_hull_of() const { return attrConvex_hull_of; }
	/**
	 * Sets the convex_hull_of attribute.
	 * @param atConvex_hull_of The new value for the convex_hull_of attribute.
	 */
	void setConvex_hull_of( const xsAnyURI &atConvex_hull_of ) { attrConvex_hull_of = atConvex_hull_of;
	 _validAttributeArray[0] = true; }

	/**
	 * Gets the source element array.
	 * @return Returns a reference to the array of source elements.
	 */
	domSource_Array &getSource_array() { return elemSource_array; }
	/**
	 * Gets the source element array.
	 * @return Returns a constant reference to the array of source elements.
	 */
	const domSource_Array &getSource_array() const { return elemSource_array; }
	/**
	 * Gets the vertices element.
	 * @return a daeSmartRef to the vertices element.
	 */
	const domVerticesRef getVertices() const { return elemVertices; }
	/**
	 * Gets the lines element array.
	 * @return Returns a reference to the array of lines elements.
	 */
	domLines_Array &getLines_array() { return elemLines_array; }
	/**
	 * Gets the lines element array.
	 * @return Returns a constant reference to the array of lines elements.
	 */
	const domLines_Array &getLines_array() const { return elemLines_array; }
	/**
	 * Gets the linestrips element array.
	 * @return Returns a reference to the array of linestrips elements.
	 */
	domLinestrips_Array &getLinestrips_array() { return elemLinestrips_array; }
	/**
	 * Gets the linestrips element array.
	 * @return Returns a constant reference to the array of linestrips elements.
	 */
	const domLinestrips_Array &getLinestrips_array() const { return elemLinestrips_array; }
	/**
	 * Gets the polygons element array.
	 * @return Returns a reference to the array of polygons elements.
	 */
	domPolygons_Array &getPolygons_array() { return elemPolygons_array; }
	/**
	 * Gets the polygons element array.
	 * @return Returns a constant reference to the array of polygons elements.
	 */
	const domPolygons_Array &getPolygons_array() const { return elemPolygons_array; }
	/**
	 * Gets the polylist element array.
	 * @return Returns a reference to the array of polylist elements.
	 */
	domPolylist_Array &getPolylist_array() { return elemPolylist_array; }
	/**
	 * Gets the polylist element array.
	 * @return Returns a constant reference to the array of polylist elements.
	 */
	const domPolylist_Array &getPolylist_array() const { return elemPolylist_array; }
	/**
	 * Gets the triangles element array.
	 * @return Returns a reference to the array of triangles elements.
	 */
	domTriangles_Array &getTriangles_array() { return elemTriangles_array; }
	/**
	 * Gets the triangles element array.
	 * @return Returns a constant reference to the array of triangles elements.
	 */
	const domTriangles_Array &getTriangles_array() const { return elemTriangles_array; }
	/**
	 * Gets the trifans element array.
	 * @return Returns a reference to the array of trifans elements.
	 */
	domTrifans_Array &getTrifans_array() { return elemTrifans_array; }
	/**
	 * Gets the trifans element array.
	 * @return Returns a constant reference to the array of trifans elements.
	 */
	const domTrifans_Array &getTrifans_array() const { return elemTrifans_array; }
	/**
	 * Gets the tristrips element array.
	 * @return Returns a reference to the array of tristrips elements.
	 */
	domTristrips_Array &getTristrips_array() { return elemTristrips_array; }
	/**
	 * Gets the tristrips element array.
	 * @return Returns a constant reference to the array of tristrips elements.
	 */
	const domTristrips_Array &getTristrips_array() const { return elemTristrips_array; }
	/**
	 * Gets the extra element array.
	 * @return Returns a reference to the array of extra elements.
	 */
	domExtra_Array &getExtra_array() { return elemExtra_array; }
	/**
	 * Gets the extra element array.
	 * @return Returns a constant reference to the array of extra elements.
	 */
	const domExtra_Array &getExtra_array() const { return elemExtra_array; }
	/**
	 * Gets the _contents array.
	 * @return Returns a reference to the _contents element array.
	 */
	daeElementRefArray &getContents() { return _contents; }
	/**
	 * Gets the _contents array.
	 * @return Returns a constant reference to the _contents element array.
	 */
	const daeElementRefArray &getContents() const { return _contents; }

protected:
	/**
	 * Constructor
	 */
	domConvex_mesh() : attrConvex_hull_of(), elemSource_array(), elemVertices(), elemLines_array(), elemLinestrips_array(), elemPolygons_array(), elemPolylist_array(), elemTriangles_array(), elemTrifans_array(), elemTristrips_array(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domConvex_mesh() {}
	/**
	 * Copy Constructor
	 */
	domConvex_mesh( const domConvex_mesh &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domConvex_mesh &operator=( const domConvex_mesh &cpy ) { (void)cpy; return *this; }

public: // STATIC METHODS
	/**
	 * Creates an instance of this class and returns a daeElementRef referencing it.
	 * @param bytes The size allocated for this instance.
	 * @return a daeElementRef referencing an instance of this object.
	 */
	static DLLSPEC daeElementRef create(daeInt bytes);
	/**
	 * Creates a daeMetaElement object that describes this element in the meta object reflection framework.
	 * If a daeMetaElement already exists it will return that instead of creating a new one. 
	 * @return A daeMetaElement describing this COLLADA element.
	 */
	static DLLSPEC daeMetaElement* registerElement();

public: // STATIC MEMBERS
	/**
	 * The daeMetaElement that describes this element in the meta object reflection framework.
	 */
	static DLLSPEC daeMetaElement* _Meta;
};


#endif
