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
#ifndef __domMesh_h__
#define __domMesh_h__

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
 * The mesh element contains vertex and primitive information sufficient to
 * describe basic geometric meshes.
 */
class domMesh : public daeElement
{

protected:  // Elements
/**
 *  The mesh element must contain one or more source elements.  @see domSource
 */
	domSource_Array elemSource_array;
/**
 *  The mesh element must contain one vertices element.  @see domVertices
 */
	domVerticesRef elemVertices;
/**
 *  The mesh element may contain any number of lines elements.  @see domLines
 */
	domLines_Array elemLines_array;
/**
 *  The mesh element may contain any number of linestrips elements.  @see
 * domLinestrips
 */
	domLinestrips_Array elemLinestrips_array;
/**
 *  The mesh element may contain any number of polygons elements.  @see domPolygons
 */
	domPolygons_Array elemPolygons_array;
/**
 *  The mesh element may contain any number of polylist elements.  @see domPolylist
 */
	domPolylist_Array elemPolylist_array;
/**
 *  The mesh element may contain any number of triangles elements.  @see domTriangles
 */
	domTriangles_Array elemTriangles_array;
/**
 *  The mesh element may contain any number of trifans elements.  @see domTrifans
 */
	domTrifans_Array elemTrifans_array;
/**
 *  The mesh element may contain any number of tristrips elements.  @see domTristrips
 */
	domTristrips_Array elemTristrips_array;
/**
 *  The extra element may appear any number of times.  @see domExtra
 */
	domExtra_Array elemExtra_array;
	/**
	 * Used to preserve order in elements that do not specify strict sequencing of sub-elements.
	 */
	daeElementRefArray _contents;


public:	//Accessors and Mutators
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
	domMesh() : elemSource_array(), elemVertices(), elemLines_array(), elemLinestrips_array(), elemPolygons_array(), elemPolylist_array(), elemTriangles_array(), elemTrifans_array(), elemTristrips_array(), elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domMesh() {}
	/**
	 * Copy Constructor
	 */
	domMesh( const domMesh &cpy ) : daeElement() { (void)cpy; }
	/**
	 * Overloaded assignment operator
	 */
	virtual domMesh &operator=( const domMesh &cpy ) { (void)cpy; return *this; }

public: // STATIC METHODS
	/**
	 * Creates an instance of this class and returns a daeElementRef referencing it.
	 * @param bytes The size allocated for this instance.
	 * @return a daeElementRef referencing an instance of this object.
	 */
	static daeElementRef create(daeInt bytes);
	/**
	 * Creates a daeMetaElement object that describes this element in the meta object reflection framework.
	 * If a daeMetaElement already exists it will return that instead of creating a new one. 
	 * @return A daeMetaElement describing this COLLADA element.
	 */
	static daeMetaElement* registerElement();

public: // STATIC MEMBERS
	/**
	 * The daeMetaElement that describes this element in the meta object reflection framework.
	 */
	static daeMetaElement* _Meta;
};


#endif
