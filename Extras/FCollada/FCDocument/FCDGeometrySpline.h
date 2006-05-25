/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/
/**
	@file FCDGeometrySpline.h
	This file contains the FCDGeometrySpline class.
	The FCDGeometrySpline class hold the information for one COLLADA geometric spline.
*/
#ifndef _FCD_GEOMETRY_SPLINE_H_
#define _FCD_GEOMETRY_SPLINE_H_

#include "FCDocument/FCDObject.h"

class FCDocument;
class FCDGeometry;

/** A dynamically-sized array of geometric spline control points. Each control point is simply one 3D position. @ingroup FCDGeometry */
typedef vector<FMVector3> FCDCVs;
/** A dynamically-sized array of weight values. Each weigth value represents a knot of a control point. @ingroup FCDGeometry */
typedef vector<double> FCDKnots;

/**
	A COLLADA geometric spline.

	A COLLADA spline contains a list of control points (CVs) that define an ordered list of 3D coordinates
	that influence the spline. The spline also contains a matching list of knots: there should be as many control
	points as there are knots.
	
	A COLLADA spline may be closed or open. If the spline is closed, then the first control point should be
	re-used when evaluating the last control point: the result should be a continuous curve, while an open
	spline will result in a discontinuity at each end.

	@todo: Insert the mathematical formula to calculate the spline position.

	@ingroup FCDGeometry
*/
class FCOLLADA_EXPORT FCDGeometrySpline : public FCDObject
{
private:
	FCDGeometry* parent;
	FCDCVs cvs;
	FCDKnots knots;
	bool isClosed;

public:
	/** Constructor: do not use directly. Use the FCDGeometry::CreateMesh function instead.
		@param document The COLLADA document that owns the new spline.
		@param parent The geometry entity that contains the new spline. */
	FCDGeometrySpline(FCDocument* document, FCDGeometry* parent);

	/** Destructor: do not use directly. All geometric splines are released with the geometry that they belong to. */
	virtual ~FCDGeometrySpline();

	/** Retrieve the parent of this geometric spline: the geometry entity.
		@return The geometry entity that this spline belongs to. */
	FCDGeometry* GetParent() { return parent; }
	const FCDGeometry* GetParent() const { return parent; } /**< See above. */

	/** Retrieves the list of control points for the spline.
		@return The list of control points. */
	inline FCDCVs& GetCVs() { return cvs; }
	inline const FCDCVs& GetCVs() const { return cvs; } /**< See above. */

	/** Retrieves the number of control points for the spline.
		@return The control point count. */
	inline size_t GetCVCount() const { return cvs.size(); }

	/** Retrieves a specific control point of the spline.
		@param index The index of the control point.
			The index should always be less than the number of control point.
		@return The control point. */
	inline FMVector3* GetCV(size_t index) { FUAssert(index < GetCVCount(), return NULL); return &(cvs.at(index)); }
	inline const FMVector3* GetCV(size_t index) const { FUAssert(index < GetCVCount(), return NULL); return &(cvs.at(index)); } /**< See above. */

	/** Retrieves the list of knots for the spline.
		@return The list of knots. */
	inline FCDKnots& GetKnots() { return knots; }
	inline const FCDKnots& GetKnots() const { return knots; } /**< See above. */

	/** Retrieves the number of knots for the spline.
		@return The knot count. */
	inline size_t GetKnotCount() const { return knots.size(); }

	/** Retrieves a specific knot of the spline.
		@param index The index of the knot. The index should always be less than the number of knots.
		@return The knot value. */
	inline double GetKnot(size_t index) const { FUAssert(index < GetKnotCount(), return 0.0); return knots.at(index); }

	/** Retrieves whether this spline is closed.
		@return Whether the spline is closed. */
	inline bool IsClosed() const { return isClosed; }

	/** Retrieves whether this spline is open.
		@return Whether the spline is open. */
	inline bool IsOpen() const { return !isClosed; }

	/** Overwrites the list of control points for this spline with a new ordered list of control points.
		@param _cvs The new control points. */
	inline void SetCVs(const FCDCVs& _cvs) { cvs = _cvs; }

	/** Overwrites the list of knots for this spline with a new ordered list of knots.
		@param _knots The new knots. */
	inline void SetKnots(const FCDKnots& _knots) { knots = _knots; }

	/** Sets the spline closed state.
		@param _isClosed The new closed state. */
	inline void SetClosed(bool _isClosed) { isClosed = _isClosed; }

	/** [INTERNAL] Reads in the \<spline\> element from a given COLLADA XML tree node.
		@param splineNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the spline.*/
	FUStatus LoadFromXML(xmlNode* splineNode);

	/** [INTERNAL] Writes out the \<spline\> element to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the spline information.
		@return The created \<spline\> element XML tree node. */
	xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_GEOMETRY_SPLINE_H_
