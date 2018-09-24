/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "hacdManifoldMesh.h"
using namespace std;

namespace HACD
{
Material::Material(void)
{
	m_diffuseColor.X() = 0.5;
	m_diffuseColor.Y() = 0.5;
	m_diffuseColor.Z() = 0.5;
	m_specularColor.X() = 0.5;
	m_specularColor.Y() = 0.5;
	m_specularColor.Z() = 0.5;
	m_ambientIntensity = 0.4;
	m_emissiveColor.X() = 0.0;
	m_emissiveColor.Y() = 0.0;
	m_emissiveColor.Z() = 0.0;
	m_shininess = 0.4;
	m_transparency = 0.0;
}

TMMVertex::TMMVertex(void)
{
	m_name = 0;
	m_id = 0;
	m_duplicate = 0;
	m_onHull = false;
	m_tag = false;
}
TMMVertex::~TMMVertex(void)
{
}
TMMEdge::TMMEdge(void)
{
	m_id = 0;
	m_triangles[0] = m_triangles[1] = m_newFace = 0;
	m_vertices[0] = m_vertices[1] = 0;
}
TMMEdge::~TMMEdge(void)
{
}
TMMTriangle::TMMTriangle(void)
{
	m_id = 0;
	for (int i = 0; i < 3; i++)
	{
		m_edges[i] = 0;
		m_vertices[0] = 0;
	}
	m_visible = false;
}
TMMTriangle::~TMMTriangle(void)
{
}
TMMesh::TMMesh(void)
{
	m_barycenter = Vec3<Real>(0, 0, 0);
	m_diag = 1;
}
TMMesh::~TMMesh(void)
{
}

void TMMesh::Print()
{
	size_t nV = m_vertices.GetSize();
	std::cout << "-----------------------------" << std::endl;
	std::cout << "vertices (" << nV << ")" << std::endl;
	for (size_t v = 0; v < nV; v++)
	{
		const TMMVertex &currentVertex = m_vertices.GetData();
		std::cout << currentVertex.m_id << ", "
				  << currentVertex.m_pos.X() << ", "
				  << currentVertex.m_pos.Y() << ", "
				  << currentVertex.m_pos.Z() << std::endl;
		m_vertices.Next();
	}

	size_t nE = m_edges.GetSize();
	std::cout << "edges (" << nE << ")" << std::endl;
	for (size_t e = 0; e < nE; e++)
	{
		const TMMEdge &currentEdge = m_edges.GetData();
		const CircularListElement<TMMVertex> *v0 = currentEdge.m_vertices[0];
		const CircularListElement<TMMVertex> *v1 = currentEdge.m_vertices[1];
		const CircularListElement<TMMTriangle> *f0 = currentEdge.m_triangles[0];
		const CircularListElement<TMMTriangle> *f1 = currentEdge.m_triangles[1];

		std::cout << "-> (" << v0->GetData().m_name << ", " << v1->GetData().m_name << ")" << std::endl;
		std::cout << "-> F0 (" << f0->GetData().m_vertices[0]->GetData().m_name << ", "
				  << f0->GetData().m_vertices[1]->GetData().m_name << ", "
				  << f0->GetData().m_vertices[2]->GetData().m_name << ")" << std::endl;
		std::cout << "-> F1 (" << f1->GetData().m_vertices[0]->GetData().m_name << ", "
				  << f1->GetData().m_vertices[1]->GetData().m_name << ", "
				  << f1->GetData().m_vertices[2]->GetData().m_name << ")" << std::endl;
		m_edges.Next();
	}
	size_t nT = m_triangles.GetSize();
	std::cout << "triangles (" << nT << ")" << std::endl;
	for (size_t t = 0; t < nT; t++)
	{
		const TMMTriangle &currentTriangle = m_triangles.GetData();
		const CircularListElement<TMMVertex> *v0 = currentTriangle.m_vertices[0];
		const CircularListElement<TMMVertex> *v1 = currentTriangle.m_vertices[1];
		const CircularListElement<TMMVertex> *v2 = currentTriangle.m_vertices[2];
		const CircularListElement<TMMEdge> *e0 = currentTriangle.m_edges[0];
		const CircularListElement<TMMEdge> *e1 = currentTriangle.m_edges[1];
		const CircularListElement<TMMEdge> *e2 = currentTriangle.m_edges[2];

		std::cout << "-> (" << v0->GetData().m_name << ", " << v1->GetData().m_name << ", " << v2->GetData().m_name << ")" << std::endl;

		std::cout << "-> E0 (" << e0->GetData().m_vertices[0]->GetData().m_name << ", "
				  << e0->GetData().m_vertices[1]->GetData().m_name << ")" << std::endl;
		std::cout << "-> E1 (" << e1->GetData().m_vertices[0]->GetData().m_name << ", "
				  << e1->GetData().m_vertices[1]->GetData().m_name << ")" << std::endl;
		std::cout << "-> E2 (" << e2->GetData().m_vertices[0]->GetData().m_name << ", "
				  << e2->GetData().m_vertices[1]->GetData().m_name << ")" << std::endl;
		m_triangles.Next();
	}
}
bool TMMesh::Save(const char *fileName)
{
	std::ofstream fout(fileName);
	std::cout << "Saving " << fileName << std::endl;
	if (SaveVRML2(fout))
	{
		fout.close();
		return true;
	}
	return false;
}
bool TMMesh::SaveVRML2(std::ofstream &fout)
{
	return SaveVRML2(fout, Material());
}
bool TMMesh::SaveVRML2(std::ofstream &fout, const Material &material)
{
	if (fout.is_open())
	{
		size_t nV = m_vertices.GetSize();
		size_t nT = m_triangles.GetSize();
		fout << "#VRML V2.0 utf8" << std::endl;
		fout << "" << std::endl;
		fout << "# Vertices: " << nV << std::endl;
		fout << "# Triangles: " << nT << std::endl;
		fout << "" << std::endl;
		fout << "Group {" << std::endl;
		fout << "	children [" << std::endl;
		fout << "		Shape {" << std::endl;
		fout << "			appearance Appearance {" << std::endl;
		fout << "				material Material {" << std::endl;
		fout << "					diffuseColor " << material.m_diffuseColor.X() << " "
			 << material.m_diffuseColor.Y() << " "
			 << material.m_diffuseColor.Z() << std::endl;
		fout << "					ambientIntensity " << material.m_ambientIntensity << std::endl;
		fout << "					specularColor " << material.m_specularColor.X() << " "
			 << material.m_specularColor.Y() << " "
			 << material.m_specularColor.Z() << std::endl;
		fout << "					emissiveColor " << material.m_emissiveColor.X() << " "
			 << material.m_emissiveColor.Y() << " "
			 << material.m_emissiveColor.Z() << std::endl;
		fout << "					shininess " << material.m_shininess << std::endl;
		fout << "					transparency " << material.m_transparency << std::endl;
		fout << "				}" << std::endl;
		fout << "			}" << std::endl;
		fout << "			geometry IndexedFaceSet {" << std::endl;
		fout << "				ccw TRUE" << std::endl;
		fout << "				solid TRUE" << std::endl;
		fout << "				convex TRUE" << std::endl;
		if (GetNVertices() > 0)
		{
			fout << "				coord DEF co Coordinate {" << std::endl;
			fout << "					point [" << std::endl;
			for (size_t v = 0; v < nV; v++)
			{
				TMMVertex &currentVertex = m_vertices.GetData();
				fout << "						" << currentVertex.m_pos.X() << " "
					 << currentVertex.m_pos.Y() << " "
					 << currentVertex.m_pos.Z() << "," << std::endl;
				currentVertex.m_id = v;
				m_vertices.Next();
			}
			fout << "					]" << std::endl;
			fout << "				}" << std::endl;
		}
		if (GetNTriangles() > 0)
		{
			fout << "				coordIndex [ " << std::endl;
			for (size_t f = 0; f < nT; f++)
			{
				TMMTriangle &currentTriangle = m_triangles.GetData();
				fout << "						" << currentTriangle.m_vertices[0]->GetData().m_id << ", "
					 << currentTriangle.m_vertices[1]->GetData().m_id << ", "
					 << currentTriangle.m_vertices[2]->GetData().m_id << ", -1," << std::endl;
				m_triangles.Next();
			}
			fout << "				]" << std::endl;
		}
		fout << "			}" << std::endl;
		fout << "		}" << std::endl;
		fout << "	]" << std::endl;
		fout << "}" << std::endl;
	}
	return true;
}
void TMMesh::GetIFS(Vec3<Real> *const points, Vec3<long> *const triangles)
{
	size_t nV = m_vertices.GetSize();
	size_t nT = m_triangles.GetSize();

	for (size_t v = 0; v < nV; v++)
	{
		points[v] = m_vertices.GetData().m_pos;
		m_vertices.GetData().m_id = v;
		m_vertices.Next();
	}
	for (size_t f = 0; f < nT; f++)
	{
		TMMTriangle &currentTriangle = m_triangles.GetData();
		triangles[f].X() = static_cast<long>(currentTriangle.m_vertices[0]->GetData().m_id);
		triangles[f].Y() = static_cast<long>(currentTriangle.m_vertices[1]->GetData().m_id);
		triangles[f].Z() = static_cast<long>(currentTriangle.m_vertices[2]->GetData().m_id);
		m_triangles.Next();
	}
}
void TMMesh::Clear()
{
	m_vertices.Clear();
	m_edges.Clear();
	m_triangles.Clear();
}
void TMMesh::Copy(TMMesh &mesh)
{
	Clear();
	// updating the id's
	size_t nV = mesh.m_vertices.GetSize();
	size_t nE = mesh.m_edges.GetSize();
	size_t nT = mesh.m_triangles.GetSize();
	for (size_t v = 0; v < nV; v++)
	{
		mesh.m_vertices.GetData().m_id = v;
		mesh.m_vertices.Next();
	}
	for (size_t e = 0; e < nE; e++)
	{
		mesh.m_edges.GetData().m_id = e;
		mesh.m_edges.Next();
	}
	for (size_t f = 0; f < nT; f++)
	{
		mesh.m_triangles.GetData().m_id = f;
		mesh.m_triangles.Next();
	}
	// copying data
	m_vertices = mesh.m_vertices;
	m_edges = mesh.m_edges;
	m_triangles = mesh.m_triangles;

	// generating mapping
	CircularListElement<TMMVertex> **vertexMap = new CircularListElement<TMMVertex> *[nV];
	CircularListElement<TMMEdge> **edgeMap = new CircularListElement<TMMEdge> *[nE];
	CircularListElement<TMMTriangle> **triangleMap = new CircularListElement<TMMTriangle> *[nT];
	for (size_t v = 0; v < nV; v++)
	{
		vertexMap[v] = m_vertices.GetHead();
		m_vertices.Next();
	}
	for (size_t e = 0; e < nE; e++)
	{
		edgeMap[e] = m_edges.GetHead();
		m_edges.Next();
	}
	for (size_t f = 0; f < nT; f++)
	{
		triangleMap[f] = m_triangles.GetHead();
		m_triangles.Next();
	}

	// updating pointers
	for (size_t v = 0; v < nV; v++)
	{
		if (vertexMap[v]->GetData().m_duplicate)
		{
			vertexMap[v]->GetData().m_duplicate = edgeMap[vertexMap[v]->GetData().m_duplicate->GetData().m_id];
		}
	}
	for (size_t e = 0; e < nE; e++)
	{
		if (edgeMap[e]->GetData().m_newFace)
		{
			edgeMap[e]->GetData().m_newFace = triangleMap[edgeMap[e]->GetData().m_newFace->GetData().m_id];
		}
		if (nT > 0)
		{
			for (int f = 0; f < 2; f++)
			{
				if (edgeMap[e]->GetData().m_triangles[f])
				{
					edgeMap[e]->GetData().m_triangles[f] = triangleMap[edgeMap[e]->GetData().m_triangles[f]->GetData().m_id];
				}
			}
		}
		for (int v = 0; v < 2; v++)
		{
			if (edgeMap[e]->GetData().m_vertices[v])
			{
				edgeMap[e]->GetData().m_vertices[v] = vertexMap[edgeMap[e]->GetData().m_vertices[v]->GetData().m_id];
			}
		}
	}
	for (size_t f = 0; f < nT; f++)
	{
		if (nE > 0)
		{
			for (int e = 0; e < 3; e++)
			{
				if (triangleMap[f]->GetData().m_edges[e])
				{
					triangleMap[f]->GetData().m_edges[e] = edgeMap[triangleMap[f]->GetData().m_edges[e]->GetData().m_id];
				}
			}
		}
		for (int v = 0; v < 3; v++)
		{
			if (triangleMap[f]->GetData().m_vertices[v])
			{
				triangleMap[f]->GetData().m_vertices[v] = vertexMap[triangleMap[f]->GetData().m_vertices[v]->GetData().m_id];
			}
		}
	}
	delete[] vertexMap;
	delete[] edgeMap;
	delete[] triangleMap;
}
long IntersectRayTriangle(const Vec3<double> &P0, const Vec3<double> &dir,
						  const Vec3<double> &V0, const Vec3<double> &V1,
						  const Vec3<double> &V2, double &t)
{
	Vec3<double> edge1, edge2, edge3;
	double det, invDet;
	edge1 = V1 - V2;
	edge2 = V2 - V0;
	Vec3<double> pvec = dir ^ edge2;
	det = edge1 * pvec;
	if (det == 0.0)
		return 0;
	invDet = 1.0 / det;
	Vec3<double> tvec = P0 - V0;
	Vec3<double> qvec = tvec ^ edge1;
	t = (edge2 * qvec) * invDet;
	if (t < 0.0)
	{
		return 0;
	}
	edge3 = V0 - V1;
	Vec3<double> I(P0 + t * dir);
	Vec3<double> s0 = (I - V0) ^ edge3;
	Vec3<double> s1 = (I - V1) ^ edge1;
	Vec3<double> s2 = (I - V2) ^ edge2;
	if (s0 * s1 > -1e-9 && s2 * s1 > -1e-9)
	{
		return 1;
	}
	return 0;
}

bool IntersectLineLine(const Vec3<double> &p1, const Vec3<double> &p2,
					   const Vec3<double> &p3, const Vec3<double> &p4,
					   Vec3<double> &pa, Vec3<double> &pb,
					   double &mua, double &mub)
{
	Vec3<double> p13, p43, p21;
	double d1343, d4321, d1321, d4343, d2121;
	double numer, denom;

	p13.X() = p1.X() - p3.X();
	p13.Y() = p1.Y() - p3.Y();
	p13.Z() = p1.Z() - p3.Z();
	p43.X() = p4.X() - p3.X();
	p43.Y() = p4.Y() - p3.Y();
	p43.Z() = p4.Z() - p3.Z();
	if (p43.X() == 0.0 && p43.Y() == 0.0 && p43.Z() == 0.0)
		return false;
	p21.X() = p2.X() - p1.X();
	p21.Y() = p2.Y() - p1.Y();
	p21.Z() = p2.Z() - p1.Z();
	if (p21.X() == 0.0 && p21.Y() == 0.0 && p21.Z() == 0.0)
		return false;

	d1343 = p13.X() * p43.X() + p13.Y() * p43.Y() + p13.Z() * p43.Z();
	d4321 = p43.X() * p21.X() + p43.Y() * p21.Y() + p43.Z() * p21.Z();
	d1321 = p13.X() * p21.X() + p13.Y() * p21.Y() + p13.Z() * p21.Z();
	d4343 = p43.X() * p43.X() + p43.Y() * p43.Y() + p43.Z() * p43.Z();
	d2121 = p21.X() * p21.X() + p21.Y() * p21.Y() + p21.Z() * p21.Z();

	denom = d2121 * d4343 - d4321 * d4321;
	if (denom == 0.0)
		return false;
	numer = d1343 * d4321 - d1321 * d4343;

	mua = numer / denom;
	mub = (d1343 + d4321 * (mua)) / d4343;

	pa.X() = p1.X() + mua * p21.X();
	pa.Y() = p1.Y() + mua * p21.Y();
	pa.Z() = p1.Z() + mua * p21.Z();
	pb.X() = p3.X() + mub * p43.X();
	pb.Y() = p3.Y() + mub * p43.Y();
	pb.Z() = p3.Z() + mub * p43.Z();

	return true;
}

long IntersectRayTriangle2(const Vec3<double> &P0, const Vec3<double> &dir,
						   const Vec3<double> &V0, const Vec3<double> &V1,
						   const Vec3<double> &V2, double &r)
{
	Vec3<double> u, v, n;  // triangle vectors
	Vec3<double> w0, w;    // ray vectors
	double a, b;           // params to calc ray-plane intersect

	// get triangle edge vectors and plane normal
	u = V1 - V0;
	v = V2 - V0;
	n = u ^ v;               // cross product
	if (n.GetNorm() == 0.0)  // triangle is degenerate
		return -1;           // do not deal with this case

	w0 = P0 - V0;
	a = -n * w0;
	b = n * dir;
	if (fabs(b) <= 0.0)
	{                  // ray is parallel to triangle plane
		if (a == 0.0)  // ray lies in triangle plane
			return 2;
		else
			return 0;  // ray disjoint from plane
	}

	// get intersect point of ray with triangle plane
	r = a / b;
	if (r < 0.0)   // ray goes away from triangle
		return 0;  // => no intersect
	// for a segment, also test if (r > 1.0) => no intersect

	Vec3<double> I = P0 + r * dir;  // intersect point of ray and plane

	// is I inside T?
	double uu, uv, vv, wu, wv, D;
	uu = u * u;
	uv = u * v;
	vv = v * v;
	w = I - V0;
	wu = w * u;
	wv = w * v;
	D = uv * uv - uu * vv;

	// get and test parametric coords
	double s, t;
	s = (uv * wv - vv * wu) / D;
	if (s < 0.0 || s > 1.0)  // I is outside T
		return 0;
	t = (uv * wu - uu * wv) / D;
	if (t < 0.0 || (s + t) > 1.0)  // I is outside T
		return 0;
	return 1;  // I is in T
}

bool TMMesh::CheckConsistancy()
{
	size_t nE = m_edges.GetSize();
	size_t nT = m_triangles.GetSize();
	for (size_t e = 0; e < nE; e++)
	{
		for (int f = 0; f < 2; f++)
		{
			if (!m_edges.GetHead()->GetData().m_triangles[f])
			{
				return false;
			}
		}
		m_edges.Next();
	}

	for (size_t f = 0; f < nT; f++)
	{
		for (int e = 0; e < 3; e++)
		{
			int found = 0;
			for (int k = 0; k < 2; k++)
			{
				if (m_triangles.GetHead()->GetData().m_edges[e]->GetData().m_triangles[k] == m_triangles.GetHead())
				{
					found++;
				}
			}
			if (found != 1)
			{
				return false;
			}
		}
		m_triangles.Next();
	}

	return true;
}
bool TMMesh::Normalize()
{
	size_t nV = m_vertices.GetSize();
	if (nV == 0)
	{
		return false;
	}
	m_barycenter = m_vertices.GetHead()->GetData().m_pos;
	Vec3<Real> min = m_barycenter;
	Vec3<Real> max = m_barycenter;
	Real x, y, z;
	for (size_t v = 1; v < nV; v++)
	{
		m_barycenter += m_vertices.GetHead()->GetData().m_pos;
		x = m_vertices.GetHead()->GetData().m_pos.X();
		y = m_vertices.GetHead()->GetData().m_pos.Y();
		z = m_vertices.GetHead()->GetData().m_pos.Z();
		if (x < min.X())
			min.X() = x;
		else if (x > max.X())
			max.X() = x;
		if (y < min.Y())
			min.Y() = y;
		else if (y > max.Y())
			max.Y() = y;
		if (z < min.Z())
			min.Z() = z;
		else if (z > max.Z())
			max.Z() = z;
		m_vertices.Next();
	}
	m_barycenter /= static_cast<Real>(nV);
	m_diag = static_cast<Real>(0.001 * (max - min).GetNorm());
	const Real invDiag = static_cast<Real>(1.0 / m_diag);
	if (m_diag != 0.0)
	{
		for (size_t v = 0; v < nV; v++)
		{
			m_vertices.GetHead()->GetData().m_pos = (m_vertices.GetHead()->GetData().m_pos - m_barycenter) * invDiag;
			m_vertices.Next();
		}
	}
	return true;
}
bool TMMesh::Denormalize()
{
	size_t nV = m_vertices.GetSize();
	if (nV == 0)
	{
		return false;
	}
	if (m_diag != 0.0)
	{
		for (size_t v = 0; v < nV; v++)
		{
			m_vertices.GetHead()->GetData().m_pos = m_vertices.GetHead()->GetData().m_pos * m_diag + m_barycenter;
			m_vertices.Next();
		}
	}
	return false;
}
}  // namespace HACD
