/*
 * SOLID - Software Library for Interference Detection
 * 
 * Copyright (C) 2001-2003  Dtecta.  All rights reserved.
 *
 * This library may be distributed under the terms of the Q Public License
 * (QPL) as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE.QPL included in the packaging of this file.
 *
 * This library may be distributed and/or modified under the terms of the
 * GNU General Public License (GPL) version 2 as published by the Free Software
 * Foundation and appearing in the file LICENSE.GPL included in the
 * packaging of this file.
 *
 * This library is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Commercial use or any other use of this library not covered by either 
 * the QPL or the GPL requires an additional license from Dtecta. 
 * Please contact info@dtecta.com for enquiries about the terms of commercial
 * use of this library.
 */

#include "Solid3JohnsonSimplexSolver.h"
#include "LinearMath/GenMinMax.h"

//#define USE_BACKUP_PROCEDURE
//#define FAST_CLOSEST

Solid3JohnsonSimplexSolver::Solid3JohnsonSimplexSolver() 
:
m_bits1(0x0),
m_all_bits(0x0)
{
}


Solid3JohnsonSimplexSolver::~Solid3JohnsonSimplexSolver()
{

}

void Solid3JohnsonSimplexSolver::reset() 
{
	m_bits1 = 0x0;
	m_all_bits = 0x0;	
}



void Solid3JohnsonSimplexSolver::addVertex(const SimdVector3& w) 
{
	assert(!fullSimplex());
	m_last = 0;
    m_last_bit = 0x1;
    while (contains(m_bits1, m_last_bit)) 
	{ 
		++m_last; 
		m_last_bit <<= 1; 
	}
	m_y[m_last] = w;
	m_ylen2[m_last] = w.length2();
    m_all_bits = m_bits1 | m_last_bit;

	update_cache();
	compute_det();
}

void Solid3JohnsonSimplexSolver::addVertex(const SimdVector3& w, const SimdPoint3& p, const SimdPoint3& q)
{
	addVertex(w);
	m_p[m_last] = p;
	m_q[m_last] = q;
}

bool Solid3JohnsonSimplexSolver::emptySimplex() const 
{ 
	return m_bits1 == 0x0; 
}

bool Solid3JohnsonSimplexSolver::fullSimplex() const 
{ 
	return m_bits1 == 0xf; 
}

SimdScalar Solid3JohnsonSimplexSolver::maxVertex() 
{ 
	return m_maxlen2; 
}

bool Solid3JohnsonSimplexSolver::closest(SimdVector3& v) 
{
#ifdef FAST_CLOSEST
	T_Bits s;
	for (s = m_bits1; s != 0x0; --s)
	{
		if (subseteq(s, m_bits1) && valid(s | m_last_bit)) 
		{
			//update bits !
			m_bits1 = s | m_last_bit;
			compute_vector(m_bits1, v);
			return true;
		}
	}
	if (valid(m_last_bit)) 
	{
		//update bits !
		m_bits1 = m_last_bit;
		m_maxlen2 = m_ylen2[m_last];
		v = m_y[m_last];
		return true;
	}
#else
	T_Bits s;
	for (s = m_all_bits; s != 0x0; --s)
	{
		if (subseteq(s, m_all_bits) && valid(s)) 
		{
			m_bits1 = s;
			compute_vector(m_bits1, v);
			return true;
		}
	}
#endif
	
	// Original GJK calls the backup procedure at this point.
#ifdef USE_BACKUP_PROCEDURE
	backup_closest(SimdVector3& v); 
#endif
	return false;  
}



int Solid3JohnsonSimplexSolver::getSimplex(SimdPoint3 *pBuf, SimdPoint3 *qBuf, SimdVector3 *yBuf) const 
{
	int num_verts = 0;
	int i;
	T_Bits bit;
	for (i = 0, bit = 0x1; i < 4; ++i, bit <<= 1) 
	{
		if (contains(m_bits1, bit)) 
		{
			pBuf[num_verts] = m_p[i];
			qBuf[num_verts] = m_q[i];
			yBuf[num_verts] = m_y[i];
			
#ifdef DEBUG
			std::cout << "Point " << i << " = " << m_y[i] << std::endl;
#endif
			
			++num_verts;
		}
	}
	return num_verts;
}

bool Solid3JohnsonSimplexSolver::inSimplex(const SimdVector3& w) 
{
	int i;
	T_Bits bit;
	for (i = 0, bit = 0x1; i < 4; ++i, bit <<= 1)
	{
		if (contains(m_all_bits, bit) && w == m_y[i])
		{
			return true;
		}
	}
	return false;
}


	
void Solid3JohnsonSimplexSolver::backup_closest(SimdVector3& v)
{ 		
	SimdScalar min_dist2 = SIMD_INFINITY;
	
  T_Bits s;
	for (s = m_all_bits; s != 0x0; --s) 
	{
		if (subseteq(s, m_all_bits) && proper(s))
		{	
			SimdVector3 u;
			compute_vector(s, u);
			SimdScalar dist2 = u.length2();
			if (dist2 < min_dist2)
			{
				min_dist2 = dist2;
				//update bits !
				m_bits1 = s;
				v = u;
			}
		}
	}
}


void Solid3JohnsonSimplexSolver::compute_points(SimdPoint3& p1, SimdPoint3& p2) 
{
	SimdScalar sum = SimdScalar(0.0);
	p1.setValue(SimdScalar(0.0), SimdScalar(0.0), SimdScalar(0.0));
	p2.setValue(SimdScalar(0.0), SimdScalar(0.0), SimdScalar(0.0));
	int i;
	T_Bits bit;
	for (i = 0, bit = 0x1; i < 4; ++i, bit <<= 1) 
	{
		if (contains(m_bits1, bit))
		{
			sum += m_det[m_bits1][i];
			p1 += m_p[i] * m_det[m_bits1][i];
			p2 += m_q[i] * m_det[m_bits1][i];
		}
	}

	assert(sum > SimdScalar(0.0));
	SimdScalar s = SimdScalar(1.0) / sum;
	p1 *= s;
	p2 *= s;
}



int Solid3JohnsonSimplexSolver::numVertices() const 
{
	int numverts = 0;
	int i,bit;
	for (i = 0, bit = 0x1; i < 4; ++i, bit <<= 1)
	{
		if (contains(m_bits1, bit))
		{
			numverts++;
		}
	}
	return numverts;
}




//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//internal

inline void Solid3JohnsonSimplexSolver::update_cache() 
{
	int i;
	T_Bits bit;
    for (i = 0, bit = 0x1; i < 4; ++i, bit <<= 1)
	{
        if (contains(m_bits1, bit)) 
		{
			m_edge[i][m_last] = m_y[i] - m_y[m_last];
			m_edge[m_last][i] = -m_edge[i][m_last];

#ifdef JOHNSON_ROBUST
			m_norm[i][m_last] = m_norm[m_last][i] = m_edge[i][m_last].length2();
#endif

		}
	}
}


bool Solid3JohnsonSimplexSolver::valid(T_Bits s) 
{
	int i;
	T_Bits bit;
	for (i = 0, bit = 0x1; i < 4; ++i, bit <<= 1)
	{
		if (contains(m_all_bits, bit)) 
		{
			if (contains(s, bit)) 
			{
				if (m_det[s][i] <= SimdScalar(0.0)) 
				{
					return false; 
				}
			}
			else if (m_det[s | bit][i] > SimdScalar(0.0))
			{ 
				return false;
			}
		}
	}
	return true;
}

bool Solid3JohnsonSimplexSolver::proper(T_Bits s)
{ 
	int i;
	T_Bits bit;
	for (i = 0, bit = 0x1; i < 4; ++i, bit <<= 1)
	{
		if (contains(s, bit) && m_det[s][i] <= SimdScalar(0.0))
		{
			return false; 
		}
	}
	return true;
}

void Solid3JohnsonSimplexSolver::compute_vector(T_Bits s, SimdVector3& v) 
{
	m_maxlen2 = SimdScalar(0.0);
	SimdScalar sum = SimdScalar(0.0);
	v .setValue(SimdScalar(0.0), SimdScalar(0.0), SimdScalar(0.0));

	int i;
	T_Bits bit;
	for (i = 0, bit = 0x1; i < 4; ++i, bit <<= 1) 
	{
		if (contains(s, bit))
		{
			sum += m_det[s][i];
			GEN_set_max(m_maxlen2, m_ylen2[i]);
			v += m_y[i] * m_det[s][i];
		}
	}
	
	assert(sum > SimdScalar(0.0));

	v /= sum;
}



#ifdef JOHNSON_ROBUST

inline void Solid3JohnsonSimplexSolver::compute_det() 
{
    m_det[m_last_bit][m_last] = 1;

	int i;
	T_Bits si;
    for (i = 0, si = 0x1; i < 4; ++i, si <<= 1) 
	{
        if (contains(m_bits1, si)) 
		{
            T_Bits s2 = si | m_last_bit;
            m_det[s2][i] = m_edge[m_last][i].dot(m_y[m_last]); 
            m_det[s2][m_last] = m_edge[i][m_last].dot(m_y[i]);

			int j;
			T_Bits sj;
            for (j = 0, sj = 0x1; j < i; ++j, sj <<= 1) 
			{
                if (contains(m_bits1, sj)) 
				{
					int k;
                    T_Bits s3 = sj | s2;			
					
					k = m_norm[i][j] < m_norm[m_last][j] ? i : m_last;
                    m_det[s3][j] = m_det[s2][i] * m_edge[k][j].dot(m_y[i]) + 
                                   m_det[s2][m_last] * m_edge[k][j].dot(m_y[m_last]);
					k = m_norm[j][i] < m_norm[m_last][i] ? j : m_last;
                    m_det[s3][i] = m_det[sj|m_last_bit][j] * m_edge[k][i].dot(m_y[j]) +  
                                   m_det[sj|m_last_bit][m_last] * m_edge[k][i].dot(m_y[m_last]);
					k = m_norm[i][m_last] < m_norm[j][m_last] ? i : j;
                    m_det[s3][m_last] = m_det[sj|si][j] * m_edge[k][m_last].dot(m_y[j]) + 
                                        m_det[sj|si][i] * m_edge[k][m_last].dot(m_y[i]);
                }
            }
        }
    }

    if (m_all_bits == 0xf) 
	{
		int k;

		k = m_norm[1][0] < m_norm[2][0] ? (m_norm[1][0] < m_norm[3][0] ? 1 : 3) : (m_norm[2][0] < m_norm[3][0] ? 2 : 3);
		
        m_det[0xf][0] = m_det[0xe][1] * m_edge[k][0].dot(m_y[1]) + 
                        m_det[0xe][2] * m_edge[k][0].dot(m_y[2]) + 
                        m_det[0xe][3] * m_edge[k][0].dot(m_y[3]);

		k = m_norm[0][1] < m_norm[2][1] ? (m_norm[0][1] < m_norm[3][1] ? 0 : 3) : (m_norm[2][1] < m_norm[3][1] ? 2 : 3);
		
        m_det[0xf][1] = m_det[0xd][0] * m_edge[k][1].dot(m_y[0]) + 
                        m_det[0xd][2] * m_edge[k][1].dot(m_y[2]) +
                        m_det[0xd][3] * m_edge[k][1].dot(m_y[3]);

		k = m_norm[0][2] < m_norm[1][2] ? (m_norm[0][2] < m_norm[3][2] ? 0 : 3) : (m_norm[1][2] < m_norm[3][2] ? 1 : 3);
		
        m_det[0xf][2] = m_det[0xb][0] * m_edge[k][2].dot(m_y[0]) + 
                        m_det[0xb][1] * m_edge[k][2].dot(m_y[1]) +  
                        m_det[0xb][3] * m_edge[k][2].dot(m_y[3]);

		k = m_norm[0][3] < m_norm[1][3] ? (m_norm[0][3] < m_norm[2][3] ? 0 : 2) : (m_norm[1][3] < m_norm[2][3] ? 1 : 2);
		
        m_det[0xf][3] = m_det[0x7][0] * m_edge[k][3].dot(m_y[0]) + 
                        m_det[0x7][1] * m_edge[k][3].dot(m_y[1]) + 
                        m_det[0x7][2] * m_edge[k][3].dot(m_y[2]);
    }
}

#else //JOHNSON_ROBUST


inline void Solid3JohnsonSimplexSolver::compute_det() 
{
    m_det[m_last_bit][m_last] = 1;

	int i;
	T_Bits si;
    for (i = 0, si = 0x1; i < 4; ++i, si <<= 1) 
	{
        if (contains(m_bits1, si)) 
		{
            T_Bits s2 = si | m_last_bit;
            m_det[s2][i] = m_edge[m_last][i].dot(m_y[m_last]); 
            m_det[s2][m_last] = m_edge[i][m_last].dot(m_y[i]);

			int j;
			T_Bits sj;
            for (j = 0, sj = 0x1; j < i; ++j, sj <<= 1)
			{
                if (contains(m_bits1, sj)) 
				{
                    T_Bits s3 = sj | s2;
                    m_det[s3][j] = m_det[s2][i] * m_edge[i][j].dot(m_y[i]) + 
                                   m_det[s2][m_last] * m_edge[i][j].dot(m_y[m_last]);
                    m_det[s3][i] = m_det[sj|m_last_bit][j] * m_edge[j][i].dot(m_y[j]) +  
                                   m_det[sj|m_last_bit][m_last] * m_edge[j][i].dot(m_y[m_last]);
                    m_det[s3][m_last] = m_det[sj|si][j] * m_edge[j][m_last].dot(m_y[j]) + 
                                        m_det[sj|si][i] * m_edge[j][m_last].dot(m_y[i]);
                }
            }
        }
    }

    if (m_all_bits == 0xf) 
	{
        m_det[0xf][0] = m_det[0xe][1] * m_edge[1][0].dot(m_y[1]) + 
                        m_det[0xe][2] * m_edge[1][0].dot(m_y[2]) + 
                        m_det[0xe][3] * m_edge[1][0].dot(m_y[3]);
        m_det[0xf][1] = m_det[0xd][0] * m_edge[0][1].dot(m_y[0]) + 
                        m_det[0xd][2] * m_edge[0][1].dot(m_y[2]) +
                        m_det[0xd][3] * m_edge[0][1].dot(m_y[3]);
        m_det[0xf][2] = m_det[0xb][0] * m_edge[0][2].dot(m_y[0]) + 
                        m_det[0xb][1] * m_edge[0][2].dot(m_y[1]) +  
                        m_det[0xb][3] * m_edge[0][2].dot(m_y[3]);
        m_det[0xf][3] = m_det[0x7][0] * m_edge[0][3].dot(m_y[0]) + 
                        m_det[0x7][1] * m_edge[0][3].dot(m_y[1]) + 
                        m_det[0x7][2] * m_edge[0][3].dot(m_y[2]);
    }
}


#endif //JOHNSON_ROBUST
