/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#ifndef _SCE_PFX_VEC_INT3_H
#define _SCE_PFX_VEC_INT3_H

#include "pfx_common.h"



namespace sce {
namespace PhysicsEffects {

class SCE_PFX_ALIGNED(16) PfxVecInt3
{
private:
PfxInt32 m_x,m_y,m_z,m_w;

public:
PfxVecInt3() {m_x=m_y=m_z=m_w=0;}
PfxVecInt3(const PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG vec) {m_x=(PfxInt32)vec[0];m_y=(PfxInt32)vec[1];m_z=(PfxInt32)vec[2];m_w=0;}
PfxVecInt3(PfxFloat fx,PfxFloat fy,PfxFloat fz) {m_x=(PfxInt32)fx;m_y=(PfxInt32)fy;m_z=(PfxInt32)fz;m_w=0;}
PfxVecInt3(PfxInt32 iv) {m_x=m_y=m_z=iv;m_w=0;}
PfxVecInt3(PfxInt32 ix,PfxInt32 iy,PfxInt32 iz) {m_x=ix;m_y=iy;m_z=iz;m_w=0;}

    inline PfxVecInt3 &operator =( const PfxVecInt3 &vec);

inline PfxInt32 get(PfxInt32 i) const {return *(&m_x+i);}
inline PfxInt32 getX() const {return m_x;}
inline PfxInt32 getY() const {return m_y;}
inline PfxInt32 getZ() const {return m_z;}
inline void set(PfxInt32 i,PfxInt32 v) {*(&m_x+i) = v;}
inline void setX(PfxInt32 v) {m_x = v;}
inline void setY(PfxInt32 v) {m_y = v;}
inline void setZ(PfxInt32 v) {m_z = v;}

    inline const PfxVecInt3 operator +( const PfxVecInt3 & vec ) const;
    inline const PfxVecInt3 operator -( const PfxVecInt3 & vec ) const;
    inline const PfxVecInt3 operator *( PfxInt32 scalar ) const;
    inline const PfxVecInt3 operator /( PfxInt32 scalar ) const;

    inline PfxVecInt3 & operator +=( const PfxVecInt3 & vec );
    inline PfxVecInt3 & operator -=( const PfxVecInt3 & vec );
    inline PfxVecInt3 & operator *=( PfxInt32 scalar );
    inline PfxVecInt3 & operator /=( PfxInt32 scalar );

    inline const PfxVecInt3 operator -() const;

operator PfxVector3() const
{
	return PfxVector3((PfxFloat)m_x,(PfxFloat)m_y,(PfxFloat)m_z);
}
};

inline PfxVecInt3 &PfxVecInt3::operator =( const PfxVecInt3 &vec)
{
    m_x = vec.m_x;
    m_y = vec.m_y;
    m_z = vec.m_z;
    return *this;
}

inline const PfxVecInt3 PfxVecInt3::operator +( const PfxVecInt3 & vec ) const
{
    return PfxVecInt3(m_x+vec.m_x, m_y+vec.m_y, m_z+vec.m_z);
}

inline const PfxVecInt3 PfxVecInt3::operator -( const PfxVecInt3 & vec ) const
{
    return PfxVecInt3(m_x-vec.m_x, m_y-vec.m_y, m_z-vec.m_z);
}

inline const PfxVecInt3 PfxVecInt3::operator *( PfxInt32 scalar ) const
{
    return PfxVecInt3(m_x*scalar, m_y*scalar, m_z*scalar);
}

inline const PfxVecInt3 PfxVecInt3::operator /( PfxInt32 scalar ) const
{
    return PfxVecInt3(m_x/scalar, m_y/scalar, m_z/scalar);
}

inline PfxVecInt3 &PfxVecInt3::operator +=( const PfxVecInt3 & vec )
{
    *this = *this + vec;
    return *this;
}

inline PfxVecInt3 &PfxVecInt3::operator -=( const PfxVecInt3 & vec )
{
    *this = *this - vec;
    return *this;
}

inline PfxVecInt3 &PfxVecInt3::operator *=( PfxInt32 scalar )
{
    *this = *this * scalar;
    return *this;
}

inline PfxVecInt3 &PfxVecInt3::operator /=( PfxInt32 scalar )
{
    *this = *this / scalar;
    return *this;
}

inline const PfxVecInt3 PfxVecInt3::operator -() const
{
return PfxVecInt3(-m_x,-m_y,-m_z);
}

inline const PfxVecInt3 operator *( PfxInt32 scalar, const PfxVecInt3 & vec )
{
    return vec * scalar;
}

inline const PfxVecInt3 mulPerElem( const PfxVecInt3 & vec0, const PfxVecInt3 & vec1 )
{
return PfxVecInt3(vec0.getX()*vec1.getX(), vec0.getY()*vec1.getY(), vec0.getZ()*vec1.getZ());
}

inline const PfxVecInt3 divPerElem( const PfxVecInt3 & vec0, const PfxVecInt3 & vec1 )
{
return PfxVecInt3(vec0.getX()/vec1.getX(), vec0.getY()/vec1.getY(), vec0.getZ()/vec1.getZ());
}

inline const PfxVecInt3 absPerElem( const PfxVecInt3 & vec )
{
return PfxVecInt3(abs(vec.getX()), abs(vec.getY()), abs(vec.getZ()));
}

inline const PfxVecInt3 maxPerElem( const PfxVecInt3 & vec0, const PfxVecInt3 & vec1 )
{
    return PfxVecInt3(
        (vec0.getX() > vec1.getX())? vec0.getX() : vec1.getX(),
        (vec0.getY() > vec1.getY())? vec0.getY() : vec1.getY(),
        (vec0.getZ() > vec1.getZ())? vec0.getZ() : vec1.getZ()
    );
}

inline const PfxVecInt3 minPerElem( const PfxVecInt3 & vec0, const PfxVecInt3 & vec1 )
{
    return PfxVecInt3(
        (vec0.getX() < vec1.getX())? vec0.getX() : vec1.getX(),
        (vec0.getY() < vec1.getY())? vec0.getY() : vec1.getY(),
        (vec0.getZ() < vec1.getZ())? vec0.getZ() : vec1.getZ()
    );
}
} //namespace PhysicsEffects
} //namespace sce


#endif // _SCE_PFX_VEC_INT3_H
