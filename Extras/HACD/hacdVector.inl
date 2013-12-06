#pragma once
#ifndef HACD_VECTOR_INL
#define HACD_VECTOR_INL
namespace HACD
{
	template <typename T> 
	inline Vec3<T> operator*(T lhs, const Vec3<T> & rhs)
	{
		return Vec3<T>(lhs * rhs.X(), lhs * rhs.Y(), lhs * rhs.Z());
	}
	template <typename T> 
	inline T & Vec3<T>::X() 
	{
		return m_data[0];
	}
	template <typename T>	
	inline  T &	Vec3<T>::Y() 
	{
		return m_data[1];
	}
	template <typename T>	
	inline  T &	Vec3<T>::Z() 
	{
		return m_data[2];
	}
	template <typename T>	
	inline  const T & Vec3<T>::X() const 
	{
		return m_data[0];
	}
	template <typename T>	
	inline  const T & Vec3<T>::Y() const 
	{
		return m_data[1];
	}
	template <typename T>	
	inline  const T & Vec3<T>::Z() const 
	{
		return m_data[2];
	}
	template <typename T>	
	inline  void Vec3<T>::Normalize()
	{
		T n = sqrt(m_data[0]*m_data[0]+m_data[1]*m_data[1]+m_data[2]*m_data[2]);
		if (n != 0.0) (*this) /= n;
	}
	template <typename T>	
	inline  T Vec3<T>::GetNorm() const 
	{ 
		return sqrt(m_data[0]*m_data[0]+m_data[1]*m_data[1]+m_data[2]*m_data[2]);
	}
	template <typename T>	
	inline  void Vec3<T>::operator= (const Vec3 & rhs)
	{ 
		this->m_data[0] = rhs.m_data[0]; 
		this->m_data[1] = rhs.m_data[1]; 
		this->m_data[2] = rhs.m_data[2]; 
	}
	template <typename T>	
	inline  void Vec3<T>::operator+=(const Vec3 & rhs)
	{ 
		this->m_data[0] += rhs.m_data[0]; 
		this->m_data[1] += rhs.m_data[1]; 
		this->m_data[2] += rhs.m_data[2]; 
	}     
	template <typename T>  
	inline void Vec3<T>::operator-=(const Vec3 & rhs)
	{ 
		this->m_data[0] -= rhs.m_data[0]; 
		this->m_data[1] -= rhs.m_data[1]; 
		this->m_data[2] -= rhs.m_data[2]; 
	}
	template <typename T>  
	inline void Vec3<T>::operator-=(T a)
	{ 
		this->m_data[0] -= a; 
		this->m_data[1] -= a; 
		this->m_data[2] -= a; 
	}
	template <typename T>  
	inline void Vec3<T>::operator+=(T a)
	{ 
		this->m_data[0] += a; 
		this->m_data[1] += a; 
		this->m_data[2] += a; 
	}
	template <typename T>  
	inline void Vec3<T>::operator/=(T a)
	{ 
		this->m_data[0] /= a; 
		this->m_data[1] /= a; 
		this->m_data[2] /= a; 
	}
	template <typename T>  
	inline void Vec3<T>::operator*=(T a)
	{ 
		this->m_data[0] *= a; 
		this->m_data[1] *= a; 
		this->m_data[2] *= a; 
	}  
	template <typename T>	
	inline Vec3<T> Vec3<T>::operator^ (const Vec3<T> & rhs) const
	{
		return Vec3<T>(m_data[1] * rhs.m_data[2] - m_data[2] * rhs.m_data[1],
					   m_data[2] * rhs.m_data[0] - m_data[0] * rhs.m_data[2],
					   m_data[0] * rhs.m_data[1] - m_data[1] * rhs.m_data[0]);
	}
	template <typename T>
	inline T Vec3<T>::operator*(const Vec3<T> & rhs) const
	{
		return (m_data[0] * rhs.m_data[0] + m_data[1] * rhs.m_data[1] + m_data[2] * rhs.m_data[2]);
	}        
	template <typename T>
	inline Vec3<T> Vec3<T>::operator+(const Vec3<T> & rhs) const
	{
		return Vec3<T>(m_data[0] + rhs.m_data[0],m_data[1] + rhs.m_data[1],m_data[2] + rhs.m_data[2]);
	}
	template <typename T> 
	inline  Vec3<T> Vec3<T>::operator-(const Vec3<T> & rhs) const
	{
		return Vec3<T>(m_data[0] - rhs.m_data[0],m_data[1] - rhs.m_data[1],m_data[2] - rhs.m_data[2]) ;
	}     
	template <typename T> 
	inline  Vec3<T> Vec3<T>::operator-() const
	{
		return Vec3<T>(-m_data[0],-m_data[1],-m_data[2]) ;
	}     

	template <typename T> 
	inline Vec3<T> Vec3<T>::operator*(T rhs) const
	{
		return Vec3<T>(rhs * this->m_data[0], rhs * this->m_data[1], rhs * this->m_data[2]);
	}
	template <typename T>
	inline Vec3<T> Vec3<T>::operator/ (T rhs) const
	{
		return Vec3<T>(m_data[0] / rhs, m_data[1] / rhs, m_data[2] / rhs);
	}
	template <typename T>
	inline Vec3<T>::Vec3(T a) 
	{ 
		m_data[0] = m_data[1] = m_data[2] = a; 
	}
	template <typename T>
	inline Vec3<T>::Vec3(T x, T y, T z)
	{
		m_data[0] = x;
		m_data[1] = y;
		m_data[2] = z;
	}
	template <typename T>
	inline Vec3<T>::Vec3(const Vec3 & rhs)
	{		
		m_data[0] = rhs.m_data[0];
		m_data[1] = rhs.m_data[1];
		m_data[2] = rhs.m_data[2];
	}
	template <typename T>
	inline Vec3<T>::~Vec3(void){};

	template <typename T>
	inline Vec3<T>::Vec3() {}
    
    template<typename T>
    inline const bool Colinear(const Vec3<T> & a, const Vec3<T> & b, const Vec3<T> & c)
    {
        return  ((c.Z() - a.Z()) * (b.Y() - a.Y()) - (b.Z() - a.Z()) * (c.Y() - a.Y()) == 0.0 /*EPS*/) &&
                ((b.Z() - a.Z()) * (c.X() - a.X()) - (b.X() - a.X()) * (c.Z() - a.Z()) == 0.0 /*EPS*/) &&
                ((b.X() - a.X()) * (c.Y() - a.Y()) - (b.Y() - a.Y()) * (c.X() - a.X()) == 0.0 /*EPS*/);
    }
    
    template<typename T>
    inline const T Volume(const Vec3<T> & a, const Vec3<T> & b, const Vec3<T> & c, const Vec3<T> & d)
    {
        return (a-d) * ((b-d) ^ (c-d));
    }
}
#endif //HACD_VECTOR_INL