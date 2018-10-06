//
// Big Vector and Sparse Matrix Classes
//
//   (c) S Melax 2006
//
// The focus is on 3D applications, so
// the big vector is an array of float3s
// and the matrix class uses 3x3 blocks.
//
// This file includes both:
//  - basic non-optimized version
//  - an expression optimized version
//
// Optimized Expressions
//
// We want to write sweet looking code such as V=As+Bt with big vectors.
// However, we dont want the extra overheads with allocating memory for temps and excessing copying.
// Instead of a full Template Metaprogramming approach, we explicitly write
// classes to specifically handle all the expressions we are likely to use.
// Most applicable lines of code will be of the same handful of basic forms,
// but with different parameters for the operands.
// In the future, if we ever need a longer expression with more operands,
// then we will just add whatever additional building blocks that are necessary - not a big deal.
// This approach is much simpler to develop, debug and optimize (restrict keyword, simd etc)
// than template metaprogramming is.  We do not rely on the implementation
// of a particular compiler to be able to expand extensive nested inline codes.
// Additionally, we reliably get our optimizations even within a debug build.
// Therefore we believe that our Optimized Expressions
// are a good compromise that give us the best of both worlds.
// The code within those important algorithms, which use this library,
// can now remain clean and readable yet still execute quickly.
//

#ifndef SM_VEC3N_H
#define SM_VEC3N_H

#include "vecmath.h"
#include "array.h"

//#include <malloc.h>
//template <class T> void * vec4<T>::operator new[](size_t n){ return _mm_malloc(n,64); }
//template <class T> void vec4<T>::operator delete[](void *a) { _mm_free(a); }

struct HalfConstraint
{
	float3 n;
	int vi;
	float s, t;
	HalfConstraint(const float3 &_n, int _vi, float _t) : n(_n), vi(_vi), s(0), t(_t) {}
	HalfConstraint() : vi(-1) {}
};

class float3Nx3N
{
public:
	class Block
	{
	public:
		float3x3 m;
		int r, c;
		float unused[16];

		Block() {}
		Block(short _r, short _c) : r(_r), c(_c) { m.x = m.y = m.z = float3(0, 0, 0); }
	};
	Array<Block> blocks;  // the first n blocks use as the diagonal.
	int n;
	void Zero();
	void InitDiagonal(float d);
	void Identity() { InitDiagonal(1.0f); }
	float3Nx3N() : n(0) {}
	float3Nx3N(int _n) : n(_n)
	{
		for (int i = 0; i < n; i++) blocks.Add(Block((short)i, (short)i));
	}
	template <class E>
	float3Nx3N &operator=(const E &expression)
	{
		expression.evalequals(*this);
		return *this;
	}
	template <class E>
	float3Nx3N &operator+=(const E &expression)
	{
		expression.evalpluseq(*this);
		return *this;
	}
	template <class E>
	float3Nx3N &operator-=(const E &expression)
	{
		expression.evalmnuseq(*this);
		return *this;
	}
};

class float3N : public Array<float3>
{
public:
	float3N(int _count = 0)
	{
		SetSize(_count);
	}
	void Zero();
	void Init(const float3 &v);  // sets each subvector to v
	template <class E>
	float3N &operator=(const E &expression)
	{
		expression.evalequals(*this);
		return *this;
	}
	template <class E>
	float3N &operator+=(const E &expression)
	{
		expression.evalpluseq(*this);
		return *this;
	}
	template <class E>
	float3N &operator-=(const E &expression)
	{
		expression.evalmnuseq(*this);
		return *this;
	}
	float3N &operator=(const float3N &V)
	{
		this->copy(V);
		return *this;
	}
};

int ConjGradient(float3N &X, float3Nx3N &A, float3N &B);
int ConjGradientFiltered(float3N &X, const float3Nx3N &A, const float3N &B, const float3Nx3N &S, Array<HalfConstraint> &H);
int ConjGradientFiltered(float3N &X, const float3Nx3N &A, const float3N &B, const float3Nx3N &S);

inline float3N &Mul(float3N &r, const float3Nx3N &m, const float3N &v)
{
	int i;
	for (i = 0; i < r.count; i++) r[i] = float3(0, 0, 0);
	for (i = 0; i < m.blocks.count; i++)
	{
		r[m.blocks[i].r] += m.blocks[i].m * v[m.blocks[i].c];
	}
	return r;
}

inline float dot(const float3N &a, const float3N &b)
{
	float d = 0;
	for (int i = 0; i < a.count; i++)
	{
		d += dot(a[i], b[i]);
	}
	return d;
}

inline void float3Nx3N::Zero()
{
	for (int i = 0; i < blocks.count; i++)
	{
		blocks[i].m = float3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);
	}
}

inline void float3Nx3N::InitDiagonal(float d)
{
	for (int i = 0; i < blocks.count; i++)
	{
		blocks[i].m = (blocks[i].c == blocks[i].r) ? float3x3(d, 0, 0, 0, d, 0, 0, 0, d) : float3x3(0, 0, 0, 0, 0, 0, 0, 0, 0);
	}
}

inline void float3N::Zero()
{
	for (int i = 0; i < count; i++)
	{
		element[i] = float3(0, 0, 0);
	}
}

inline void float3N::Init(const float3 &v)
{
	for (int i = 0; i < count; i++)
	{
		element[i] = v;
	}
}

#ifdef WE_LIKE_SLOW_CODE

// Unoptimized Slow Basic Version of big vector operators.
// Uses typical implmentation for operators +/-*=
// These operators cause lots of unnecessary construction, memory allocation, and copying.

inline float3N operator+(const float3N &a, const float3N &b)
{
	float3N r(a.count);
	for (int i = 0; i < a.count; i++) r[i] = a[i] + b[i];
	return r;
}

inline float3N operator*(const float3N &a, const float &s)
{
	float3N r(a.count);
	for (int i = 0; i < a.count; i++) r[i] = a[i] * s;
	return r;
}
inline float3N operator/(const float3N &a, const float &s)
{
	float3N r(a.count);
	return Mul(r, a, 1.0f / s);
}
inline float3N operator-(const float3N &a, const float3N &b)
{
	float3N r(a.count);
	for (int i = 0; i < a.count; i++) r[i] = a[i] - b[i];
	return r;
}
inline float3N operator-(const float3N &a)
{
	float3N r(a.count);
	for (int i = 0; i < a.count; i++) r[i] = -a[i];
	return r;
}

inline float3N operator*(const float3Nx3N &m, const float3N &v)
{
	float3N r(v.count);
	return Mul(r, m, v);
}
inline float3N &operator-=(float3N &A, const float3N &B)
{
	assert(A.count == B.count);
	for (int i = 0; i < A.count; i++) A[i] -= B[i];
	return A;
}
inline float3N &operator+=(float3N &A, const float3N &B)
{
	assert(A.count == B.count);
	for (int i = 0; i < A.count; i++) A[i] += B[i];
	return A;
}

#else

// Optimized Expressions

class exVneg
{
public:
	const float3N &v;
	exVneg(const float3N &_v) : v(_v) {}
	void evalequals(float3N &r) const
	{
		for (int i = 0; i < v.count; i++) r[i] = -v[i];
	}
	void evalpluseq(float3N &r) const
	{
		for (int i = 0; i < v.count; i++) r[i] += -v[i];
	}
	void evalmnuseq(float3N &r) const
	{
		for (int i = 0; i < v.count; i++) r[i] -= -v[i];
	}
};

class exVaddV
{
public:
	const float3N &a;
	const float3N &b;
	exVaddV(const float3N &_a, const float3N &_b) : a(_a), b(_b) {}
	void evalequals(float3N &r) const
	{
		for (int i = 0; i < a.count; i++) r[i] = a[i] + b[i];
	}
	void evalpluseq(float3N &r) const
	{
		for (int i = 0; i < a.count; i++) r[i] += a[i] + b[i];
	}
	void evalmnuseq(float3N &r) const
	{
		for (int i = 0; i < a.count; i++) r[i] -= a[i] + b[i];
	}
};

class exVsubV
{
public:
	const float3N &a;
	const float3N &b;
	exVsubV(const float3N &_a, const float3N &_b) : a(_a), b(_b) {}
	void evalequals(float3N &r) const
	{
		for (int i = 0; i < a.count; i++) r[i] = a[i] - b[i];
	}
	void evalpluseq(float3N &r) const
	{
		for (int i = 0; i < a.count; i++) r[i] += a[i] - b[i];
	}
	void evalmnuseq(float3N &r) const
	{
		for (int i = 0; i < a.count; i++) r[i] -= a[i] - b[i];
	}
};

class exVs
{
public:
	const float3N &v;
	const float s;
	exVs(const float3N &_v, const float &_s) : v(_v), s(_s) {}
	void evalequals(float3N &r) const
	{
		for (int i = 0; i < v.count; i++) r[i] = v[i] * s;
	}
	void evalpluseq(float3N &r) const
	{
		for (int i = 0; i < v.count; i++) r[i] += v[i] * s;
	}
	void evalmnuseq(float3N &r) const
	{
		for (int i = 0; i < v.count; i++) r[i] -= v[i] * s;
	}
};
class exAsaddB
{
public:
	const float3N &a;
	const float3N &b;
	const float s;
	exAsaddB(const float3N &_a, const float &_s, const float3N &_b) : a(_a), s(_s), b(_b) {}
	void evalequals(float3N &r) const
	{
		for (int i = 0; i < a.count; i++) r[i] = a[i] * s + b[i];
	}
	void evalpluseq(float3N &r) const
	{
		for (int i = 0; i < a.count; i++) r[i] += a[i] * s + b[i];
	}
	void evalmnuseq(float3N &r) const
	{
		for (int i = 0; i < a.count; i++) r[i] -= a[i] * s + b[i];
	}
};
class exAsaddBt
{
public:
	const float3N &a;
	const float3N &b;
	const float s;
	const float t;
	exAsaddBt(const float3N &_a, const float &_s, const float3N &_b, const float &_t) : a(_a), s(_s), b(_b), t(_t) {}
	void evalequals(float3N &r) const
	{
		for (int i = 0; i < a.count; i++) r[i] = a[i] * s + b[i] * t;
	}
	void evalpluseq(float3N &r) const
	{
		for (int i = 0; i < a.count; i++) r[i] += a[i] * s + b[i] * t;
	}
	void evalmnuseq(float3N &r) const
	{
		for (int i = 0; i < a.count; i++) r[i] -= a[i] * s + b[i] * t;
	}
};

class exMv
{
public:
	const float3Nx3N &m;
	const float3N &v;
	exMv(const float3Nx3N &_m, const float3N &_v) : m(_m), v(_v) {}
	void evalequals(float3N &r) const { Mul(r, m, v); }
};

class exMs
{
public:
	const float3Nx3N &m;
	const float s;
	exMs(const float3Nx3N &_m, const float &_s) : m(_m), s(_s) {}
	void evalequals(float3Nx3N &r) const
	{
		for (int i = 0; i < r.blocks.count; i++) r.blocks[i].m = m.blocks[i].m * s;
	}
	void evalpluseq(float3Nx3N &r) const
	{
		for (int i = 0; i < r.blocks.count; i++) r.blocks[i].m += m.blocks[i].m * s;
	}
	void evalmnuseq(float3Nx3N &r) const
	{
		for (int i = 0; i < r.blocks.count; i++) r.blocks[i].m -= m.blocks[i].m * s;
	}
};

class exMAsMBt
{
public:
	const float3Nx3N &a;
	const float s;
	const float3Nx3N &b;
	const float t;
	exMAsMBt(const float3Nx3N &_a, const float &_s, const float3Nx3N &_b, const float &_t) : a(_a), s(_s), b(_b), t(_t) {}
	void evalequals(float3Nx3N &r) const
	{
		for (int i = 0; i < r.blocks.count; i++) r.blocks[i].m = a.blocks[i].m * s + b.blocks[i].m * t;
	}
	void evalpluseq(float3Nx3N &r) const
	{
		for (int i = 0; i < r.blocks.count; i++) r.blocks[i].m += a.blocks[i].m * s + b.blocks[i].m * t;
	}
	void evalmnuseq(float3Nx3N &r) const
	{
		for (int i = 0; i < r.blocks.count; i++) r.blocks[i].m -= a.blocks[i].m * s + b.blocks[i].m * t;
	}
};

inline exVaddV operator+(const float3N &a, const float3N &b) { return exVaddV(a, b); }
inline exVsubV operator+(const exVneg &E, const float3N &b) { return exVsubV(b, E.v); }
inline exVsubV operator-(const float3N &a, const float3N &b) { return exVsubV(a, b); }
inline exVs operator*(const float3N &V, const float &s) { return exVs(V, s); }
inline exVs operator*(const exVs &E, const float &s) { return exVs(E.v, E.s * s); }
inline exAsaddB operator+(const exVs &E, const float3N &b) { return exAsaddB(E.v, E.s, b); }
inline exAsaddB operator+(const float3N &b, const exVs &E) { return exAsaddB(E.v, E.s, b); }
inline exAsaddB operator-(const float3N &b, const exVs &E) { return exAsaddB(E.v, -E.s, b); }
inline exAsaddBt operator+(const exVs &Ea, const exVs &Eb) { return exAsaddBt(Ea.v, Ea.s, Eb.v, Eb.s); }
inline exAsaddBt operator-(const exVs &Ea, const exVs &Eb) { return exAsaddBt(Ea.v, Ea.s, Eb.v, -Eb.s); }
inline exMv operator*(const float3Nx3N &m, const float3N &v) { return exMv(m, v); }
inline exMs operator*(const exMs &E, const float &s) { return exMs(E.m, E.s * s); }
inline exMs operator*(const float3Nx3N &m, const float &s) { return exMs(m, s); }
inline exMAsMBt operator+(const exMs &Ea, const exMs &Eb) { return exMAsMBt(Ea.m, Ea.s, Eb.m, Eb.s); }
inline exMAsMBt operator-(const exMs &Ea, const exMs &Eb) { return exMAsMBt(Ea.m, Ea.s, Eb.m, -Eb.s); }

#endif

#endif
