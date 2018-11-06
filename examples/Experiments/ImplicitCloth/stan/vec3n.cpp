//
// Big Vector and Sparse Matrix Classes
//

#include <float.h>

#include "vec3n.h"

float conjgrad_lasterror;
float conjgrad_epsilon = 0.1f;
int conjgrad_loopcount;
int conjgrad_looplimit = 100;

/*EXPORTVAR(conjgrad_lasterror);
EXPORTVAR(conjgrad_epsilon  );
EXPORTVAR(conjgrad_loopcount);
EXPORTVAR(conjgrad_looplimit);
*/

int ConjGradient(float3N &X, float3Nx3N &A, float3N &B)
{
	// Solves for unknown X in equation AX=B
	conjgrad_loopcount = 0;
	int n = B.count;
	float3N q(n), d(n), tmp(n), r(n);
	r = B - Mul(tmp, A, X);  // just use B if X known to be zero
	d = r;
	float s = dot(r, r);
	float starget = s * squared(conjgrad_epsilon);
	while (s > starget && conjgrad_loopcount++ < conjgrad_looplimit)
	{
		Mul(q, A, d);  // q = A*d;
		float a = s / dot(d, q);
		X = X + d * a;
		if (conjgrad_loopcount % 50 == 0)
		{
			r = B - Mul(tmp, A, X);
		}
		else
		{
			r = r - q * a;
		}
		float s_prev = s;
		s = dot(r, r);
		d = r + d * (s / s_prev);
	}
	conjgrad_lasterror = s;
	return conjgrad_loopcount < conjgrad_looplimit;  // true means we reached desired accuracy in given time - ie stable
}

int ConjGradientMod(float3N &X, float3Nx3N &A, float3N &B, int3 hack)
{
	// obsolete!!!
	// Solves for unknown X in equation AX=B
	conjgrad_loopcount = 0;
	int n = B.count;
	float3N q(n), d(n), tmp(n), r(n);
	r = B - Mul(tmp, A, X);  // just use B if X known to be zero
	r[hack[0]] = r[hack[1]] = r[hack[2]] = float3(0, 0, 0);
	d = r;
	float s = dot(r, r);
	float starget = s * squared(conjgrad_epsilon);
	while (s > starget && conjgrad_loopcount++ < conjgrad_looplimit)
	{
		Mul(q, A, d);  // q = A*d;
		q[hack[0]] = q[hack[1]] = q[hack[2]] = float3(0, 0, 0);
		float a = s / dot(d, q);
		X = X + d * a;
		if (conjgrad_loopcount % 50 == 0)
		{
			r = B - Mul(tmp, A, X);
			r[hack[0]] = r[hack[1]] = r[hack[2]] = float3(0, 0, 0);
		}
		else
		{
			r = r - q * a;
		}
		float s_prev = s;
		s = dot(r, r);
		d = r + d * (s / s_prev);
		d[hack[0]] = d[hack[1]] = d[hack[2]] = float3(0, 0, 0);
	}
	conjgrad_lasterror = s;
	return conjgrad_loopcount < conjgrad_looplimit;  // true means we reached desired accuracy in given time - ie stable
}

static inline void filter(float3N &V, const float3Nx3N &S)
{
	for (int i = 0; i < S.blocks.count; i++)
	{
		V[S.blocks[i].r] = V[S.blocks[i].r] * S.blocks[i].m;
	}
}

int ConjGradientFiltered(float3N &X, const float3Nx3N &A, const float3N &B, const float3Nx3N &S)
{
	// Solves for unknown X in equation AX=B
	conjgrad_loopcount = 0;
	int n = B.count;
	float3N q(n), d(n), tmp(n), r(n);
	r = B - Mul(tmp, A, X);  // just use B if X known to be zero
	filter(r, S);
	d = r;
	float s = dot(r, r);
	float starget = s * squared(conjgrad_epsilon);
	while (s > starget && conjgrad_loopcount++ < conjgrad_looplimit)
	{
		Mul(q, A, d);  // q = A*d;
		filter(q, S);
		float a = s / dot(d, q);
		X = X + d * a;
		if (conjgrad_loopcount % 50 == 0)
		{
			r = B - Mul(tmp, A, X);
			filter(r, S);
		}
		else
		{
			r = r - q * a;
		}
		float s_prev = s;
		s = dot(r, r);
		d = r + d * (s / s_prev);
		filter(d, S);
	}
	conjgrad_lasterror = s;
	return conjgrad_loopcount < conjgrad_looplimit;  // true means we reached desired accuracy in given time - ie stable
}

// test big vector math library:
static void testfloat3N()
{
	float3N a(2), b(2), c(2);
	a[0] = float3(1, 2, 3);
	a[1] = float3(4, 5, 6);
	b[0] = float3(10, 20, 30);
	b[1] = float3(40, 50, 60);
	//      c =  a+b+b * 10.0f;
	//      float d = dot(a+b,-b);
	int k;
	k = 0;
}
class dotest
{
public:
	dotest() { testfloat3N(); }
} do_test_at_program_startup;
