/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Takahiro Harada


__inline
float2 make_float2(float x, float y)
{
	float2 v;
	v.s[0] = x; v.s[1] = y;
	return v;
}

__inline
float2 make_float2(float x)
{
	return make_float2(x,x);
}

__inline
float2 make_float2(const int2& x)
{
	return make_float2((float)x.s[0], (float)x.s[1]);
}




__inline
float2 operator-(const float2& a)
{
	return make_float2(-a.x, -a.y);
}

__inline
float2 operator*(const float2& a, const float2& b)
{
	float2 out;
	out.s[0] = a.s[0]*b.s[0];
	out.s[1] = a.s[1]*b.s[1];
	return out;
}

__inline
float2 operator*(float a, const float2& b)
{
	return make_float2(a*b.s[0], a*b.s[1]);
}

__inline
float2 operator*(const float2& b, float a)
{
	return make_float2(a*b.s[0], a*b.s[1]);
}

__inline
void operator*=(float2& a, const float2& b)
{
	a.s[0]*=b.s[0];
	a.s[1]*=b.s[1];
}

__inline
void operator*=(float2& a, float b)
{
	a.s[0]*=b;
	a.s[1]*=b;
}

__inline
float2 operator/(const float2& a, const float2& b)
{
	float2 out;
	out.s[0] = a.s[0]/b.s[0];
	out.s[1] = a.s[1]/b.s[1];
	return out;
}

__inline
float2 operator/(const float2& b, float a)
{
	return make_float2(b.s[0]/a, b.s[1]/a);
}

__inline
void operator/=(float2& a, const float2& b)
{
	a.s[0]/=b.s[0];
	a.s[1]/=b.s[1];
}

__inline
void operator/=(float2& a, float b)
{
	a.s[0]/=b;
	a.s[1]/=b;
}
//

__inline
float2 operator+(const float2& a, const float2& b)
{
	float2 out;
	out.s[0] = a.s[0]+b.s[0];
	out.s[1] = a.s[1]+b.s[1];
	return out;
}

__inline
float2 operator+(const float2& a, float b)
{
	float2 out;
	out.s[0] = a.s[0]+b;
	out.s[1] = a.s[1]+b;
	return out;
}

__inline
float2 operator-(const float2& a, const float2& b)
{
	float2 out;
	out.s[0] = a.s[0]-b.s[0];
	out.s[1] = a.s[1]-b.s[1];
	return out;
}

__inline
float2 operator-(const float2& a, float b)
{
	float2 out;
	out.s[0] = a.s[0]-b;
	out.s[1] = a.s[1]-b;
	return out;
}

__inline
void operator+=(float2& a, const float2& b)
{
	a.s[0]+=b.s[0];
	a.s[1]+=b.s[1];
}

__inline
void operator+=(float2& a, float b)
{
	a.s[0]+=b;
	a.s[1]+=b;
}

__inline
void operator-=(float2& a, const float2& b)
{
	a.s[0]-=b.s[0];
	a.s[1]-=b.s[1];
}

__inline
void operator-=(float2& a, float b)
{
	a.s[0]-=b;
	a.s[1]-=b;
}
