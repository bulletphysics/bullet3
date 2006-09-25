// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
// Scalar.cpp
//
// Copyright (c) 2006 Simon Hobbs
//
// This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
#ifdef WIN32
#if _MSC_VER >= 1310

#include "Scalar.h"

const Scalar Scalar::Consts::MinusOne(-1.0f);
const Scalar Scalar::Consts::Zero(0.0f);
const Scalar Scalar::Consts::Half(0.5f);
const Scalar Scalar::Consts::One(1.0f);
const Scalar Scalar::Consts::Three(3.0f);

const Scalar Scalar::Consts::MinValue(-3.402823466e+38f);
const Scalar Scalar::Consts::MaxValue(3.402823466e+38f);
const Scalar Scalar::Consts::Epsilon(1.192092896e-07f);

const Scalar Scalar::Consts::PosInfinity(0x7f800000, true);
const Scalar Scalar::Consts::NegInfinity(0xff800000, true);

const Scalar Scalar::Consts::AbsMask(0x7fffffff, true);

#endif
#endif //WIN32

