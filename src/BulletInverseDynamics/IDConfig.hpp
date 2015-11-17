///@file Configuration for Inverse Dynamics Library,
///      such as choice of linear algebra library and underlying scalar type
#ifndef IDCONFIG_HPP_
#define IDCONFIG_HPP_
// If we have a custom configuration, compile without using other parts of bullet.
#ifdef BT_CUSTOM_INVERSE_DYNAMICS_CONFIG_H
#define BT_ID_WO_BULLET
#endif
// error messages
#include "IDErrorMessages.hpp"

#ifdef BT_CUSTOM_INVERSE_DYNAMICS_CONFIG_H
#define INVDYN_INCLUDE_HELPER_2(x) #x
#define INVDYN_INCLUDE_HELPER(x) INVDYN_INCLUDE_HELPER_2(x)
#include INVDYN_INCLUDE_HELPER(BT_CUSTOM_INVERSE_DYNAMICS_CONFIG_H)
#else
// Use default configuration with bullet's types
// Use the same scalar type as rest of bullet library
#include "LinearMath/btScalar.h"
typedef btScalar idScalar;
// use bullet types for arrays and array indices
#include "Bullet3Common/b3AlignedObjectArray.h"
// this is to make it work with C++2003, otherwise we could do this:
// template <typename T>
// using idArray = b3AlignedObjectArray<T>;
template <typename T>
struct idArray {
    typedef b3AlignedObjectArray<T> type;
};
typedef int idArrayIdx;
#define ID_DECLARE_ALIGNED_ALLOCATOR B3_DECLARE_ALIGNED_ALLOCATOR

// use bullet's allocator functions
#define idMalloc btAllocFunc
#define idFree btFreeFunc

#define ID_LINEAR_MATH_USE_BULLET
#include "details/IDLinearMathInterface.hpp"
#endif
#endif
