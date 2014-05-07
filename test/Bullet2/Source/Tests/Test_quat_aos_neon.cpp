//
//  Test_quat_aos_neon.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc., Inc.
//


#include "LinearMath/btScalar.h"
#if defined (BT_USE_SSE_IN_API) || defined (BT_USE_NEON)

#include "Test_quat_aos_neon.h"
#include "vector.h"
#include "Utils.h"
#include "main.h"

#include <vectormath/vmInclude.h>


//typedef Vectormath::Aos::Vector3    vmVector3;
//typedef Vectormath::Aos::Quat       vmQuat;
//typedef Vectormath::Aos::Matrix3    vmMatrix3;
//typedef Vectormath::Aos::Transform3 vmTransform3;
//typedef Vectormath::Aos::Point3     vmPoint3;


typedef Vectormath::Aos::Vector4    vmVector4;

// reference code for testing purposes
ATTRIBUTE_ALIGNED16(class) Quat_ref
{
    float mX;
    float mY;
    float mZ;
    float mW;
    
public:
    // Default constructor; does no initialization
    // 
    inline Quat_ref( ) { };
    
    // Copy a quaternion
    // 
    inline Quat_ref( const Quat_ref & quat );
    
    // Construct a quaternion from x, y, z, and w elements
    // 
    inline Quat_ref( float x, float y, float z, float w );
    
    // Construct a quaternion from a 3-D vector and a scalar
    // 
    inline Quat_ref( const vmVector3 & xyz, float w );
    
    // Copy elements from a 4-D vector into a quaternion
    // 
    explicit inline Quat_ref( const vmVector4 & vec );
    
    // Convert a rotation matrix to a unit-length quaternion
    // 
    explicit inline Quat_ref( const vmMatrix3 & rotMat );
    
    // Set all elements of a quaternion to the same scalar value
    // 
    explicit inline Quat_ref( float scalar );
    
    // Assign one quaternion to another
    // 
    inline Quat_ref & operator =( const Quat_ref & quat );
    
    // Set the x, y, and z elements of a quaternion
    // NOTE: 
    // This function does not change the w element.
    // 
    inline Quat_ref & setXYZ( const vmVector3 & vec );
    
    // Get the x, y, and z elements of a quaternion
    // 
    inline const vmVector3 getXYZ( ) const;
    
    // Set the x element of a quaternion
    // 
    inline Quat_ref & setX( float x );
    
    // Set the y element of a quaternion
    // 
    inline Quat_ref & setY( float y );
    
    // Set the z element of a quaternion
    // 
    inline Quat_ref & setZ( float z );
    
    // Set the w element of a quaternion
    // 
    inline Quat_ref & setW( float w );
    
    // Get the x element of a quaternion
    // 
    inline float getX( ) const;
    
    // Get the y element of a quaternion
    // 
    inline float getY( ) const;
    
    // Get the z element of a quaternion
    // 
    inline float getZ( ) const;
    
    // Get the w element of a quaternion
    // 
    inline float getW( ) const;
    
    // Set an x, y, z, or w element of a quaternion by index
    // 
    inline Quat_ref & setElem( int idx, float value );
    
    // Get an x, y, z, or w element of a quaternion by index
    // 
    inline float getElem( int idx ) const;
    
    // Subscripting operator to set or get an element
    // 
    inline float & operator []( int idx );
    
    // Subscripting operator to get an element
    // 
    inline float operator []( int idx ) const;
    
    // Add two quaternions
    // 
    inline const Quat_ref operator +( const Quat_ref & quat ) const;
    
    // Subtract a quaternion from another quaternion
    // 
    inline const Quat_ref operator -( const Quat_ref & quat ) const;
    
    // Multiply two quaternions
    // 
    inline const Quat_ref operator *( const Quat_ref & quat ) const;
    
    // Multiply a quaternion by a scalar
    // 
    inline const Quat_ref operator *( float scalar ) const;
    
    // Divide a quaternion by a scalar
    // 
    inline const Quat_ref operator /( float scalar ) const;
    
    // Perform compound assignment and addition with a quaternion
    // 
    inline Quat_ref & operator +=( const Quat_ref & quat );
    
    // Perform compound assignment and subtraction by a quaternion
    // 
    inline Quat_ref & operator -=( const Quat_ref & quat );
    
    // Perform compound assignment and multiplication by a quaternion
    // 
    inline Quat_ref & operator *=( const Quat_ref & quat );
    
    // Perform compound assignment and multiplication by a scalar
    // 
    inline Quat_ref & operator *=( float scalar );
    
    // Perform compound assignment and division by a scalar
    // 
    inline Quat_ref & operator /=( float scalar );
    
    // Negate all elements of a quaternion
    // 
    inline const Quat_ref operator -( ) const;
    
    // Construct an identity quaternion
    // 
    static inline const Quat_ref identity( );
    
    // Construct a quaternion to rotate between two unit-length 3-D vectors
    // NOTE: 
    // The result is unpredictable if unitVec0 and unitVec1 point in opposite directions.
    // 
    static inline const Quat_ref rotation( const vmVector3 & unitVec0, const vmVector3 & unitVec1 );
    
    // Construct a quaternion to rotate around a unit-length 3-D vector
    // 
    static inline const Quat_ref rotation( float radians, const vmVector3 & unitVec );
    
    // Construct a quaternion to rotate around the x axis
    // 
    static inline const Quat_ref rotationX( float radians );
    
    // Construct a quaternion to rotate around the y axis
    // 
    static inline const Quat_ref rotationY( float radians );
    
    // Construct a quaternion to rotate around the z axis
    // 
    static inline const Quat_ref rotationZ( float radians );
    
};

inline Quat_ref::Quat_ref( const Quat_ref & quat )
{
    mX = quat.mX;
    mY = quat.mY;
    mZ = quat.mZ;
    mW = quat.mW;
}

inline Quat_ref::Quat_ref( float _x, float _y, float _z, float _w )
{
    mX = _x;
    mY = _y;
    mZ = _z;
    mW = _w;
}

inline Quat_ref::Quat_ref( const vmVector3 & xyz, float _w )
{
    this->setXYZ( xyz );
    this->setW( _w );
}

inline Quat_ref::Quat_ref( const vmVector4 & vec )
{
    mX = vec.getX();
    mY = vec.getY();
    mZ = vec.getZ();
    mW = vec.getW();
}

inline Quat_ref::Quat_ref( float scalar )
{
    mX = scalar;
    mY = scalar;
    mZ = scalar;
    mW = scalar;
}

inline const Quat_ref Quat_ref::identity( )
{
    return Quat_ref( 0.0f, 0.0f, 0.0f, 1.0f );
}


inline void loadXYZW_ref( Quat_ref & quat, const float * fptr )
{
    quat = Quat_ref( fptr[0], fptr[1], fptr[2], fptr[3] );
}

inline void storeXYZW_ref( const Quat_ref & quat, float * fptr )
{
    fptr[0] = quat.getX();
    fptr[1] = quat.getY();
    fptr[2] = quat.getZ();
    fptr[3] = quat.getW();
}

inline Quat_ref & Quat_ref::operator =( const Quat_ref & quat )
{
    mX = quat.mX;
    mY = quat.mY;
    mZ = quat.mZ;
    mW = quat.mW;
    return *this;
}

inline Quat_ref & Quat_ref::setXYZ( const vmVector3 & vec )
{
    mX = vec.getX();
    mY = vec.getY();
    mZ = vec.getZ();
    return *this;
}

inline const vmVector3 Quat_ref::getXYZ( ) const
{
    return vmVector3( mX, mY, mZ );
}

inline Quat_ref & Quat_ref::setX( float _x )
{
    mX = _x;
    return *this;
}

inline float Quat_ref::getX( ) const
{
    return mX;
}

inline Quat_ref & Quat_ref::setY( float _y )
{
    mY = _y;
    return *this;
}

inline float Quat_ref::getY( ) const
{
    return mY;
}

inline Quat_ref & Quat_ref::setZ( float _z )
{
    mZ = _z;
    return *this;
}

inline float Quat_ref::getZ( ) const
{
    return mZ;
}

inline Quat_ref & Quat_ref::setW( float _w )
{
    mW = _w;
    return *this;
}

inline float Quat_ref::getW( ) const
{
    return mW;
}

inline Quat_ref & Quat_ref::setElem( int idx, float value )
{
    *(&mX + idx) = value;
    return *this;
}

inline float Quat_ref::getElem( int idx ) const
{
    return *(&mX + idx);
}

inline float & Quat_ref::operator []( int idx )
{
    return *(&mX + idx);
}

inline float Quat_ref::operator []( int idx ) const
{
    return *(&mX + idx);
}

inline const Quat_ref Quat_ref::operator +( const Quat_ref & quat ) const
{
    return Quat_ref(
                ( mX + quat.mX ),
                ( mY + quat.mY ),
                ( mZ + quat.mZ ),
                ( mW + quat.mW )
                );
}

inline const Quat_ref Quat_ref::operator -( const Quat_ref & quat ) const
{
    return Quat_ref(
                ( mX - quat.mX ),
                ( mY - quat.mY ),
                ( mZ - quat.mZ ),
                ( mW - quat.mW )
                );
}

inline const Quat_ref Quat_ref::operator *( float scalar ) const
{
    return Quat_ref(
                ( mX * scalar ),
                ( mY * scalar ),
                ( mZ * scalar ),
                ( mW * scalar )
                );
}

inline Quat_ref & Quat_ref::operator +=( const Quat_ref & quat )
{
    *this = *this + quat;
    return *this;
}

inline Quat_ref & Quat_ref::operator -=( const Quat_ref & quat )
{
    *this = *this - quat;
    return *this;
}

inline Quat_ref & Quat_ref::operator *=( float scalar )
{
    *this = *this * scalar;
    return *this;
}

inline const Quat_ref Quat_ref::operator /( float scalar ) const
{
    return Quat_ref(
                ( mX / scalar ),
                ( mY / scalar ),
                ( mZ / scalar ),
                ( mW / scalar )
                );
}

inline Quat_ref & Quat_ref::operator /=( float scalar )
{
    *this = *this / scalar;
    return *this;
}

inline const Quat_ref Quat_ref::operator -( ) const
{
    return Quat_ref(
                -mX,
                -mY,
                -mZ,
                -mW
                );
}

inline const Quat_ref operator *( float scalar, const Quat_ref & quat )
{
    return quat * scalar;
}

inline float dot( const Quat_ref & quat0, const Quat_ref & quat1 )
{
    float result;
    result = ( quat0.getX() * quat1.getX() );
    result = ( result + ( quat0.getY() * quat1.getY() ) );
    result = ( result + ( quat0.getZ() * quat1.getZ() ) );
    result = ( result + ( quat0.getW() * quat1.getW() ) );
    return result;
}

inline const Quat_ref lerp( float t, const Quat_ref & quat0, const Quat_ref & quat1 )
{
    return ( quat0 + ( ( quat1 - quat0 ) * t ) );
}

inline const Quat_ref slerp( float t, const Quat_ref & unitQuat0, const Quat_ref & unitQuat1 )
{
    Quat_ref start;
    float recipSinAngle, scale0, scale1, cosAngle, angle;
    cosAngle = dot( unitQuat0, unitQuat1 );
    if ( cosAngle < 0.0f ) {
        cosAngle = -cosAngle;
        start = ( -unitQuat0 );
    } else {
        start = unitQuat0;
    }
    if ( cosAngle < _VECTORMATH_SLERP_TOL ) {
        angle = acosf( cosAngle );
        recipSinAngle = ( 1.0f / sinf( angle ) );
        scale0 = ( sinf( ( ( 1.0f - t ) * angle ) ) * recipSinAngle );
        scale1 = ( sinf( ( t * angle ) ) * recipSinAngle );
    } else {
        scale0 = ( 1.0f - t );
        scale1 = t;
    }
    return ( ( start * scale0 ) + ( unitQuat1 * scale1 ) );
}

inline const Quat_ref squad( float t, const Quat_ref & unitQuat0, const Quat_ref & unitQuat1, const Quat_ref & unitQuat2, const Quat_ref & unitQuat3 )
{
    Quat_ref tmp0, tmp1;
    tmp0 = slerp( t, unitQuat0, unitQuat3 );
    tmp1 = slerp( t, unitQuat1, unitQuat2 );
    return slerp( ( ( 2.0f * t ) * ( 1.0f - t ) ), tmp0, tmp1 );
}

inline float norm( const Quat_ref & quat )
{
    float result;
    result = ( quat.getX() * quat.getX() );
    result = ( result + ( quat.getY() * quat.getY() ) );
    result = ( result + ( quat.getZ() * quat.getZ() ) );
    result = ( result + ( quat.getW() * quat.getW() ) );
    return result;
}

inline float length( const Quat_ref & quat )
{
    return ::sqrtf( norm( quat ) );
}

inline const Quat_ref normalize( const Quat_ref & quat )
{
    float lenSqr, lenInv;
    lenSqr = norm( quat );
    lenInv = ( 1.0f / sqrtf( lenSqr ) );
    return Quat_ref(
                ( quat.getX() * lenInv ),
                ( quat.getY() * lenInv ),
                ( quat.getZ() * lenInv ),
                ( quat.getW() * lenInv )
                );
}

inline const Quat_ref Quat_ref::rotation( const vmVector3 & unitVec0, const vmVector3 & unitVec1 )
{
    float cosHalfAngleX2, recipCosHalfAngleX2;
    cosHalfAngleX2 = sqrtf( ( 2.0f * ( 1.0f + dot( unitVec0, unitVec1 ) ) ) );
    recipCosHalfAngleX2 = ( 1.0f / cosHalfAngleX2 );
    return Quat_ref( ( cross( unitVec0, unitVec1 ) * recipCosHalfAngleX2 ), ( cosHalfAngleX2 * 0.5f ) );
}

inline const Quat_ref Quat_ref::rotation( float radians, const vmVector3 & unitVec )
{
    float s, c, angle;
    angle = ( radians * 0.5f );
    s = sinf( angle );
    c = cosf( angle );
    return Quat_ref( ( unitVec * s ), c );
}

inline const Quat_ref Quat_ref::rotationX( float radians )
{
    float s, c, angle;
    angle = ( radians * 0.5f );
    s = sinf( angle );
    c = cosf( angle );
    return Quat_ref( s, 0.0f, 0.0f, c );
}

inline const Quat_ref Quat_ref::rotationY( float radians )
{
    float s, c, angle;
    angle = ( radians * 0.5f );
    s = sinf( angle );
    c = cosf( angle );
    return Quat_ref( 0.0f, s, 0.0f, c );
}

inline const Quat_ref Quat_ref::rotationZ( float radians )
{
    float s, c, angle;
    angle = ( radians * 0.5f );
    s = sinf( angle );
    c = cosf( angle );
    return Quat_ref( 0.0f, 0.0f, s, c );
}

inline const Quat_ref Quat_ref::operator *( const Quat_ref & quat ) const
{
    return Quat_ref(
                ( ( ( ( mW * quat.mX ) + ( mX * quat.mW ) ) + ( mY * quat.mZ ) ) - ( mZ * quat.mY ) ),
                ( ( ( ( mW * quat.mY ) + ( mY * quat.mW ) ) + ( mZ * quat.mX ) ) - ( mX * quat.mZ ) ),
                ( ( ( ( mW * quat.mZ ) + ( mZ * quat.mW ) ) + ( mX * quat.mY ) ) - ( mY * quat.mX ) ),
                ( ( ( ( mW * quat.mW ) - ( mX * quat.mX ) ) - ( mY * quat.mY ) ) - ( mZ * quat.mZ ) )
                );
}

inline Quat_ref & Quat_ref::operator *=( const Quat_ref & quat )
{
    *this = *this * quat;
    return *this;
}

inline const vmVector3 rotate( const Quat_ref & quat, const vmVector3 & vec )
{
    float tmpX, tmpY, tmpZ, tmpW;
    tmpX = ( ( ( quat.getW() * vec.getX() ) + ( quat.getY() * vec.getZ() ) ) - ( quat.getZ() * vec.getY() ) );
    tmpY = ( ( ( quat.getW() * vec.getY() ) + ( quat.getZ() * vec.getX() ) ) - ( quat.getX() * vec.getZ() ) );
    tmpZ = ( ( ( quat.getW() * vec.getZ() ) + ( quat.getX() * vec.getY() ) ) - ( quat.getY() * vec.getX() ) );
    tmpW = ( ( ( quat.getX() * vec.getX() ) + ( quat.getY() * vec.getY() ) ) + ( quat.getZ() * vec.getZ() ) );
    return vmVector3(
                   ( ( ( ( tmpW * quat.getX() ) + ( tmpX * quat.getW() ) ) - ( tmpY * quat.getZ() ) ) + ( tmpZ * quat.getY() ) ),
                   ( ( ( ( tmpW * quat.getY() ) + ( tmpY * quat.getW() ) ) - ( tmpZ * quat.getX() ) ) + ( tmpX * quat.getZ() ) ),
                   ( ( ( ( tmpW * quat.getZ() ) + ( tmpZ * quat.getW() ) ) - ( tmpX * quat.getY() ) ) + ( tmpY * quat.getX() ) )
                   );
}

inline const Quat_ref conj( const Quat_ref & quat )
{
    return Quat_ref( -quat.getX(), -quat.getY(), -quat.getZ(), quat.getW() );
}

inline const Quat_ref select( const Quat_ref & quat0, const Quat_ref & quat1, bool select1 )
{
    return Quat_ref(
                ( select1 )? quat1.getX() : quat0.getX(),
                ( select1 )? quat1.getY() : quat0.getY(),
                ( select1 )? quat1.getZ() : quat0.getZ(),
                ( select1 )? quat1.getW() : quat0.getW()
                );
}



#define LOOPCOUNT 1000
#define NUM_CYCLES 10000
#define DATA_SIZE 1024

int Test_quat_aos_neon(void)
{
       
    return 0;
}

#endif

