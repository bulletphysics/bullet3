//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#ifndef PX_PHYSICS_EXTENSIONS_MASS_PROPERTIES_H
#define PX_PHYSICS_EXTENSIONS_MASS_PROPERTIES_H
/** \addtogroup extensions
  @{
*/

#include "PxPhysXConfig.h"
#include "foundation/PxMath.h"
#include "foundation/PxMathUtils.h"
#include "foundation/PxVec3.h"
#include "foundation/PxMat33.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"
#include "geometry/PxGeometry.h"
#include "geometry/PxBoxGeometry.h"
#include "geometry/PxSphereGeometry.h"
#include "geometry/PxCapsuleGeometry.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "geometry/PxConvexMesh.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Utility class to compute and manipulate mass and inertia tensor properties.

In most cases #PxRigidBodyExt::updateMassAndInertia(), #PxRigidBodyExt::setMassAndUpdateInertia() should be enough to
setup the mass properties of a rigid body. This utility class targets users that need to customize the mass properties
computation.
*/
class PxMassProperties
{
public:
	/**
	\brief Default constructor.
	*/
	PX_FORCE_INLINE PxMassProperties() : inertiaTensor(PxIdentity), centerOfMass(0.0f), mass(1.0f) {}

	/**
	\brief Construct from individual elements.
	*/
	PX_FORCE_INLINE PxMassProperties(const PxReal m, const PxMat33& inertiaT, const PxVec3& com) : inertiaTensor(inertiaT), centerOfMass(com), mass(m) {}

	/**
	\brief Compute mass properties based on a provided geometry structure.

	This constructor assumes the geometry has a density of 1. Mass and inertia tensor scale linearly with density.

	\param[in] geometry The geometry to compute the mass properties for. Supported geometry types are: sphere, box, capsule and convex mesh.
	*/
	PxMassProperties(const PxGeometry& geometry)
	{
		switch (geometry.getType())
		{
			case PxGeometryType::eSPHERE:
			{
				const PxSphereGeometry& s = static_cast<const PxSphereGeometry&>(geometry);
				mass = (4.0f / 3.0f) * PxPi * s.radius * s.radius * s.radius;
				inertiaTensor = PxMat33::createDiagonal(PxVec3(2.0f / 5.0f * mass * s.radius * s.radius));
				centerOfMass = PxVec3(0.0f);
			}
			break;

			case PxGeometryType::eBOX:
			{
				const PxBoxGeometry& b = static_cast<const PxBoxGeometry&>(geometry);
				mass = b.halfExtents.x * b.halfExtents.y * b.halfExtents.z * 8.0f;
				PxVec3 d2 = b.halfExtents.multiply(b.halfExtents);
				inertiaTensor = PxMat33::createDiagonal(PxVec3(d2.y + d2.z, d2.x + d2.z, d2.x + d2.y)) * (mass * 1.0f / 3.0f);
				centerOfMass = PxVec3(0.0f);
			}
			break;

			case PxGeometryType::eCAPSULE:
			{
				const PxCapsuleGeometry& c = static_cast<const PxCapsuleGeometry&>(geometry);
				PxReal r = c.radius, h = c.halfHeight;
				mass = ((4.0f / 3.0f) * r + 2 * c.halfHeight) * PxPi * r * r;

				PxReal a = r*r*r * (8.0f / 15.0f) + h*r*r * (3.0f / 2.0f) + h*h*r * (4.0f / 3.0f) + h*h*h * (2.0f / 3.0f);
				PxReal b = r*r*r * (8.0f / 15.0f) + h*r*r;
				inertiaTensor = PxMat33::createDiagonal(PxVec3(b, a, a) * PxPi * r * r);
				centerOfMass = PxVec3(0.0f);
			}
			break;

			case PxGeometryType::eCONVEXMESH:
			{
				const PxConvexMeshGeometry& c = static_cast<const PxConvexMeshGeometry&>(geometry);
				PxVec3 unscaledCoM;
				PxMat33 unscaledInertiaTensorNonCOM; // inertia tensor of convex mesh in mesh local space
				PxMat33 unscaledInertiaTensorCOM;
				PxReal unscaledMass;
				c.convexMesh->getMassInformation(unscaledMass, unscaledInertiaTensorNonCOM, unscaledCoM);				

				// inertia tensor relative to center of mass
				unscaledInertiaTensorCOM[0][0] = unscaledInertiaTensorNonCOM[0][0] - unscaledMass*PxReal((unscaledCoM.y*unscaledCoM.y+unscaledCoM.z*unscaledCoM.z));
				unscaledInertiaTensorCOM[1][1] = unscaledInertiaTensorNonCOM[1][1] - unscaledMass*PxReal((unscaledCoM.z*unscaledCoM.z+unscaledCoM.x*unscaledCoM.x));
				unscaledInertiaTensorCOM[2][2] = unscaledInertiaTensorNonCOM[2][2] - unscaledMass*PxReal((unscaledCoM.x*unscaledCoM.x+unscaledCoM.y*unscaledCoM.y));
				unscaledInertiaTensorCOM[0][1] = unscaledInertiaTensorCOM[1][0] = (unscaledInertiaTensorNonCOM[0][1] + unscaledMass*PxReal(unscaledCoM.x*unscaledCoM.y));
				unscaledInertiaTensorCOM[1][2] = unscaledInertiaTensorCOM[2][1] = (unscaledInertiaTensorNonCOM[1][2] + unscaledMass*PxReal(unscaledCoM.y*unscaledCoM.z));
				unscaledInertiaTensorCOM[0][2] = unscaledInertiaTensorCOM[2][0] = (unscaledInertiaTensorNonCOM[0][2] + unscaledMass*PxReal(unscaledCoM.z*unscaledCoM.x));

				const PxMeshScale& s = c.scale;
				mass = unscaledMass * s.scale.x * s.scale.y * s.scale.z;
				centerOfMass = s.rotation.rotate(s.scale.multiply(s.rotation.rotateInv(unscaledCoM)));
				inertiaTensor = scaleInertia(unscaledInertiaTensorCOM, s.rotation, s.scale);
			}
			break;

			case PxGeometryType::eHEIGHTFIELD:
			case PxGeometryType::ePLANE:
			case PxGeometryType::eTRIANGLEMESH:
			case PxGeometryType::eINVALID:
			case PxGeometryType::eGEOMETRY_COUNT:
			{
				*this = PxMassProperties();
			}
			break;
		}

		PX_ASSERT(inertiaTensor.column0.isFinite() && inertiaTensor.column1.isFinite() && inertiaTensor.column2.isFinite());
		PX_ASSERT(centerOfMass.isFinite());
		PX_ASSERT(PxIsFinite(mass));
	}

	/**
	\brief Scale mass properties.

	\param[in] scale The linear scaling factor to apply to the mass properties.
	\return The scaled mass properties.
	*/
	PX_FORCE_INLINE PxMassProperties operator*(const PxReal scale) const
	{
		PX_ASSERT(PxIsFinite(scale));

		return PxMassProperties(mass * scale, inertiaTensor * scale, centerOfMass);
	}

	/**
	\brief Translate the center of mass by a given vector and adjust the inertia tensor accordingly.

	\param[in] t The translation vector for the center of mass.
	*/
	PX_FORCE_INLINE void translate(const PxVec3& t)
	{
		PX_ASSERT(t.isFinite());

		inertiaTensor = translateInertia(inertiaTensor, mass, t);
		centerOfMass += t;

		PX_ASSERT(inertiaTensor.column0.isFinite() && inertiaTensor.column1.isFinite() && inertiaTensor.column2.isFinite());
		PX_ASSERT(centerOfMass.isFinite());
	}

	/**
	\brief Get the entries of the diagonalized inertia tensor and the corresponding reference rotation.

	\param[in] inertia The inertia tensor to diagonalize.
	\param[out] massFrame The frame the diagonalized tensor refers to.
	\return The entries of the diagonalized inertia tensor.
	*/
	PX_FORCE_INLINE static PxVec3 getMassSpaceInertia(const PxMat33& inertia, PxQuat& massFrame)
	{
		PX_ASSERT(inertia.column0.isFinite() && inertia.column1.isFinite() && inertia.column2.isFinite());

		PxVec3 diagT = PxDiagonalize(inertia, massFrame);
		PX_ASSERT(diagT.isFinite());
		PX_ASSERT(massFrame.isFinite());
		return diagT;
	}

	/**
	\brief Translate an inertia tensor using the parallel axis theorem

	\param[in] inertia The inertia tensor to translate.
	\param[in] mass The mass of the object.
	\param[in] t The relative frame to translate the inertia tensor to.
	\return The translated inertia tensor.
	*/
	PX_FORCE_INLINE static PxMat33 translateInertia(const PxMat33& inertia, const PxReal mass, const PxVec3& t)
	{
		PX_ASSERT(inertia.column0.isFinite() && inertia.column1.isFinite() && inertia.column2.isFinite());
		PX_ASSERT(PxIsFinite(mass));
		PX_ASSERT(t.isFinite());

		PxMat33 s(	PxVec3(0,t.z,-t.y),
					PxVec3(-t.z,0,t.x),
					PxVec3(t.y,-t.x,0) );

		PxMat33 translatedIT = s.getTranspose() * s * mass + inertia;
		PX_ASSERT(translatedIT.column0.isFinite() && translatedIT.column1.isFinite() && translatedIT.column2.isFinite());
		return translatedIT;
	}

	/**
	\brief Rotate an inertia tensor around the center of mass

	\param[in] inertia The inertia tensor to rotate.
	\param[in] q The rotation to apply to the inertia tensor.
	\return The rotated inertia tensor.
	*/
	PX_FORCE_INLINE static PxMat33 rotateInertia(const PxMat33& inertia, const PxQuat& q)
	{
		PX_ASSERT(inertia.column0.isFinite() && inertia.column1.isFinite() && inertia.column2.isFinite());
		PX_ASSERT(q.isUnit());

		PxMat33 m(q);
		PxMat33 rotatedIT = m * inertia * m.getTranspose();
		PX_ASSERT(rotatedIT.column0.isFinite() && rotatedIT.column1.isFinite() && rotatedIT.column2.isFinite());
		return rotatedIT;
	}

	/**
	\brief Non-uniform scaling of the inertia tensor

	\param[in] inertia The inertia tensor to scale.
	\param[in] scaleRotation The frame of the provided scaling factors.
	\param[in] scale The scaling factor for each axis (relative to the frame specified in scaleRotation).
	\return The scaled inertia tensor.
	*/
	static PxMat33 scaleInertia(const PxMat33& inertia, const PxQuat& scaleRotation, const PxVec3& scale)
	{
		PX_ASSERT(inertia.column0.isFinite() && inertia.column1.isFinite() && inertia.column2.isFinite());
		PX_ASSERT(scaleRotation.isUnit());
		PX_ASSERT(scale.isFinite());

		PxMat33 localInertiaT = rotateInertia(inertia, scaleRotation); // rotate inertia into scaling frame
		PxVec3 diagonal(localInertiaT[0][0], localInertiaT[1][1], localInertiaT[2][2]);

		PxVec3 xyz2 = PxVec3(diagonal.dot(PxVec3(0.5f))) - diagonal; // original x^2, y^2, z^2
		PxVec3 scaledxyz2 = xyz2.multiply(scale).multiply(scale);

		PxReal	xx = scaledxyz2.y + scaledxyz2.z,
				yy = scaledxyz2.z + scaledxyz2.x,
				zz = scaledxyz2.x + scaledxyz2.y;

		PxReal	xy = localInertiaT[0][1] * scale.x * scale.y,
				xz = localInertiaT[0][2] * scale.x * scale.z,
				yz = localInertiaT[1][2] * scale.y * scale.z;

		PxMat33 scaledInertia(	PxVec3(xx, xy, xz),
								PxVec3(xy, yy, yz),
								PxVec3(xz, yz, zz));

		PxMat33 scaledIT = rotateInertia(scaledInertia * (scale.x * scale.y * scale.z), scaleRotation.getConjugate());
		PX_ASSERT(scaledIT.column0.isFinite() && scaledIT.column1.isFinite() && scaledIT.column2.isFinite());
		return scaledIT;
	}

	/**
	\brief Sum up individual mass properties.

	\param[in] props Array of mass properties to sum up.
	\param[in] transforms Reference transforms for each mass properties entry.
	\param[in] count The number of mass properties to sum up.
	\return The summed up mass properties.
	*/
	static PxMassProperties sum(const PxMassProperties* props, const PxTransform* transforms, const PxU32 count)
	{
		PxReal combinedMass = 0.0f;
		PxVec3 combinedCoM(0.0f);
		PxMat33 combinedInertiaT = PxMat33(PxZero);

		for(PxU32 i = 0; i < count; i++)
		{
			PX_ASSERT(props[i].inertiaTensor.column0.isFinite() && props[i].inertiaTensor.column1.isFinite() && props[i].inertiaTensor.column2.isFinite());
			PX_ASSERT(props[i].centerOfMass.isFinite());
			PX_ASSERT(PxIsFinite(props[i].mass));

			combinedMass += props[i].mass;
			const PxVec3 comTm = transforms[i].transform(props[i].centerOfMass);
			combinedCoM += comTm * props[i].mass;
		}

		combinedCoM /= combinedMass;

		for(PxU32 i = 0; i < count; i++)
		{
			const PxVec3 comTm = transforms[i].transform(props[i].centerOfMass);
			combinedInertiaT += translateInertia(rotateInertia(props[i].inertiaTensor, transforms[i].q), props[i].mass, combinedCoM - comTm);
		}

		PX_ASSERT(combinedInertiaT.column0.isFinite() && combinedInertiaT.column1.isFinite() && combinedInertiaT.column2.isFinite());
		PX_ASSERT(combinedCoM.isFinite());
		PX_ASSERT(PxIsFinite(combinedMass));

		return PxMassProperties(combinedMass, combinedInertiaT, combinedCoM);
	}


	PxMat33 inertiaTensor;			//!< The inertia tensor of the object.
	PxVec3  centerOfMass;			//!< The center of mass of the object.
	PxReal	mass;					//!< The mass of the object.
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
