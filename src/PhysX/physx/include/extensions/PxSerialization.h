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
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#ifndef PX_SERIALIZATION_H
#define PX_SERIALIZATION_H
/** \addtogroup extensions
  @{
*/

#include "PxPhysXConfig.h"
#include "common/PxBase.h"
#include "cooking/PxCooking.h"
#include "foundation/PxIO.h"
#include "common/PxTolerancesScale.h"
#include "common/PxTypeInfo.h"
#include "common/PxStringTable.h"

//
// Important: if you adjust the comment about binary compatible versions below, don't forget to adjust the compatibility list in
// sBinaryCompatibleVersionsbinary as well
//
/**
PX_BINARY_SERIAL_VERSION is used to specify the binary data format compatibility additionally to the physics sdk version. 
The binary format version is defined as "PX_PHYSICS_VERSION_MAJOR.PX_PHYSICS_VERSION_MINOR.PX_PHYSICS_VERSION_BUGFIX-PX_BINARY_SERIAL_VERSION".
The following binary format versions are compatible with the current physics version:
  (no compatible versions)

The PX_BINARY_SERIAL_VERSION for a given PhysX release is typically 0. If incompatible modifications are made to a customer specific branch the
number should be increased.
*/
#define PX_BINARY_SERIAL_VERSION 0


#if !PX_DOXYGEN
namespace physx
{
#endif

	class PxBinaryConverter;

/**
\brief Utility functions for serialization

@see PxCollection, PxSerializationRegistry
*/
class PxSerialization
{
public:
	/**
	\brief Additional PxScene and PxPhysics options stored in XML serialized data.

	The PxXmlMiscParameter parameter can be serialized and deserialized along with PxCollection instances (XML only).
	This is for application use only and has no impact on how objects are serialized or deserialized. 
	@see PxSerialization::createCollectionFromXml, PxSerialization::serializeCollectionToXml
	*/
	struct PxXmlMiscParameter
	{
		/**
		\brief Up vector for the scene reference coordinate system.
		*/
		PxVec3				upVector;

		/**
		\brief Tolerances scale to be used for the scene.
		*/
		PxTolerancesScale	scale;
		
		PxXmlMiscParameter() : upVector(0) {}
		PxXmlMiscParameter(PxVec3& inUpVector, PxTolerancesScale inScale) : upVector(inUpVector), scale(inScale) {}
	};

	/**
	\brief Returns whether the collection is serializable with the externalReferences collection.

	Some definitions to explain whether a collection can be serialized or not:

	For definitions of <b>requires</b> and <b>complete</b> see #PxSerialization::complete

	A serializable object is <b>subordinate</b> if it cannot be serialized on its own
	The following objects are subordinate:
	- articulation links
	- articulation joints
	- joints

	A collection C can be serialized with external references collection D iff
	- C is complete relative to D (no dangling references)
	- Every object in D required by an object in C has a valid ID (no unnamed references)
	- Every subordinate object in C is required by another object in C (no orphans)

	\param[in] collection Collection to be checked
	\param[in] sr PxSerializationRegistry instance with information about registered classes.
	\param[in] externalReferences the external References collection
	\return  Whether the collection is serializable
	@see PxSerialization::complete, PxSerialization::serializeCollectionToBinary, PxSerialization::serializeCollectionToXml, PxSerializationRegistry
	*/
	static	bool			isSerializable(PxCollection& collection, PxSerializationRegistry& sr, const PxCollection* externalReferences = NULL);

	/**
	\brief Adds to a collection all objects such that it can be successfully serialized.
	
	A collection C is complete relative to an other collection D if every object required by C is either in C or D.
	This function adds objects to a collection, such that it becomes complete with respect to the exceptFor collection.
	Completeness is needed for serialization. See #PxSerialization::serializeCollectionToBinary, 
	#PxSerialization::serializeCollectionToXml.

	Sdk objects require other sdk object according to the following rules: 
	 - joints require their actors and constraint
	 - rigid actors require their shapes
	 - shapes require their material(s) and mesh (triangle mesh, convex mesh or height field), if any
	 - articulations require their links and joints
	 - aggregates require their actors

	If followJoints is specified another rule is added:
	 - actors require their joints
	
	Specifying followJoints will make whole jointed actor chains being added to the collection. Following chains 
	is interrupted whenever a object in exceptFor is encountered.

	\param[in,out] collection Collection which is completed
	\param[in] sr PxSerializationRegistry instance with information about registered classes.
	\param[in] exceptFor Optional exemption collection
	\param[in] followJoints Specifies whether joints should be added for jointed actors
	@see PxCollection, PxSerialization::serializeCollectionToBinary, PxSerialization::serializeCollectionToXml, PxSerializationRegistry
	*/
	static	void			complete(PxCollection& collection, PxSerializationRegistry& sr, const PxCollection* exceptFor = NULL, bool followJoints = false);
	
	/**
	\brief Creates PxSerialObjectId values for unnamed objects in a collection.

	Creates PxSerialObjectId names for unnamed objects in a collection starting at a base value and incrementing, 
	skipping values that are already assigned to objects in the collection.

	\param[in,out] collection Collection for which names are created
	\param[in] base Start address for PxSerialObjectId names
	@see PxCollection
	*/
	static	void			createSerialObjectIds(PxCollection& collection, const PxSerialObjectId base);
			
	/**
	\brief Creates a PxCollection from XML data.

	\param inputData The input data containing the XML collection.
	\param cooking PxCooking instance used for sdk object instantiation.
	\param sr PxSerializationRegistry instance with information about registered classes.
	\param externalRefs PxCollection used to resolve external references.
	\param stringTable PxStringTable instance used for storing object names.
	\param outArgs Optional parameters of physics and scene deserialized from XML. See #PxSerialization::PxXmlMiscParameter
	\return a pointer to a PxCollection if successful or NULL if it failed.

	@see PxCollection, PxSerializationRegistry, PxInputData, PxStringTable, PxCooking, PxSerialization::PxXmlMiscParameter
	*/
	static	PxCollection*	createCollectionFromXml(PxInputData& inputData, PxCooking& cooking, PxSerializationRegistry& sr, const PxCollection* externalRefs = NULL, PxStringTable* stringTable = NULL, PxXmlMiscParameter* outArgs = NULL);
	
	/**
	\brief Deserializes a PxCollection from memory.

	Creates a collection from memory. If the collection has external dependencies another collection 
	can be provided to resolve these.

	The memory block provided has to be 128 bytes aligned and contain a contiguous serialized collection as written
	by PxSerialization::serializeCollectionToBinary. The contained binary data needs to be compatible with the current binary format version
	which is defined by "PX_PHYSICS_VERSION_MAJOR.PX_PHYSICS_VERSION_MINOR.PX_PHYSICS_VERSION_BUGFIX-PX_BINARY_SERIAL_VERSION".
	For a list of compatible sdk releases refer to the documentation of PX_BINARY_SERIAL_VERSION.

	\param[in] memBlock Pointer to memory block containing the serialized collection
	\param[in] sr PxSerializationRegistry instance with information about registered classes.
	\param[in] externalRefs Collection to resolve external dependencies

	@see PxCollection, PxSerialization::complete, PxSerialization::serializeCollectionToBinary, PxSerializationRegistry, PX_BINARY_SERIAL_VERSION
	*/
	static	PxCollection*	createCollectionFromBinary(void* memBlock, PxSerializationRegistry& sr, const PxCollection* externalRefs = NULL);

	/**
	\brief Serializes a physics collection to an XML output stream.

	The collection to be serialized needs to be complete @see PxSerialization.complete.
	Optionally the XML may contain meshes in binary cooked format for fast loading. It does this when providing a valid non-null PxCooking pointer.

	\note Serialization of objects in a scene that is simultaneously being simulated is not supported and leads to undefined behavior. 

	\param outputStream Stream to save collection to.
	\param collection PxCollection instance which is serialized. The collection needs to be complete with respect to the externalRefs collection.
	\param sr PxSerializationRegistry instance with information about registered classes.
	\param cooking Optional pointer to cooking instance. If provided, cooked mesh data is cached for fast loading.
	\param externalRefs Collection containing external references.
	\param inArgs Optional parameters of physics and scene serialized to XML along with the collection. See #PxSerialization::PxXmlMiscParameter
	\return true if the collection is successfully serialized.

	@see PxCollection, PxOutputStream, PxSerializationRegistry, PxCooking, PxSerialization::PxXmlMiscParameter
	*/
	static	bool			serializeCollectionToXml(PxOutputStream& outputStream, PxCollection& collection,  PxSerializationRegistry& sr, PxCooking* cooking = NULL, const PxCollection* externalRefs = NULL, PxXmlMiscParameter* inArgs = NULL);
	
	/**
	\brief Serializes a collection to a binary stream.

	Serializes a collection to a stream. In order to resolve external dependencies the externalReferences collection has to be provided. 
	Optionally names of objects that where set for example with #PxActor::setName are serialized along with the objects.

	The collection can be successfully serialized if isSerializable(collection) returns true. See #isSerializable.

	The implementation of the output stream needs to fulfill the requirements on the memory block input taken by
	PxSerialization::createCollectionFromBinary.

	\note Serialization of objects in a scene that is simultaneously being simulated is not supported and leads to undefined behavior. 

	\param[out] outputStream into which the collection is serialized
	\param[in] collection Collection to be serialized
	\param[in] sr PxSerializationRegistry instance with information about registered classes.
	\param[in] externalRefs Collection used to resolve external dependencies
	\param[in] exportNames Specifies whether object names are serialized
	\return Whether serialization was successful

	@see PxCollection, PxOutputStream, PxSerialization::complete, PxSerialization::createCollectionFromBinary, PxSerializationRegistry
	*/
	static	bool			serializeCollectionToBinary(PxOutputStream& outputStream, PxCollection& collection, PxSerializationRegistry& sr, const PxCollection* externalRefs = NULL, bool exportNames = false );

	/** 
	\brief Dumps the binary meta-data to a stream.

	A meta-data file contains information about the SDK's internal classes and about custom user types ready 
	for serialization. Such a file is needed to convert binary-serialized data from one platform to another (re-targeting). 
	The converter needs meta-data files for the source and target platforms to perform conversions.

	Custom user types can be supported with PxSerializationRegistry::registerBinaryMetaDataCallback (see the guide for more information). 
	
	\param[out] outputStream Stream to write meta data to	
	\param[in] sr PxSerializationRegistry instance with information about registered classes used for conversion.
	
	@see PxOutputStream, PxSerializationRegistry
	*/
	static	void			dumpBinaryMetaData(PxOutputStream& outputStream, PxSerializationRegistry& sr);

	/**
	\brief Creates binary converter for re-targeting binary-serialized data.
	
	\return Binary converter instance.
	*/
	static PxBinaryConverter* createBinaryConverter();

	/**
	\brief Creates an application managed registry for serialization.
	
	\param[in] physics Physics SDK to generate create serialization registry
	
	\return PxSerializationRegistry instance.

	@see PxSerializationRegistry
	*/
	static PxSerializationRegistry* createSerializationRegistry(PxPhysics& physics);	
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
