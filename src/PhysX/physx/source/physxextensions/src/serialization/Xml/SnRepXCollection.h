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
#ifndef PX_REPXCOLLECTION_H
#define PX_REPXCOLLECTION_H

#include "common/PxTolerancesScale.h"
#include "PxRepXSerializer.h"

namespace physx { namespace Sn {
	
	struct	XmlNode;
	
	struct RepXCollectionItem
	{
		PxRepXObject		liveObject;
		XmlNode*			descriptor;
		RepXCollectionItem( PxRepXObject inItem = PxRepXObject(), XmlNode* inDescriptor = NULL )
			: liveObject( inItem )
			, descriptor( inDescriptor )
		{
		}
	};

	struct RepXDefaultEntry
	{
		const char* name;
		const char* value;
		RepXDefaultEntry( const char* pn, const char* val ) : name( pn ), value( val ){}
	};

	/**
	*	The result of adding an object to the collection.
	*/
	struct RepXAddToCollectionResult
	{
		enum Enum
		{
			Success,
			SerializerNotFound,
			InvalidParameters, //Null data passed in.
			AlreadyInCollection
		};

		PxSerialObjectId	collectionId;
		Enum				result;

		RepXAddToCollectionResult( Enum inResult = Success, const PxSerialObjectId inId = 0 )
			: collectionId( inId )
			, result( inResult )
		{
		}
		bool isValid() { return result == Success && collectionId != 0; }
	};
	/**
	*	A RepX collection contains a set of static data objects that can be transformed
	*	into live objects.  It uses RepX serializer to do two transformations:
	*	live object <-> collection object (descriptor)
	*	collection object <-> file system.
	*
	*	A live object is considered to be something live in the physics
	*	world such as a material or a rigidstatic.
	*
	*	A collection object is a piece of data from which a live object
	*	of identical characteristics can be created.  
	*
	*	Clients need to pass PxCollection so that objects can resolve
	*	references.  In addition, objects must be added in an order such that
	*	references can be resolved in the first place.  So objects must be added
	*	to the collection *after* objects they are dependent upon.
	*
	*	When deserializing from a file, the collection will allocate char*'s that will
	*	not be freed when the collection itself is freed.  The user must be responsible
	*	for these character allocations.
	*/
	class RepXCollection 
	{
	protected:
		virtual ~RepXCollection(){}

	public:
		virtual void destroy() = 0;

		/**
		*	Set the scale on this collection.  The scale is saved with the collection.
		*
		*	If the scale wasn't set, it will be invalid.
		*/
		virtual void  setTolerancesScale( const PxTolerancesScale& inScale ) = 0;

		/**
		*	Get the scale that was set at collection creation time or at load time.
		*	If this is a loaded file and the source data does not contain a scale
		*	this value will be invalid (PxTolerancesScale::isValid()).
		*/
		virtual PxTolerancesScale getTolerancesScale() const = 0;

		/**
		*	Set the up vector on this collection.  The up vector is saved with the collection.
		*
		*	If the up vector wasn't set, it will be (0,0,0).
		*/
		virtual void  setUpVector( const PxVec3& inUpVector ) = 0;

		/**
		* If the up vector wasn't set, it will be (0,0,0).  Else this will be the up vector
		* optionally set when the collection was created.
		*/
		virtual PxVec3	getUpVector() const = 0;

		virtual const char* getVersion() = 0;
		static const char* getLatestVersion();

		//Necessary accessor functions for translation/upgrading.
		virtual const RepXCollectionItem* begin() const = 0;
		virtual const RepXCollectionItem* end() const = 0;


		//Performs a deep copy of the repx node.
		virtual XmlNode* copyRepXNode( const XmlNode* srcNode ) = 0;

		virtual void addCollectionItem( RepXCollectionItem inItem ) = 0;

		//Create a new repx node with this name.  Its value is unset.
		virtual XmlNode& createRepXNode( const char* name ) = 0;

		virtual RepXCollection& createCollection( const char* inVersionStr ) = 0;
		//Release this when finished.
		virtual XmlReaderWriter& createNodeEditor() = 0;

		virtual PxAllocatorCallback& getAllocator() = 0;

		virtual bool instantiateCollection( PxRepXInstantiationArgs& inArgs, PxCollection& inPxCollection ) = 0;

		
		virtual RepXAddToCollectionResult addRepXObjectToCollection( const PxRepXObject& inObject, PxCollection* inCollection, PxRepXInstantiationArgs& inArgs ) = 0;

		/**
		 *	Save this collection out to a file stream.  Uses the RepX serialize to perform 
		 *	collection object->file conversions.
		 *
		 *	/param[in] inStream Write-only stream to save collection out to.
		 */
		virtual void save( PxOutputStream& inStream ) = 0;
	};
} }

#endif
