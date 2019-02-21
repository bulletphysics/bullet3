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

#include "foundation/PxMemory.h"
#include "CmPhysXCommon.h"
#include "SnXmlImpl.h" 
#include "SnXmlReader.h"
#include "SnXmlMemoryAllocator.h"
#include "PsFoundation.h"
#include "SnRepXCollection.h"
#include "SnRepXUpgrader.h"

using namespace physx::profile;

namespace physx { namespace Sn {

	#define DEFINE_REPX_DEFAULT_PROPERTY( name, val ) RepXDefaultEntry( name, val ),

	static RepXDefaultEntry gRepX1_0Defaults[] = {
	#include "SnRepX1_0Defaults.h"
	};
	static PxU32 gNumRepX1_0Default = sizeof( gRepX1_0Defaults ) / sizeof ( *gRepX1_0Defaults );

	static RepXDefaultEntry gRepX3_1Defaults[] = {
	#include "SnRepX3_1Defaults.h"
	};
	static PxU32 gNumRepX3_1Defaults = sizeof( gRepX3_1Defaults ) / sizeof ( *gRepX3_1Defaults );

	static RepXDefaultEntry gRepX3_2Defaults[] = {
	#include "SnRepX3_2Defaults.h"
	};
	static PxU32 gNumRepX3_2Defaults = sizeof( gRepX3_2Defaults ) / sizeof ( *gRepX3_2Defaults );
	
	inline const char* nextPeriod( const char* str )
	{
		for( ++str; str && *str && *str != '.'; ++str ); //empty loop intentional
		return str;
	}

	inline bool safeStrEq(const char* lhs, const char* rhs)
	{
		if (lhs == rhs)
			return true;
		//If they aren't equal, and one of them is null,
		//then they can't be equal.
		//This is assuming that the null char* is not equal to
		//the empty "" char*.
		if (!lhs || !rhs)
			return false;

		return ::strcmp(lhs, rhs) == 0;
	}
	
	typedef PxProfileHashMap<const char*, PxU32> TNameOffsetMap;
	
	void setMissingPropertiesToDefault( XmlNode* topNode, XmlReaderWriter& editor, const RepXDefaultEntry* defaults, PxU32 numDefaults, TNameOffsetMap& map )
	{
		for ( XmlNode* child = topNode->mFirstChild; child != NULL; child = child->mNextSibling )
			setMissingPropertiesToDefault( child, editor, defaults, numDefaults, map );

		const TNameOffsetMap::Entry* entry( map.find( topNode->mName ) );
		if ( entry )
		{
			XmlReaderWriter& theReader( editor );
			theReader.setNode( *topNode );
			char nameBuffer[512] = {0};
			size_t nameLen = strlen( topNode->mName );
			//For each default property entry for this node type.
			for ( const RepXDefaultEntry* item = defaults + entry->second; strncmp( item->name, topNode->mName, nameLen ) == 0; ++item )
			{
				bool childAdded = false;
				const char* nameStart = item->name + nameLen;
				++nameStart;
				theReader.pushCurrentContext();
				const char* str = nameStart;
				while( *str )
				{
					 const char *period = nextPeriod( str );
					 size_t len = size_t(PxMin( period - str, ptrdiff_t(1023) )); //can't be too careful these days.
					 PxMemCopy( nameBuffer, str, PxU32(len) );
					 nameBuffer[len] = 0;
					 if ( theReader.gotoChild( nameBuffer ) == false )
					 {
						 childAdded = true;
						 theReader.addOrGotoChild( nameBuffer );
					 }
					 if (*period )
						 str = period + 1;
					 else
						 str = period;
				}
				if ( childAdded )
					theReader.setCurrentItemValue( item->value );
				theReader.popCurrentContext();
			}
		}
	}

	
	static void setMissingPropertiesToDefault( RepXCollection& collection, XmlReaderWriter& editor, const RepXDefaultEntry* defaults, PxU32 numDefaults )
	{
		PxProfileAllocatorWrapper wrapper( collection.getAllocator() );
		//Release all strings at once, instead of piece by piece
		XmlMemoryAllocatorImpl alloc( collection.getAllocator() );
		//build a hashtable of the initial default value strings.
		TNameOffsetMap nameOffsets( wrapper );
		for ( PxU32 idx = 0; idx < numDefaults; ++idx )
		{
			const RepXDefaultEntry& item( defaults[idx] );
			size_t nameLen = 0;
			const char* periodPtr = nextPeriod (item.name);
			for ( ; periodPtr && *periodPtr; ++periodPtr ) if( *periodPtr == '.' )	break;
			if ( periodPtr == NULL || *periodPtr != '.' ) continue;
			nameLen = size_t(periodPtr - item.name);
			char* newMem = reinterpret_cast<char*>(alloc.allocate( PxU32(nameLen + 1) ));
			PxMemCopy( newMem, item.name, PxU32(nameLen) );
			newMem[nameLen] = 0;
		
			if ( nameOffsets.find( newMem ) )
				alloc.deallocate( reinterpret_cast<PxU8*>(newMem) );
			else
				nameOffsets.insert( newMem, idx );
		}
		//Run through each collection item, and recursively find it and its children
		//If an object's name is in the hash map, check and add any properties that don't exist.
		//else return.
		for ( const RepXCollectionItem* item = collection.begin(), *end = collection.end(); item != end; ++ item )
		{
			RepXCollectionItem theItem( *item );
			setMissingPropertiesToDefault( theItem.descriptor, editor, defaults, numDefaults, nameOffsets );
		}
	}

	struct RecursiveTraversal
	{
		RecursiveTraversal(XmlReaderWriter& editor): mEditor(editor) {}
		void traverse()
		{
			mEditor.pushCurrentContext();
			updateNode();
			for(bool exists = mEditor.gotoFirstChild(); exists; exists = mEditor.gotoNextSibling())
				traverse();
			mEditor.popCurrentContext();
		}
		virtual void updateNode() = 0;
        virtual ~RecursiveTraversal() {}
		XmlReaderWriter& mEditor;
	protected:
		RecursiveTraversal& operator=(const RecursiveTraversal&){return *this;}
	};


	RepXCollection& RepXUpgrader::upgrade10CollectionTo3_1Collection(RepXCollection& src)
	{
		XmlReaderWriter& editor( src.createNodeEditor() );
		setMissingPropertiesToDefault(src, editor, gRepX1_0Defaults, gNumRepX1_0Default );

		
		RepXCollection* dest = &src.createCollection("3.1.1");
		
		for ( const RepXCollectionItem* item = src.begin(), *end = src.end(); item != end; ++ item )
		{
			//either src or dest could do the copy operation, it doesn't matter who does it.
			RepXCollectionItem newItem( item->liveObject, src.copyRepXNode( item->descriptor ) );
			editor.setNode( *const_cast<XmlNode*>( newItem.descriptor ) );
			//Some old files have this name in their system.
			editor.renameProperty( "MassSpaceInertia", "MassSpaceInertiaTensor" );
			editor.renameProperty( "SleepEnergyThreshold", "SleepThreshold" );

			if ( strstr( newItem.liveObject.typeName, "Joint" ) || strstr( newItem.liveObject.typeName, "joint" ) )
			{
				//Joints changed format a bit.  old joints looked like:
				/*
				<Actor0 >1627536</Actor0>
				<Actor1 >1628368</Actor1>
				<LocalPose0 >0 0 0 1 0.5 0.5 0.5</LocalPose0>
				<LocalPose1 >0 0 0 1 0.3 0.3 0.3</LocalPose1>*/
				//New joints look like:
				/*
				<Actors >
					<actor0 >58320336</actor0>
					<actor1 >56353568</actor1>
				</Actors>
				<LocalPose >
					<eACTOR0 >0 0 0 1 0.5 0.5 0.5</eACTOR0>
					<eACTOR1 >0 0 0 1 0.3 0.3 0.3</eACTOR1>
				</LocalPose>
				*/
				const char* actor0, *actor1, *lp0, *lp1;
				editor.readAndRemoveProperty( "Actor0", actor0 );
				editor.readAndRemoveProperty( "Actor1", actor1 ); 
				editor.readAndRemoveProperty( "LocalPose0", lp0 );
				editor.readAndRemoveProperty( "LocalPose1", lp1 );

				editor.addOrGotoChild( "Actors" );
				editor.writePropertyIfNotEmpty( "actor0", actor0 );
				editor.writePropertyIfNotEmpty( "actor1", actor1 );
				editor.leaveChild();

				editor.addOrGotoChild( "LocalPose" );
				editor.writePropertyIfNotEmpty( "eACTOR0", lp0 );
				editor.writePropertyIfNotEmpty( "eACTOR1", lp1 );
				editor.leaveChild();
			}



			//now desc owns the new node.  Collections share a single allocation pool, however,
			//which will get destroyed when all the collections referencing it are destroyed themselves.
			//Data on nodes is shared between nodes, but the node structure itself is allocated.
			dest->addCollectionItem( newItem );
		}
		editor.release();
		src.destroy();
		return *dest;
	}
	
	RepXCollection& RepXUpgrader::upgrade3_1CollectionTo3_2Collection(RepXCollection& src)
	{
		XmlReaderWriter& editor( src.createNodeEditor() );
		setMissingPropertiesToDefault(src, editor, gRepX3_1Defaults, gNumRepX3_1Defaults );

		RepXCollection* dest = &src.createCollection("3.2.0");

		for ( const RepXCollectionItem* item = src.begin(), *end = src.end(); item != end; ++ item )
		{
			//either src or dest could do the copy operation, it doesn't matter who does it.
			RepXCollectionItem newItem( item->liveObject, src.copyRepXNode( item->descriptor ) );
			editor.setNode( *const_cast<XmlNode*>( newItem.descriptor ) );
			
			if ( strstr( newItem.liveObject.typeName, "PxMaterial" ) )
			{
				editor.removeChild( "DynamicFrictionV" );
				editor.removeChild( "StaticFrictionV" );
				editor.removeChild( "dirOfAnisotropy" );	
			}
			//now desc owns the new node.  Collections share a single allocation pool, however,
			//which will get destroyed when all the collections referencing it are destroyed themselves.
			//Data on nodes is shared between nodes, but the node structure itself is allocated.
			dest->addCollectionItem( newItem );
		}
		editor.release();
		src.destroy();
		return *dest;
	}
	
	RepXCollection& RepXUpgrader::upgrade3_2CollectionTo3_3Collection(RepXCollection& src)
	{
		XmlReaderWriter& editor( src.createNodeEditor() );
		setMissingPropertiesToDefault(src, editor, gRepX3_2Defaults, gNumRepX3_2Defaults );

		RepXCollection* dest = &src.createCollection("3.3.0");

		

		struct RenameSpringToStiffness : public RecursiveTraversal
		{
			RenameSpringToStiffness(XmlReaderWriter& editor_): RecursiveTraversal(editor_) {}
		
			void updateNode()
			{
				mEditor.renameProperty("Spring", "Stiffness");
				mEditor.renameProperty("TangentialSpring", "TangentialStiffness");
			}
		};


		struct UpdateArticulationSwingLimit : public RecursiveTraversal
		{
			UpdateArticulationSwingLimit(XmlReaderWriter& editor_): RecursiveTraversal(editor_) {}
		
			void updateNode()
			{
				if(!Ps::stricmp(mEditor.getCurrentItemName(), "yLimit") && !Ps::stricmp(mEditor.getCurrentItemValue(), "0"))
					mEditor.setCurrentItemValue("0.785398");

				if(!Ps::stricmp(mEditor.getCurrentItemName(), "zLimit") && !Ps::stricmp(mEditor.getCurrentItemValue(), "0"))
					mEditor.setCurrentItemValue("0.785398");

				if(!Ps::stricmp(mEditor.getCurrentItemName(), "TwistLimit"))
				{
					mEditor.gotoFirstChild();
					PxReal lower = PxReal(strtod(mEditor.getCurrentItemValue(), NULL));
					mEditor.gotoNextSibling();
					PxReal upper = PxReal(strtod(mEditor.getCurrentItemValue(), NULL));
					mEditor.leaveChild();
					if(lower>=upper)
					{
						mEditor.writePropertyIfNotEmpty("lower", "-0.785398");
						mEditor.writePropertyIfNotEmpty("upper", "0.785398");
					}
				}
			}
		};


		for ( const RepXCollectionItem* item = src.begin(), *end = src.end(); item != end; ++ item )
		{
			//either src or dest could do the copy operation, it doesn't matter who does it.
			RepXCollectionItem newItem( item->liveObject, src.copyRepXNode( item->descriptor ) );	

			if ( strstr( newItem.liveObject.typeName, "PxCloth" ) || strstr( newItem.liveObject.typeName, "PxClothFabric" ) )
			{  
				physx::shdfnd::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "Didn't suppot PxCloth upgrate from 3.2 to 3.3! ");
				continue;
			}

			if ( strstr( newItem.liveObject.typeName, "PxParticleSystem" ) || strstr( newItem.liveObject.typeName, "PxParticleFluid" ) )
			{
				editor.setNode( *const_cast<XmlNode*>( newItem.descriptor ) );
				editor.renameProperty( "PositionBuffer", "Positions" );
				editor.renameProperty( "VelocityBuffer", "Velocities" );
				editor.renameProperty( "RestOffsetBuffer", "RestOffsets" );
			}
			
			if(strstr(newItem.liveObject.typeName, "PxPrismaticJoint" ) 
			|| strstr(newItem.liveObject.typeName, "PxRevoluteJoint")
			|| strstr(newItem.liveObject.typeName, "PxSphericalJoint")
			|| strstr(newItem.liveObject.typeName, "PxD6Joint")
			|| strstr(newItem.liveObject.typeName, "PxArticulation"))
			{
				editor.setNode( *const_cast<XmlNode*>( newItem.descriptor ) );
				RenameSpringToStiffness(editor).traverse();
			}

			if(strstr(newItem.liveObject.typeName, "PxArticulation"))
			{
				editor.setNode( *const_cast<XmlNode*>( newItem.descriptor ) );
				UpdateArticulationSwingLimit(editor).traverse();
			}



			//now dest owns the new node.  Collections share a single allocation pool, however,
			//which will get destroyed when all the collections referencing it are destroyed themselves.
			//Data on nodes is shared between nodes, but the node structure itself is allocated.
			
			dest->addCollectionItem( newItem );
			
		}
		editor.release();
		src.destroy();
		
		return *dest;
	}

	RepXCollection& RepXUpgrader::upgrade3_3CollectionTo3_4Collection(RepXCollection& src)
	{
		RepXCollection* dest = &src.createCollection("3.4.0");

		for ( const RepXCollectionItem* item = src.begin(), *end = src.end(); item != end; ++ item )
		{
			if(strstr(item->liveObject.typeName, "PxTriangleMesh"))
			{
				PxRepXObject newMeshRepXObj("PxBVH33TriangleMesh", item->liveObject.serializable, item->liveObject.id);                      
				XmlNode* newMeshNode = src.copyRepXNode( item->descriptor );
				newMeshNode->mName = "PxBVH33TriangleMesh";
				RepXCollectionItem newMeshItem(newMeshRepXObj, newMeshNode);
				dest->addCollectionItem( newMeshItem );
				continue;
			}

			RepXCollectionItem newItem( item->liveObject, src.copyRepXNode( item->descriptor ) );	
			dest->addCollectionItem( newItem );
		}
		src.destroy();		
		return *dest;
	}

	RepXCollection& RepXUpgrader::upgrade3_4CollectionTo4_0Collection(RepXCollection& src)
	{
		RepXCollection* dest = &src.createCollection("4.0.0");

		for (const RepXCollectionItem* item = src.begin(), *end = src.end(); item != end; ++item)
		{
			if (strstr(item->liveObject.typeName, "PxParticleFluid") || 
				strstr(item->liveObject.typeName, "PxParticleSystem") ||
				strstr(item->liveObject.typeName, "PxClothFabric") ||
				strstr(item->liveObject.typeName, "PxCloth"))
			{
				continue;
			}

			RepXCollectionItem newItem(item->liveObject, src.copyRepXNode(item->descriptor));
			dest->addCollectionItem(newItem);
		}
		src.destroy();
		return *dest;
	}

	RepXCollection& RepXUpgrader::upgradeCollection(RepXCollection& src)
	{
		const char* srcVersion = src.getVersion();
		if( safeStrEq( srcVersion, RepXCollection::getLatestVersion() ))
           return src;		

		typedef RepXCollection& (*UPGRADE_FUNCTION)(RepXCollection& src);

		struct Upgrade { const char* versionString; UPGRADE_FUNCTION upgradeFunction; };
		
		static const Upgrade upgradeTable[] =
		{
			{   "1.0", upgrade10CollectionTo3_1Collection	},
			{   "3.1", NULL									},
			{ "3.1.1", upgrade3_1CollectionTo3_2Collection	},
			{ "3.2.0", upgrade3_2CollectionTo3_3Collection	},
			{ "3.3.0", NULL									},
			{ "3.3.1", NULL									},
			{ "3.3.2", NULL									},
			{ "3.3.3", NULL									},
			{ "3.3.4", upgrade3_3CollectionTo3_4Collection	},
			{ "3.4.0", NULL									},
			{ "3.4.1", NULL									},
			{ "3.4.2", upgrade3_4CollectionTo4_0Collection	}
		}; //increasing order and complete
		
		const PxU32 upgradeTableSize = sizeof(upgradeTable)/sizeof(upgradeTable[0]);

		PxU32 repxVersion = UINT16_MAX;
		
		for (PxU32 i=0; i<upgradeTableSize; i++)
	    {
			if( safeStrEq( srcVersion, upgradeTable[i].versionString ))
			{
				repxVersion = i;
				break;
			}
		}

		RepXCollection* dest = &src;
		for( PxU32 j = repxVersion; j < upgradeTableSize; j++ )
		{
			if( upgradeTable[j].upgradeFunction )
				dest = &(upgradeTable[j].upgradeFunction)(*dest);
		}

		return *dest;
	}
} }
