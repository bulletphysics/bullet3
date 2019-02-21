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
#ifndef PX_XML_READER_H
#define PX_XML_READER_H

#include "foundation/PxSimpleTypes.h"
#include "extensions/PxRepXSimpleType.h"

namespace physx {
	namespace Sn { struct XmlNode; }
	
	/**
	 *	Reader used to read data out of the repx format.
	 */
	class XmlReader
	{
	protected:
		virtual ~XmlReader(){}
	public:
		/** Read a key-value pair out of the database */
		virtual bool read( const char* inName, const char*& outData ) = 0;
		/** Read an object id out of the database */
		virtual bool read( const char* inName, PxSerialObjectId& outId ) = 0;
		/** Goto a child element by name.  That child becomes this reader's context */
		virtual bool gotoChild( const char* inName ) = 0;
		/** Goto the first child regardless of name */
		virtual bool gotoFirstChild() = 0;
		/** Goto the next sibling regardless of name */
		virtual bool gotoNextSibling() = 0;
		/** Count all children of the current object */
		virtual PxU32 countChildren() = 0;
		/** Get the name of the current item */
		virtual const char* getCurrentItemName() = 0;
		/** Get the value of the current item */
		virtual const char* getCurrentItemValue() = 0;
		/** Leave the current child */
		virtual bool leaveChild() = 0;
		/** Get reader for the parental object */
		virtual XmlReader* getParentReader() = 0;

		/**
		 *	Ensures we don't leave the reader in an odd state
		 *	due to not leaving a given child
		 */
		virtual void pushCurrentContext() = 0;
		/** Pop the current context back to where it during push*/
		virtual void popCurrentContext() = 0; 
	};

	//Used when upgrading a repx collection
	class XmlReaderWriter : public XmlReader
	{
	public:
		//Clears the stack of nodes (push/pop current node reset)
		//and sets the current node to inNode.
		virtual void setNode( Sn::XmlNode& node ) = 0;
		//If the child exists, add it.
		//the either way goto that child.
		virtual void addOrGotoChild( const char* name ) = 0;
		//Value is copied into the collection, inValue has no further references
		//to it.
		virtual void setCurrentItemValue( const char* value ) = 0;
		//Removes the child but does not release the char* name or char* data ptrs.
		//Those pointers are never released and are shared among collections.
		//Thus copying nodes is cheap and safe.
		virtual bool removeChild( const char* name ) = 0;
		virtual void release() = 0;

		bool renameProperty( const char* oldName, const char* newName )
		{
			if ( gotoChild( oldName ) )
			{
				const char* value = getCurrentItemValue();
				leaveChild();
				removeChild( oldName );
				addOrGotoChild( newName );
				setCurrentItemValue( value );
				leaveChild();
				return true;
			}
			return false;
		}
		bool readAndRemoveProperty( const char* name, const char*& outValue )
		{
			bool retval = read( name, outValue ); 
			if ( retval ) removeChild( name );
			return retval;
		}

		bool writePropertyIfNotEmpty( const char* name, const char* value )
		{
			if ( value && *value )
			{ 
				addOrGotoChild( name ); 
				setCurrentItemValue( value ); 
				leaveChild(); 
				return true;
			}
			return false;
		}
	};

} 
#endif
