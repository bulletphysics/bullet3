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
#ifndef PX_SIMPLEXMLWRITER_H
#define PX_SIMPLEXMLWRITER_H

#include "PsArray.h"
#include "SnXmlMemoryPoolStreams.h"
#include "CmPhysXCommon.h"

namespace physx { namespace Sn {
	class XmlWriter
	{
	public:
		
		struct STagWatcher
		{
			typedef XmlWriter TXmlWriterType;
			TXmlWriterType& mWriter;
			STagWatcher( const STagWatcher& inOther );
			STagWatcher& operator-( const STagWatcher& inOther );
			STagWatcher( TXmlWriterType& inWriter, const char* inTagName )
				: mWriter( inWriter )
			{
				mWriter.beginTag( inTagName );
			}
			~STagWatcher() { mWriter.endTag(); }
		};

		virtual ~XmlWriter(){}
		virtual void beginTag( const char* inTagname ) = 0;
		virtual void endTag() = 0;
		virtual void addAttribute( const char* inName, const char* inValue ) = 0;
		virtual void writeContentTag( const char* inTag, const char* inContent ) = 0;
		virtual void addContent( const char* inContent ) = 0;
		virtual PxU32 tabCount() = 0;
	};

	template<typename TStreamType>
	class XmlWriterImpl : public XmlWriter
	{
		PxProfileAllocatorWrapper mWrapper;
		TStreamType& mStream;
		XmlWriterImpl( const XmlWriterImpl& inOther );
		XmlWriterImpl& operator=( const XmlWriterImpl& inOther );
		PxProfileArray<const char*>	mTags;
		bool								mTagOpen;
		PxU32							mInitialTagDepth;
	public:

		XmlWriterImpl( TStreamType& inStream, PxAllocatorCallback& inAllocator, PxU32 inInitialTagDepth = 0 )
			: mWrapper( inAllocator )
			, mStream( inStream )
			, mTags( mWrapper )
			, mTagOpen( false )
			, mInitialTagDepth( inInitialTagDepth )
		{
		}
		virtual ~XmlWriterImpl()
		{
			while( mTags.size() )
				endTag();
		}
		PxU32 tabCount() { return mTags.size() + mInitialTagDepth; }

		void writeTabs( PxU32 inSize )
		{
			inSize += mInitialTagDepth;
			for ( PxU32 idx =0; idx < inSize; ++idx )
				mStream << "\t";
		}
		void beginTag( const char* inTagname )
		{
			closeTag();
			writeTabs(mTags.size());
			mTags.pushBack( inTagname );
			mStream << "<" << inTagname;
			mTagOpen = true;
		}
		void addAttribute( const char* inName, const char* inValue )
		{
			PX_ASSERT( mTagOpen );
			mStream << " " << inName << "=" << "\"" << inValue << "\"";
		}
		void closeTag(bool useNewline = true)
		{
			if ( mTagOpen )
			{
				mStream << " " << ">";
				if (useNewline )
					mStream << "\n";
			}
			mTagOpen = false;
		}
		void doEndOpenTag()
		{
			mStream << "</" << mTags.back() << ">" << "\n";
		}
		void endTag()
		{
			PX_ASSERT( mTags.size() );
			if ( mTagOpen )
				mStream << " " << "/>" << "\n";
			else
			{
				writeTabs(mTags.size()-1);
				doEndOpenTag();
			}
			mTagOpen = false;
			mTags.popBack();
		}
		void addContent( const char* inContent )
		{
			closeTag(false);
			mStream << inContent;
		}
		void writeContentTag( const char* inTag, const char* inContent )
		{
			beginTag( inTag );
			addContent( inContent );
			doEndOpenTag();
			mTags.popBack();
		}
		void insertXml( const char* inXml )
		{
			closeTag();
			mStream << inXml;
		}
	};

	struct BeginTag
	{
		const char* mTagName;
		BeginTag( const char* inTagName )
			: mTagName( inTagName ) { }
	};

	struct EndTag
	{
		EndTag() {}
	};

	struct Att
	{
		const char* mAttName;
		const char* mAttValue;
		Att( const char* inAttName, const char* inAttValue )
			: mAttName( inAttName )
			, mAttValue( inAttValue ) {	}
	};

	struct Content
	{
		const char* mContent;
		Content( const char* inContent )
			: mContent( inContent ) { }
	};

	
	struct ContentTag
	{
		const char* mTagName;
		const char* mContent;
		ContentTag( const char* inTagName, const char* inContent )
			: mTagName( inTagName )
			, mContent( inContent ) { }
	};

	inline XmlWriter& operator<<( XmlWriter& inWriter, const BeginTag& inTag ) { inWriter.beginTag( inTag.mTagName ); return inWriter; }
	inline XmlWriter& operator<<( XmlWriter& inWriter, const EndTag& inTag ) { inWriter.endTag(); return inWriter; }
	inline XmlWriter& operator<<( XmlWriter& inWriter, const Att& inTag ) { inWriter.addAttribute(inTag.mAttName, inTag.mAttValue); return inWriter; }
	inline XmlWriter& operator<<( XmlWriter& inWriter, const Content& inTag ) { inWriter.addContent(inTag.mContent); return inWriter; }
	inline XmlWriter& operator<<( XmlWriter& inWriter, const ContentTag& inTag ) { inWriter.writeContentTag(inTag.mTagName, inTag.mContent); return inWriter; }

	inline void writeProperty( XmlWriter& inWriter, MemoryBuffer& tempBuffer, const char* inPropName )
	{
		PxU8 data = 0;
		tempBuffer.write( &data, sizeof(PxU8) );
		inWriter.writeContentTag( inPropName, reinterpret_cast<const char*>( tempBuffer.mBuffer ) );
		tempBuffer.clear();
	}

	template<typename TDataType>
	inline void writeProperty( XmlWriter& inWriter, MemoryBuffer& tempBuffer, const char* inPropName, TDataType inValue )
	{
		tempBuffer << inValue;
		writeProperty( inWriter, tempBuffer, inPropName );
	}
} }
#endif