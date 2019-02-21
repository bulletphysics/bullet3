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

#ifndef PXPVDSDK_PXPVDUSERRENDERIMPL_H
#define PXPVDSDK_PXPVDUSERRENDERIMPL_H

#include "PxPvdUserRenderer.h"

namespace physx
{
namespace pvdsdk
{

struct PvdUserRenderTypes
{
	enum Enum
	{
		Unknown = 0,
#define DECLARE_PVD_IMMEDIATE_RENDER_TYPE(type) type,
#define DECLARE_PVD_IMMEDIATE_RENDER_TYPE_NO_COMMA(type) type
#include "PxPvdUserRenderTypes.h"
#undef DECLARE_PVD_IMMEDIATE_RENDER_TYPE_NO_COMMA
#undef DECLARE_PVD_IMMEDIATE_RENDER_TYPE
	};
};

class RenderSerializer
{
  protected:
	virtual ~RenderSerializer()
	{
	}

  public:
	virtual void streamify(uint64_t& val) = 0;
	virtual void streamify(float& val) = 0;
	virtual void streamify(uint32_t& val) = 0;
	virtual void streamify(uint8_t& val) = 0;
	virtual void streamify(DataRef<uint8_t>& val) = 0;
	virtual void streamify(DataRef<PvdDebugPoint>& val) = 0;
	virtual void streamify(DataRef<PvdDebugLine>& val) = 0;
	virtual void streamify(DataRef<PvdDebugTriangle>& val) = 0;
	virtual void streamify(PvdDebugText& val) = 0;
	virtual bool isGood() = 0;
	virtual uint32_t hasData() = 0;

	void streamify(PvdUserRenderTypes::Enum& val)
	{
		uint8_t data = static_cast<uint8_t>(val);
		streamify(data);
		val = static_cast<PvdUserRenderTypes::Enum>(data);
	}
	void streamify(PxVec3& val)
	{
		streamify(val[0]);
		streamify(val[1]);
		streamify(val[2]);
	}

	void streamify(PvdColor& val)
	{
		streamify(val.r);
		streamify(val.g);
		streamify(val.b);
		streamify(val.a);
	}
	void streamify(PxTransform& val)
	{
		streamify(val.q.x);
		streamify(val.q.y);
		streamify(val.q.z);
		streamify(val.q.w);
		streamify(val.p.x);
		streamify(val.p.y);
		streamify(val.p.z);
	}
	void streamify(bool& val)
	{
		uint8_t tempVal = uint8_t(val ? 1 : 0);
		streamify(tempVal);
		val = tempVal ? true : false;
	}
};

template <typename TBulkRenderType>
struct BulkRenderEvent
{
	DataRef<TBulkRenderType> mData;
	BulkRenderEvent(const TBulkRenderType* data, uint32_t count) : mData(data, count)
	{
	}
	BulkRenderEvent()
	{
	}
	void serialize(RenderSerializer& serializer)
	{
		serializer.streamify(mData);
	}
};
struct SetInstanceIdRenderEvent
{
	uint64_t mInstanceId;
	SetInstanceIdRenderEvent(uint64_t iid) : mInstanceId(iid)
	{
	}
	SetInstanceIdRenderEvent()
	{
	}
	void serialize(RenderSerializer& serializer)
	{
		serializer.streamify(mInstanceId);
	}
};
struct PointsRenderEvent : BulkRenderEvent<PvdDebugPoint>
{
	PointsRenderEvent(const PvdDebugPoint* data, uint32_t count) : BulkRenderEvent<PvdDebugPoint>(data, count)
	{
	}
	PointsRenderEvent()
	{
	}
};
struct LinesRenderEvent : BulkRenderEvent<PvdDebugLine>
{
	LinesRenderEvent(const PvdDebugLine* data, uint32_t count) : BulkRenderEvent<PvdDebugLine>(data, count)
	{
	}
	LinesRenderEvent()
	{
	}
};
struct TrianglesRenderEvent : BulkRenderEvent<PvdDebugTriangle>
{
	TrianglesRenderEvent(const PvdDebugTriangle* data, uint32_t count) : BulkRenderEvent<PvdDebugTriangle>(data, count)
	{
	}
	TrianglesRenderEvent()
	{
	}
};
struct DebugRenderEvent
{
	DataRef<PvdDebugPoint> mPointData;
	DataRef<PvdDebugLine> mLineData;
	DataRef<PvdDebugTriangle> mTriangleData;
	DebugRenderEvent(const PvdDebugPoint* pointData, uint32_t pointCount, const PvdDebugLine* lineData,
	                 uint32_t lineCount, const PvdDebugTriangle* triangleData, uint32_t triangleCount)
	: mPointData(pointData, pointCount), mLineData(lineData, lineCount), mTriangleData(triangleData, triangleCount)
	{
	}

	DebugRenderEvent()
	{
	}
	void serialize(RenderSerializer& serializer)
	{
		serializer.streamify(mPointData);
		serializer.streamify(mLineData);
		serializer.streamify(mTriangleData);
	}
};

struct TextRenderEvent
{
	PvdDebugText mText;
	TextRenderEvent(const PvdDebugText& text)
	{
		mText.color = text.color;
		mText.position = text.position;
		mText.size = text.size;
		mText.string = text.string;
	}
	TextRenderEvent()
	{
	}
	void serialize(RenderSerializer& serializer)
	{
		serializer.streamify(mText);
	}
};

struct JointFramesRenderEvent
{
	PxTransform parent;
	PxTransform child;
	JointFramesRenderEvent(const PxTransform& p, const PxTransform& c) : parent(p), child(c)
	{
	}
	JointFramesRenderEvent()
	{
	}
	void serialize(RenderSerializer& serializer)
	{
		serializer.streamify(parent);
		serializer.streamify(child);
	}
};
struct LinearLimitRenderEvent
{
	PxTransform t0;
	PxTransform t1;
	float value;
	bool active;
	LinearLimitRenderEvent(const PxTransform& _t0, const PxTransform& _t1, float _value, bool _active)
	: t0(_t0), t1(_t1), value(_value), active(_active)
	{
	}
	LinearLimitRenderEvent()
	{
	}
	void serialize(RenderSerializer& serializer)
	{
		serializer.streamify(t0);
		serializer.streamify(t1);
		serializer.streamify(value);
		serializer.streamify(active);
	}
};
struct AngularLimitRenderEvent
{
	PxTransform t0;
	float lower;
	float upper;
	bool active;
	AngularLimitRenderEvent(const PxTransform& _t0, float _lower, float _upper, bool _active)
	: t0(_t0), lower(_lower), upper(_upper), active(_active)
	{
	}
	AngularLimitRenderEvent()
	{
	}
	void serialize(RenderSerializer& serializer)
	{
		serializer.streamify(t0);
		serializer.streamify(lower);
		serializer.streamify(upper);
		serializer.streamify(active);
	}
};
struct LimitConeRenderEvent
{
	PxTransform t;
	float ySwing;
	float zSwing;
	bool active;
	LimitConeRenderEvent(const PxTransform& _t, float _ySwing, float _zSwing, bool _active)
	: t(_t), ySwing(_ySwing), zSwing(_zSwing), active(_active)
	{
	}
	LimitConeRenderEvent()
	{
	}
	void serialize(RenderSerializer& serializer)
	{
		serializer.streamify(t);
		serializer.streamify(ySwing);
		serializer.streamify(zSwing);
		serializer.streamify(active);
	}
};
struct DoubleConeRenderEvent
{
	PxTransform t;
	float angle;
	bool active;
	DoubleConeRenderEvent(const PxTransform& _t, float _angle, bool _active) : t(_t), angle(_angle), active(_active)
	{
	}
	DoubleConeRenderEvent()
	{
	}
	void serialize(RenderSerializer& serializer)
	{
		serializer.streamify(t);
		serializer.streamify(angle);
		serializer.streamify(active);
	}
};

template <typename TDataType>
struct RenderSerializerMap
{
	void serialize(RenderSerializer& s, TDataType& d)
	{
		d.serialize(s);
	}
};
template <>
struct RenderSerializerMap<uint8_t>
{
	void serialize(RenderSerializer& s, uint8_t& d)
	{
		s.streamify(d);
	}
};

template <>
struct RenderSerializerMap<PvdDebugPoint>
{
	void serialize(RenderSerializer& s, PvdDebugPoint& d)
	{
		s.streamify(d.pos);
		s.streamify(d.color);
	}
};

template <>
struct RenderSerializerMap<PvdDebugLine>
{
	void serialize(RenderSerializer& s, PvdDebugLine& d)
	{
		s.streamify(d.pos0);
		s.streamify(d.color0);
		s.streamify(d.pos1);
		s.streamify(d.color1);
	}
};

template <>
struct RenderSerializerMap<PvdDebugTriangle>
{
	void serialize(RenderSerializer& s, PvdDebugTriangle& d)
	{
		s.streamify(d.pos0);
		s.streamify(d.color0);
		s.streamify(d.pos1);
		s.streamify(d.color1);
		s.streamify(d.pos2);
		s.streamify(d.color2);
	}
};

template <typename TDataType>
struct PvdTypeToRenderType
{
	bool compile_error;
};

#define DECLARE_PVD_IMMEDIATE_RENDER_TYPE(type)     \
	template <>                                     \
	struct PvdTypeToRenderType<type##RenderEvent>	\
	{                                               \
		enum Enum                                   \
		{                                           \
			EnumVal = PvdUserRenderTypes::type      \
		};                                          \
	};

#include "PxPvdUserRenderTypes.h"
#undef DECLARE_PVD_IMMEDIATE_RENDER_TYPE

template <typename TDataType>
PvdUserRenderTypes::Enum getPvdRenderTypeFromType()
{
	return static_cast<PvdUserRenderTypes::Enum>(PvdTypeToRenderType<TDataType>::EnumVal);
}

}
}

#endif // PXPVDSDK_PXPVDUSERRENDERIMPL_H
