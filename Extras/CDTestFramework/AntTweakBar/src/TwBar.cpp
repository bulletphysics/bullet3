//	---------------------------------------------------------------------------
//
//	@file		TwBar.cpp
//	@author		Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//				Copyright © 2005, 2006 Philippe Decaudin.
//              For conditions of distribution and use, see License.txt
//
//	note:		TAB=4 
//
//	---------------------------------------------------------------------------


#include "TwPrecomp.h"

#include <AntTweakBar.h>
#include "TwMgr.h"
#include "TwBar.h"
#include "TwColors.h"
  
using namespace std;

extern const char *g_ErrNotFound;
const char *g_ErrUnknownAttrib	= "Unknown attribute";
const char *g_ErrNotGroup		= "Value is not a group";
const char *g_ErrNoValue		= "Value required";
const char *g_ErrBadValue		= "Bad value";
const char *g_ErrUnknownType	= "Unknown type";
const char *g_ErrNotEnum		= "Must be of type Enum";

#undef PERF			// comment to print benchs
#define PERF(cmd)


PerfTimer g_BarTimer;

#define ANT_SET_CURSOR(_Name)		g_TwMgr->SetCursor(g_TwMgr->m_Cursor##_Name)
#define ANT_SET_ROTO_CURSOR(_Num)	g_TwMgr->SetCursor(g_TwMgr->m_RotoCursors[_Num])

#if !defined(ANT_WINDOWS)
#	define _stricmp strcasecmp
#	define _strdup strdup
#endif 	// defined(ANT_WINDOWS)

#if !defined(M_PI)
#	define M_PI 3.1415926535897932384626433832795
#endif	// !defined(M_PI)

const float  FLOAT_MAX  = 3.0e+38f;
const double DOUBLE_MAX = 1.0e+308;
const double DOUBLE_EPS = 1.0e-307;


//	---------------------------------------------------------------------------

CTwVarAtom::CTwVarAtom()
{
	m_Type = TW_TYPE_UNDEF;
	m_Ptr = NULL;
	m_SetCallback = NULL;
	m_GetCallback = NULL;
	m_ButtonCallback = NULL;
	m_ClientData = NULL;
	m_ReadOnly = false;
	m_NoSlider = false;
	m_KeyIncr[0] = 0;
	m_KeyIncr[1] = 0;
	m_KeyDecr[0] = 0;
	m_KeyDecr[1] = 0;
	memset(&m_Val, 0, sizeof(UVal));
}

CTwVarAtom::~CTwVarAtom()
{
	if( m_Type==TW_TYPE_BOOL8 || m_Type==TW_TYPE_BOOL16 || m_Type==TW_TYPE_BOOL32 || m_Type==TW_TYPE_BOOLCPP )
	{
		if( m_Val.m_Bool.m_FreeTrueString && m_Val.m_Bool.m_TrueString!=NULL )
		{
			free(m_Val.m_Bool.m_TrueString);
			m_Val.m_Bool.m_TrueString = NULL;
		}
		if( m_Val.m_Bool.m_FreeFalseString && m_Val.m_Bool.m_FalseString!=NULL )
		{
			free(m_Val.m_Bool.m_FalseString);
			m_Val.m_Bool.m_FalseString = NULL;
		}
	}
	/*
	else if( m_Type==TW_TYPE_ENUM8 || m_Type==TW_TYPE_ENUM16 || m_Type==TW_TYPE_ENUM32 )
	{
		if( m_Val.m_Enum.m_Entries!=NULL )
		{
			delete m_Val.m_Enum.m_Entries;
			m_Val.m_Enum.m_Entries = NULL;
		}
	}
	*/
}

//	---------------------------------------------------------------------------

void CTwVarAtom::ValueToString(std::string *_Str) const
{
	assert(_Str!=NULL);
	static const char *ErrStr = "unreachable";
	char Tmp[1024];
	if( m_Type==TW_TYPE_UNDEF || m_Type==TW_TYPE_HELP_ATOM || m_Type==TW_TYPE_HELP_GRP || m_Type==TW_TYPE_BUTTON )	// has no value
	{
		*_Str = "";
		return;
	}
	else if( m_Type==TW_TYPE_HELP_HEADER )
	{
		*_Str = "KEY+  KEY-";
		return;
	}
	else if( m_Type==TW_TYPE_SHORTCUT )	// special case for help bar: display shortcut
	{
		*_Str = "";
		if( m_ReadOnly && m_Val.m_Shortcut.m_Incr[0]==0 && m_Val.m_Shortcut.m_Decr[0]==0 )
			(*_Str) = "(read only)";
		else
		{
			if( m_Val.m_Shortcut.m_Incr[0]>0 )
				TwGetKeyString(_Str, m_Val.m_Shortcut.m_Incr[0], m_Val.m_Shortcut.m_Incr[1]);
			else
				(*_Str) += "(none)";
			if( m_Val.m_Shortcut.m_Decr[0]>0 )
			{
				(*_Str) += "  ";
				TwGetKeyString(_Str, m_Val.m_Shortcut.m_Decr[0], m_Val.m_Shortcut.m_Decr[1]);
			}
		}
		return;
	}
	else if( m_Type==TW_TYPE_HELP_STRUCT )
	{
		int idx = m_Val.m_HelpStruct.m_StructType - TW_TYPE_STRUCT_BASE;
		if( idx>=0 && idx<(int)g_TwMgr->m_Structs.size() )
		{
			if( g_TwMgr->m_Structs[idx].m_Name.length()>0 )
				(*_Str) = '{' + g_TwMgr->m_Structs[idx].m_Name + '}';
			else
				(*_Str) = "{struct}";
		}
		return;
	}
	if( m_Ptr==NULL && m_GetCallback==NULL )
	{
		*_Str = ErrStr;
		return;
	}
	bool UseGet = (m_GetCallback!=NULL);	
	switch( m_Type )
	{
	case TW_TYPE_BOOLCPP:
		{
			bool Val = 0;
			if( UseGet )
 				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(bool *)m_Ptr;
			if( Val )
				*_Str = (m_Val.m_Bool.m_TrueString!=NULL) ? m_Val.m_Bool.m_TrueString : "ON";
			else
				*_Str = (m_Val.m_Bool.m_FalseString!=NULL) ? m_Val.m_Bool.m_FalseString : "OFF";
		}
		break;
	case TW_TYPE_BOOL8:
		{
			char Val = 0;
			if( UseGet )
 				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(char *)m_Ptr;
			if( Val )
				*_Str = (m_Val.m_Bool.m_TrueString!=NULL) ? m_Val.m_Bool.m_TrueString : "ON";
			else
				*_Str = (m_Val.m_Bool.m_FalseString!=NULL) ? m_Val.m_Bool.m_FalseString : "OFF";
		}
		break;
	case TW_TYPE_BOOL16:
		{
			short Val = 0;
			if( UseGet )
 				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(short *)m_Ptr;
			if( Val )
				*_Str = (m_Val.m_Bool.m_TrueString!=NULL) ? m_Val.m_Bool.m_TrueString : "ON";
			else
				*_Str = (m_Val.m_Bool.m_FalseString!=NULL) ? m_Val.m_Bool.m_FalseString : "OFF";
		}
		break;
	case TW_TYPE_BOOL32:
		{
			int Val = 0;
			if( UseGet )
 				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(int *)m_Ptr;
			if( Val )
				*_Str = (m_Val.m_Bool.m_TrueString!=NULL) ? m_Val.m_Bool.m_TrueString : "ON";
			else
				*_Str = (m_Val.m_Bool.m_FalseString!=NULL) ? m_Val.m_Bool.m_FalseString : "OFF";
		}
		break;
	case TW_TYPE_CHAR:
		{
			unsigned char Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(unsigned char *)m_Ptr;
			if( Val!=0 )
			{
				int d = Val;
				if( m_Val.m_Char.m_Hexa )
					sprintf(Tmp, "%c (0x%.2X)", Val, d);
				else
					sprintf(Tmp, "%c (%d)", Val, d);
				*_Str = Tmp;
			}
			else
				*_Str = " (0)";
		}
		break;
	case TW_TYPE_INT8:
		{
			signed char Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(signed char *)m_Ptr;
			int d = Val;
			if( m_Val.m_Int8.m_Hexa )
				sprintf(Tmp, "0x%.2X", d&0xff);
			else
				sprintf(Tmp, "%d", d);
			*_Str = Tmp;
		}
		break;
	case TW_TYPE_UINT8:
		{
			unsigned char Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(unsigned char *)m_Ptr;
			unsigned int d = Val;
			if( m_Val.m_UInt8.m_Hexa )
				sprintf(Tmp, "0x%.2X", d);
			else		
				sprintf(Tmp, "%u", d);
			*_Str = Tmp;
		}
		break;
	case TW_TYPE_INT16:
		{
			short Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(short *)m_Ptr;
			int d = Val;
			if( m_Val.m_Int16.m_Hexa )
				sprintf(Tmp, "0x%.4X", d&0xffff);
			else
				sprintf(Tmp, "%d", d);
			*_Str = Tmp;
		}
		break;
	case TW_TYPE_UINT16:
		{
			unsigned short Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(unsigned short *)m_Ptr;
			unsigned int d = Val;
			if( m_Val.m_UInt16.m_Hexa )
				sprintf(Tmp, "0x%.4X", d);
			else
				sprintf(Tmp, "%u", d);
			*_Str = Tmp;
		}
		break;
	case TW_TYPE_INT32:
		{
			int Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(int *)m_Ptr;
			if( m_Val.m_Int32.m_Hexa )
				sprintf(Tmp, "0x%.8X", Val);
			else
				sprintf(Tmp, "%d", Val);
			*_Str = Tmp;
		}
		break;
	case TW_TYPE_UINT32:
		{
			unsigned int Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(unsigned int *)m_Ptr;
			if( m_Val.m_UInt32.m_Hexa )
				sprintf(Tmp, "0x%.8X", Val);
			else
				sprintf(Tmp, "%u", Val);
			*_Str = Tmp;
		}
		break;
	case TW_TYPE_FLOAT:
		{
			float Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(float *)m_Ptr;
			if( m_Val.m_Float32.m_Precision<0 )
				sprintf(Tmp, "%g", Val);
			else
			{
				char Fmt[32];
				sprintf(Fmt, "%%.%df", (int)m_Val.m_Float32.m_Precision);
				sprintf(Tmp, Fmt, Val);
			}
			*_Str = Tmp;
		}
		break;	
	case TW_TYPE_DOUBLE:
		{
			double Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(double *)m_Ptr;
			if( m_Val.m_Float64.m_Precision<0 )
				sprintf(Tmp, "%lg", Val);
			else
			{
				char Fmt[128];
				sprintf(Fmt, "%%.%dlf", (int)m_Val.m_Float64.m_Precision);
				sprintf(Tmp, Fmt, Val);
			}
			*_Str = Tmp;
		}
		break;
	/*
	case TW_TYPE_ENUM8:
	case TW_TYPE_ENUM16:
	case TW_TYPE_ENUM32:
		{
			unsigned int d = 0;
			if( m_Type==TW_TYPE_ENUM8 )
			{
				unsigned char Val = 0;
				if( UseGet )
					m_GetCallback(&Val, m_ClientData);
				else
					Val = *(unsigned char *)m_Ptr;
				d = Val;
			}
			else if( m_Type==TW_TYPE_ENUM16 )
			{
				unsigned short Val = 0;
				if( UseGet )
					m_GetCallback(&Val, m_ClientData);
				else
					Val = *(unsigned short *)m_Ptr;
				d = Val;
			}
			else
			{
				assert(m_Type==TW_TYPE_ENUM32);
				unsigned int Val = 0;
				if( UseGet )
					m_GetCallback(&Val, m_ClientData);
				else
					Val = *(unsigned int *)m_Ptr;
				d = Val;
			}
			bool Found = false;
			if( m_Val.m_Enum.m_Entries!=NULL )
			{
				UVal::CEnumVal::CEntries::iterator It = m_Val.m_Enum.m_Entries->find(d);
				if( It!=m_Val.m_Enum.m_Entries->end() )
				{
					*_Str = It->second;
					Found = true;
				}
			}
			if( !Found )
			{
				sprintf(Tmp, "%u", d);
				*_Str = Tmp;
			}
		}
		break;
	*/
	default:
		if( m_Type>=TW_TYPE_ENUM_BASE && m_Type<TW_TYPE_ENUM_BASE+(int)g_TwMgr->m_Enums.size() )
		{
			unsigned int Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(unsigned int *)m_Ptr;

			CTwMgr::CEnum& e = g_TwMgr->m_Enums[m_Type-TW_TYPE_ENUM_BASE];
			CTwMgr::CEnum::CEntries::iterator It = e.m_Entries.find(Val);
			if( It!=e.m_Entries.end() )
				*_Str = It->second;
			else
			{
				sprintf(Tmp, "%u", Val);
				*_Str = Tmp;
			}
		}
		else
			*_Str = "unknown type";
	}
}

//	---------------------------------------------------------------------------

double CTwVarAtom::ValueToDouble() const
{
	if( m_Ptr==NULL && m_GetCallback==NULL )
		return 0;	// unreachable
	bool UseGet = (m_GetCallback!=NULL);
	switch( m_Type )
	{
	case TW_TYPE_BOOLCPP:
		{
			bool Val = 0;
			if( UseGet )
 				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(bool *)m_Ptr;
			if( Val )
				return 1;
			else
				return 0;
		}
		break;
	case TW_TYPE_BOOL8:
		{
			char Val = 0;
			if( UseGet )
 				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(char *)m_Ptr;
			if( Val )
				return 1;
			else
				return 0;
		}
		break;
	case TW_TYPE_BOOL16:
		{
			short Val = 0;
			if( UseGet )
 				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(short *)m_Ptr;
			if( Val )
				return 1;
			else
				return 0;
		}
		break;
	case TW_TYPE_BOOL32:
		{
			int Val = 0;
			if( UseGet )
 				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(int *)m_Ptr;
			if( Val )
				return 1;
			else
				return 0;
		}
		break;
	case TW_TYPE_CHAR:
		{
			unsigned char Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(unsigned char *)m_Ptr;
			return Val;
		}
		break;
	case TW_TYPE_INT8:
		{
			signed char Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(signed char *)m_Ptr;
			int d = Val;
			return d;
		}
		break;
	case TW_TYPE_UINT8:
		{
			unsigned char Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(unsigned char *)m_Ptr;
			unsigned int d = Val;
			return d;
		}
		break;
	case TW_TYPE_INT16:
		{
			short Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(short *)m_Ptr;
			int d = Val;
			return d;
		}
		break;
	case TW_TYPE_UINT16:
		{
			unsigned short Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(unsigned short *)m_Ptr;
			unsigned int d = Val;
			return d;
		}
		break;
	case TW_TYPE_INT32:
		{
			int Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(int *)m_Ptr;
			return Val;
		}
		break;
	case TW_TYPE_UINT32:
		{
			unsigned int Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(unsigned int *)m_Ptr;
			return Val;
		}
		break;
	case TW_TYPE_FLOAT:
		{
			float Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(float *)m_Ptr;
			return Val;
		}
		break;	
	case TW_TYPE_DOUBLE:
		{
			double Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(double *)m_Ptr;
			return Val;
		}
		break;
	/*
	case TW_TYPE_ENUM8:
	case TW_TYPE_ENUM16:
	case TW_TYPE_ENUM32:
		{
			unsigned int d = 0;
			if( m_Type==TW_TYPE_ENUM8 )
			{
				unsigned char Val = 0;
				if( UseGet )
					m_GetCallback(&Val, m_ClientData);
				else
					Val = *(unsigned char *)m_Ptr;
				d = Val;
			}
			else if( m_Type==TW_TYPE_ENUM16 )
			{
				unsigned short Val = 0;
				if( UseGet )
					m_GetCallback(&Val, m_ClientData);
				else
					Val = *(unsigned short *)m_Ptr;
				d = Val;
			}
			else
			{
				assert(m_Type==TW_TYPE_ENUM32);
				unsigned int Val = 0;
				if( UseGet )
					m_GetCallback(&Val, m_ClientData);
				else
					Val = *(unsigned int *)m_Ptr;
				d = Val;
			}
			return d;
		}
		break;
	*/
	default:
		if( m_Type>=TW_TYPE_ENUM_BASE && m_Type<TW_TYPE_ENUM_BASE+(int)g_TwMgr->m_Enums.size() )
		{
			unsigned int Val = 0;
			if( UseGet )
				m_GetCallback(&Val, m_ClientData);
			else
				Val = *(unsigned int *)m_Ptr;
			return Val;
		}
		else
			return 0; // unknown type
	}
}

//	---------------------------------------------------------------------------

void CTwVarAtom::ValueFromDouble(double _Val)
{
	if( m_Ptr==NULL && m_SetCallback==NULL )
		return;	// unreachable
	bool UseSet = (m_SetCallback!=NULL);
	switch( m_Type )
	{
	case TW_TYPE_BOOLCPP:
		{
			bool Val = (_Val!=0);
			if( UseSet )
 				m_SetCallback(&Val, m_ClientData);
			else
				*(bool*)m_Ptr = Val;
		}
		break;
	case TW_TYPE_BOOL8:
		{
			char Val = (_Val!=0) ? 1 : 0;
			if( UseSet )
 				m_SetCallback(&Val, m_ClientData);
			else
				*(char*)m_Ptr = Val;
		}
		break;
	case TW_TYPE_BOOL16:
		{
			short Val = (_Val!=0) ? 1 : 0;
			if( UseSet )
 				m_SetCallback(&Val, m_ClientData);
			else
				*(short*)m_Ptr = Val;
		}
		break;
	case TW_TYPE_BOOL32:
		{
			int Val = (_Val!=0) ? 1 : 0;
			if( UseSet )
 				m_SetCallback(&Val, m_ClientData);
			else
				*(int*)m_Ptr = Val;
		}
		break;
	case TW_TYPE_CHAR:
		{
			unsigned char Val = (unsigned char)_Val;
			if( UseSet )
 				m_SetCallback(&Val, m_ClientData);
			else
				*(unsigned char*)m_Ptr = Val;
		}
		break;
	case TW_TYPE_INT8:
		{
			signed char Val = (signed char)_Val;
			if( UseSet )
 				m_SetCallback(&Val, m_ClientData);
			else
				*(signed char*)m_Ptr = Val;
		}
		break;
	case TW_TYPE_UINT8:
	//case TW_TYPE_ENUM8:
		{
			unsigned char Val = (unsigned char)_Val;
			if( UseSet )
 				m_SetCallback(&Val, m_ClientData);
			else
				*(unsigned char*)m_Ptr = Val;
		}
		break;
	case TW_TYPE_INT16:
		{
			short Val = (short)_Val;
			if( UseSet )
 				m_SetCallback(&Val, m_ClientData);
			else
				*(short*)m_Ptr = Val;
		}
		break;
	case TW_TYPE_UINT16:
	//case TW_TYPE_ENUM16:
		{
			unsigned short Val = (unsigned short)_Val;
			if( UseSet )
 				m_SetCallback(&Val, m_ClientData);
			else
				*(unsigned short*)m_Ptr = Val;
		}
		break;
	case TW_TYPE_INT32:
		{
			int Val = (int)_Val;
			if( UseSet )
 				m_SetCallback(&Val, m_ClientData);
			else
				*(int*)m_Ptr = Val;
		}
		break;
	case TW_TYPE_UINT32:
	//case TW_TYPE_ENUM32:
		{
			unsigned int Val = (unsigned int)_Val;
			if( UseSet )
 				m_SetCallback(&Val, m_ClientData);
			else
				*(unsigned int*)m_Ptr = Val;
		}
		break;
	case TW_TYPE_FLOAT:
		{
			float Val = (float)_Val;
			if( UseSet )
 				m_SetCallback(&Val, m_ClientData);
			else
				*(float*)m_Ptr = Val;
		}
		break;
	case TW_TYPE_DOUBLE:
		{
			double Val = (double)_Val;
			if( UseSet )
 				m_SetCallback(&Val, m_ClientData);
			else
				*(double*)m_Ptr = Val;
		}
		break;
	default:
		if( m_Type>=TW_TYPE_ENUM_BASE && m_Type<TW_TYPE_ENUM_BASE+(int)g_TwMgr->m_Enums.size() )
		{
			unsigned int Val = (unsigned int)_Val;
			if( UseSet )
 				m_SetCallback(&Val, m_ClientData);
			else
				*(unsigned int*)m_Ptr = Val;
		}
	}
}

//	---------------------------------------------------------------------------

void CTwVarAtom::MinMaxStepToDouble(double *_Min, double *_Max, double *_Step) const
{
	double max = DOUBLE_MAX;
	double min = -DOUBLE_MAX;
	double step = 1;

	switch( m_Type )
	{
	case TW_TYPE_BOOLCPP:
	case TW_TYPE_BOOL8:
	case TW_TYPE_BOOL16:
	case TW_TYPE_BOOL32:
		min = 0;
		max = 1;
		step = 1;
		break;
	case TW_TYPE_CHAR:
		min = (double)m_Val.m_Char.m_Min;
		max = (double)m_Val.m_Char.m_Max;
		step = (double)m_Val.m_Char.m_Step;
		break;
	case TW_TYPE_INT8:
		min = (double)m_Val.m_Int8.m_Min;
		max = (double)m_Val.m_Int8.m_Max;
		step = (double)m_Val.m_Int8.m_Step;
		break;
	case TW_TYPE_UINT8:
		min = (double)m_Val.m_UInt8.m_Min;
		max = (double)m_Val.m_UInt8.m_Max;
		step = (double)m_Val.m_UInt8.m_Step;
		break;
	case TW_TYPE_INT16:
		min = (double)m_Val.m_Int16.m_Min;
		max = (double)m_Val.m_Int16.m_Max;
		step = (double)m_Val.m_Int16.m_Step;
		break;
	case TW_TYPE_UINT16:
		min = (double)m_Val.m_UInt16.m_Min;
		max = (double)m_Val.m_UInt16.m_Max;
		step = (double)m_Val.m_UInt16.m_Step;
		break;
	case TW_TYPE_INT32:
		min = (double)m_Val.m_Int32.m_Min;
		max = (double)m_Val.m_Int32.m_Max;
		step = (double)m_Val.m_Int32.m_Step;
		break;
	case TW_TYPE_UINT32:
		min = (double)m_Val.m_UInt32.m_Min;
		max = (double)m_Val.m_UInt32.m_Max;
		step = (double)m_Val.m_UInt32.m_Step;
		break;
	case TW_TYPE_FLOAT:
		min = (double)m_Val.m_Float32.m_Min;
		max = (double)m_Val.m_Float32.m_Max;
		step = (double)m_Val.m_Float32.m_Step;
		break;
	case TW_TYPE_DOUBLE:
		min = m_Val.m_Float64.m_Min;
		max = m_Val.m_Float64.m_Max;
		step = m_Val.m_Float64.m_Step;
		break;
	default:
		{}	// nothing
	}

	if( _Min!=NULL )
		*_Min = min;
	if( _Max!=NULL )
		*_Max = max;
	if( _Step!=NULL )
		*_Step = step;
}

//	---------------------------------------------------------------------------

const CTwVar *CTwVarAtom::Find(const char *_Name, CTwVarGroup **_Parent, int *_Index) const
{
	if( strcmp(_Name, m_Name.c_str())==0 )
	{
		if( _Parent!=NULL )
			*_Parent = NULL;
		if( _Index!=NULL )
			*_Index = -1;
		return this;
	}
	else
		return NULL;
}

//	---------------------------------------------------------------------------

enum EVarAttribs
{
	V_LABEL = 1,
	V_HELP,
	V_GROUP,
	V_SHOW,
	V_HIDE,
	V_READONLY,
	V_READWRITE,
	V_ENDTAG
};

int CTwVar::HasAttrib(const char *_Attrib, bool *_HasValue) const
{
	*_HasValue = true;
	if( _stricmp(_Attrib, "label")==0 )
		return V_LABEL;
	else if( _stricmp(_Attrib, "help")==0 )
		return V_HELP;
	else if( _stricmp(_Attrib, "group")==0 )
		return V_GROUP;

	*_HasValue = false;
	if( _stricmp(_Attrib, "show")==0 )
		return V_SHOW;
	else if( _stricmp(_Attrib, "hide")==0 )
		return V_HIDE;
	if( _stricmp(_Attrib, "readonly")==0 )
		return V_READONLY;
	else if( _stricmp(_Attrib, "readwrite")==0 )
		return V_READWRITE;

	return 0; // not found
}

int CTwVar::SetAttrib(int _AttribID, const char *_Value, TwBar *_Bar, struct CTwVarGroup *_VarParent, int _VarIndex)
{
	switch( _AttribID )
	{
		/*
	case V_LABEL:
		if( _Value && strlen(_Value)>0 )
		{
			m_Label = _Value;
			_Bar->NotUpToDate();
			return 1;
		}
		else
		{
			g_TwMgr->SetLastError(g_ErrNoValue);
			return 0;
		}
		*/
	case V_LABEL:
	case V_HELP:
		if( _Value && strlen(_Value)>0 )
		{
			/*
			if( IsGroup() && static_cast<CTwVarGroup *>(this)->m_StructValuePtr!=NULL )
			{
				int Idx = static_cast<CTwVarGroup *>(this)->m_StructType-TW_TYPE_STRUCT_BASE;
				if( Idx>=0 && Idx<(int)g_TwMgr->m_Structs.size() )
					if( _AttribID==V_LABEL )
						g_TwMgr->m_Structs[Idx].m_Label = _Value;
					else // V_HELP
						g_TwMgr->m_Structs[Idx].m_Help = _Value;
			}
			else
			*/
			{
				CTwVarGroup *Parent = NULL;
				CTwVar *ThisVar = _Bar->Find(m_Name.c_str(), &Parent);
				if( this==ThisVar && Parent!=NULL && Parent->m_StructValuePtr!=NULL )
				{
					int Idx = Parent->m_StructType-TW_TYPE_STRUCT_BASE;
					if( Idx>=0 && Idx<(int)g_TwMgr->m_Structs.size() )
					{
						size_t nl = m_Name.length();
						for( size_t im=0; im<g_TwMgr->m_Structs[Idx].m_Members.size(); ++im )
						{
							size_t ml = g_TwMgr->m_Structs[Idx].m_Members[im].m_Name.length();
							if( nl>=ml && strcmp(g_TwMgr->m_Structs[Idx].m_Members[im].m_Name.c_str(), m_Name.c_str()+(nl-ml))==0 )
							{
								if( _AttribID==V_LABEL )
									g_TwMgr->m_Structs[Idx].m_Members[im].m_Label = _Value;
								else // V_HELP
									g_TwMgr->m_Structs[Idx].m_Members[im].m_Help = _Value;
								break;
							}
						}
					}
				}
				else
				{
					if( _AttribID==V_LABEL )
						m_Label = _Value;
					else // V_HELP
						m_Help = _Value;
				}
			}
			_Bar->NotUpToDate();
			return 1;
		}
		else
		{
			g_TwMgr->SetLastError(g_ErrNoValue);
			return 0;
		}
	case V_GROUP:
		{
			CTwVarGroup *Grp = NULL;
			if( _Value==NULL || strlen(_Value)<=0 )
				Grp = &(_Bar->m_VarRoot);
			else
			{
				CTwVar *v = _Bar->Find(_Value, NULL, NULL);
				if( v && !v->IsGroup() )
				{
					g_TwMgr->SetLastError(g_ErrNotGroup);
					return 0;
				}
				Grp = static_cast<CTwVarGroup *>(v);
				if( Grp==NULL )
				{
					Grp = new CTwVarGroup;
					Grp->m_Name = _Value;
					Grp->m_Open = true;
					Grp->m_SummaryCallback = NULL;
					Grp->m_SummaryClientData = NULL;
					Grp->m_StructValuePtr = NULL;
					if( g_TwMgr->m_HelpBar )
						Grp->m_Color = g_TwMgr->m_HelpBar->m_ColGrpText;
					else
						Grp->m_Color = COLOR32_WHITE;
					_Bar->m_VarRoot.m_Vars.push_back(Grp);
				}
			}
			Grp->m_Vars.push_back(this);
			if( _VarParent!=NULL && _VarIndex>=0 )
			{
				_VarParent->m_Vars.erase(_VarParent->m_Vars.begin()+_VarIndex);
				if( _VarParent!=&(_Bar->m_VarRoot) && _VarParent->m_Vars.size()<=0 )
					TwRemoveVar(_Bar, _VarParent->m_Name.c_str());
			}
			_Bar->NotUpToDate();
			return 1;			
		}
	case V_SHOW:
		if( !m_Visible )
		{
			m_Visible = true;
			_Bar->NotUpToDate();
		}
		return 1;
	case V_HIDE:
		if( m_Visible )
		{
			m_Visible = false;
			_Bar->NotUpToDate();
		}
		return 1;
	case V_READONLY:
		SetReadOnly(true);
		_Bar->NotUpToDate();
		return 1;
	case V_READWRITE:
		SetReadOnly(false);
		_Bar->NotUpToDate();
		return 1;
	default:
		g_TwMgr->SetLastError(g_ErrUnknownAttrib);
		return 0;
	}
}

//	---------------------------------------------------------------------------

enum EVarAtomAttribs
{
	VA_KEY_INCR = V_ENDTAG+1,
	VA_KEY_DECR,
	VA_MIN,
	VA_MAX,
	VA_STEP,
	VA_PRECISION,
	VA_HEXA,
	VA_DECIMAL,
	VA_TRUE,
	VA_FALSE,
	VA_VAL,
};

int CTwVarAtom::HasAttrib(const char *_Attrib, bool *_HasValue) const
{
	*_HasValue = true;
	if( _stricmp(_Attrib, "keyincr")==0 || _stricmp(_Attrib, "key")==0 )
		return VA_KEY_INCR;
	else if( _stricmp(_Attrib, "keydecr")==0 )
		return VA_KEY_DECR;
	else if( _stricmp(_Attrib, "min")==0 )
		return VA_MIN;
	else if( _stricmp(_Attrib, "max")==0 )
		return VA_MAX;
	else if( _stricmp(_Attrib, "step")==0 )
		return VA_STEP;
	else if( _stricmp(_Attrib, "precision")==0 )
		return VA_PRECISION;
	else if( _stricmp(_Attrib, "hexa")==0 )
	{
		*_HasValue = false;
		return VA_HEXA;
	}
	else if( _stricmp(_Attrib, "decimal")==0 )
	{
		*_HasValue = false;
		return VA_DECIMAL;
	}
	else if( _stricmp(_Attrib, "true")==0 )
		return VA_TRUE;
	else if( _stricmp(_Attrib, "false")==0 )
		return VA_FALSE;
	else if( _stricmp(_Attrib, "val")==0 )
		return VA_VAL;

	return CTwVar::HasAttrib(_Attrib, _HasValue);
}

int CTwVarAtom::SetAttrib(int _AttribID, const char *_Value, TwBar *_Bar, struct CTwVarGroup *_VarParent, int _VarIndex)
{
	switch( _AttribID )
	{
	case VA_KEY_INCR:
		{
			int Key = 0;
			int Mod = 0;
			if( TwGetKeyCode(&Key, &Mod, _Value) )
			{
				m_KeyIncr[0] = Key;
				m_KeyIncr[1] = Mod;
				return 1;
			}
			else
				return 0;
		}
	case VA_KEY_DECR:
		{
			int Key = 0;
			int Mod = 0;
			if( TwGetKeyCode(&Key, &Mod, _Value) )
			{
				m_KeyDecr[0] = Key;
				m_KeyDecr[1] = Mod;
				return 1;
			}
			else
				return 0;
		}
	case VA_TRUE:
		if( (m_Type==TW_TYPE_BOOL8 || m_Type==TW_TYPE_BOOL16 || m_Type==TW_TYPE_BOOL32 || m_Type==TW_TYPE_BOOLCPP) && _Value!=NULL )
		{
			if( m_Val.m_Bool.m_FreeTrueString && m_Val.m_Bool.m_TrueString!=NULL )
				free(m_Val.m_Bool.m_TrueString);
			m_Val.m_Bool.m_TrueString = _strdup(_Value);
			m_Val.m_Bool.m_FreeTrueString = true;
			return 1;
		}
		else
			return 0;
	case VA_FALSE:
		if( (m_Type==TW_TYPE_BOOL8 || m_Type==TW_TYPE_BOOL16 || m_Type==TW_TYPE_BOOL32 || m_Type==TW_TYPE_BOOLCPP) && _Value!=NULL )
		{
			if( m_Val.m_Bool.m_FreeFalseString && m_Val.m_Bool.m_FalseString!=NULL )
				free(m_Val.m_Bool.m_FalseString);
			m_Val.m_Bool.m_FalseString = _strdup(_Value);
			m_Val.m_Bool.m_FreeFalseString = true;
			return 1;
		}
		else
			return 0;
	case VA_MIN:
	case VA_MAX:
	case VA_STEP:
		if( _Value && strlen(_Value)>0 )
		{
			void *Ptr = NULL;
			const char *Fmt = NULL;
			int d = 0;
			unsigned int u = 0;
			int Num = (_AttribID==VA_STEP) ? 2 : ((_AttribID==VA_MAX) ? 1 : 0);
			switch( m_Type )
			{
			case TW_TYPE_CHAR:
				//Ptr = (&m_Val.m_Char.m_Min) + Num;
				//Fmt = "%c";
				Ptr = &u;
				Fmt = "%u";
				break;
			case TW_TYPE_INT16:
				Ptr = (&m_Val.m_Int16.m_Min) + Num;
				Fmt = "%hd";
				break;
			case TW_TYPE_INT32:
				Ptr = (&m_Val.m_Int32.m_Min) + Num;
				Fmt = "%d";
				break;
			case TW_TYPE_UINT16:
				Ptr = (&m_Val.m_UInt16.m_Min) + Num;
				Fmt = "%hu";
				break;
			case TW_TYPE_UINT32:
				Ptr = (&m_Val.m_UInt32.m_Min) + Num;
				Fmt = "%u";
				break;
			case TW_TYPE_FLOAT:
				Ptr = (&m_Val.m_Float32.m_Min) + Num;
				Fmt = "%f";
				break;
			case TW_TYPE_DOUBLE:
				Ptr = (&m_Val.m_Float64.m_Min) + Num;
				Fmt = "%lf";
				break;
			case TW_TYPE_INT8:
				Ptr = &d;
				Fmt = "%d";
				break;
			case TW_TYPE_UINT8:
				Ptr = &u;
				Fmt = "%u";
				break;
			default:
				g_TwMgr->SetLastError(g_ErrUnknownType);
				return 0;
			}

			if( Fmt!=NULL && Ptr!=NULL && sscanf(_Value, Fmt, Ptr)==1 )
			{
				if( m_Type==TW_TYPE_CHAR )
					*((&m_Val.m_Char.m_Min)+Num) = (unsigned char)(u);
				else if( m_Type==TW_TYPE_INT8 )
					*((&m_Val.m_Int8.m_Min)+Num) = (signed char)(d);
				else if( m_Type==TW_TYPE_UINT8 )
					*((&m_Val.m_UInt8.m_Min)+Num) = (unsigned char)(u);

				// set precision
				if( _AttribID==VA_STEP && ((m_Type==TW_TYPE_FLOAT && m_Val.m_Float32.m_Precision<0) || (m_Type==TW_TYPE_DOUBLE && m_Val.m_Float64.m_Precision<0)) )
				{
					double Step = fabs( (m_Type==TW_TYPE_FLOAT) ? m_Val.m_Float32.m_Step : m_Val.m_Float64.m_Step );
					signed char *Precision = (m_Type==TW_TYPE_FLOAT) ? &m_Val.m_Float32.m_Precision : &m_Val.m_Float64.m_Precision;
					if( Step>=1 )
						*Precision = 0;
					else if( Step>=0.1 )
						*Precision = 1;
					else if( Step>=0.01 )
						*Precision = 2;
					else if( Step>=0.001 )
						*Precision = 3;
					else if( Step>=0.0001 )
						*Precision = 4;
					else if( Step>=0.00001 )
						*Precision = 5;
					else if( Step>=0.000001 )
						*Precision = 6;
					else if( Step>=0.0000001 )
						*Precision = 7;
					else if( Step>=0.00000001 )
						*Precision = 8;
					else if( Step>=0.000000001 )
						*Precision = 9;
					else if( Step>=0.0000000001 )
						*Precision = 10;
					else if( Step>=0.00000000001 )
						*Precision = 11;
					else if( Step>=0.000000000001 )
						*Precision = 12;
					else
						*Precision = -1;
				}

				return 1;
			}
			else
			{
				g_TwMgr->SetLastError(g_ErrBadValue);
				return 0;
			}
		}
		else
		{
			g_TwMgr->SetLastError(g_ErrNoValue);
			return 0;
		}
	case VA_PRECISION:
		if( _Value && strlen(_Value)>0 )
		{
			int Precision = 0;
			if( sscanf(_Value, "%d", &Precision)==1 && Precision>-128 && Precision<128 )
			{
				if( m_Type==TW_TYPE_FLOAT )
					m_Val.m_Float32.m_Precision = (signed char)Precision;
				else if ( m_Type==TW_TYPE_DOUBLE )
					m_Val.m_Float64.m_Precision = (signed char)Precision;
				return 1;
			}
			else
			{
				g_TwMgr->SetLastError(g_ErrBadValue);
				return 0;
			}
		}
		else
		{
			g_TwMgr->SetLastError(g_ErrNoValue);
			return 0;
		}
	case VA_HEXA:
	case VA_DECIMAL:
		{
			bool hexa = (_AttribID==VA_HEXA);
			switch( m_Type )
			{
			case TW_TYPE_CHAR:
				m_Val.m_Char.m_Hexa = hexa;
				return 1;
			case TW_TYPE_INT8:
				m_Val.m_Int8.m_Hexa = hexa;
				return 1;
			case TW_TYPE_INT16:
				m_Val.m_Int16.m_Hexa = hexa;
				return 1;
			case TW_TYPE_INT32:
				m_Val.m_Int32.m_Hexa = hexa;
				return 1;
			case TW_TYPE_UINT8:
				m_Val.m_UInt8.m_Hexa = hexa;
				return 1;
			case TW_TYPE_UINT16:
				m_Val.m_UInt16.m_Hexa = hexa;
				return 1;
			case TW_TYPE_UINT32:
				m_Val.m_UInt32.m_Hexa = hexa;
				return 1;
			default:
				return 0;
			}
		}
	case VA_VAL:
		if( _Value && strlen(_Value)>0 && m_Type>=TW_TYPE_ENUM_BASE && m_Type<TW_TYPE_ENUM_BASE+(int)g_TwMgr->m_Enums.size() )
		{
			const char *s = _Value;
			int n = 0, i = 0;
			unsigned int u;
			bool Cont;
			do
			{
				Cont = false;
				i = 0;
				char Sep;
				n = sscanf(s, "%u %c%n", &u, &Sep, &i);
				if( n==2 && i>0 && ( Sep=='<' || Sep=='{' || Sep=='[' || Sep=='(' ) )
				{
					if( Sep=='<' )	// Change to closing separator
						Sep = '>';
					else if( Sep=='{' )
						Sep = '}';
					else if( Sep=='[' )
						Sep = ']';
					else if( Sep=='(' )
						Sep = ')';
					s += i;
					i = 0;
					while( s[i]!=Sep && s[i]!=0 )
						++i;
					if( s[i]==Sep )
					{
						//if( m_Val.m_Enum.m_Entries==NULL )
						//	m_Val.m_Enum.m_Entries = new UVal::CEnumVal::CEntries;
						//UVal::CEnumVal::CEntries::value_type v(u, "");
						CTwMgr::CEnum::CEntries::value_type v(u, "");
						if( i>0 )
							v.second.assign(s, i);
						//m_Val.m_Enum.m_Entries->insert(v);
						pair<CTwMgr::CEnum::CEntries::iterator, bool> ret;
						ret = g_TwMgr->m_Enums[m_Type-TW_TYPE_ENUM_BASE].m_Entries.insert(v);
						if( !ret.second ) // force overwrite if element already exists
						{
							g_TwMgr->m_Enums[m_Type-TW_TYPE_ENUM_BASE].m_Entries.erase(ret.first);
							g_TwMgr->m_Enums[m_Type-TW_TYPE_ENUM_BASE].m_Entries.insert(v);
						}

						s += i+1;
						i = 0;
						n = sscanf(s, " ,%n", &i);
						if( n==0 && i>=1 )
						{
							s += i;
							Cont = true;
						}
					}
					else
					{
						g_TwMgr->SetLastError(g_ErrBadValue);
						return 0;
					}
				}
				else
				{
					g_TwMgr->SetLastError(g_ErrBadValue);
					return 0;
				}
			} while( Cont );
			return 1;
		}
		else
		{
			g_TwMgr->SetLastError(g_ErrNoValue);
			return 0;
		}
		break;
	default:
		return CTwVar::SetAttrib(_AttribID, _Value, _Bar, _VarParent, _VarIndex);
	}
}

//	---------------------------------------------------------------------------

void CTwVarAtom::Increment(int _Step)
{
	if( _Step==0 )
		return;
	switch( m_Type )
	{
	case TW_TYPE_BOOL8:
		{
			char v = false;
			if( m_Ptr!=NULL )
				v = *((char *)m_Ptr);
			else if( m_GetCallback!=NULL )
				m_GetCallback(&v, m_ClientData);
			if( v )
				v = false;
			else
				v = true;
			if( m_Ptr!=NULL )
				*((char *)m_Ptr) = v;
			else if( m_SetCallback!=NULL )
				m_SetCallback(&v, m_ClientData);
		}
		break;
	case TW_TYPE_BOOL16:
		{
			short v = false;
			if( m_Ptr!=NULL )
				v = *((short *)m_Ptr);
			else if( m_GetCallback!=NULL )
				m_GetCallback(&v, m_ClientData);
			if( v )
				v = false;
			else
				v = true;
			if( m_Ptr!=NULL )
				*((short *)m_Ptr) = v;
			else if( m_SetCallback!=NULL )
				m_SetCallback(&v, m_ClientData);
		}
		break;
	case TW_TYPE_BOOL32:
		{
			int v = false;
			if( m_Ptr!=NULL )
				v = *((int *)m_Ptr);
			else if( m_GetCallback!=NULL )
				m_GetCallback(&v, m_ClientData);
			if( v )
				v = false;
			else
				v = true;
			if( m_Ptr!=NULL )
				*((int *)m_Ptr) = v;
			else if( m_SetCallback!=NULL )
				m_SetCallback(&v, m_ClientData);
		}
		break;
	case TW_TYPE_BOOLCPP:
		{
			bool v = false;
			if( m_Ptr!=NULL )
				v = *((bool *)m_Ptr);
			else if( m_GetCallback!=NULL )
				m_GetCallback(&v, m_ClientData);
			if( v )
				v = false;
			else
				v = true;
			if( m_Ptr!=NULL )
				*((bool *)m_Ptr) = v;
			else if( m_SetCallback!=NULL )
				m_SetCallback(&v, m_ClientData);
		}
		break;
	case TW_TYPE_CHAR:
		{
			unsigned char v = 0;
			if( m_Ptr!=NULL )
				v = *((unsigned char *)m_Ptr);
			else if( m_GetCallback!=NULL )
				m_GetCallback(&v, m_ClientData);
			int iv = _Step*(int)m_Val.m_Char.m_Step + (int)v;
			if( iv<m_Val.m_Char.m_Min )
				iv = m_Val.m_Char.m_Min;
			if( iv>m_Val.m_Char.m_Max )
				iv = m_Val.m_Char.m_Max;
			if( iv<0 )
				iv = 0;
			else if( iv>0xff )
				iv = 0xff;
			v = (unsigned char)iv;
			if( m_Ptr!=NULL )
				*((unsigned char *)m_Ptr) = v;
			else if( m_SetCallback!=NULL )
				m_SetCallback(&v, m_ClientData);
		}
		break;
	case TW_TYPE_INT8:
		{
			signed char v = 0;
			if( m_Ptr!=NULL )
				v = *((signed char *)m_Ptr);
			else if( m_GetCallback!=NULL )
				m_GetCallback(&v, m_ClientData);
			int iv = _Step*(int)m_Val.m_Int8.m_Step + (int)v;
			if( iv<m_Val.m_Int8.m_Min )
				iv = m_Val.m_Int8.m_Min;
			if( iv>m_Val.m_Int8.m_Max )
				iv = m_Val.m_Int8.m_Max;
			v = (signed char)iv;
			if( m_Ptr!=NULL )
				*((signed char *)m_Ptr) = v;
			else if( m_SetCallback!=NULL )
				m_SetCallback(&v, m_ClientData);
		}
		break;
	case TW_TYPE_UINT8:
		{
			unsigned char v = 0;
			if( m_Ptr!=NULL )
				v = *((unsigned char *)m_Ptr);
			else if( m_GetCallback!=NULL )
				m_GetCallback(&v, m_ClientData);
			int iv = _Step*(int)m_Val.m_UInt8.m_Step + (int)v;
			if( iv<m_Val.m_UInt8.m_Min )
				iv = m_Val.m_UInt8.m_Min;
			if( iv>m_Val.m_UInt8.m_Max )
				iv = m_Val.m_UInt8.m_Max;
			if( iv<0 )
				iv = 0;
			else if( iv>0xff )
				iv = 0xff;
			v = (unsigned char)iv;
			if( m_Ptr!=NULL )
				*((unsigned char *)m_Ptr) = v;
			else if( m_SetCallback!=NULL )
				m_SetCallback(&v, m_ClientData);
		}
		break;
	case TW_TYPE_INT16:
		{
			short v = 0;
			if( m_Ptr!=NULL )
				v = *((short *)m_Ptr);
			else if( m_GetCallback!=NULL )
				m_GetCallback(&v, m_ClientData);
			int iv = _Step*(int)m_Val.m_Int16.m_Step + (int)v;
			if( iv<m_Val.m_Int16.m_Min )
				iv = m_Val.m_Int16.m_Min;
			if( iv>m_Val.m_Int16.m_Max )
				iv = m_Val.m_Int16.m_Max;
			v = (short)iv;
			if( m_Ptr!=NULL )
				*((short *)m_Ptr) = v;
			else if( m_SetCallback!=NULL )
				m_SetCallback(&v, m_ClientData);
		}
		break;
	case TW_TYPE_UINT16:
		{
			unsigned short v = 0;
			if( m_Ptr!=NULL )
				v = *((unsigned short *)m_Ptr);
			else if( m_GetCallback!=NULL )
				m_GetCallback(&v, m_ClientData);
			int iv = _Step*(int)m_Val.m_UInt16.m_Step + (int)v;
			if( iv<m_Val.m_UInt16.m_Min )
				iv = m_Val.m_UInt16.m_Min;
			if( iv>m_Val.m_UInt16.m_Max )
				iv = m_Val.m_UInt16.m_Max;
			if( iv<0 )
				iv = 0;
			else if( iv>0xffff )
				iv = 0xffff;
			v = (unsigned short)iv;
			if( m_Ptr!=NULL )
				*((unsigned short *)m_Ptr) = v;
			else if( m_SetCallback!=NULL )
				m_SetCallback(&v, m_ClientData);
		}
		break;
	case TW_TYPE_INT32:
		{
			int v = 0;
			if( m_Ptr!=NULL )
				v = *((int *)m_Ptr);
			else if( m_GetCallback!=NULL )
				m_GetCallback(&v, m_ClientData);
			double dv = (double)_Step*(double)m_Val.m_Int32.m_Step + (double)v;
			if( dv>(double)0x7fffffff )
				v = 0x7fffffff;
			else if( dv<(double)(-0x7fffffff-1) )
				v = -0x7fffffff-1;
			else
				v = _Step*m_Val.m_Int32.m_Step + v;
			if( v<m_Val.m_Int32.m_Min )
				v = m_Val.m_Int32.m_Min;
			if( v>m_Val.m_Int32.m_Max )
				v = m_Val.m_Int32.m_Max;
			if( m_Ptr!=NULL )
				*((int *)m_Ptr) = v;
			else if( m_SetCallback!=NULL )
				m_SetCallback(&v, m_ClientData);
		}
		break;
	case TW_TYPE_UINT32:
		{
			unsigned int v = 0;
			if( m_Ptr!=NULL )
				v = *((unsigned int *)m_Ptr);
			else if( m_GetCallback!=NULL )
				m_GetCallback(&v, m_ClientData);
			double dv = (double)_Step*(double)m_Val.m_UInt32.m_Step + (double)v;
			if( dv>(double)0xffffffff )
				v = 0xffffffff;
			else if( dv<0 )
				v = 0;
			else
				v = _Step*m_Val.m_UInt32.m_Step + v;
			if( v<m_Val.m_UInt32.m_Min )
				v = m_Val.m_UInt32.m_Min;
			if( v>m_Val.m_UInt32.m_Max )
				v = m_Val.m_UInt32.m_Max;
			if( m_Ptr!=NULL )
				*((unsigned int *)m_Ptr) = v;
			else if( m_SetCallback!=NULL )
				m_SetCallback(&v, m_ClientData);
		}
		break;
	case TW_TYPE_FLOAT:
		{
			float v = 0;
			if( m_Ptr!=NULL )
				v = *((float *)m_Ptr);
			else if( m_GetCallback!=NULL )
				m_GetCallback(&v, m_ClientData);
			v += _Step*m_Val.m_Float32.m_Step;
			if( v<m_Val.m_Float32.m_Min )
				v = m_Val.m_Float32.m_Min;
			if( v>m_Val.m_Float32.m_Max )
				v = m_Val.m_Float32.m_Max;
			if( m_Ptr!=NULL )
				*((float *)m_Ptr) = v;
			else if( m_SetCallback!=NULL )
				m_SetCallback(&v, m_ClientData);
		}
		break;
	case TW_TYPE_DOUBLE:
		{
			double v = 0;
			if( m_Ptr!=NULL )
				v = *((double *)m_Ptr);
			else if( m_GetCallback!=NULL )
				m_GetCallback(&v, m_ClientData);
			v += _Step*m_Val.m_Float64.m_Step;
			if( v<m_Val.m_Float64.m_Min )
				v = m_Val.m_Float64.m_Min;
			if( v>m_Val.m_Float64.m_Max )
				v = m_Val.m_Float64.m_Max;
			if( m_Ptr!=NULL )
				*((double *)m_Ptr) = v;
			else if( m_SetCallback!=NULL )
				m_SetCallback(&v, m_ClientData);
		}
		break;
	/*
	case TW_TYPE_ENUM8:
		{
			assert(_Step==1 || _Step==-1);
			unsigned char v = 0;
			if( m_Ptr!=NULL )
				v = *((unsigned char *)m_Ptr);
			else if( m_GetCallback!=NULL )
				m_GetCallback(&v, m_ClientData);
			if( m_Val.m_Enum.m_Entries!=NULL )
			{
				UVal::CEnumVal::CEntries::iterator It = m_Val.m_Enum.m_Entries->find(v);
				if( It==m_Val.m_Enum.m_Entries->end() )
					It = m_Val.m_Enum.m_Entries->begin();
				else if( _Step==1 )
				{
					++It;
					if( It==m_Val.m_Enum.m_Entries->end() )
						It = m_Val.m_Enum.m_Entries->begin();
				}
				else if( _Step==-1 )
				{
					if( It==m_Val.m_Enum.m_Entries->begin() )
						It = m_Val.m_Enum.m_Entries->end();
					if( It!=m_Val.m_Enum.m_Entries->begin() )
						--It;
				}
				if( It != m_Val.m_Enum.m_Entries->end() )
				{
					v = (unsigned char)(It->first);
					if( m_Ptr!=NULL )
						*((unsigned char *)m_Ptr) = v;
					else if( m_SetCallback!=NULL )
						m_SetCallback(&v, m_ClientData);
				}
			}
		}
		break;
	case TW_TYPE_ENUM16:
		{
			assert(_Step==1 || _Step==-1);
			unsigned short v = 0;
			if( m_Ptr!=NULL )
				v = *((unsigned short *)m_Ptr);
			else if( m_GetCallback!=NULL )
				m_GetCallback(&v, m_ClientData);
			if( m_Val.m_Enum.m_Entries!=NULL )
			{
				UVal::CEnumVal::CEntries::iterator It = m_Val.m_Enum.m_Entries->find(v);
				if( It==m_Val.m_Enum.m_Entries->end() )
					It = m_Val.m_Enum.m_Entries->begin();
				else if( _Step==1 )
				{
					++It;
					if( It==m_Val.m_Enum.m_Entries->end() )
						It = m_Val.m_Enum.m_Entries->begin();
				}
				else if( _Step==-1 )
				{
					if( It==m_Val.m_Enum.m_Entries->begin() )
						It = m_Val.m_Enum.m_Entries->end();
					if( It!=m_Val.m_Enum.m_Entries->begin() )
						--It;
				}
				if( It != m_Val.m_Enum.m_Entries->end() )
				{
					v = (unsigned short)(It->first);
					if( m_Ptr!=NULL )
						*((unsigned short *)m_Ptr) = v;
					else if( m_SetCallback!=NULL )
						m_SetCallback(&v, m_ClientData);
				}
			}
		}
		break;
	case TW_TYPE_ENUM32:
		{
			assert(_Step==1 || _Step==-1);
			unsigned int v = 0;
			if( m_Ptr!=NULL )
				v = *((unsigned int *)m_Ptr);
			else if( m_GetCallback!=NULL )
				m_GetCallback(&v, m_ClientData);
			if( m_Val.m_Enum.m_Entries!=NULL )
			{
				UVal::CEnumVal::CEntries::iterator It = m_Val.m_Enum.m_Entries->find(v);
				if( It==m_Val.m_Enum.m_Entries->end() )
					It = m_Val.m_Enum.m_Entries->begin();
				else if( _Step==1 )
				{
					++It;
					if( It==m_Val.m_Enum.m_Entries->end() )
						It = m_Val.m_Enum.m_Entries->begin();
				}
				else if( _Step==-1 )
				{
					if( It==m_Val.m_Enum.m_Entries->begin() )
						It = m_Val.m_Enum.m_Entries->end();
					if( It!=m_Val.m_Enum.m_Entries->begin() )
						--It;
				}
				if( It!=m_Val.m_Enum.m_Entries->end() )
				{
					v = (unsigned int)(It->first);
					if( m_Ptr!=NULL )
						*((unsigned int *)m_Ptr) = v;
					else if( m_SetCallback!=NULL )
						m_SetCallback(&v, m_ClientData);
				}
			}
		}
		break;
	*/
	default:
		if( m_Type==TW_TYPE_BUTTON )
		{
			if( m_ButtonCallback!=NULL )
				m_ButtonCallback(m_ClientData);
		}
		else if( m_Type>=TW_TYPE_ENUM_BASE && m_Type<TW_TYPE_ENUM_BASE+(int)g_TwMgr->m_Enums.size() )
		{
			assert(_Step==1 || _Step==-1);
			unsigned int v = 0;
			if( m_Ptr!=NULL )
				v = *((unsigned int *)m_Ptr);
			else if( m_GetCallback!=NULL )
				m_GetCallback(&v, m_ClientData);
			CTwMgr::CEnum& e = g_TwMgr->m_Enums[m_Type-TW_TYPE_ENUM_BASE];
			CTwMgr::CEnum::CEntries::iterator It = e.m_Entries.find(v);
			if( It==e.m_Entries.end() )
				It = e.m_Entries.begin();
			else if( _Step==1 )
			{
				++It;
				if( It==e.m_Entries.end() )
					It = e.m_Entries.begin();
			}
			else if( _Step==-1 )
			{
				if( It==e.m_Entries.begin() )
					It = e.m_Entries.end();
				if( It!=e.m_Entries.begin() )
					--It;
			}
			if( It!=e.m_Entries.end() )
			{
				v = (unsigned int)(It->first);
				if( m_Ptr!=NULL )
					*((unsigned int *)m_Ptr) = v;
				else if( m_SetCallback!=NULL )
					m_SetCallback(&v, m_ClientData);
			}
		}
		else
			fprintf(stderr, "CTwVarAtom::Increment : unknown or unimplemented type\n");
	}
}

//	---------------------------------------------------------------------------

void CTwVarAtom::SetDefaults()
{
	switch( m_Type )
	{
	case TW_TYPE_BOOL8:
	case TW_TYPE_BOOL16:
	case TW_TYPE_BOOL32:
	case TW_TYPE_BOOLCPP:
		m_NoSlider = true;
		break;
	case TW_TYPE_CHAR:
		m_Val.m_Char.m_Max = 0xff;
		m_Val.m_Char.m_Min = 0;
		m_Val.m_Char.m_Step = 1;
		m_Val.m_Char.m_Precision = -1;
		m_Val.m_Char.m_Hexa = false;
		break;
	case TW_TYPE_INT8:
		m_Val.m_Int8.m_Max = 0x7f;
		m_Val.m_Int8.m_Min = -m_Val.m_Int8.m_Max-1;
		m_Val.m_Int8.m_Step = 1;
		m_Val.m_Int8.m_Precision = -1;
		m_Val.m_Int8.m_Hexa = false;
		break;
	case TW_TYPE_UINT8:
		m_Val.m_UInt8.m_Max = 0xff;
		m_Val.m_UInt8.m_Min = 0;
		m_Val.m_UInt8.m_Step = 1;
		m_Val.m_UInt8.m_Precision = -1;
		m_Val.m_UInt8.m_Hexa = false;
		break;
	case TW_TYPE_INT16:
		m_Val.m_Int16.m_Max = 0x7fff;
		m_Val.m_Int16.m_Min = -m_Val.m_Int16.m_Max-1;
		m_Val.m_Int16.m_Step = 1;
		m_Val.m_Int16.m_Precision = -1;
		m_Val.m_Int16.m_Hexa = false;
		break;
	case TW_TYPE_UINT16:
		m_Val.m_UInt16.m_Max = 0xffff;
		m_Val.m_UInt16.m_Min = 0;
		m_Val.m_UInt16.m_Step = 1;
		m_Val.m_UInt16.m_Precision = -1;
		m_Val.m_UInt16.m_Hexa = false;
		break;
	case TW_TYPE_INT32:
		m_Val.m_Int32.m_Max = 0x7fffffff;
		m_Val.m_Int32.m_Min = -m_Val.m_Int32.m_Max-1;
		m_Val.m_Int32.m_Step = 1;
		m_Val.m_Int32.m_Precision = -1;
		m_Val.m_Int32.m_Hexa = false;
		break;
	case TW_TYPE_UINT32:
		m_Val.m_UInt32.m_Max = 0xffffffff;
		m_Val.m_UInt32.m_Min = 0;
		m_Val.m_UInt32.m_Step = 1;
		m_Val.m_UInt32.m_Precision = -1;
		m_Val.m_UInt32.m_Hexa = false;
		break;
	case TW_TYPE_FLOAT:
		m_Val.m_Float32.m_Max = FLOAT_MAX;
		m_Val.m_Float32.m_Min = -FLOAT_MAX;
		m_Val.m_Float32.m_Step = 1;
		m_Val.m_Float32.m_Precision = -1;
		m_Val.m_Float32.m_Hexa = false;
		break;
	case TW_TYPE_DOUBLE:
		m_Val.m_Float64.m_Max = DOUBLE_MAX;
		m_Val.m_Float64.m_Min = -DOUBLE_MAX;
		m_Val.m_Float64.m_Step = 1;
		m_Val.m_Float64.m_Precision = -1;
		m_Val.m_Float64.m_Hexa = false;
		break;
	/*
	case TW_TYPE_ENUM8:
	case TW_TYPE_ENUM16:
	case TW_TYPE_ENUM32:
		m_NoSlider = true;
		break;
	*/
	default:
		{}	// nothing
	}

	// special types
	if( m_Type==TW_TYPE_BUTTON || (m_Type>=TW_TYPE_ENUM_BASE && m_Type<TW_TYPE_ENUM_BASE+(int)g_TwMgr->m_Enums.size()) )
		m_NoSlider = true;
}

//	---------------------------------------------------------------------------

/*
int CTwVarAtom::DefineEnum(const TwEnumVal *_EnumValues, unsigned int _NbValues)
{
	assert(_EnumValues!=NULL);
	if( m_Type!=TW_TYPE_ENUM8 && m_Type!=TW_TYPE_ENUM16 && m_Type!=TW_TYPE_ENUM32 )
	{
		g_TwMgr->SetLastError(g_ErrNotEnum);
		return 0;
	}
	if( m_Val.m_Enum.m_Entries==NULL )
		m_Val.m_Enum.m_Entries = new UVal::CEnumVal::CEntries;
	for(unsigned int i=0; i<_NbValues; ++i)
	{
		UVal::CEnumVal::CEntries::value_type Entry(_EnumValues[i].Value, (_EnumValues[i].Label!=NULL)?_EnumValues[i].Label:"");
		pair<UVal::CEnumVal::CEntries::iterator, bool> Result = m_Val.m_Enum.m_Entries->insert(Entry);
		if( !Result.second )
			(Result.first)->second = Entry.second;
	}
	return 1;
}
*/

//	---------------------------------------------------------------------------

enum EVarGroupAttribs
{
	VG_OPEN = V_ENDTAG+1,
	VG_CLOSE,
	VG_TYPEID,	// used internally for structs
	VG_VALPTR,	// used internally for structs
	VG_ALPHA,	// tw_type_color* only
	VG_NOALPHA,	// tw_type_color* only
	VG_HLS,		// tw_type_color* only
	VG_RGB,		// tw_type_color* only
	VG_ORDER,	// tw_type_color* only
};

int CTwVarGroup::HasAttrib(const char *_Attrib, bool *_HasValue) const
{
	*_HasValue = false;
	if( _stricmp(_Attrib, "open")==0 )
		return VG_OPEN;
	else if( _stricmp(_Attrib, "close")==0 )
		return VG_CLOSE;
	else if( _stricmp(_Attrib, "typeid")==0 )
	{
		*_HasValue = true;
		return VG_TYPEID;
	}
	else if( _stricmp(_Attrib, "valptr")==0 )
	{
		*_HasValue = true;
		return VG_VALPTR;
	}
	else if( _stricmp(_Attrib, "alpha")==0 )
		return VG_ALPHA;
	else if( _stricmp(_Attrib, "noalpha")==0 )
		return VG_NOALPHA;
	else if( _stricmp(_Attrib, "hls")==0 )
		return VG_HLS;
	else if( _stricmp(_Attrib, "rgb")==0 )
		return VG_RGB;
	else if( _stricmp(_Attrib, "order")==0 )
	{
		*_HasValue = true;
		return VG_ORDER;
	}

	return CTwVar::HasAttrib(_Attrib, _HasValue);
}

int CTwVarGroup::SetAttrib(int _AttribID, const char *_Value, TwBar *_Bar, struct CTwVarGroup *_VarParent, int _VarIndex)
{
	switch( _AttribID )
	{
	case VG_OPEN:
		if( !m_Open )
		{
			m_Open = true;
			_Bar->NotUpToDate();
		}
		return 1;
	case VG_CLOSE:
		if( m_Open )
		{
			m_Open = false;
			_Bar->NotUpToDate();
		}
		return 1;
	case VG_TYPEID:
		{
			int type = TW_TYPE_UNDEF;
			if( _Value!=NULL && sscanf(_Value, "%d", &type)==1 )
			{
				int idx = type - TW_TYPE_STRUCT_BASE;
				if( idx>=0 && idx<(int)g_TwMgr->m_Structs.size() )
				{
					m_SummaryCallback   = g_TwMgr->m_Structs[idx].m_SummaryCallback;
					m_SummaryClientData = g_TwMgr->m_Structs[idx].m_SummaryClientData;
					m_StructType = (TwType)type;
					return 1;
				}
			}
			return 0;
		}
	case VG_VALPTR:
		{
			void *structValuePtr = NULL;
			if( _Value!=NULL && sscanf(_Value, "%p", &structValuePtr)==1 )
			{
				m_StructValuePtr = structValuePtr;
				m_Color = _Bar->m_ColStructText;
				return 1;
			}
			return 0;
		}
	case VG_ALPHA:
		if( m_SummaryCallback==CColorExt::SummaryCB && m_StructValuePtr!=NULL )	// is tw_type_color?
			if( static_cast<CColorExt *>(m_StructValuePtr)->m_CanHaveAlpha )
			{
				static_cast<CColorExt *>(m_StructValuePtr)->m_HasAlpha = true;
				_Bar->NotUpToDate();
				return 1;
			}
		return 0;
	case VG_NOALPHA:
		if( m_SummaryCallback==CColorExt::SummaryCB && m_StructValuePtr!=NULL )	// is tw_type_color?
		{
			static_cast<CColorExt *>(m_StructValuePtr)->m_HasAlpha = false;
			_Bar->NotUpToDate();
			return 1;
		}
		else
			return 0;
	case VG_HLS:
		if( m_SummaryCallback==CColorExt::SummaryCB && m_StructValuePtr!=NULL )	// is tw_type_color?
		{
			static_cast<CColorExt *>(m_StructValuePtr)->m_HLS = true;
			_Bar->NotUpToDate();
			return 1;
		}
		else
			return 0;
	case VG_RGB:
		if( m_SummaryCallback==CColorExt::SummaryCB && m_StructValuePtr!=NULL )	// is tw_type_color?
		{
			static_cast<CColorExt *>(m_StructValuePtr)->m_HLS = false;
			_Bar->NotUpToDate();
			return 1;
		}
		else
			return 0;
	case VG_ORDER:
		if( m_SummaryCallback==CColorExt::SummaryCB && m_StructValuePtr!=NULL )	// is tw_type_color?
		{
			if( _Value!=NULL )
			{
				if( _stricmp(_Value, "ogl")==0 )
					static_cast<CColorExt *>(m_StructValuePtr)->m_OGL = true;
				else if( _stricmp(_Value, "dx")==0 )
					static_cast<CColorExt *>(m_StructValuePtr)->m_OGL = false;
				else
					return 0;
				return 1;
			}
			return 0;
		}
		else
			return 0;
	default:
		return CTwVar::SetAttrib(_AttribID, _Value, _Bar, _VarParent, _VarIndex);
	}
}

//	---------------------------------------------------------------------------

const CTwVar *CTwVarGroup::Find(const char *_Name, CTwVarGroup **_Parent, int *_Index) const
{
	if( strcmp(_Name, m_Name.c_str())==0 )
	{
		if( _Parent!=NULL )
			*_Parent = NULL;
		if( _Index!=NULL )
			*_Index = -1;
		return this;
	}
	else
	{
		const CTwVar *v;
		for( size_t i=0; i<m_Vars.size(); ++ i )
			if( m_Vars[i]!=NULL )
			{
				v = m_Vars[i]->Find(_Name, _Parent, _Index);
				if( v!=NULL )
				{
					if( _Parent!=NULL && *_Parent==NULL )
					{
						*_Parent = const_cast<CTwVarGroup *>(this);
						if( _Index!=NULL )
							*_Index  = (int)i;
					}
					return v;
				}
			}
		return NULL;
	}
}

//	---------------------------------------------------------------------------

size_t CTwVar::GetDataSize(TwType _Type)
{
	switch( _Type )
	{
	case TW_TYPE_BOOLCPP:
		return sizeof(bool);
	case TW_TYPE_BOOL8:
	case TW_TYPE_CHAR:
	case TW_TYPE_INT8:
	case TW_TYPE_UINT8:
	//case TW_TYPE_ENUM8:
		return 1;
	case TW_TYPE_BOOL16:
	case TW_TYPE_INT16:
	case TW_TYPE_UINT16:
	//case TW_TYPE_ENUM16:
		return 2;
	case TW_TYPE_BOOL32:
	case TW_TYPE_INT32:
	case TW_TYPE_UINT32:
	case TW_TYPE_FLOAT:
	//case TW_TYPE_ENUM32:
		return 4;
	case TW_TYPE_DOUBLE:
		return 8;
	default:
		if( g_TwMgr && _Type>=TW_TYPE_STRUCT_BASE && _Type<TW_TYPE_STRUCT_BASE+(int)g_TwMgr->m_Structs.size() )
		{
			const CTwMgr::CStruct& s = g_TwMgr->m_Structs[_Type-TW_TYPE_STRUCT_BASE];
			return s.m_Size;
			/*
			size_t size = 0;
			for( size_t i=0; i<s.m_Members.size(); ++i )
				size += s.m_Members[i].m_Size;
			return size;
			*/
		}
		else if( g_TwMgr && _Type>=TW_TYPE_ENUM_BASE && _Type<TW_TYPE_ENUM_BASE+(int)g_TwMgr->m_Enums.size() )
			return 4;
		else	// includes TW_TYPE_BUTTON
			return 0;
	}
}

//	---------------------------------------------------------------------------

CTwBar::CTwBar(const char *_Name)
{
	assert(g_TwMgr!=NULL && g_TwMgr->m_Graph!=NULL);

	m_Name = _Name;
	m_Visible = true;
	m_VarRoot.m_IsRoot = true;
	m_VarRoot.m_Open = true;
	m_VarRoot.m_SummaryCallback = NULL;
	m_VarRoot.m_SummaryClientData = NULL;
	m_VarRoot.m_StructValuePtr = NULL;

	m_UpToDate = false;
	int n = (int)g_TwMgr->m_Bars.size();
	m_PosX = 24*n-8;
	m_PosY = 24*n-8;
	m_Width = 200;
	m_Height = 320;
	int cr, cg, cb;
	ColorHLSToRGBi(g_TwMgr->m_BarInitColorHue%256, 127, 255, &cr, &cg, &cb);
	g_TwMgr->m_BarInitColorHue -= 16;
	if( g_TwMgr->m_BarInitColorHue<0 )
		g_TwMgr->m_BarInitColorHue += 256;
	m_Color = Color32FromARGBi(0xf0, cr, cg, cb);
	//m_Color = 0xd7008f8f;
	m_Font = g_TwMgr->m_CurrentFont;
	//m_Font = g_DefaultNormalFont;
	//m_Font = g_DefaultSmallFont;
	//m_Font = g_DefaultLargeFont;
	m_TitleWidth = 0;
	m_Sep = 1;
	m_ValuesWidth = 10*(m_Font->m_CharHeight/2); // about 10 characters
	m_NbHierLines = 0;
	m_NbDisplayedLines = 0;
	m_FirstLine = 0;
	m_LastUpdateTime = 0;
	m_UpdatePeriod = 2;
	m_ScrollYW = 0;
	m_ScrollYH = 0;
	m_ScrollY0 = 0;
	m_ScrollY1 = 0;

	m_DrawHandles = false;
	m_DrawIncrDecrBtn = false;
	m_DrawClickBtn = false;
	m_MouseDrag = false;
	m_MouseDragVar = false;
	m_MouseDragTitle = false;
	m_MouseDragScroll = false;
	m_MouseDragResizeUR = false;
	m_MouseDragResizeUL = false;
	m_MouseDragResizeLR = false;
	m_MouseDragResizeLL = false;
	m_MouseDragValWidth = false;
	m_MouseOriginX = 0;
	m_MouseOriginY = 0;
	m_VarHasBeenIncr = true;
	m_FirstLine0 = 0;
	m_HighlightedLine = -1;
	m_HighlightedLinePrev = -1;
	m_HighlightIncrBtn = false;
	m_HighlightDecrBtn = false;
	m_HighlightClickBtn = false;
	m_HighlightTitle = false;
	m_HighlightScroll = false;
	m_HighlightUpScroll = false;
	m_HighlightDnScroll = false;
	m_HighlightMinimize = false;
	m_HighlightFont = false;
	m_HighlightValWidth = false;

	m_IsMinimized = false;
	m_MinNumber = 0;
	m_MinPosX = 0;
	m_MinPosY = 0;
	m_HighlightMaximize = false;
	m_IsHelpBar = false;
	m_IsPopupList = false;
	m_VarEnumLinkedToPopupList = NULL;
	m_BarLinkedToPopupList = NULL;

	m_TitleTextObj = g_TwMgr->m_Graph->NewTextObj();
	m_LabelsTextObj = g_TwMgr->m_Graph->NewTextObj();
	m_ValuesTextObj = g_TwMgr->m_Graph->NewTextObj();
	m_ShortcutTextObj = g_TwMgr->m_Graph->NewTextObj();
	m_ShortcutLine = -1;

	m_RotoMinRadius = 24;
	m_RotoNbSubdiv = 256;	// number of steps for one turn

	UpdateColors();
	NotUpToDate();
}

//	---------------------------------------------------------------------------

CTwBar::~CTwBar()
{
	if( m_IsMinimized )
		g_TwMgr->Maximize(this);
	if( m_TitleTextObj )
		g_TwMgr->m_Graph->DeleteTextObj(m_TitleTextObj);
	if( m_LabelsTextObj )
		g_TwMgr->m_Graph->DeleteTextObj(m_LabelsTextObj);
	if( m_ValuesTextObj )
		g_TwMgr->m_Graph->DeleteTextObj(m_ValuesTextObj);
	if( m_ShortcutTextObj )
		g_TwMgr->m_Graph->DeleteTextObj(m_ShortcutTextObj);
}

//	---------------------------------------------------------------------------

const CTwVar *CTwBar::Find(const char *_Name, CTwVarGroup **_Parent, int *_Index) const
{
	return m_VarRoot.Find(_Name, _Parent, _Index);
}

CTwVar *CTwBar::Find(const char *_Name, CTwVarGroup **_Parent, int *_Index)
{
	return const_cast<CTwVar *>(const_cast<const CTwBar *>(this)->Find(_Name, _Parent, _Index));
}

//	---------------------------------------------------------------------------

enum EBarAttribs
{
	BAR_LABEL = 1,
	BAR_HELP,
	BAR_COLOR,
	BAR_SHOW,
	BAR_HIDE,
	BAR_ICONIFY,
	BAR_SIZE,
	BAR_POSITION,
	BAR_REFRESH,
	BAR_FONT_SIZE,
	BAR_VALUES_WIDTH,
};

int CTwBar::HasAttrib(const char *_Attrib, bool *_HasValue) const
{
	*_HasValue = true;
	if( _stricmp(_Attrib, "label")==0 )
		return BAR_LABEL;
	else if( _stricmp(_Attrib, "help")==0 )
		return BAR_HELP;
	else if( _stricmp(_Attrib, "color")==0 )
		return BAR_COLOR;
	else if( _stricmp(_Attrib, "size")==0 )
		return BAR_SIZE;
	else if( _stricmp(_Attrib, "position")==0 )
		return BAR_POSITION;
	else if( _stricmp(_Attrib, "refresh")==0 )
		return BAR_REFRESH;
	else if( _stricmp(_Attrib, "fontsize")==0 )
		return BAR_FONT_SIZE;
	else if( _stricmp(_Attrib, "valueswidth")==0 )
		return BAR_VALUES_WIDTH;

	*_HasValue = false;
	if( _stricmp(_Attrib, "show")==0 )
		return BAR_SHOW;
	else if( _stricmp(_Attrib, "hide")==0 )
		return BAR_HIDE;
	else if( _stricmp(_Attrib, "iconify")==0 )
		return BAR_ICONIFY;

	return 0; // not found
}

int CTwBar::SetAttrib(int _AttribID, const char *_Value)
{
	switch( _AttribID )
	{
	case BAR_LABEL:
		if( _Value && strlen(_Value)>0 )
		{
			m_Label = _Value;
			NotUpToDate();
			return 1;
		}
		else
		{
			g_TwMgr->SetLastError(g_ErrNoValue);
			return 0;
		}
	case BAR_HELP:
		if( _Value && strlen(_Value)>0 )
		{
			m_Help = _Value;
			NotUpToDate();
			return 1;
		}
		else
		{
			g_TwMgr->SetLastError(g_ErrNoValue);
			return 0;
		}
	case BAR_COLOR:
		if( _Value && strlen(_Value)>0 )
		{
			int v0, v1, v2, v3;
			int n = sscanf(_Value, "%d%d%d%d", &v0, &v1, &v2, &v3);
			color32 c;
			if( n==3 )
				c = Color32FromARGBi(255, v0, v1, v2);
			else if( n==4 )
				c = Color32FromARGBi(v0, v1, v2, v3);
			else
			{
				g_TwMgr->SetLastError(g_ErrBadValue);
				return 0;
			}
			m_Color = c;
			NotUpToDate();
			return 1;
		}
		else
		{
			g_TwMgr->SetLastError(g_ErrNoValue);
			return 0;
		}
	case BAR_SIZE:
		if( _Value && strlen(_Value)>0 )
		{
			int sx, sy;
			int n = sscanf(_Value, "%d%d", &sx, &sy);
			if( n==2 && sx>0 && sy>0 )
			{
				m_Width = sx;
				m_Height = sy;
				NotUpToDate();
				return 1;
			}
			else
			{
				g_TwMgr->SetLastError(g_ErrBadValue);
				return 0;
			}
		}
		else
		{
			g_TwMgr->SetLastError(g_ErrNoValue);
			return 0;
		}
	case BAR_POSITION:
		if( _Value && strlen(_Value)>0 )
		{
			int x, y;
			int n = sscanf(_Value, "%d%d", &x, &y);
			if( n==2 && x>=0 && y>=0 )
			{
				m_PosX = x;
				m_PosY = y;
				NotUpToDate();
				return 1;
			}
			else
			{
				g_TwMgr->SetLastError(g_ErrBadValue);
				return 0;
			}
		}
		else
		{
			g_TwMgr->SetLastError(g_ErrNoValue);
			return 0;
		}
	case BAR_REFRESH:
		if( _Value && strlen(_Value)>0 )
		{
			float r;
			int n = sscanf(_Value, "%f", &r);
			if( n==1 && r>=0 )
			{
				m_UpdatePeriod = r;
				return 1;
			}
			else
			{
				g_TwMgr->SetLastError(g_ErrBadValue);
				return 0;
			}
		}
		else
		{
			g_TwMgr->SetLastError(g_ErrNoValue);
			return 0;
		}
	case BAR_FONT_SIZE:
		if( _Value && strlen(_Value)>0 )
		{
			int s;
			int n = sscanf(_Value, "%d", &s);
			if( n==1 && s>=1 && s<=3 )
			{
				if( s==1 )
					g_TwMgr->SetFont(g_DefaultSmallFont, true);
				else if( s==2 )
					g_TwMgr->SetFont(g_DefaultNormalFont, true);
				else if( s==3 )
					g_TwMgr->SetFont(g_DefaultLargeFont, true);
				return 1;
			}
			else
			{
				g_TwMgr->SetLastError(g_ErrBadValue);
				return 0;
			}
		}
		else
		{
			g_TwMgr->SetLastError(g_ErrNoValue);
			return 0;
		}
	case BAR_VALUES_WIDTH:
		if( _Value && strlen(_Value)>0 )
		{
			int w;
			int n = sscanf(_Value, "%d", &w);
			if( n==1 && w>0 )
			{
				m_ValuesWidth = w;
				NotUpToDate();
				return 1;
			}
			else
			{
				g_TwMgr->SetLastError(g_ErrBadValue);
				return 0;
			}
		}
		else
		{
			g_TwMgr->SetLastError(g_ErrNoValue);
			return 0;
		}
	case BAR_SHOW:
		TwSetBarState(this, TW_STATE_SHOWN);
		return 1;
	case BAR_HIDE:
		TwSetBarState(this, TW_STATE_HIDDEN);
		return 1;
	case BAR_ICONIFY:
		TwSetBarState(this, TW_STATE_ICONIFIED);
		return 1;
	default:
		g_TwMgr->SetLastError(g_ErrUnknownAttrib);
		return 0;
	}
}

//	---------------------------------------------------------------------------

void CTwBar::NotUpToDate()
{
	m_UpToDate = false;
}

//	---------------------------------------------------------------------------

void CTwBar::UpdateColors()
{
	float a, r, g, b, h, l, s;
	Color32ToARGBf(m_Color, &a, &r, &g, &b);
	ColorRGBToHLSf(r, g, b, &h, &l, &s);
	if( s>0.8f )
		s = 0.8f;
	
	ColorHLSToRGBf(h, 0.78f, s, &r, &g, &b);
	m_ColBg = Color32FromARGBf(a, r, g, b);
	ColorHLSToRGBf(h, 0.70f, s, &r, &g, &b);
	m_ColBg1 = Color32FromARGBf(a, r, g, b);
	ColorHLSToRGBf(h, 0.62f, s, &r, &g, &b);
	m_ColBg2 = Color32FromARGBf(a, r, g, b);

	ColorHLSToRGBf(h, 0.8f, s, &r, &g, &b);
	m_ColHighBg = Color32FromARGBf(a, r, g, b);
	//m_ColHighBg = Color32FromARGBf(a, 0.95f, 0.95f, 0.2f);
	
	m_ColLabelText = COLOR32_BLACK;
	m_ColStructText = 0xff005000;

	ColorHLSToRGBf(h, 0.9f, s, &r, &g, &b);
	m_ColValBg = Color32FromARGBf(a, r, g, b);
	m_ColStructBg = Color32FromARGBf(0.7f*a, r, g, b);

	//m_ColValText = COLOR32_BLACK;
	m_ColValText = 0xff000080;
	m_ColValTextRO = 0xff505050;
	//m_ColValMin = 0xff800000;
	//m_ColValMax = 0xff800000;
	m_ColValMin = 0xff0000f0;
	m_ColValMax = 0xff0000f0;

	ColorHLSToRGBf(h, 0.4f, s, &r, &g, &b);
	m_ColTitleBg = Color32FromARGBf(a, r, g, b);
	m_ColTitleText = COLOR32_WHITE;
	m_ColTitleShadow = COLOR32_BLACK;
	ColorHLSToRGBf(h, 0.2f, s, &r, &g, &b);
	m_ColTitleHighBg = Color32FromARGBf(a, r, g, b);

	ColorHLSToRGBf(h, 0.8f, s, &r, &g, &b);
	m_ColLine = Color32FromARGBf(1, r, g, b); // 0xfff0f0f0;
	m_ColLineShadow = COLOR32_BLACK;
	m_ColUnderline = 0xff202000;
	ColorHLSToRGBf(h, 0.3f, s, &r, &g, &b);
	m_ColBtn = Color32FromARGBf(0.6f*a, r, g, b);
	ColorHLSToRGBf(h, 0.1f, s, &r, &g, &b);
	m_ColHighBtn = Color32FromARGBf(0.6f*a, r, g, b);

	ColorHLSToRGBf(h, 0.4f, s, &r, &g, &b);
	m_ColGrpBg = Color32FromARGBf(a, r, g, b);
	m_ColGrpText = COLOR32_WHITE;

	ColorHLSToRGBf(h, 0.75f, s, &r, &g, &b);
	m_ColHelpBg = Color32FromARGBf(a, r, g, b);
	m_ColHelpText = Color32FromARGBf(1, 0, 0.4f, 0);

	ColorHLSToRGBf(h, 0.45f, s, &r, &g, &b);
	m_ColHierBg = Color32FromARGBf(0.75f*a, r, g, b);

	m_ColShortcutText = 0xfff0f0f0;
	m_ColShortcutBg = Color32FromARGBf(0.5f*a, 0.1f, 0.1f, 0.1f);
	m_ColInfoText = Color32FromARGBf(1.0f, 0.7f, 0.7f, 0.7f);

	m_ColRoto = Color32FromARGBf(1, 0.75f, 0.75f, 0.75f);
	m_ColRotoVal = Color32FromARGBf(1, 1.0f, 0.2f, 0.2f);
	m_ColRotoBound = Color32FromARGBf(1, 0.4f, 0.4f, 0.4f);
}

//	---------------------------------------------------------------------------

CTwVarGroup::~CTwVarGroup()
{
	for( vector<CTwVar*>::iterator it= m_Vars.begin(); it!=m_Vars.end(); ++it )
		if( *it != NULL )
		{
			CTwVar *Var = *it;
			delete Var;
			*it = NULL;
		}
}

//	---------------------------------------------------------------------------

static inline int IncrBtnWidth(int _CharHeight) 
{ 
	return ((2*_CharHeight)/3+2)&0xfffe; // force even value 
}

//	---------------------------------------------------------------------------

void CTwBar::BrowseHierarchy(int *_CurrLine, int _CurrLevel, const CTwVar *_Var, int _First, int _Last)
{
	assert(_Var!=NULL);
	if( !_Var->m_IsRoot )
	{
		if( (*_CurrLine)>=_First && (*_CurrLine)<=_Last )
		{
			CHierTag Tag;
			Tag.m_Level = _CurrLevel;
			Tag.m_Var = const_cast<CTwVar *>(_Var);
			Tag.m_Closing = false;
			m_HierTags.push_back(Tag);
		}
		*_CurrLine += 1;
	}
	else
	{
		*_CurrLine = 0;
		_CurrLevel = -1;
		m_HierTags.resize(0);
	}

	if( _Var->IsGroup() )
	{
		const CTwVarGroup *Grp = static_cast<const CTwVarGroup *>(_Var);
		if( Grp->m_Open )
			for( vector<CTwVar*>::const_iterator it=Grp->m_Vars.begin(); it!=Grp->m_Vars.end(); ++it )
				if( (*it)->m_Visible )
					BrowseHierarchy(_CurrLine, _CurrLevel+1, *it, _First, _Last);
		if( m_HierTags.size()>0 )
			m_HierTags[m_HierTags.size()-1].m_Closing = true;
	}
}

//	---------------------------------------------------------------------------

void CTwBar::ListLabels(vector<string>& _Labels, vector<color32>& _Colors, const CTexFont *_Font, int _AtomWidthMax, int _GroupWidthMax)
{
	const int NbEtc = 2;
	string ValStr;
	int Len, i, x, Etc;
	const unsigned char *Text;
	unsigned char ch;
	int WidthMax;

	int nh = (int)m_HierTags.size();
	for( int h=0; h<nh; ++h )
	{
		Len = (int)m_HierTags[h].m_Var->m_Label.length();
		if( Len>0 )
			Text = (const unsigned char *)(m_HierTags[h].m_Var->m_Label.c_str());
		else
		{
			Text = (const unsigned char *)(m_HierTags[h].m_Var->m_Name.c_str());
			Len = (int)m_HierTags[h].m_Var->m_Name.length();
		}
		x = 0;
		Etc = 0;
		_Labels.push_back("");	// add a new text line
		_Colors.push_back(m_HierTags[h].m_Var->m_Color);
		string& CurrentLabel = _Labels[_Labels.size()-1];
		if( m_HierTags[h].m_Var->IsGroup() && static_cast<const CTwVarGroup *>(m_HierTags[h].m_Var)->m_SummaryCallback==NULL )
			WidthMax = _GroupWidthMax;
		else if( !m_HierTags[h].m_Var->IsGroup() && static_cast<const CTwVarAtom *>(m_HierTags[h].m_Var)->m_Type==TW_TYPE_BUTTON )
			WidthMax = _GroupWidthMax - 2*IncrBtnWidth(m_Font->m_CharHeight);
		else
			WidthMax = _AtomWidthMax;
		for( i=0; i<m_HierTags[h].m_Level; ++i )
		{
			CurrentLabel += ' ';
			CurrentLabel += ' ';
			x += 2*_Font->m_CharWidth[(int)' '];
		}
		for( i=0; i<Len; ++i )
		{
			ch = (Etc==0) ? Text[i] : '.';
			CurrentLabel += ch;
			x += _Font->m_CharWidth[(int)ch];
			if( Etc>0 )
			{
				++Etc;
				if( Etc>NbEtc )
					break;
			}
			else if( i<Len-2 && x+(NbEtc+2)*_Font->m_CharWidth[(int)'.']>=WidthMax && !(m_HierTags[h].m_Var->m_DontClip))
				Etc = 1;
		}		
	}
}

//	---------------------------------------------------------------------------

void CTwBar::ListValues(vector<string>& _Values, vector<color32>& _Colors, vector<color32>& _BgColors, const CTexFont *_Font, int _WidthMax)
{
	CTwFPU fpu;	// force fpu precision

	const int NbEtc = 2;
	const CTwVarAtom *Atom = NULL;
	string ValStr;
	int Len, i, x, Etc;
	const unsigned char *Text;
	unsigned char ch;
	bool ReadOnly;
	bool IsMax;
	bool IsMin;
	size_t SummaryMaxLength = max(_WidthMax/_Font->m_CharWidth[(int)'I'], 4);
	static vector<char> Summary;
	Summary.resize(SummaryMaxLength+32);

	int nh = (int)m_HierTags.size();
	for( int h=0; h<nh; ++h )
		if( !m_HierTags[h].m_Var->IsGroup() || m_IsHelpBar 
			|| (m_HierTags[h].m_Var->IsGroup() && static_cast<const CTwVarGroup *>(m_HierTags[h].m_Var)->m_SummaryCallback!=NULL) )
		{
			ReadOnly = true;
			IsMax = false;
			IsMin = false;
			if( !m_HierTags[h].m_Var->IsGroup() )
			{
				Atom = static_cast<const CTwVarAtom *>(m_HierTags[h].m_Var);
				Atom->ValueToString(&ValStr);
				if( !m_IsHelpBar || (Atom->m_Type==TW_TYPE_SHORTCUT && (Atom->m_Val.m_Shortcut.m_Incr[0]>0 || Atom->m_Val.m_Shortcut.m_Decr[0]>0)) )
					ReadOnly = Atom->m_ReadOnly;
				if( !Atom->m_NoSlider )
				{
					double v, vmin, vmax;
					v = Atom->ValueToDouble();
					Atom->MinMaxStepToDouble(&vmin, &vmax, NULL);
					IsMax = (v>=vmax);
					IsMin = (v<=vmin);
				}
			}
			else if(m_HierTags[h].m_Var->IsGroup() && static_cast<const CTwVarGroup *>(m_HierTags[h].m_Var)->m_SummaryCallback!=NULL)
			{
				const CTwVarGroup *Grp = static_cast<const CTwVarGroup *>(m_HierTags[h].m_Var);
				Summary[0] = '\0';
				if( Grp->m_SummaryCallback==CTwMgr::CStruct::DefaultSummary )
					Grp->m_SummaryCallback(&Summary[0], SummaryMaxLength, Grp, Grp->m_SummaryClientData);
				else
					Grp->m_SummaryCallback(&Summary[0], SummaryMaxLength, Grp->m_StructValuePtr, Grp->m_SummaryClientData);
				ValStr = (const char *)(&Summary[0]);
			}
			else
				ValStr = "";	// is a group in the help bar
			Len = (int)ValStr.length();
			Text = (const unsigned char *)(ValStr.c_str());
			x = 0;
			Etc = 0;
			_Values.push_back("");	// add a new text line
			if( ReadOnly || (IsMin && IsMax) )
                _Colors.push_back(m_ColValTextRO);
			else if( IsMin )
				_Colors.push_back(m_ColValMin);
			else if( IsMax )
				_Colors.push_back(m_ColValMax);
			else
				_Colors.push_back(m_ColValText);
			if( m_HierTags[h].m_Var->IsGroup() )
			{
				const CTwVarGroup *Grp = static_cast<const CTwVarGroup *>(m_HierTags[h].m_Var);
				// if typecolor set bgcolor 
				if( Grp->m_SummaryCallback==CColorExt::SummaryCB )
					_BgColors.push_back(0xff000000);
				else
					_BgColors.push_back(m_ColStructBg);
			}
			else
				_BgColors.push_back(m_ColValBg);

			string& CurrentValue = _Values[_Values.size()-1];
			int wmax = _WidthMax;
			if( m_HighlightedLine==h && m_DrawIncrDecrBtn )
				wmax -= 2*IncrBtnWidth(m_Font->m_CharHeight);
			for( i=0; i<Len; ++i )
			{
				ch = (Etc==0) ? Text[i] : '.';
				CurrentValue += ch;
				x += _Font->m_CharWidth[(int)ch];
				if( Etc>0 )
				{
					++Etc;
					if( Etc>NbEtc )
						break;
				}
				else if( i<Len-2 && x+(NbEtc+2)*_Font->m_CharWidth[(int)'.']>=wmax )
					Etc = 1;
			}
		}
		else
		{
			_Values.push_back("");	// add a new empty line
			_Colors.push_back(COLOR32_BLACK);
			_BgColors.push_back(0x00000000);
		}
}

//	---------------------------------------------------------------------------

static int ClampText(string& _Text, const CTexFont *_Font, int _WidthMax)
{
	int Len = (int)_Text.length();
	unsigned char ch;
	int Width = 0;
	int i;
	for( i=0; i<Len; ++i )
	{
		ch = _Text.at(i);
		if( i<Len-1 && Width+_Font->m_CharWidth[(int)'.']>=_WidthMax )
			break;
		Width += _Font->m_CharWidth[ch];
	}
	if( i<Len ) // clamp
	{
		_Text.resize(i+2);
		_Text.at(i+0) = '.';
		_Text.at(i+1) = '.';
		Width += 2*_Font->m_CharWidth[(int)'.'];
	}
	return Width;
}

//	---------------------------------------------------------------------------

void CTwBar::Update()
{
	assert(m_UpToDate==false);
	assert(m_Font);
	ITwGraph *Gr = g_TwMgr->m_Graph;

	bool DoEndDraw = false;
	if( !Gr->IsDrawing() )
	{
		Gr->BeginDraw(g_TwMgr->m_WndWidth, g_TwMgr->m_WndHeight);
		DoEndDraw = true;
	}

	int PrevPosY = m_PosY;
	int vpx, vpy, vpw, vph;
	vpx = 0;
	vpy = 0;
	vpw = g_TwMgr->m_WndWidth;
	vph = g_TwMgr->m_WndHeight;
	if( !m_IsMinimized && vpw>0 && vph>0 )
	{
		bool Modif = false;
		if( m_Width>vpw )
		{
			m_Width = vpw;
			Modif = true;
		}
		if( m_Width<8*m_Font->m_CharHeight )
		{
			m_Width = 8*m_Font->m_CharHeight;
			Modif = true;
		}
		if( m_Height>vph )
		{
			m_Height = vph;
			Modif = true;
		}
		if( m_Height<5*m_Font->m_CharHeight )
		{
			m_Height = 5*m_Font->m_CharHeight;
			Modif = true;
		}
		if( m_PosX+m_Width>vpx+vpw )
			m_PosX = vpx+vpw-m_Width;
		if( m_PosX<vpx )
			m_PosX = vpx;
		if( m_PosY+m_Height>vpy+vph )
			m_PosY = vpy+vph-m_Height;
		if( m_PosY<vpy )
			m_PosY = vpy;
		m_ScrollY0 += m_PosY-PrevPosY;
		m_ScrollY1 += m_PosY-PrevPosY;
		if( m_ValuesWidth<2*m_Font->m_CharHeight )
		{
			m_ValuesWidth = 2*m_Font->m_CharHeight;
			Modif = true;
		}
		if( m_ValuesWidth>m_Width-4*m_Font->m_CharHeight )
		{
			m_ValuesWidth = m_Width-4*m_Font->m_CharHeight;
			Modif = true;
		}
		if( Modif && m_IsHelpBar )
		{
			g_TwMgr->m_HelpBarNotUpToDate = true;
			g_TwMgr->m_KeyPressedBuildText = true;
			g_TwMgr->m_InfoBuildText = true;
		}
	}

	UpdateColors();

	// update geometry relatively to (m_PosX, m_PosY)
	if( !m_IsPopupList )
	{
		//m_VarX0 = 2*m_Font->m_CharHeight+m_Sep;
		m_VarX0 = m_Font->m_CharHeight+m_Sep;
		//m_VarX2 = m_Width - 4;
		m_VarX2 = m_Width - m_Font->m_CharHeight - m_Sep-2;
		m_VarX1 = m_VarX2 - m_ValuesWidth;
	}
	else
	{
		//m_VarX0 = m_Font->m_CharHeight+6+m_Sep;
		m_VarX0 = 2;
		//m_VarX2 = m_Width - 4;
		m_VarX2 = m_Width - m_Font->m_CharHeight - m_Sep-2;
		m_VarX1 = m_VarX2;
	}
	if( m_VarX1<m_VarX0+32 )
		m_VarX1 = m_VarX0+32;
	if( m_VarX1>m_VarX2 )
		m_VarX1 = m_VarX2;
	if( !m_IsPopupList )
	{
		m_VarY0 = m_Font->m_CharHeight+2+m_Sep+6;
		m_VarY1 = m_Height-m_Font->m_CharHeight-2-m_Sep;
		m_VarY2 = m_Height-1;
	}
	else
	{
		m_VarY0 = 4;
		m_VarY1 = m_Height-2-m_Sep;
		m_VarY2 = m_Height-1;
	}

	int NbLines = (m_VarY1-m_VarY0+1)/(m_Font->m_CharHeight+m_Sep);
	if( NbLines<= 0 )
		NbLines = 1;
	if( !m_IsMinimized )
	{
		int LineNum = 0;
		BrowseHierarchy(&LineNum, 0, &m_VarRoot, m_FirstLine, m_FirstLine+NbLines); // add a dummy tag at the end to avoid wrong 'tag-closing' problems
		if( (int)m_HierTags.size()>NbLines )
			m_HierTags.resize(NbLines); // remove the last dummy tag
		m_NbHierLines = LineNum;
		m_NbDisplayedLines = (int)m_HierTags.size();
	}

	// scroll bar
	int y0 = m_PosY+m_VarY0;
	int y1 = m_PosY+m_VarY1;
	int x0 = m_PosX+2;
	int x1 = m_PosX+m_Font->m_CharHeight-2;
	if( ((x0+x1)&1)==1 )
		x1 += 1;
	int w  = x1-x0+1;
	int h  = y1-y0-2*w;
	int hscr = (m_NbHierLines>0) ? ((h*m_NbDisplayedLines)/m_NbHierLines) : h;
	if( hscr<=4 )
		hscr = 4;
	if( hscr>h )
		hscr = h;
	int yscr = (m_NbHierLines>0) ? ((h*m_FirstLine)/m_NbHierLines) : 0;
	if( yscr<=0 )
		yscr = 0;
	if( yscr>h-4 )
		yscr = h-4;
	if( yscr+hscr>h )
		hscr = h-yscr;
	if( hscr>h )
		hscr = h;
	if( hscr<=4 )
		hscr = 4;
	m_ScrollYW = w;
	m_ScrollYH = h;
	m_ScrollY0 = y0+w+yscr;
	m_ScrollY1 = y0+w+yscr+hscr;

	// Build title
	string Title;
	if( m_Label.size()>0 )
		Title = m_Label;
	else
		Title = m_Name;
	m_TitleWidth = ClampText(Title, m_Font, (!m_IsMinimized)?(m_Width-5*m_Font->m_CharHeight):(16*m_Font->m_CharHeight));
	Gr->BuildText(m_TitleTextObj, &Title, NULL, NULL, 1, m_Font, 0, 0);

	if( !m_IsMinimized )
	{
		// Build labels
		vector<string>  Labels;
		vector<color32> Colors;
		ListLabels(Labels, Colors, m_Font, m_VarX1-m_VarX0, m_VarX2-m_VarX0);
		Gr->BuildText(m_LabelsTextObj, &(Labels[0]), &(Colors[0]), NULL, (int)Labels.size(), m_Font, 1, 0);

		// Should draw click button?
		m_DrawClickBtn    = ( m_VarX2-m_VarX1>4*IncrBtnWidth(m_Font->m_CharHeight)
							  && m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size()
							  && m_HierTags[m_HighlightedLine].m_Var!=NULL 
							  && !m_HierTags[m_HighlightedLine].m_Var->IsGroup()
							  && !static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_ReadOnly
							  && (    static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type==TW_TYPE_BUTTON ));
							//	   || static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type==TW_TYPE_BOOLCPP
							//	   || static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type==TW_TYPE_BOOL8
							//	   || static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type==TW_TYPE_BOOL16
							//	   || static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type==TW_TYPE_BOOL32 ));

		// Should draw [-/+] button?
		m_DrawIncrDecrBtn = ( m_VarX2-m_VarX1>4*IncrBtnWidth(m_Font->m_CharHeight)
							  && m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size()
							  && m_HierTags[m_HighlightedLine].m_Var!=NULL 
							  && !m_HierTags[m_HighlightedLine].m_Var->IsGroup()
							  && static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type!=TW_TYPE_BUTTON
							  && !static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_ReadOnly
							  && !static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_NoSlider );

		// Build values
		vector<string>& Values = Labels;	// reuse
		Values.resize(0);
		Colors.resize(0);
		vector<color32> BgColors;
		ListValues(Values, Colors, BgColors, m_Font, m_VarX2-m_VarX1);
		assert(BgColors.size()==Values.size());
		Gr->BuildText(m_ValuesTextObj, &(Values[0]), &(Colors[0]), &(BgColors[0]), (int)Values.size(), m_Font, 1, m_VarX2-m_VarX1);

		// Build key shortcut text
		string Shortcut;
		m_ShortcutLine = -1;
		if( m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size() && m_HierTags[m_HighlightedLine].m_Var!=NULL && !m_HierTags[m_HighlightedLine].m_Var->IsGroup() )
		{
			const CTwVarAtom *Atom = static_cast<const CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var);
			if( Atom->m_KeyIncr[0]>0 || Atom->m_KeyDecr[0]>0 )
			{
				if( Atom->m_KeyIncr[0]>0 && Atom->m_KeyDecr[0]>0 )
					Shortcut = "Keys: ";
				else
					Shortcut = "Key: ";
				if( Atom->m_KeyIncr[0]>0 )
					TwGetKeyString(&Shortcut, Atom->m_KeyIncr[0], Atom->m_KeyIncr[1]);
				else
					Shortcut += "(none)";
				if( Atom->m_KeyDecr[0]>0 )
				{
					Shortcut += "  ";
					TwGetKeyString(&Shortcut, Atom->m_KeyDecr[0], Atom->m_KeyDecr[1]);
				}
				m_ShortcutLine = m_HighlightedLine;
			}
		}
		ClampText(Shortcut, m_Font, m_Width-3*m_Font->m_CharHeight);
		Gr->BuildText(m_ShortcutTextObj, &Shortcut, NULL, NULL, 1, m_Font, 0, 0);
	}

	if( DoEndDraw )
		Gr->EndDraw();

	m_UpToDate = true;
	m_LastUpdateTime = float(g_BarTimer.GetTime());
}

//	---------------------------------------------------------------------------

void CTwBar::DrawHierHandle()
{
	assert(m_Font);
	ITwGraph *Gr = g_TwMgr->m_Graph;

	//int x0 = m_PosX+m_Font->m_CharHeight+1;
	int x0 = m_PosX+2;
	int x2 = m_PosX+m_VarX0-5;
	if( (x2-x0)&1 )
		--x2;
	int x1 = (x0+x2)/2;
	int w = x2-x0+1;
	int y0 = m_PosY+m_VarY0;
	int y1;
	int dh0 = (m_Font->m_CharHeight+m_Sep-1-w)/2;
	if( dh0<0 )
		dh0 = 0;
	int dh1 = dh0+w-1;
	int i, h=0;

	if( !m_IsPopupList )
	{
		CTwVarGroup *Grp;
		int nh = (int)m_HierTags.size();
		for( h=0; h<nh; ++h )
		{
			y1 = y0 + m_Font->m_CharHeight+m_Sep-1;
			if( m_HierTags[h].m_Var->IsGroup() )
				Grp = static_cast<CTwVarGroup *>(m_HierTags[h].m_Var);
			else
				Grp = NULL;

			if( Grp )
			{
				Gr->DrawLine(x2+1,y0+dh0+1,x2+1,y0+dh1+1, m_ColLineShadow);
				Gr->DrawLine(x0+1,y0+dh1+1,x2+2,y0+dh1+1, m_ColLineShadow);
				if( Grp->m_Open )
				{
					Gr->DrawLine(x1,y0+dh1, x1,y1+1, m_ColLine);
					Gr->DrawLine(x1+1,y0+dh1, x1+1,y1+1, m_ColLineShadow);
				}
			}

			if( m_HierTags[h].m_Level>0 )
			{
				if( Grp!=NULL )
				{
					Gr->DrawLine(x1,y0, x1,y0+dh0, m_ColLine);
					Gr->DrawLine(x1+1,y0, x1+1,y0+dh0, m_ColLineShadow);
				}

				if( !m_HierTags[h].m_Closing && Grp==NULL )
				{
					Gr->DrawLine(x1,y0, x1,y1+1, m_ColLine);
					Gr->DrawLine(x1+1,y0, x1+1,y1+1, m_ColLineShadow);
				}

				if( m_HierTags[h].m_Closing && Grp==NULL )
				{
					Gr->DrawLine(x1,y0+dh0+w/2, x2+2,y0+dh0+w/2, m_ColLine);
					Gr->DrawLine(x1+1,y0+dh0+w/2+1, x2+3,y0+dh0+w/2+1, m_ColLineShadow);
					Gr->DrawLine(x1,y0, x1,y0+dh0+w/2, m_ColLine);
					Gr->DrawLine(x1+1,y0, x1+1,y0+dh0+w/2, m_ColLineShadow);
					if( m_HierTags[h].m_Level!=1 && h+1<nh && m_HierTags[h+1].m_Level!=0 )
					{
						Gr->DrawLine(x1,y0+dh0+w/2+2, x1,y1+1, m_ColLine);
						Gr->DrawLine(x1+1,y0+dh0+w/2+3, x1+1,y1+1, m_ColLineShadow);
					}
				}
			}

			if( Grp )
			{
				//Gr->DrawRect(x0+1,y0+dh0+1,x2-1,y0+dh1-1, (h==m_HighlightedLine) ? m_ColHighBtn : m_ColBtn);
				Gr->DrawRect(x0,y0+dh0,x2,y0+dh1, (h==m_HighlightedLine) ? m_ColHighBtn : m_ColBtn);
				if( m_DrawHandles )
				{
					Gr->DrawLine(x0,y0+dh0,x2,y0+dh0, m_ColLine);
					Gr->DrawLine(x2,y0+dh0,x2,y0+dh1+1, m_ColLine);
					Gr->DrawLine(x2,y0+dh1,x0,y0+dh1, m_ColLine);
					Gr->DrawLine(x0,y0+dh1,x0,y0+dh0, m_ColLine);
				}
				
				Gr->DrawLine(x0+2,y0+dh0+w/2, x2-1,y0+dh0+w/2, m_ColTitleText);
				if( !Grp->m_Open )
					Gr->DrawLine(x1,y0+dh0+2, x1,y0+dh1-1, m_ColTitleText);

				if( m_ColGrpBg!=0 && Grp->m_StructValuePtr==NULL )
				{
					color32 cb = (Grp->m_StructType==TW_TYPE_HELP_STRUCT) ? m_ColStructBg : m_ColGrpBg;
					//int decal = m_Font->m_CharHeight/2-2+2*m_HierTags[h].m_Level;
					//if( decal>m_Font->m_CharHeight-3 )
					//	decal = m_Font->m_CharHeight-3;
					int margin = m_Font->m_CharWidth[(int)' ']*m_HierTags[h].m_Level;
					//Gr->DrawRect(m_PosX+m_VarX0+margin, y0+decal, m_PosX+m_VarX2, y0+m_Font->m_CharHeight-1, cb);
					Gr->DrawRect(m_PosX+m_VarX0+margin-1, y0+1, m_PosX+m_VarX2, y0+m_Font->m_CharHeight, cb);// m_ColHierBg);
					//Gr->DrawRect(m_PosX+m_VarX0-4, y0+m_Font->m_CharHeight/2-1, m_PosX+m_VarX0+margin-2, y0+m_Font->m_CharHeight/2, m_ColHierBg);
				}
			}
			else if( static_cast<CTwVarAtom *>(m_HierTags[h].m_Var)->m_Type==TW_TYPE_HELP_GRP && m_ColHelpBg!=0 )
				Gr->DrawRect(m_PosX+m_VarX0+m_HierTags[h].m_Var->m_LeftMargin, y0+m_HierTags[h].m_Var->m_TopMargin, m_PosX+m_VarX2, y0+m_Font->m_CharHeight-1, m_ColHelpBg);
			else if( static_cast<CTwVarAtom *>(m_HierTags[h].m_Var)->m_Type==TW_TYPE_HELP_HEADER && m_ColHelpBg!=0 )
				Gr->DrawRect(m_PosX+m_VarX0+m_HierTags[h].m_Var->m_LeftMargin, y0+m_HierTags[h].m_Var->m_TopMargin, m_PosX+m_VarX2, y0+m_Font->m_CharHeight-1, m_ColHelpBg);
			/*
			else if( static_cast<CTwVarAtom *>(m_HierTags[h].m_Var)->m_Type==TW_TYPE_BUTTON && m_ColBtn!=0 )
			{
				// draw button
				int cbx0 = m_PosX+m_VarX2-2*bw+bw/2, cby0 = y0+2, cbx1 = m_PosX+m_VarX2-2-bw/2, cby1 = y0+m_Font->m_CharHeight-4;
				if( m_HighlightClickBtn )
				{
					Gr->DrawRect(cbx0+2, cby0+2, cbx1+2, cby1+2, m_ColBtn);
					Gr->DrawLine(cbx0+3, cby1+3, cbx1+4, cby1+3, 0x7F000000);
					Gr->DrawLine(cbx1+3, cby0+3, cbx1+3, cby1+3, 0x7F000000);						
				}
				else
				{
					Gr->DrawRect(cbx0+3, cby1+1, cbx1+3, cby1+3, 0x7F000000);
					Gr->DrawRect(cbx1+1, cby0+3, cbx1+3, cby1, 0x7F000000);
					Gr->DrawRect(cbx0, cby0, cbx1, cby1, m_ColBtn);
				}
			}
			*/

			y0 = y1+1;
		}
	}

	if( m_NbDisplayedLines<m_NbHierLines )
	{
		// Draw scroll bar
		y0 = m_PosY+m_VarY0;
		y1 = m_PosY+m_VarY1;
		//x0 = m_PosX+2;
		//x1 = m_PosX+m_Font->m_CharHeight-2;
		x0 = m_PosX + m_VarX2+4;
		x1 = x0 + m_Font->m_CharHeight-4;
		if( ((x0+x1)&1)==1 )
			x1 += 1;
		w  = m_ScrollYW;
		h  = m_ScrollYH;

		Gr->DrawRect(x0+2,y0+w, x1-2,y1-1-w, (m_ColBg&0xffffff)|0x3f000000);
		if( m_DrawHandles || m_IsPopupList )
		{
			// scroll handle shadow lines
			Gr->DrawLine(x1-1,m_ScrollY0+1, x1-1,m_ScrollY1+1, m_ColLineShadow);
			Gr->DrawLine(x0+2,m_ScrollY1+1, x1,m_ScrollY1+1, m_ColLineShadow);
			
			// up & down arrow
			for( i=0; i<(x1-x0-2)/2; ++i )
			{
				Gr->DrawLine(x0+2+i,y0+w-2*i, x1-i,y0+w-2*i, m_ColLineShadow);
				Gr->DrawLine(x0+1+i,y0+w-1-2*i, x1-1-i,y0+w-1-2*i, m_HighlightUpScroll?m_ColBtn:m_ColLine);

				Gr->DrawLine(x0+2+i,y1-w+2+2*i, x1-i,y1-w+2+2*i, m_ColLineShadow);
				Gr->DrawLine(x0+1+i,y1-w+1+2*i, x1-1-i,y1-w+1+2*i, m_HighlightDnScroll?m_ColBtn:m_ColLine);
			}

			// middle lines
			Gr->DrawLine((x0+x1)/2-1,y0+w, (x0+x1)/2-1,m_ScrollY0, m_ColLine);
			Gr->DrawLine((x0+x1)/2,y0+w, (x0+x1)/2,m_ScrollY0, m_ColLine);
			Gr->DrawLine((x0+x1)/2+1,y0+w, (x0+x1)/2+1,m_ScrollY0, m_ColLineShadow);
			Gr->DrawLine((x0+x1)/2-1,m_ScrollY1, (x0+x1)/2-1,y1-w+1, m_ColLine);
			Gr->DrawLine((x0+x1)/2,m_ScrollY1, (x0+x1)/2,y1-w+1, m_ColLine);
			Gr->DrawLine((x0+x1)/2+1,m_ScrollY1, (x0+x1)/2+1,y1-w+1, m_ColLineShadow);
			// scroll handle lines
			Gr->DrawRect(x0+2,m_ScrollY0+1, x1-3,m_ScrollY1-1, m_HighlightScroll?m_ColHighBtn:m_ColBtn);
			Gr->DrawLine(x1-2,m_ScrollY0, x1-2,m_ScrollY1, m_ColLine);
			Gr->DrawLine(x0+1,m_ScrollY0, x0+1,m_ScrollY1, m_ColLine);
			Gr->DrawLine(x0+1,m_ScrollY1, x1-1,m_ScrollY1, m_ColLine);
			Gr->DrawLine(x0+1,m_ScrollY0, x1-2,m_ScrollY0, m_ColLine);
		}
		else
			Gr->DrawRect(x0+3,m_ScrollY0+1, x1-3,m_ScrollY1-1, (m_ColBtn&0xffffff)|0x3f000000);
	}

	if( m_DrawHandles && !m_IsPopupList )
	{
		// Draw resize handles
		//   lower-left
		Gr->DrawLine(m_PosX+3, m_PosY+m_Height-m_Font->m_CharHeight+3, m_PosX+3, m_PosY+m_Height-4, m_ColLine);
		Gr->DrawLine(m_PosX+4, m_PosY+m_Height-m_Font->m_CharHeight+4, m_PosX+4, m_PosY+m_Height-3, m_ColLineShadow);
		Gr->DrawLine(m_PosX+3, m_PosY+m_Height-4, m_PosX+m_Font->m_CharHeight-4, m_PosY+m_Height-4, m_ColLine);
		Gr->DrawLine(m_PosX+4, m_PosY+m_Height-3, m_PosX+m_Font->m_CharHeight-3, m_PosY+m_Height-3, m_ColLineShadow);
		//   lower-right
		Gr->DrawLine(m_PosX+m_Width-4, m_PosY+m_Height-m_Font->m_CharHeight+3, m_PosX+m_Width-4, m_PosY+m_Height-4, m_ColLine);
		Gr->DrawLine(m_PosX+m_Width-3, m_PosY+m_Height-m_Font->m_CharHeight+4, m_PosX+m_Width-3, m_PosY+m_Height-3, m_ColLineShadow);
		Gr->DrawLine(m_PosX+m_Width-4, m_PosY+m_Height-4, m_PosX+m_Width-m_Font->m_CharHeight+3, m_PosY+m_Height-4, m_ColLine);
		Gr->DrawLine(m_PosX+m_Width-3, m_PosY+m_Height-3, m_PosX+m_Width-m_Font->m_CharHeight+4, m_PosY+m_Height-3, m_ColLineShadow);
		//   upper-left
		Gr->DrawLine(m_PosX+3, m_PosY+m_Font->m_CharHeight-4, m_PosX+3, m_PosY+3, m_ColLine);
		Gr->DrawLine(m_PosX+4, m_PosY+m_Font->m_CharHeight-3, m_PosX+4, m_PosY+4, m_ColLineShadow);
		Gr->DrawLine(m_PosX+3, m_PosY+3, m_PosX+m_Font->m_CharHeight-4, m_PosY+3, m_ColLine);
		Gr->DrawLine(m_PosX+4, m_PosY+4, m_PosX+m_Font->m_CharHeight-3, m_PosY+4, m_ColLineShadow);
		//   upper-right
		Gr->DrawLine(m_PosX+m_Width-4, m_PosY+3, m_PosX+m_Width-m_Font->m_CharHeight+3, m_PosY+3, m_ColLine);
		Gr->DrawLine(m_PosX+m_Width-3, m_PosY+4, m_PosX+m_Width-m_Font->m_CharHeight+4, m_PosY+4, m_ColLineShadow);
		Gr->DrawLine(m_PosX+m_Width-4, m_PosY+m_Font->m_CharHeight-4, m_PosX+m_Width-4, m_PosY+3, m_ColLine);
		Gr->DrawLine(m_PosX+m_Width-3, m_PosY+m_Font->m_CharHeight-3, m_PosX+m_Width-3, m_PosY+4, m_ColLineShadow);

		// Draw minimize button
		int xm = m_PosX+m_Width-2*m_Font->m_CharHeight, wm=m_Font->m_CharHeight-6;
		wm = (wm<6) ? 6 : wm;
		Gr->DrawRect(xm+1, m_PosY+4, xm+wm-1, m_PosY+3+wm, m_HighlightMinimize?m_ColHighBtn:m_ColBtn);
		Gr->DrawLine(xm, m_PosY+3, xm+wm, m_PosY+3, m_ColLine);
		Gr->DrawLine(xm+wm, m_PosY+3, xm+wm, m_PosY+3+wm, m_ColLine);
		Gr->DrawLine(xm+wm, m_PosY+3+wm, xm, m_PosY+3+wm, m_ColLine);
		Gr->DrawLine(xm, m_PosY+3+wm, xm, m_PosY+3, m_ColLine);
		Gr->DrawLine(xm+wm+1, m_PosY+4, xm+wm+1, m_PosY+4+wm, m_ColLineShadow);
		Gr->DrawLine(xm+wm+1, m_PosY+4+wm, xm, m_PosY+4+wm, m_ColLineShadow);
		Gr->DrawLine(xm+wm/3+((wm<9)?1:0), m_PosY+4+wm/3-((wm<9)?0:1), xm+wm/2+1, m_PosY+2+wm, m_ColTitleText, true);
		Gr->DrawLine(xm+wm-wm/3+((wm<9)?0:1), m_PosY+4+wm/3-((wm<9)?0:1), xm+wm/2, m_PosY+2+wm, m_ColTitleText, true);

		// Draw font button
		xm = m_PosX+m_Font->m_CharHeight+2;
		Gr->DrawRect(xm+1, m_PosY+4, xm+wm-1, m_PosY+3+wm, m_HighlightFont?m_ColHighBtn:m_ColBtn);
		Gr->DrawLine(xm, m_PosY+3, xm+wm, m_PosY+3, m_ColLine);
		Gr->DrawLine(xm+wm, m_PosY+3, xm+wm, m_PosY+3+wm, m_ColLine);
		Gr->DrawLine(xm+wm, m_PosY+3+wm, xm, m_PosY+3+wm, m_ColLine);
		Gr->DrawLine(xm, m_PosY+3+wm, xm, m_PosY+3, m_ColLine);
		Gr->DrawLine(xm+wm+1, m_PosY+4, xm+wm+1, m_PosY+4+wm, m_ColLineShadow);
		Gr->DrawLine(xm+wm+1, m_PosY+4+wm, xm, m_PosY+4+wm, m_ColLineShadow);
		Gr->DrawLine(xm+wm/2-wm/6, m_PosY+3+wm/3, xm+wm/2+wm/6+1, m_PosY+3+wm/3, m_ColTitleText);
		Gr->DrawLine(xm+wm/2-wm/6, m_PosY+3+wm/3, xm+wm/2-wm/6, m_PosY+4+wm-wm/3+(wm>11?1:0), m_ColTitleText);
		Gr->DrawLine(xm+wm/2-wm/6, m_PosY+3+wm/2+(wm>11?1:0), xm+wm/2+wm/6, m_PosY+3+wm/2+(wm>11?1:0), m_ColTitleText);
	}
}

//	---------------------------------------------------------------------------

void CTwBar::Draw()
{
	PERF( PerfTimer Timer; double DT; )

	assert(m_Font);
	ITwGraph *Gr = g_TwMgr->m_Graph;

	if( float(g_BarTimer.GetTime())>m_LastUpdateTime+m_UpdatePeriod )
		NotUpToDate();

	if( m_HighlightedLine!=m_HighlightedLinePrev )
	{
		m_HighlightedLinePrev = m_HighlightedLine;
		NotUpToDate();
	}

	if( m_IsHelpBar && g_TwMgr->m_HelpBarNotUpToDate )
		g_TwMgr->UpdateHelpBar();

	if( !m_UpToDate )
		Update();

	if( !m_IsMinimized )
	{
		int y = m_PosY+1;

		// Draw title
		if( !m_IsPopupList )
		{
			PERF( Timer.Reset(); )
			Gr->DrawRect(m_PosX, m_PosY, m_PosX+m_Width-1, m_PosY+m_Font->m_CharHeight, (m_HighlightTitle||m_MouseDragTitle) ? m_ColTitleHighBg : m_ColTitleBg);
			if( m_ColTitleShadow!=0 )
				Gr->DrawText(m_TitleTextObj, m_PosX+(m_Width-m_TitleWidth)/2+1, m_PosY+1, m_ColTitleShadow, 0);
			Gr->DrawText(m_TitleTextObj, m_PosX+(m_Width-m_TitleWidth)/2, m_PosY, m_ColTitleText, 0);
			y = m_PosY+m_Font->m_CharHeight+1;
			Gr->DrawLine(m_PosX, y, m_PosX+m_Width-1, y, m_ColTitleShadow);
			y++;
			PERF( DT = Timer.GetTime(); printf("Title=%.4fms ", 1000.0*DT); )
		}

		// Draw background
		PERF( Timer.Reset(); )
		//Gr->DrawRect(m_PosX, y, m_PosX+m_Width-1, m_PosY+m_Height-1, m_ColBg);
		Gr->DrawRect(m_PosX, y, m_PosX+m_Width-1, m_PosY+m_Height-1, m_ColBg2, m_ColBg1, m_ColBg1, m_ColBg);
		Gr->DrawRect(m_PosX, y, m_PosX+m_VarX0-5, m_PosY+m_Height-1, m_ColHierBg);
		Gr->DrawRect(m_PosX+m_VarX2+3, y, m_PosX+m_Width-1, m_PosY+m_Height-1, m_ColHierBg);
		// Draw highlighted line
		if( m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size() && m_HierTags[m_HighlightedLine].m_Var!=NULL
			&& (m_HierTags[m_HighlightedLine].m_Var->IsGroup() || (!static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_ReadOnly && !m_IsHelpBar)) )
			{
				int y0 = m_PosY + m_VarY0 + m_HighlightedLine*(m_Font->m_CharHeight+m_Sep);
				Gr->DrawRect(m_PosX+m_VarX0, y0, m_PosX+m_VarX2, y0+m_Font->m_CharHeight-1, m_ColHighBg);
				int eps = (g_TwMgr->m_GraphAPI==TW_OPENGL) ? 1 : 0;
				Gr->DrawLine(m_PosX+m_VarX0, y0+m_Font->m_CharHeight+eps, m_PosX+m_VarX2, y0+m_Font->m_CharHeight+eps, m_ColUnderline);
			}
		int dshad = 3;	// bar shadows
		color32 cshad = (((m_Color>>24)/2)<<24) & 0xFF000000;
		Gr->DrawRect(m_PosX, m_PosY+m_Height, m_PosX+dshad, m_PosY+m_Height+dshad, 0, cshad, 0, 0);
		Gr->DrawRect(m_PosX+dshad+1, m_PosY+m_Height, m_PosX+m_Width-1, m_PosY+m_Height+dshad, cshad, cshad, 0, 0);
		Gr->DrawRect(m_PosX+m_Width, m_PosY+m_Height, m_PosX+m_Width+dshad, m_PosY+m_Height+dshad, cshad, 0, 0, 0);
		Gr->DrawRect(m_PosX+m_Width, m_PosY, m_PosX+m_Width+dshad, m_PosY+dshad, 0, 0, cshad, 0);
		Gr->DrawRect(m_PosX+m_Width, m_PosY+dshad+1, m_PosX+m_Width+dshad, m_PosY+m_Height-1, cshad, 0, cshad, 0);
		color32 clight = 0x5FFFFFFF;
		Gr->DrawLine(m_PosX, m_PosY, m_PosX, m_PosY+m_Height, clight);
		Gr->DrawLine(m_PosX, m_PosY, m_PosX+m_Width, m_PosY, clight);
		PERF( DT = Timer.GetTime(); printf("Bg=%.4fms ", 1000.0*DT); )

		// Draw hierarchy handle
		PERF( Timer.Reset(); )
		DrawHierHandle();
		PERF( DT = Timer.GetTime(); printf("Handles=%.4fms ", 1000.0*DT); )

		// Draw labels
		PERF( Timer.Reset(); )
		Gr->DrawText(m_LabelsTextObj, m_PosX+m_VarX0, m_PosY+m_VarY0, 0 /*m_ColLabelText*/, 0);
		PERF( DT = Timer.GetTime(); printf("Labels=%.4fms ", 1000.0*DT); )

		// Draw values
		if( !m_IsPopupList )
		{
			PERF( Timer.Reset(); )
			Gr->DrawText(m_ValuesTextObj, m_PosX+m_VarX1, m_PosY+m_VarY0, 0 /*m_ColValText*/, 0 /*m_ColValBg*/);
			PERF( DT = Timer.GetTime(); printf("Values=%.4fms ", 1000.0*DT); )
		}

		// Draw preview for color values and draw buttons
		int h, nh = (int)m_HierTags.size();
		int yh = m_PosY+m_VarY0;
		int bw = IncrBtnWidth(m_Font->m_CharHeight);
		for( h=0; h<nh; ++h )
		{
			if( m_HierTags[h].m_Var->IsGroup() )
			{
				const CTwVarGroup * Grp = static_cast<const CTwVarGroup *>(m_HierTags[h].m_Var);
				if( Grp->m_SummaryCallback==CColorExt::SummaryCB && Grp->m_StructValuePtr!=NULL )
				{
					// draw color value
					int ydecal = (g_TwMgr->m_GraphAPI==TW_OPENGL) ? 1 : 0;
					const int checker = 8;
					for( int c=0; c<checker; ++c )
						Gr->DrawRect(m_PosX+m_VarX1+(c*(m_VarX2-m_VarX1))/checker, yh+1+ydecal+((c%2)*(m_Font->m_CharHeight-2))/2, m_PosX+m_VarX1-1+((c+1)*(m_VarX2-m_VarX1))/checker, yh+ydecal+(((c%2)+1)*(m_Font->m_CharHeight-2))/2, 0xffffffff);
					Gr->DrawRect(m_PosX+m_VarX1, yh+1+ydecal, m_PosX+m_VarX2-1, yh+ydecal+m_Font->m_CharHeight-2, 0xbfffffff);
					const CColorExt *colExt = static_cast<const CColorExt *>(Grp->m_StructValuePtr);
					color32 col = Color32FromARGBi((colExt->m_HasAlpha ? colExt->A : 255), colExt->R, colExt->G, colExt->B);
					if( col!=0 )
						Gr->DrawRect(m_PosX+m_VarX1, yh+1+ydecal, m_PosX+m_VarX2-1, yh+ydecal+m_Font->m_CharHeight-2, col);
					/*
					Gr->DrawLine(m_PosX+m_VarX1-1, yh, m_PosX+m_VarX2+1, yh, 0xff000000);
					Gr->DrawLine(m_PosX+m_VarX1-1, yh+m_Font->m_CharHeight, m_PosX+m_VarX2+1, yh+m_Font->m_CharHeight, 0xff000000);
					Gr->DrawLine(m_PosX+m_VarX1-1, yh, m_PosX+m_VarX1-1, yh+m_Font->m_CharHeight, 0xff000000);
					Gr->DrawLine(m_PosX+m_VarX2, yh, m_PosX+m_VarX2, yh+m_Font->m_CharHeight, 0xff000000);
					*/
				}
			}
			else if( static_cast<CTwVarAtom *>(m_HierTags[h].m_Var)->m_Type==TW_TYPE_BUTTON && !m_IsPopupList )
			{
				// draw button
				int cbx0 = m_PosX+m_VarX2-2*bw+bw/2, cby0 = yh+2, cbx1 = m_PosX+m_VarX2-2-bw/2, cby1 = yh+m_Font->m_CharHeight-4;
				if( !static_cast<CTwVarAtom *>(m_HierTags[h].m_Var)->m_ReadOnly )
				{
					if( m_HighlightClickBtn && h==m_HighlightedLine )
					{
						Gr->DrawRect(cbx0+2, cby0+2, cbx1+2, cby1+2, m_ColHighBtn);
						Gr->DrawLine(cbx0+3, cby1+3, cbx1+4, cby1+3, 0xAF000000);
						Gr->DrawLine(cbx1+3, cby0+3, cbx1+3, cby1+3, 0xAF000000);						
					}
					else
					{
						Gr->DrawRect(cbx0+3, cby1+1, cbx1+3, cby1+3, (h==m_HighlightedLine)?0xAF000000:0x7F000000);
						Gr->DrawRect(cbx1+1, cby0+3, cbx1+3, cby1, (h==m_HighlightedLine)?0xAF000000:0x7F000000);
						Gr->DrawRect(cbx0, cby0, cbx1, cby1, (h==m_HighlightedLine)?m_ColHighBtn:m_ColBtn);
					}
				}
			}
			yh += m_Font->m_CharHeight+m_Sep;
		}

		if( m_DrawHandles && !m_IsPopupList )
		{
			// Draw -/+/click buttons
			if( (m_DrawIncrDecrBtn || m_DrawClickBtn) && m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size() )
			{
				int y0 = m_PosY + m_VarY0 + m_HighlightedLine*(m_Font->m_CharHeight+m_Sep);
				if( m_DrawIncrDecrBtn )
				{
					Gr->DrawRect(m_PosX+m_VarX2-2*bw+1, y0+1, m_PosX+m_VarX2-bw-1, y0+m_Font->m_CharHeight-2, m_HighlightDecrBtn?m_ColHighBtn:m_ColBtn);
					Gr->DrawRect(m_PosX+m_VarX2-bw+1, y0+1, m_PosX+m_VarX2-1, y0+m_Font->m_CharHeight-2, m_HighlightIncrBtn?m_ColHighBtn:m_ColBtn);
					// [-]
					Gr->DrawLine(m_PosX+m_VarX2-2*bw+3, y0+m_Font->m_CharHeight/2, m_PosX+m_VarX2-bw-2, y0+m_Font->m_CharHeight/2, m_ColTitleText);
					// [+]
					Gr->DrawLine(m_PosX+m_VarX2-bw+3, y0+m_Font->m_CharHeight/2, m_PosX+m_VarX2-2, y0+m_Font->m_CharHeight/2, m_ColTitleText);
					Gr->DrawLine(m_PosX+m_VarX2-bw/2, y0+m_Font->m_CharHeight/2-bw/2+2, m_PosX+m_VarX2-bw/2, y0+m_Font->m_CharHeight/2+bw/2-1, m_ColTitleText);

					// [o] rotoslider
					//Gr->DrawRect(m_PosX+m_VarX2-3*bw+1, y0+1, m_PosX+m_VarX2-2*bw-1, y0+m_Font->m_CharHeight-2, (!m_HighlightDecrBtn && !m_HighlightIncrBtn)?m_ColHighBtn:m_ColBtn);
					//Gr->DrawLine(m_PosX+m_VarX2-3*bw+bw/2-0, y0+m_Font->m_CharHeight/2-1, m_PosX+m_VarX2-3*bw+bw/2+1, y0+m_Font->m_CharHeight/2-1, m_ColTitleText);
					//Gr->DrawLine(m_PosX+m_VarX2-3*bw+bw/2-1, y0+m_Font->m_CharHeight/2+0, m_PosX+m_VarX2-3*bw+bw/2+2, y0+m_Font->m_CharHeight/2+0, m_ColTitleText);
					//Gr->DrawLine(m_PosX+m_VarX2-3*bw+bw/2-1, y0+m_Font->m_CharHeight/2+1, m_PosX+m_VarX2-3*bw+bw/2+2, y0+m_Font->m_CharHeight/2+1, m_ColTitleText);
					//Gr->DrawLine(m_PosX+m_VarX2-3*bw+bw/2-0, y0+m_Font->m_CharHeight/2+2, m_PosX+m_VarX2-3*bw+bw/2+1, y0+m_Font->m_CharHeight/2+2, m_ColTitleText);
				}
			}

			// Draw value width slider
			if( !m_HighlightValWidth )
			{
				Gr->DrawRect(m_PosX+m_VarX1-2, m_PosY+m_VarY0-8, m_PosX+m_VarX1-1, m_PosY+m_VarY0-4, m_ColTitleText);
				Gr->DrawLine(m_PosX+m_VarX1-1, m_PosY+m_VarY0-3, m_PosX+m_VarX1, m_PosY+m_VarY0-3, m_ColLineShadow);
				Gr->DrawLine(m_PosX+m_VarX1, m_PosY+m_VarY0-3, m_PosX+m_VarX1, m_PosY+m_VarY0-8, m_ColLineShadow);
			}
			else
			{
				Gr->DrawRect(m_PosX+m_VarX1-2, m_PosY+m_VarY0-8, m_PosX+m_VarX1-1, m_PosY+m_VarY1, m_ColTitleText);
				Gr->DrawLine(m_PosX+m_VarX1-1, m_PosY+m_VarY1+1, m_PosX+m_VarX1, m_PosY+m_VarY1+1, m_ColLineShadow);
				Gr->DrawLine(m_PosX+m_VarX1, m_PosY+m_VarY1+1, m_PosX+m_VarX1, m_PosY+m_VarY0-8, m_ColLineShadow);
			}
		}

		// Draw key shortcut text
		if( m_HighlightedLine>=0 && m_HighlightedLine==m_ShortcutLine && !m_IsPopupList )
		{
			PERF( Timer.Reset(); )	
			Gr->DrawRect(m_PosX+m_Font->m_CharHeight-2, m_PosY+m_VarY1+1, m_PosX+m_Width-m_Font->m_CharHeight-2, m_PosY+m_VarY1+1+m_Font->m_CharHeight, m_ColShortcutBg);
			Gr->DrawText(m_ShortcutTextObj, m_PosX+m_Font->m_CharHeight, m_PosY+m_VarY1+1, m_ColShortcutText, 0);
			PERF( DT = Timer.GetTime(); printf("Shortcut=%.4fms ", 1000.0*DT); )
		}
		else if( m_IsHelpBar )
		{
			if( g_TwMgr->m_KeyPressedTextObj && g_TwMgr->m_KeyPressedStr.size()>0 )	// Draw key pressed
			{
				if( g_TwMgr->m_KeyPressedBuildText )
				{
					string Str = g_TwMgr->m_KeyPressedStr;
					ClampText(Str, m_Font, m_Width-2*m_Font->m_CharHeight);
					g_TwMgr->m_Graph->BuildText(g_TwMgr->m_KeyPressedTextObj, &Str, NULL, NULL, 1, g_TwMgr->m_HelpBar->m_Font, 0, 0);
					g_TwMgr->m_KeyPressedBuildText = false;
					g_TwMgr->m_KeyPressedTime = (float)g_BarTimer.GetTime();
				}
				if( (float)g_BarTimer.GetTime()>g_TwMgr->m_KeyPressedTime+1.0f ) // draw key pressed at least 1 second
					g_TwMgr->m_KeyPressedStr = "";
				PERF( Timer.Reset(); )	
				Gr->DrawRect(m_PosX+m_Font->m_CharHeight-2, m_PosY+m_VarY1+1, m_PosX+m_Width-m_Font->m_CharHeight-2, m_PosY+m_VarY1+1+m_Font->m_CharHeight, m_ColShortcutBg);
				Gr->DrawText(g_TwMgr->m_KeyPressedTextObj, m_PosX+m_Font->m_CharHeight, m_PosY+m_VarY1+1, m_ColShortcutText, 0);
				PERF( DT = Timer.GetTime(); printf("KeyPressed=%.4fms ", 1000.0*DT); )	
			}
			else
			{
				if( g_TwMgr->m_InfoBuildText )
				{
					string Info = "> AntTweakBar";
					//string Info = "> AntTweakBar - www.antisphere.com";
					char Ver[64];
					sprintf(Ver, " (v%d.%02d)", TW_VERSION/100, TW_VERSION%100);
					Info += Ver;
					ClampText(Info, m_Font, m_Width-2*m_Font->m_CharHeight);
					g_TwMgr->m_Graph->BuildText(g_TwMgr->m_InfoTextObj, &Info, NULL, NULL, 1, g_TwMgr->m_HelpBar->m_Font, 0, 0);
					g_TwMgr->m_InfoBuildText = false;
				}
				PERF( Timer.Reset(); )	
				Gr->DrawRect(m_PosX+m_Font->m_CharHeight-2, m_PosY+m_VarY1+1, m_PosX+m_Width-m_Font->m_CharHeight-2, m_PosY+m_VarY1+1+m_Font->m_CharHeight, m_ColShortcutBg);
				Gr->DrawText(g_TwMgr->m_InfoTextObj, m_PosX+m_Font->m_CharHeight, m_PosY+m_VarY1+1, m_ColInfoText, 0);
				PERF( DT = Timer.GetTime(); printf("Info=%.4fms ", 1000.0*DT); )	
			}
		}

		if( !m_IsPopupList )
		{
			// Draw RotoSlider
			RotoDraw();
		}

		if( g_TwMgr->m_PopupBar!=NULL && this!=g_TwMgr->m_PopupBar )
		{
			// darken bar if a popup bar is displayed
			Gr->DrawRect(m_PosX, m_PosY, m_PosX+m_Width-1, m_PosY+m_Height-1, 0x1F000000);
		}

	}
	else // minimized
	{
		int vpx, vpy, vpw, vph;
		vpx = 0;
		vpy = 0;
		vpw = g_TwMgr->m_WndWidth;
		vph = g_TwMgr->m_WndHeight;
		int n = vph/m_Font->m_CharHeight-1;
		if( n<1 )
			n = 1;
		m_MinPosY = vph-((m_MinNumber%n)+1)*m_Font->m_CharHeight;
		m_MinPosX = (m_MinNumber/n)*m_Font->m_CharHeight;

		if( m_HighlightMaximize )
		{
			// Draw title
			Gr->DrawRect(m_MinPosX, m_MinPosY, m_MinPosX+m_TitleWidth+2*m_Font->m_CharHeight, m_MinPosY+m_Font->m_CharHeight, m_ColTitleBg);
			if( m_ColTitleShadow!=0 )
				Gr->DrawText(m_TitleTextObj, m_MinPosX+(3*m_Font->m_CharHeight)/2, m_MinPosY+1, m_ColTitleShadow, 0);
			Gr->DrawText(m_TitleTextObj, m_MinPosX+(3*m_Font->m_CharHeight)/2, m_MinPosY, m_ColTitleText, 0);
		}

		if( !m_IsHelpBar )
		{
			// Draw maximize button
			int xm = m_MinPosX+2, wm=m_Font->m_CharHeight-6;
			wm = (wm<6) ? 6 : wm;
			Gr->DrawRect(xm+1, m_MinPosY+4, xm+wm-1, m_MinPosY+3+wm, m_HighlightMaximize?m_ColHighBtn:m_ColBtn);
			Gr->DrawLine(xm, m_MinPosY+3, xm+wm, m_MinPosY+3, m_ColLine);
			Gr->DrawLine(xm+wm, m_MinPosY+3, xm+wm, m_MinPosY+3+wm, m_ColLine);
			Gr->DrawLine(xm+wm, m_MinPosY+3+wm, xm, m_MinPosY+3+wm, m_ColLine);
			Gr->DrawLine(xm, m_MinPosY+3+wm, xm, m_MinPosY+3, m_ColLine);
			Gr->DrawLine(xm+wm+1, m_MinPosY+4, xm+wm+1, m_MinPosY+4+wm, m_ColLineShadow);
			Gr->DrawLine(xm+wm+1, m_MinPosY+4+wm, xm, m_MinPosY+4+wm, m_ColLineShadow);
			Gr->DrawLine(xm+wm/3, m_MinPosY+3+wm-wm/3, xm+wm/2+1, m_MinPosY+5, m_ColTitleText, true);
			Gr->DrawLine(xm+wm-wm/3+1, m_MinPosY+3+wm-wm/3, xm+wm/2, m_MinPosY+5, m_ColTitleText, true);
		}
		else
		{
			// Draw help button
			int xm = m_MinPosX+2, wm=m_Font->m_CharHeight-6;
			wm = (wm<6) ? 6 : wm;
			Gr->DrawRect(xm+1, m_MinPosY+4, xm+wm-1, m_MinPosY+3+wm, m_HighlightMaximize?m_ColHighBtn:m_ColBtn);
			Gr->DrawLine(xm, m_MinPosY+3, xm+wm, m_MinPosY+3, m_ColLine);
			Gr->DrawLine(xm+wm, m_MinPosY+3, xm+wm, m_MinPosY+3+wm, m_ColLine);
			Gr->DrawLine(xm+wm, m_MinPosY+3+wm, xm, m_MinPosY+3+wm, m_ColLine);
			Gr->DrawLine(xm, m_MinPosY+3+wm, xm, m_MinPosY+3, m_ColLine);
			Gr->DrawLine(xm+wm+1, m_MinPosY+4, xm+wm+1, m_MinPosY+4+wm, m_ColLineShadow);
			Gr->DrawLine(xm+wm+1, m_MinPosY+4+wm, xm, m_MinPosY+4+wm, m_ColLineShadow);
			Gr->DrawLine(xm+wm/2-wm/6, m_MinPosY+3+wm/4, xm+wm-wm/3, m_MinPosY+3+wm/4, m_ColTitleText);
			Gr->DrawLine(xm+wm-wm/3, m_MinPosY+3+wm/4, xm+wm-wm/3, m_MinPosY+3+wm/2, m_ColTitleText);
			Gr->DrawLine(xm+wm-wm/3, m_MinPosY+3+wm/2, xm+wm/2, m_MinPosY+3+wm/2, m_ColTitleText);
			Gr->DrawLine(xm+wm/2, m_MinPosY+3+wm/2, xm+wm/2, m_MinPosY+3+wm-wm/4, m_ColTitleText);
			Gr->DrawLine(xm+wm/2, m_MinPosY+3+wm-wm/4+1, xm+wm/2, m_MinPosY+3+wm-wm/4+2, m_ColTitleText);
		}
	}
}

//	---------------------------------------------------------------------------

bool CTwBar::MouseMotion(int _X, int _Y)
{
	assert(g_TwMgr->m_Graph && g_TwMgr->m_WndHeight>0 && g_TwMgr->m_WndWidth>0);
	if( !m_UpToDate )
		Update();
	
	bool Handled = false;
	if( !m_IsMinimized )
	{
		bool InBar = (_X>=m_PosX && _X<m_PosX+m_Width && _Y>=m_PosY && _Y<m_PosY+m_Height);
		for( size_t ib=0; ib<g_TwMgr->m_Bars.size(); ++ib )
			if( g_TwMgr->m_Bars[ib]!=NULL )
			{
				g_TwMgr->m_Bars[ib]->m_DrawHandles = false;
				g_TwMgr->m_Bars[ib]->m_HighlightTitle = false;
			}
		m_DrawHandles = InBar;

		if( !m_MouseDrag )
		{	
			Handled = InBar;
			m_HighlightedLine = -1;
			m_HighlightIncrBtn = false;
			m_HighlightDecrBtn = false;
			m_HighlightClickBtn = false;
			m_HighlightTitle = false;
			m_HighlightScroll = false;
			m_HighlightUpScroll = false;
			m_HighlightDnScroll = false;
			m_HighlightMinimize = false;
			m_HighlightFont = false;
			m_HighlightValWidth = false;
			//if( InBar && _X>m_PosX+m_Font->m_CharHeight+1 && _X<m_PosX+m_VarX2 && _Y>=m_PosY+m_VarY0 && _Y<m_PosY+m_VarY1 )
			if( InBar && _X>m_PosX+2 && _X<m_PosX+m_VarX2 && _Y>=m_PosY+m_VarY0 && _Y<m_PosY+m_VarY1 )
			{
				m_HighlightedLine = (_Y-m_PosY-m_VarY0)/(m_Font->m_CharHeight+m_Sep);
				if( m_HighlightedLine>=(int)m_HierTags.size() )
					m_HighlightedLine = -1;
				if( m_HighlightedLine<0 || m_HierTags[m_HighlightedLine].m_Var==NULL || m_HierTags[m_HighlightedLine].m_Var->IsGroup() )
					ANT_SET_CURSOR(Arrow);
				else
				{
					if( !m_HierTags[m_HighlightedLine].m_Var->IsGroup() && static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_NoSlider )
					{
						if( static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_ReadOnly && !m_IsHelpBar )
							ANT_SET_CURSOR(No); //(Arrow);
						else
							ANT_SET_CURSOR(Arrow);
					}
					else if( m_DrawIncrDecrBtn && _X>=m_PosX+m_VarX2-IncrBtnWidth(m_Font->m_CharHeight) )	// [+] button
					{
						m_HighlightIncrBtn = true;
						ANT_SET_CURSOR(Arrow);
					}
					else if( m_DrawIncrDecrBtn && _X>=m_PosX+m_VarX2-2*IncrBtnWidth(m_Font->m_CharHeight) )	// [-] button
					{
						m_HighlightDecrBtn = true;
						ANT_SET_CURSOR(Arrow);
					}
					else if( !m_HierTags[m_HighlightedLine].m_Var->IsGroup() && static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_ReadOnly )
					{
						if( !m_IsHelpBar )
							ANT_SET_CURSOR(No);
						else
							ANT_SET_CURSOR(Arrow);
					}
					else
						ANT_SET_CURSOR(Point);
						//ANT_SET_CURSOR(WE);
				}
			}
			else if( InBar && !m_IsPopupList && _X>=m_PosX+2*m_Font->m_CharHeight && _X<m_PosX+m_Width-2*m_Font->m_CharHeight && _Y<m_PosY+m_Font->m_CharHeight )
			{
				m_HighlightTitle = true;
				ANT_SET_CURSOR(Move);
			}
			else if ( InBar && !m_IsPopupList && _X>=m_PosX+m_VarX1-3 && _X<m_PosX+m_VarX1+3 && _Y>m_PosY+m_Font->m_CharHeight && _Y<m_PosY+m_VarY0 )
			{
				m_HighlightValWidth = true;
				ANT_SET_CURSOR(WE);
			}
			//else if( InBar && m_NbDisplayedLines<m_NbHierLines && _X>=m_PosX && _X<m_PosX+m_Font->m_CharHeight && _Y>=m_ScrollY0 && _Y<m_ScrollY1 )
			else if( InBar && m_NbDisplayedLines<m_NbHierLines && _X>=m_PosX+m_VarX2+2 && _X<m_PosX+m_Width-2 && _Y>=m_ScrollY0 && _Y<m_ScrollY1 )
			{
				m_HighlightScroll = true;
				ANT_SET_CURSOR(NS);
			}
			else if( InBar && _X>=m_PosX+m_VarX2+2 && _X<m_PosX+m_Width-2 && _Y>=m_PosY+m_VarY0 && _Y<m_ScrollY0 )
			{
				m_HighlightUpScroll = true;
				ANT_SET_CURSOR(Arrow);
			}
			else if( InBar && _X>=m_PosX+m_VarX2+2 && _X<m_PosX+m_Width-2 && _Y>=m_ScrollY1 && _Y<m_PosY+m_VarY1 )
			{
				m_HighlightDnScroll = true;
				ANT_SET_CURSOR(Arrow);
			}
			else if( InBar && !m_IsPopupList && _X>=m_PosX && _X<m_PosX+m_Font->m_CharHeight && _Y>=m_PosY && _Y<m_PosY+m_Font->m_CharHeight )
				ANT_SET_CURSOR(TopLeft);
			else if( InBar && !m_IsPopupList && _X>=m_PosX && _X<m_PosX+m_Font->m_CharHeight && _Y>=m_PosY+m_Height-m_Font->m_CharHeight && _Y<m_PosY+m_Height )
				ANT_SET_CURSOR(BottomLeft);
			else if( InBar && !m_IsPopupList && _X>=m_PosX+m_Width-m_Font->m_CharHeight && _X<m_PosX+m_Width && _Y>=m_PosY && _Y<m_PosY+m_Font->m_CharHeight )
				ANT_SET_CURSOR(TopRight);
			else if( InBar && !m_IsPopupList && _X>=m_PosX+m_Width-m_Font->m_CharHeight && _X<m_PosX+m_Width && _Y>=m_PosY+m_Height-m_Font->m_CharHeight && _Y<m_PosY+m_Height )
				ANT_SET_CURSOR(BottomRight);
			else if( InBar && !m_IsPopupList && _X>=m_PosX+m_Font->m_CharHeight && _X<m_PosX+2*m_Font->m_CharHeight && _Y<m_PosY+m_Font->m_CharHeight )
			{
				m_HighlightFont = true;
				ANT_SET_CURSOR(Arrow);
			}
			else if( InBar && !m_IsPopupList && _X>=m_PosX+m_Width-2*m_Font->m_CharHeight && _X<m_PosX+m_Width-m_Font->m_CharHeight && _Y<m_PosY+m_Font->m_CharHeight )
			{
				m_HighlightMinimize = true;
				ANT_SET_CURSOR(Arrow);
			}
			else if( m_IsHelpBar && InBar && _X>=m_PosX+m_VarX0 && _X<m_PosX+m_Width-m_Font->m_CharHeight && _Y>m_PosY+m_Height-m_Font->m_CharHeight && _Y<m_PosY+m_Height )
				ANT_SET_CURSOR(Hand);	// web link
			else // if( InBar )
				ANT_SET_CURSOR(Arrow);
		}
		else
		{
			if( m_MouseDragVar && m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size() && m_HierTags[m_HighlightedLine].m_Var && !m_HierTags[m_HighlightedLine].m_Var->IsGroup() )
			{
				/*
				CTwVarAtom *Var = static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var);
				int Delta = _X-m_MouseOriginX;
				if( Delta!=0 )
				{
					if( !Var->m_NoSlider && !Var->m_ReadOnly )
					{
						Var->Increment(Delta);
						NotUpToDate();
					}
					m_VarHasBeenIncr = true;
				}
				m_MouseOriginX = _X;
				m_MouseOriginY = _Y;
				if( !Var->m_NoSlider && !Var->m_ReadOnly )
					ANT_SET_CURSOR(Center);
					//ANT_SET_CURSOR(WE);
				else
					ANT_SET_CURSOR(Arrow);
				Handled = true;
				*/

				// move rotoslider
				if( !static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_NoSlider )
					RotoOnMouseMove(_X, _Y);

				if( static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_ReadOnly )
					ANT_SET_CURSOR(No);
				else if( static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_NoSlider )
					ANT_SET_CURSOR(Arrow);
				m_VarHasBeenIncr = true;
				Handled = true;
				m_DrawHandles = true;
			}
			else if( m_MouseDragTitle )
			{
				int y = m_PosY;
				m_PosX += _X-m_MouseOriginX;
				m_PosY += _Y-m_MouseOriginY;
				m_MouseOriginX = _X;
				m_MouseOriginY = _Y;
				int vpx, vpy, vpw, vph;
				vpx = 0;
				vpy = 0;
				vpw = g_TwMgr->m_WndWidth;
				vph = g_TwMgr->m_WndHeight;
				if( m_PosX+m_Width>vpx+vpw )
					m_PosX = vpx+vpw-m_Width;
				if( m_PosX<vpx )
					m_PosX = vpx;
				if( m_PosY+m_Height>vpy+vph )
					m_PosY = vpy+vph-m_Height;
				if( m_PosY<vpy )
					m_PosY = vpy;
				m_ScrollY0 += m_PosY-y;
				m_ScrollY1 += m_PosY-y;
				ANT_SET_CURSOR(Move);
				Handled = true;
			}
			else if( m_MouseDragValWidth )
			{
			 	m_ValuesWidth += m_MouseOriginX-_X;
				m_MouseOriginX = _X;
				NotUpToDate();
				if( m_IsHelpBar )
					g_TwMgr->m_HelpBarNotUpToDate = true;
				ANT_SET_CURSOR(WE);
				Handled = true;
				m_DrawHandles = true;
			}
			else if( m_MouseDragScroll )
			{
				if( m_ScrollYH>0 )
				{
					int dl = ((_Y-m_MouseOriginY)*m_NbHierLines)/m_ScrollYH;
					if( m_FirstLine0+dl<0 )
						m_FirstLine = 0;
					else if( m_FirstLine0+dl+m_NbDisplayedLines>m_NbHierLines )
						m_FirstLine = m_NbHierLines-m_NbDisplayedLines;
					else
						m_FirstLine = m_FirstLine0+dl;
 					NotUpToDate();
				}
				ANT_SET_CURSOR(NS);
				Handled = true;
				m_DrawHandles = true;
			}
			else if( m_MouseDragResizeUL )
			{
				m_PosX += _X-m_MouseOriginX;
				m_PosY += _Y-m_MouseOriginY;
				m_Width -= _X-m_MouseOriginX;
				m_Height -= _Y-m_MouseOriginY;
				m_MouseOriginX = _X;
				m_MouseOriginY = _Y;
				ANT_SET_CURSOR(TopLeft);
				NotUpToDate();
				if( m_IsHelpBar )
				{
					g_TwMgr->m_HelpBarNotUpToDate = true;
					g_TwMgr->m_HelpBarUpdateNow = true;
				}
				g_TwMgr->m_KeyPressedBuildText = true;
				g_TwMgr->m_InfoBuildText = true;
				Handled = true;
				m_DrawHandles = true;
			}
			else if( m_MouseDragResizeUR )
			{
				m_PosY += _Y-m_MouseOriginY;
				m_Width += _X-m_MouseOriginX;
				m_Height -= _Y-m_MouseOriginY;
				m_MouseOriginX = _X;
				m_MouseOriginY = _Y;
				ANT_SET_CURSOR(TopRight);
				NotUpToDate();
				if( m_IsHelpBar )
				{
					g_TwMgr->m_HelpBarNotUpToDate = true;
					g_TwMgr->m_HelpBarUpdateNow = true;
				}
				g_TwMgr->m_KeyPressedBuildText = true;
				g_TwMgr->m_InfoBuildText = true;
				Handled = true;
				m_DrawHandles = true;
			}
			else if( m_MouseDragResizeLL )
			{
				m_PosX += _X-m_MouseOriginX;
				m_Width -= _X-m_MouseOriginX;
				m_Height += _Y-m_MouseOriginY;
				m_MouseOriginX = _X;
				m_MouseOriginY = _Y;
				ANT_SET_CURSOR(BottomLeft);
				NotUpToDate();
				if( m_IsHelpBar )
				{
					g_TwMgr->m_HelpBarNotUpToDate = true;
					g_TwMgr->m_HelpBarUpdateNow = true;
				}
				g_TwMgr->m_KeyPressedBuildText = true;
				g_TwMgr->m_InfoBuildText = true;
				Handled = true;
				m_DrawHandles = true;
			}
			else if( m_MouseDragResizeLR )
			{
				m_Width += _X-m_MouseOriginX;
				m_Height += _Y-m_MouseOriginY;
				m_MouseOriginX = _X;
				m_MouseOriginY = _Y;
				ANT_SET_CURSOR(BottomRight);
				NotUpToDate();
				if( m_IsHelpBar )
				{
					g_TwMgr->m_HelpBarNotUpToDate = true;
					g_TwMgr->m_HelpBarUpdateNow = true;
				}
				g_TwMgr->m_KeyPressedBuildText = true;
				g_TwMgr->m_InfoBuildText = true;
				Handled = true;
				m_DrawHandles = true;
			}
			//else if( InBar )
			//	ANT_SET_CURSOR(Arrow);
		}
	}
	else // minimized
	{
		if( _X>=m_MinPosX+2 && _X<m_MinPosX+m_Font->m_CharHeight && _Y>m_MinPosY && _Y<m_MinPosY+m_Font->m_CharHeight-2 )
		{
			m_HighlightMaximize = true;
			if( !m_IsHelpBar )
				ANT_SET_CURSOR(Arrow);
			else
				ANT_SET_CURSOR(Help);
			Handled = true;
		}
		else
			m_HighlightMaximize = false;
	}

	return Handled;
}

//	---------------------------------------------------------------------------

#ifdef ANT_WINDOWS
#	pragma optimize("", off)
//	disable optimizations because the conversion of Enum from unsigned int to double is not always exact if optimized and GraphAPI=DirectX !
#endif
static void ANT_CALL PopupCallback(void *_ClientData)
{
	if( g_TwMgr!=NULL && g_TwMgr->m_PopupBar!=NULL )
	{
		unsigned int Enum = *(unsigned int *)&_ClientData;
		CTwVarAtom *Var = g_TwMgr->m_PopupBar->m_VarEnumLinkedToPopupList;
		CTwBar *Bar = g_TwMgr->m_PopupBar->m_BarLinkedToPopupList;
		if( Bar!=NULL && Var!=NULL && !Var->m_ReadOnly && (Var->m_Type>=TW_TYPE_ENUM_BASE && Var->m_Type<=TW_TYPE_ENUM_BASE+(int)g_TwMgr->m_Enums.size()) )
		{
			Var->ValueFromDouble(Enum);
			Bar->UnHighlightLine();
			Bar->NotUpToDate();
		}
		TwDeleteBar(g_TwMgr->m_PopupBar);
		g_TwMgr->m_PopupBar = NULL;
	}
}
#ifdef ANT_WINDOWS
#	pragma optimize("", on)
#endif

//	---------------------------------------------------------------------------

bool CTwBar::MouseButton(ETwMouseButtonID _Button, bool _Pressed, int _X, int _Y)
{
	assert(g_TwMgr->m_Graph && g_TwMgr->m_WndHeight>0 && g_TwMgr->m_WndWidth>0);
	bool Handled = false;
	if( !m_UpToDate )
		Update();

	if( !m_IsMinimized )
	{
		Handled = (_X>=m_PosX && _X<m_PosX+m_Width && _Y>=m_PosY && _Y<m_PosY+m_Height);
		if( _Button==TW_MOUSE_LEFT && m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size() && m_HierTags[m_HighlightedLine].m_Var )
		{
			if( m_HierTags[m_HighlightedLine].m_Var->IsGroup() )
			{
				if( _Pressed && !g_TwMgr->m_IsRepeatingMousePressed )
				{
					CTwVarGroup *Grp = static_cast<CTwVarGroup *>(m_HierTags[m_HighlightedLine].m_Var);
					Grp->m_Open = !Grp->m_Open;
					NotUpToDate();
					ANT_SET_CURSOR(Arrow);
				}
			}
			else if( _Pressed && m_HighlightIncrBtn )
			{
				static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->Increment(1);
				NotUpToDate();
			}
			else if( _Pressed && m_HighlightDecrBtn )
			{
				static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->Increment(-1);
				NotUpToDate();
			}
			else if( _Pressed && !m_MouseDrag )
			{
				m_MouseDrag = true;
				m_MouseDragVar = true;
				m_MouseOriginX = _X;
				m_MouseOriginY = _Y;
				m_VarHasBeenIncr = false;
				CTwVarAtom * Var = static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var);
				if( !Var->m_NoSlider && !Var->m_ReadOnly )
				{
					// begin rotoslider
					RotoOnLButtonDown(_X, _Y);
					m_MouseDrag = true;
					m_MouseDragVar = true;
				}
				else if( (Var->m_Type==TW_TYPE_BOOL8 || Var->m_Type==TW_TYPE_BOOL16 || Var->m_Type==TW_TYPE_BOOL32 || Var->m_Type==TW_TYPE_BOOLCPP) && !Var->m_ReadOnly )
				{
					Var->Increment(1);
					//m_HighlightClickBtn = true;
					m_VarHasBeenIncr = true;
					m_MouseDragVar = false;
					m_MouseDrag = false;
					NotUpToDate();
				}
				else if( Var->m_Type==TW_TYPE_BUTTON && !Var->m_ReadOnly )
				{
					m_HighlightClickBtn = true;
					m_MouseDragVar = false;
					m_MouseDrag = false;
				}
				//else if( (Var->m_Type==TW_TYPE_ENUM8 || Var->m_Type==TW_TYPE_ENUM16 || Var->m_Type==TW_TYPE_ENUM32) && !Var->m_ReadOnly )
				else if( (Var->m_Type>=TW_TYPE_ENUM_BASE && Var->m_Type<=TW_TYPE_ENUM_BASE+(int)g_TwMgr->m_Enums.size()) && !Var->m_ReadOnly && !g_TwMgr->m_IsRepeatingMousePressed )
				{
					m_MouseDragVar = false;
					m_MouseDrag = false;
					if( g_TwMgr->m_PopupBar!=NULL )
					{
						TwDeleteBar(g_TwMgr->m_PopupBar);
						g_TwMgr->m_PopupBar = NULL;
					}
					// popup list
					CTwMgr::CEnum& e = g_TwMgr->m_Enums[Var->m_Type-TW_TYPE_ENUM_BASE];
					g_TwMgr->m_PopupBar = TwNewBar("~ Enum Popup ~");
					g_TwMgr->m_PopupBar->m_IsPopupList = true;
					g_TwMgr->m_PopupBar->m_Color = m_Color;
					g_TwMgr->m_PopupBar->m_PosX = m_PosX + m_VarX1 - 2;
					g_TwMgr->m_PopupBar->m_PosY = m_PosY + m_VarY0 + (m_HighlightedLine+1)*(m_Font->m_CharHeight+m_Sep);
					g_TwMgr->m_PopupBar->m_Width = m_Width - 2*m_Font->m_CharHeight;
					int popHeight0 = (int)e.m_Entries.size()*(m_Font->m_CharHeight+m_Sep) + m_Font->m_CharHeight/2+2;
					int popHeight = popHeight0;
					if( g_TwMgr->m_PopupBar->m_PosY+popHeight+2 > g_TwMgr->m_WndHeight )
						popHeight = g_TwMgr->m_WndHeight-g_TwMgr->m_PopupBar->m_PosY-2;
					if( popHeight<popHeight0/2 && popHeight<g_TwMgr->m_WndHeight/2 )
						popHeight = min(popHeight0, g_TwMgr->m_WndHeight/2);
					if( popHeight<3*(m_Font->m_CharHeight+m_Sep) )
						popHeight = 3*(m_Font->m_CharHeight+m_Sep);
					g_TwMgr->m_PopupBar->m_Height = popHeight;
					g_TwMgr->m_PopupBar->m_VarEnumLinkedToPopupList = Var;
					g_TwMgr->m_PopupBar->m_BarLinkedToPopupList = this;
					for( CTwMgr::CEnum::CEntries::iterator It=e.m_Entries.begin(); It!=e.m_Entries.end(); ++It )
					{
						char ID[64];
						sprintf(ID, "%u", It->first);
						//ultoa(It->first, ID, 10);
						TwAddButton(g_TwMgr->m_PopupBar, ID, PopupCallback, *(void**)&(It->first), NULL);
						CTwVar *Btn = g_TwMgr->m_PopupBar->Find(ID);
						if( Btn!=NULL )
							Btn->m_Label = It->second.c_str();
					}
					g_TwMgr->m_HelpBarNotUpToDate = false;
				}
				else if( Var->m_ReadOnly )
					ANT_SET_CURSOR(No);
				else
					ANT_SET_CURSOR(Arrow);
			}
			else if ( !_Pressed && m_MouseDragVar )
			{
				m_MouseDrag = false;
				m_MouseDragVar = false;
				if( !Handled )
					m_DrawHandles = false;
				Handled = true;
				// end rotoslider
				RotoOnLButtonUp(_X, _Y);

				/* Incr/decr on right or left click
				if( !m_VarHasBeenIncr && !static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_ReadOnly )
				{
					if( _Button==TW_MOUSE_LEFT )
						static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->Increment(-1);
					else if( _Button==TW_MOUSE_RIGHT )
						static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->Increment(1);
					NotUpToDate();
				}
				*/

				if( static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_ReadOnly )
					ANT_SET_CURSOR(No);
				else
					ANT_SET_CURSOR(Arrow);
			}
			else if( !_Pressed && m_HighlightClickBtn )	// a button variable is activated
			{
				m_HighlightClickBtn = false;
				m_MouseDragVar = false;
				m_MouseDrag = false;
				Handled = true;
				NotUpToDate();
				CTwVarAtom * Var = static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var);
				if( !Var->m_ReadOnly && Var->m_Type==TW_TYPE_BUTTON && Var->m_ButtonCallback!=NULL )
					Var->m_ButtonCallback(Var->m_ClientData);
			}
			else if( !_Pressed )
			{
				m_MouseDragVar = false;
				m_MouseDrag = false;
			}
		}
		else if( _Pressed && !m_MouseDrag && !m_IsPopupList && _Button==TW_MOUSE_LEFT && _X>=m_PosX+2*m_Font->m_CharHeight && _X<m_PosX+m_Width-2*m_Font->m_CharHeight && _Y>=m_PosY && _Y<m_PosY+m_Font->m_CharHeight )
		{
			m_MouseDrag = true;
			m_MouseDragTitle = true;
			m_MouseOriginX = _X;
			m_MouseOriginY = _Y;
			m_HighlightTitle = true;
			ANT_SET_CURSOR(Move);
		}
		else if( !_Pressed && m_MouseDragTitle )
		{
			m_MouseDrag = false;
			m_MouseDragTitle = false;
			ANT_SET_CURSOR(Arrow);
		}
		else if ( _Pressed && !m_MouseDrag && !m_IsPopupList && _Button==TW_MOUSE_LEFT && _X>=m_PosX+m_VarX1-3 && _X<m_PosX+m_VarX1+3 && _Y>m_PosY+m_Font->m_CharHeight && _Y<m_PosY+m_VarY0 )
		{
			m_MouseDrag = true;
			m_MouseDragValWidth = true;
			m_MouseOriginX = _X;
			m_MouseOriginY = _Y;
			ANT_SET_CURSOR(WE);
		}
		else if( !_Pressed && m_MouseDragValWidth )
		{
			m_MouseDrag = false;
			m_MouseDragValWidth = false;
			ANT_SET_CURSOR(Arrow);
		}
		else if( _Pressed && !m_MouseDrag && m_NbDisplayedLines<m_NbHierLines && _Button==TW_MOUSE_LEFT && _X>=m_PosX+m_VarX2+2 && _X<m_PosX+m_Width-2 && _Y>=m_ScrollY0 && _Y<m_ScrollY1 )
		{
			m_MouseDrag = true;
			m_MouseDragScroll = true;
			m_MouseOriginX = _X;
			m_MouseOriginY = _Y;
			m_FirstLine0 = m_FirstLine;
			ANT_SET_CURSOR(NS);
		}
		else if( !_Pressed && m_MouseDragScroll )
		{
			m_MouseDrag = false;
			m_MouseDragScroll = false;
			ANT_SET_CURSOR(Arrow);
		}
		else if( _Pressed && _Button==TW_MOUSE_LEFT && _X>=m_PosX+m_VarX2+2 && _X<m_PosX+m_Width-2 && _Y>=m_PosY+m_VarY0 && _Y<m_ScrollY0 )
		{
			if( m_FirstLine>0 )
			{
				--m_FirstLine;
				NotUpToDate();
			}
		}
		else if( _Pressed && _Button==TW_MOUSE_LEFT && _X>=m_PosX+m_VarX2+2 && _X<m_PosX+m_Width-2 && _Y>=m_ScrollY1 && _Y<m_PosY+m_VarY1 )
		{
			if( m_FirstLine<m_NbHierLines-m_NbDisplayedLines )
			{
				++m_FirstLine;
				NotUpToDate();
			}
		}
		else if( _Pressed && !m_MouseDrag && !m_IsPopupList && _Button==TW_MOUSE_LEFT && _X>=m_PosX && _X<m_PosX+m_Font->m_CharHeight && _Y>=m_PosY && _Y<m_PosY+m_Font->m_CharHeight )
		{
			m_MouseDrag = true;
			m_MouseDragResizeUL = true;
			m_MouseOriginX = _X;
			m_MouseOriginY = _Y;
			ANT_SET_CURSOR(TopLeft);
		}
		else if( !_Pressed && m_MouseDragResizeUL )
		{
			m_MouseDrag = false;
			m_MouseDragResizeUL = false;
			ANT_SET_CURSOR(Arrow);
		}
		else if( _Pressed && !m_MouseDrag && !m_IsPopupList && _Button==TW_MOUSE_LEFT && _X>=m_PosX+m_Width-m_Font->m_CharHeight && _X<m_PosX+m_Width && _Y>=m_PosY && _Y<m_PosY+m_Font->m_CharHeight )
		{
			m_MouseDrag = true;
			m_MouseDragResizeUR = true;
			m_MouseOriginX = _X;
			m_MouseOriginY = _Y;
			ANT_SET_CURSOR(TopRight);
		}
		else if( !_Pressed && m_MouseDragResizeUR )
		{
			m_MouseDrag = false;
			m_MouseDragResizeUR = false;
			ANT_SET_CURSOR(Arrow);
		}
		else if( _Pressed && !m_MouseDrag && !m_IsPopupList && _Button==TW_MOUSE_LEFT && _X>=m_PosX && _X<m_PosX+m_Font->m_CharHeight && _Y>=m_PosY+m_Height-m_Font->m_CharHeight && _Y<m_PosY+m_Height )
		{
			m_MouseDrag = true;
			m_MouseDragResizeLL = true;
			m_MouseOriginX = _X;
			m_MouseOriginY = _Y;
			ANT_SET_CURSOR(BottomLeft);
		}
		else if( !_Pressed && m_MouseDragResizeLL )
		{
			m_MouseDrag = false;
			m_MouseDragResizeLL = false;
			ANT_SET_CURSOR(Arrow);
		}
		else if( _Pressed && !m_MouseDrag && !m_IsPopupList && _Button==TW_MOUSE_LEFT && _X>=m_PosX+m_Width-m_Font->m_CharHeight && _X<m_PosX+m_Width && _Y>=m_PosY+m_Height-m_Font->m_CharHeight && _Y<m_PosY+m_Height )
		{
			m_MouseDrag = true;
			m_MouseDragResizeLR = true;
			m_MouseOriginX = _X;
			m_MouseOriginY = _Y;
			ANT_SET_CURSOR(BottomRight);
		}
		else if( !_Pressed && m_MouseDragResizeLR )
		{
			m_MouseDrag = false;
			m_MouseDragResizeLR = false;
			ANT_SET_CURSOR(Arrow);
		}
		else if( _Pressed && !m_IsPopupList && _X>=m_PosX+m_Font->m_CharHeight && _X<m_PosX+2*m_Font->m_CharHeight && _Y>m_PosY && _Y<m_PosY+m_Font->m_CharHeight )
		{
			// change font
			if( _Button==TW_MOUSE_LEFT )
			{
				if( m_Font==g_DefaultSmallFont )
					g_TwMgr->SetFont(g_DefaultNormalFont, true);
				else if( m_Font==g_DefaultNormalFont )
					g_TwMgr->SetFont(g_DefaultLargeFont, true);
				else if( m_Font==g_DefaultLargeFont )
					g_TwMgr->SetFont(g_DefaultSmallFont, true);
				else
					g_TwMgr->SetFont(g_DefaultNormalFont, true);
			}
			else if( _Button==TW_MOUSE_RIGHT )
			{
				if( m_Font==g_DefaultSmallFont )
					g_TwMgr->SetFont(g_DefaultLargeFont, true);
				else if( m_Font==g_DefaultNormalFont )
					g_TwMgr->SetFont(g_DefaultSmallFont, true);
				else if( m_Font==g_DefaultLargeFont )
					g_TwMgr->SetFont(g_DefaultNormalFont, true);
				else
					g_TwMgr->SetFont(g_DefaultNormalFont, true);
			}

			ANT_SET_CURSOR(Arrow);
		}
		else if( _Pressed && !m_IsPopupList && _Button==TW_MOUSE_LEFT && _X>=m_PosX+m_Width-2*m_Font->m_CharHeight && _X<m_PosX+m_Width-m_Font->m_CharHeight && _Y>m_PosY && _Y<m_PosY+m_Font->m_CharHeight )
		{
			// minimize
			g_TwMgr->Minimize(this);
			ANT_SET_CURSOR(Arrow);
		}
		else if( m_IsHelpBar && _Pressed && !g_TwMgr->m_IsRepeatingMousePressed && _X>=m_PosX+m_VarX0 && _X<m_PosX+m_Width-m_Font->m_CharHeight && _Y>m_PosY+m_Height-m_Font->m_CharHeight && _Y<m_PosY+m_Height )
		{
			#ifdef ANT_WINDOWS
				ShellExecute(NULL, "open", "http://www.antisphere.com/Tools/AntTweakBar", NULL, NULL, SW_SHOWNORMAL);
			#endif
			ANT_SET_CURSOR(Hand);
		}
	}
	else // minimized
	{
		if( _Pressed && m_HighlightMaximize )
		{
			m_HighlightMaximize = false;
			g_TwMgr->Maximize(this);
			ANT_SET_CURSOR(Arrow);
			Handled = true;
		}
	}

	return Handled;
}


//	---------------------------------------------------------------------------

bool CTwBar::MouseWheel(int _Pos, int _PrevPos, int _MouseX, int _MouseY)
{
	assert(g_TwMgr->m_Graph && g_TwMgr->m_WndHeight>0 && g_TwMgr->m_WndWidth>0);
	if( !m_UpToDate )
		Update();
	
	bool Handled = false;
	if( !m_IsMinimized && _MouseX>=m_PosX && _MouseX<m_PosX+m_Width && _MouseY>=m_PosY && _MouseY<m_PosY+m_Height )
	{
		if( _Pos>_PrevPos && m_FirstLine>0 )
		{
			--m_FirstLine;
			NotUpToDate();
		}
		else if( _Pos<_PrevPos && m_FirstLine<m_NbHierLines-m_NbDisplayedLines )
		{
			++m_FirstLine;
			NotUpToDate();
		}

		if( _Pos!=_PrevPos )
			Handled = true;
	}

	return Handled;
}

//	---------------------------------------------------------------------------

CTwVarAtom *CTwVarGroup::FindShortcut(int _Key, int _Modifiers, bool *_DoIncr)
{
	CTwVarAtom *Atom;
	int Mask = 0xffffffff;
	if( _Key>' ' && _Key<256 ) // don't test SHIFT if _Key is a common key
		Mask &= ~TW_KMOD_SHIFT;

	// don't test KMOD_NUM and KMOD_CAPS modifiers coming from SDL
	Mask &= ~(0x1000);	// 0x1000 is the KMOD_NUM value defined in SDL_keysym.h
	Mask &= ~(0x2000);	// 0x2000 is the KMOD_CAPS value defined in SDL_keysym.h

	// complete partial modifiers comming from SDL
	if( _Modifiers & TW_KMOD_SHIFT )
		_Modifiers |= TW_KMOD_SHIFT;
	if( _Modifiers & TW_KMOD_CTRL )
		_Modifiers |= TW_KMOD_CTRL;
	if( _Modifiers & TW_KMOD_ALT )
		_Modifiers |= TW_KMOD_ALT;
	if( _Modifiers & TW_KMOD_META )
		_Modifiers |= TW_KMOD_META;

	for(size_t i=0; i<m_Vars.size(); ++i)
		if( m_Vars[i]!=NULL )
		{
			if( m_Vars[i]->IsGroup() )
			{
				Atom = static_cast<CTwVarGroup *>(m_Vars[i])->FindShortcut(_Key, _Modifiers, _DoIncr);
				if( Atom!=NULL )
					return Atom;
			}
			else
			{
				Atom = static_cast<CTwVarAtom *>(m_Vars[i]);
				if( Atom->m_KeyIncr[0]==_Key && (Atom->m_KeyIncr[1]&Mask)==(_Modifiers&Mask) )
				{
					if( _DoIncr!=NULL )
						*_DoIncr = true;
					return Atom;
				}
				else if( Atom->m_KeyDecr[0]==_Key && (Atom->m_KeyDecr[1]&Mask)==(_Modifiers&Mask) )
				{
					if( _DoIncr!=NULL )
						*_DoIncr = false;
					return Atom;
				}
			}
		}
	return NULL;
}

bool CTwBar::KeyPressed(int _Key, int _Modifiers)
{
	assert(g_TwMgr->m_Graph && g_TwMgr->m_WndHeight>0 && g_TwMgr->m_WndWidth>0);
	bool Handled = false;
	if( !m_UpToDate )
		Update();

	if( _Key>0 && _Key<TW_KEY_LAST )
	{
		/* cf TranslateKey in TwMgr.cpp
		// CTRL special cases
		if( (_Modifiers&TW_KMOD_CTRL) && !(_Modifiers&TW_KMOD_ALT || _Modifiers&TW_KMOD_META) && _Key>0 && _Key<32 )
			_Key += 'a'-1;

		// PAD translation (for SDL keysym)
		if( _Key>=256 && _Key<=272 ) // 256=SDLK_KP0 ... 272=SDLK_KP_EQUALS
		{
			bool Num = ((_Modifiers&TW_KMOD_SHIFT) && !(_Modifiers&0x1000)) || (!(_Modifiers&TW_KMOD_SHIFT) && (_Modifiers&0x1000)); // 0x1000 is SDL's KMOD_NUM
			_Modifiers &= ~TW_KMOD_SHIFT;	// remove shift modifier
			if( _Key==266 )			 // SDLK_KP_PERIOD
				_Key = Num ? '.' : TW_KEY_DELETE;
			else if( _Key==267 )	 // SDLK_KP_DIVIDE
				_Key = '/';
			else if( _Key==268 )	 // SDLK_KP_MULTIPLY
				_Key = '*';
			else if( _Key==269 )	 // SDLK_KP_MINUS
				_Key = '-';
			else if( _Key==270 )	 // SDLK_KP_PLUS
				_Key = '+';
			else if( _Key==271 )	 // SDLK_KP_ENTER
				_Key = TW_KEY_RETURN;
			else if( _Key==272 )	 // SDLK_KP_EQUALS
				_Key = '=';
			else if( Num )			 // num SDLK_KP0..9
				_Key += '0' - 256;
			else if( _Key==256 )	 // non-num SDLK_KP01
				_Key = TW_KEY_INSERT;
			else if( _Key==257 )	 // non-num SDLK_KP1
				_Key = TW_KEY_END;
			else if( _Key==258 )	 // non-num SDLK_KP2
				_Key = TW_KEY_DOWN;
			else if( _Key==259 )	 // non-num SDLK_KP3
				_Key = TW_KEY_PAGE_DOWN;
			else if( _Key==260 )	 // non-num SDLK_KP4
				_Key = TW_KEY_LEFT;
			else if( _Key==262 )	 // non-num SDLK_KP6
				_Key = TW_KEY_RIGHT;
			else if( _Key==263 )	 // non-num SDLK_KP7
				_Key = TW_KEY_HOME;
			else if( _Key==264 )	 // non-num SDLK_KP8
				_Key = TW_KEY_UP;
			else if( _Key==265 )	 // non-num SDLK_KP9
				_Key = TW_KEY_PAGE_UP;
		}
		*/

		/*
		string Str;
		TwGetKeyString(&Str, _Key, _Modifiers);
		printf("key: %d 0x%04xd %s\n", _Key, _Modifiers, Str.c_str());
		*/
		bool DoIncr = true;
		CTwVarAtom *Atom = m_VarRoot.FindShortcut(_Key, _Modifiers, &DoIncr);
		if( Atom!=NULL && Atom->m_Visible )
		{
			if( !Atom->m_ReadOnly )
				Atom->Increment( DoIncr ? +1 : -1 );
			NotUpToDate();
			Show(Atom);
			Handled = true;
		}
	}
	return Handled;
}

//	---------------------------------------------------------------------------

bool CTwBar::Show(CTwVar *_Var)
{
	if( _Var==NULL || !_Var->m_Visible )
		return false;
	if( !m_UpToDate )
		Update();

	if( OpenHier(&m_VarRoot, _Var) )
	{
		if( !m_UpToDate )
			Update();
		int l = LineInHier(&m_VarRoot, _Var);
		if( l>=0 )
		{
			int NbLines = (m_VarY1-m_VarY0+1)/(m_Font->m_CharHeight+m_Sep);
			if( NbLines<= 0 )
				NbLines = 1;
			if( l<m_FirstLine || l>=m_FirstLine+NbLines )
			{
				m_FirstLine = l-NbLines/2;
				if( m_FirstLine<0 )
					m_FirstLine = 0;
				NotUpToDate();
				Update();
				if( m_NbDisplayedLines<NbLines )
				{
					m_FirstLine -= NbLines-m_NbDisplayedLines;
					if( m_FirstLine<0 )
						m_FirstLine = 0;					
					NotUpToDate();
				}
			}
			m_HighlightedLine = l-m_FirstLine;
			return true;
		}
	}

	return false;
}

//	---------------------------------------------------------------------------

bool CTwBar::OpenHier(CTwVarGroup *_Root, CTwVar *_Var)
{
	assert( _Root!=NULL );
	for(size_t i=0; i<_Root->m_Vars.size(); ++i)
		if( _Root->m_Vars[i]!=NULL )
		{
			if( _Var==_Root->m_Vars[i] 
				|| (_Root->m_Vars[i]->IsGroup() && OpenHier(static_cast<CTwVarGroup *>(_Root->m_Vars[i]), _Var)) )
			{
				_Root->m_Open = true;
				NotUpToDate();
				return true;
			}
		}
	return false;
}

//	---------------------------------------------------------------------------

int CTwBar::LineInHier(CTwVarGroup *_Root, CTwVar *_Var)
{
	assert( _Root!=NULL );
	int l = 0;
	for(size_t i=0; i<_Root->m_Vars.size(); ++i)
		if( _Root->m_Vars[i]!=NULL && _Root->m_Vars[i]->m_Visible )
		{
			if( _Var==_Root->m_Vars[i] )
				return l;
			else if( _Root->m_Vars[i]->IsGroup() && static_cast<CTwVarGroup *>(_Root->m_Vars[i])->m_Open )
			{
				++l;
				int ll = LineInHier(static_cast<CTwVarGroup *>(_Root->m_Vars[i]), _Var);
				if( ll>=0 )
					return l+ll;
				else
					l += -ll-2;
			}
			++l;
		}
	return -l-1;
}

//	---------------------------------------------------------------------------

static void DrawArc(int _X, int _Y, int _Radius, float _StartAngleDeg, float _EndAngleDeg, color32 _Color)	// angles in degree
{
	ITwGraph *Gr = g_TwMgr->m_Graph;
	if( Gr==NULL || !Gr->IsDrawing() || _Radius==0 || _StartAngleDeg==_EndAngleDeg )
		return;

	float startAngle = (float)M_PI*_StartAngleDeg/180;
	float endAngle = (float)M_PI*_EndAngleDeg/180;
	float stepAngle = 8/(float)_Radius;	// segment length = 8 pixels
	if( stepAngle>(float)M_PI/4 )
		stepAngle = (float)M_PI/4;
	bool fullCircle = fabsf(endAngle-startAngle)>=2.0f*(float)M_PI+fabsf(stepAngle);
	int numSteps;
	if( fullCircle )
	{
		numSteps = int((2.0f*(float)M_PI)/stepAngle);
		startAngle = 0;
		endAngle = 2.0f*(float)M_PI;
	}
	else
		numSteps = int(fabsf(endAngle-startAngle)/stepAngle);
	if( startAngle>endAngle )
		stepAngle = -stepAngle;

	int x0 = int(_X + _Radius * cosf(startAngle) + 0.5f);
	int y0 = int(_Y - _Radius * sinf(startAngle) + 0.5f);
	int x1, y1;
	float angle = startAngle+stepAngle;

	for( int i=0; i<numSteps; ++i, angle+=stepAngle )
	{
		x1 = int(_X + _Radius * cosf(angle) + 0.5f);
		y1 = int(_Y - _Radius * sinf(angle) + 0.5f);
		Gr->DrawLine(x0, y0, x1, y1, _Color, true);
		x0 = x1;
		y0 = y1;
	}

	if( fullCircle )
	{
		x1 = int(_X + _Radius * cosf(startAngle) + 0.5f);
		y1 = int(_Y - _Radius * sinf(startAngle) + 0.5f);
	}
	else
	{
		x1 = int(_X + _Radius * cosf(endAngle) + 0.5f);
		y1 = int(_Y - _Radius * sinf(endAngle) + 0.5f);
	}
	Gr->DrawLine(x0, y0, x1, y1, _Color, true);
}

//	---------------------------------------------------------------------------

CTwBar::CRotoSlider::CRotoSlider()
{
	m_Var = NULL;
	m_Active = false;
	m_ActiveMiddle = false;
	m_Subdiv = 256; // will be recalculate in RotoOnLButtonDown
}

void CTwBar::RotoDraw()
{
	ITwGraph *Gr = g_TwMgr->m_Graph;
	if( Gr==NULL || !Gr->IsDrawing() )
		return;

	if( m_Roto.m_Active )
	{
		DrawArc(m_Roto.m_Origin.x, m_Roto.m_Origin.y, 32, 0, 360, m_ColRoto);
		DrawArc(m_Roto.m_Origin.x+1, m_Roto.m_Origin.y, 32, 0, 360, m_ColRoto);
		DrawArc(m_Roto.m_Origin.x, m_Roto.m_Origin.y+1, 32, 0, 360, m_ColRoto);

		if( m_Roto.m_HasPrevious )
		{
			double varMax = RotoGetMax();
			double varMin = RotoGetMin();
			double varStep = RotoGetStep();
			if( varMax<DOUBLE_MAX && varMin>-DOUBLE_MAX && fabs(varStep)>DOUBLE_EPS && m_Roto.m_Subdiv>0 )
			{
				double dtMax = 360.0*(varMax-m_Roto.m_ValueAngle0)/((double)m_Roto.m_Subdiv*varStep);//+2;
				double dtMin = 360.0*(varMin-m_Roto.m_ValueAngle0)/((double)m_Roto.m_Subdiv*varStep);//-2;

				if( dtMax>=0 && dtMax<360 && dtMin<=0 && dtMin>-360 && fabs(dtMax-dtMin)<=360 )
				{
					int x1, y1, x2, y2;
					double da = 2.0*M_PI/m_Roto.m_Subdiv;

					x1 = m_Roto.m_Origin.x + (int)(40*cos(-M_PI*(m_Roto.m_Angle0+dtMax)/180-da));
					y1 = m_Roto.m_Origin.y + (int)(40*sin(-M_PI*(m_Roto.m_Angle0+dtMax)/180-da)+0.5);
					x2 = m_Roto.m_Origin.x + (int)(40*cos(-M_PI*(m_Roto.m_Angle0+dtMax-10)/180-da));
					y2 = m_Roto.m_Origin.y + (int)(40*sin(-M_PI*(m_Roto.m_Angle0+dtMax-10)/180-da)+0.5);
					Gr->DrawLine(m_Roto.m_Origin.x, m_Roto.m_Origin.y, x1, y1, m_ColRotoBound, true);
					Gr->DrawLine(m_Roto.m_Origin.x+1, m_Roto.m_Origin.y, x1+1, y1, m_ColRotoBound, true);
					Gr->DrawLine(m_Roto.m_Origin.x, m_Roto.m_Origin.y+1, x1, y1+1, m_ColRotoBound, true);
					Gr->DrawLine(x1, y1, x2, y2, m_ColRotoBound, true);
					Gr->DrawLine(x1+1, y1, x2+1, y2, m_ColRotoBound, true);
					Gr->DrawLine(x1, y1+1, x2, y2+1, m_ColRotoBound, true);

					x1 = m_Roto.m_Origin.x + (int)(40*cos(-M_PI*(m_Roto.m_Angle0+dtMin)/180+da));
					y1 = m_Roto.m_Origin.y + (int)(40*sin(-M_PI*(m_Roto.m_Angle0+dtMin)/180+da)+0.5);
					x2 = m_Roto.m_Origin.x + (int)(40*cos(-M_PI*(m_Roto.m_Angle0+dtMin+10)/180+da));
					y2 = m_Roto.m_Origin.y + (int)(40*sin(-M_PI*(m_Roto.m_Angle0+dtMin+10)/180+da)+0.5);
					Gr->DrawLine(m_Roto.m_Origin.x, m_Roto.m_Origin.y, x1, y1, m_ColRotoBound, true);
					Gr->DrawLine(m_Roto.m_Origin.x+1, m_Roto.m_Origin.y, x1+1, y1, m_ColRotoBound, true);
					Gr->DrawLine(m_Roto.m_Origin.x, m_Roto.m_Origin.y+1, x1, y1+1, m_ColRotoBound, true);
					Gr->DrawLine(x1, y1, x2, y2, m_ColRotoBound, true);
					Gr->DrawLine(x1+1, y1, x2+1, y2, m_ColRotoBound, true);
					Gr->DrawLine(x1, y1+1, x2, y2+1, m_ColRotoBound, true);
				}
			}
		}

		Gr->DrawLine(m_Roto.m_Origin.x+1, m_Roto.m_Origin.y, m_Roto.m_Current.x+1, m_Roto.m_Current.y, m_ColRotoVal, true);
		Gr->DrawLine(m_Roto.m_Origin.x, m_Roto.m_Origin.y+1, m_Roto.m_Current.x, m_Roto.m_Current.y+1, m_ColRotoVal, true);
		Gr->DrawLine(m_Roto.m_Origin.x, m_Roto.m_Origin.y, m_Roto.m_Current.x, m_Roto.m_Current.y, m_ColRotoVal, true);

		if( fabs(m_Roto.m_AngleDT)>=1 )
		{
			DrawArc(m_Roto.m_Origin.x, m_Roto.m_Origin.y, 32, float(m_Roto.m_Angle0), float(m_Roto.m_Angle0+m_Roto.m_AngleDT-1), m_ColRotoVal);
			DrawArc(m_Roto.m_Origin.x+1, m_Roto.m_Origin.y, 32, float(m_Roto.m_Angle0), float(m_Roto.m_Angle0+m_Roto.m_AngleDT-1), m_ColRotoVal);
			DrawArc(m_Roto.m_Origin.x, m_Roto.m_Origin.y+1, 32, float(m_Roto.m_Angle0), float(m_Roto.m_Angle0+m_Roto.m_AngleDT-1), m_ColRotoVal);
		}
	}
}

double CTwBar::RotoGetValue() const
{
	assert(m_Roto.m_Var!=NULL);
	return m_Roto.m_Var->ValueToDouble();
}

void CTwBar::RotoSetValue(double _Val)
{
	assert(m_Roto.m_Var!=NULL);
	if( _Val!=m_Roto.m_CurrentValue )
	{
		m_Roto.m_CurrentValue = _Val;
		m_Roto.m_Var->ValueFromDouble(_Val);
		NotUpToDate();
	}
}

double CTwBar::RotoGetMin() const
{
	assert(m_Roto.m_Var!=NULL);
	double min = -DOUBLE_MAX;
	m_Roto.m_Var->MinMaxStepToDouble(&min, NULL, NULL);
	return min;
}

double CTwBar::RotoGetMax() const
{
	assert(m_Roto.m_Var!=NULL);
	double max = DOUBLE_MAX;
	m_Roto.m_Var->MinMaxStepToDouble(NULL, &max, NULL);
	return max;
}

double CTwBar::RotoGetStep() const
{
	assert(m_Roto.m_Var!=NULL);
	double step = 1;
	m_Roto.m_Var->MinMaxStepToDouble(NULL, NULL, &step);
	return step;
}

double CTwBar::RotoGetSteppedValue() const
{
	double d = m_Roto.m_PreciseValue-m_Roto.m_Value0;
	double n = int(d/RotoGetStep());
	return m_Roto.m_Value0 + RotoGetStep()*n;
}

void CTwBar::RotoOnMouseMove(int _X, int _Y)
{
	CPoint p(_X, _Y);
	if( m_Roto.m_Active )
	{
		m_Roto.m_Current = p;
		RotoSetValue(RotoGetSteppedValue());
		//DrawManip();

		int ti = -1;
		double t = 0;
		float r = sqrtf(float(	(m_Roto.m_Current.x-m_Roto.m_Origin.x)*(m_Roto.m_Current.x-m_Roto.m_Origin.x) 
							  + (m_Roto.m_Current.y-m_Roto.m_Origin.y)*(m_Roto.m_Current.y-m_Roto.m_Origin.y)));
		if( r>m_RotoMinRadius )
		{
			t = - atan2(double(m_Roto.m_Current.y-m_Roto.m_Origin.y), double(m_Roto.m_Current.x-m_Roto.m_Origin.x));
			ti = (int((t/(2.0*M_PI)+1.0)*NB_ROTO_CURSORS+0.5)) % NB_ROTO_CURSORS;
			if( m_Roto.m_HasPrevious )
			{
				CPoint v0 = m_Roto.m_Previous-m_Roto.m_Origin;
				CPoint v1 = m_Roto.m_Current-m_Roto.m_Origin;
				double l0 = sqrt(double(v0.x*v0.x+v0.y*v0.y));
				double l1 = sqrt(double(v1.x*v1.x+v1.y*v1.y));
				double dt = acos(max(-1+1.0e-30,min(1-1.0e-30,double(v0.x*v1.x+v0.y*v1.y)/(l0*l1))));
				if( v0.x*v1.y-v0.y*v1.x>0 )
					dt = - dt;
				double preciseInc = double(m_Roto.m_Subdiv) * dt/(2.0*M_PI) * RotoGetStep();
				if( preciseInc>RotoGetStep() || preciseInc<-RotoGetStep() )
				{
					m_Roto.m_PreciseValue += preciseInc;
					if( m_Roto.m_PreciseValue>RotoGetMax() )
					{
						m_Roto.m_PreciseValue = RotoGetMax();
						m_Roto.m_Value0 = RotoGetMax();

						double da = 360*(RotoGetMax()-m_Roto.m_ValueAngle0)/(double(m_Roto.m_Subdiv)*RotoGetStep());
						m_Roto.m_Angle0 = ((int((t/(2.0*M_PI)+1.0)*360.0+0.5)) % 360) - da;
						m_Roto.m_AngleDT = da;
					}
					else if( m_Roto.m_PreciseValue<RotoGetMin() )
					{
						m_Roto.m_PreciseValue = RotoGetMin();
						m_Roto.m_Value0 = RotoGetMin();

						double da = 360*(RotoGetMin()-m_Roto.m_ValueAngle0)/(double(m_Roto.m_Subdiv)*RotoGetStep());
						m_Roto.m_Angle0 = ((int((t/(2.0*M_PI)+1.0)*360.0+0.5)) % 360) - da;
						m_Roto.m_AngleDT = da;
					}
					m_Roto.m_Previous = m_Roto.m_Current;
					m_Roto.m_AngleDT += 180.0*dt/M_PI;
				}
			}
			else
			{
				m_Roto.m_Previous = m_Roto.m_Current;
				m_Roto.m_Value0 = RotoGetValue();
				m_Roto.m_PreciseValue = m_Roto.m_Value0;
				m_Roto.m_HasPrevious = true;
				m_Roto.m_Angle0 = (int((t/(2.0*M_PI)+1.0)*360.0+0.5)) % 360;
				m_Roto.m_ValueAngle0 = m_Roto.m_Value0;
				m_Roto.m_AngleDT = 0;
			}
		}
		else
		{
			if( m_Roto.m_HasPrevious )
			{
				RotoSetValue(RotoGetSteppedValue());
				m_Roto.m_Value0 = RotoGetValue();
				m_Roto.m_ValueAngle0 = m_Roto.m_Value0;
				m_Roto.m_PreciseValue = m_Roto.m_Value0;
				m_Roto.m_Angle0 = 0;	
			}
			m_Roto.m_HasPrevious = false;
			m_Roto.m_AngleDT = 0;
		}
		if( ti>=0 && ti<NB_ROTO_CURSORS )
			ANT_SET_ROTO_CURSOR(ti);
		else
			ANT_SET_CURSOR(Center);
	}
	else
	{
		ANT_SET_CURSOR(Point);
	}
}

void CTwBar::RotoOnLButtonDown(int _X, int _Y)
{
	CPoint p(_X, _Y);
	if( !m_Roto.m_Active && m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size() && m_HierTags[m_HighlightedLine].m_Var && !m_HierTags[m_HighlightedLine].m_Var->IsGroup() )
	{
		m_Roto.m_Var = static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var);
		int y = m_PosY + m_VarY0 + m_HighlightedLine*(m_Font->m_CharHeight+m_Sep) + m_Font->m_CharHeight/2;
		m_Roto.m_Origin = CPoint(p.x, y); //r.CenterPoint().y);
		m_Roto.m_Current = p;
		m_Roto.m_Active = true;
		m_Roto.m_HasPrevious = false;
		m_Roto.m_Angle0 = 0;
		m_Roto.m_AngleDT = 0;
		//SetCapture();

		m_Roto.m_Value0 = RotoGetValue();
		m_Roto.m_CurrentValue = m_Roto.m_Value0;
		m_Roto.m_ValueAngle0 = m_Roto.m_Value0;
		m_Roto.m_PreciseValue = m_Roto.m_Value0;
		//RotoSetValue(RotoGetSteppedValue());  Not here
		//DrawManip();

		m_Roto.m_Subdiv = m_RotoNbSubdiv;
		// re-adjust m_Subdiv if needed:
		double min=-DOUBLE_MAX, max=DOUBLE_MAX, step=1;
		m_Roto.m_Var->MinMaxStepToDouble(&min, &max, &step);
		if( fabs(step)>0 && min>-DOUBLE_MAX && max<DOUBLE_MAX )
		{
			double dsubdiv = fabs(max-min)/fabs(step)+0.5;
			if( dsubdiv<m_RotoNbSubdiv/3 )
				m_Roto.m_Subdiv = 3*(int)dsubdiv;
		}

		ANT_SET_CURSOR(Center);
	}
}

void CTwBar::RotoOnLButtonUp(int /*_X*/, int /*_Y*/)
{
	if( !m_Roto.m_ActiveMiddle )
	{
		if( m_Roto.m_Var )
			RotoSetValue(RotoGetSteppedValue());

		m_Roto.m_Var = NULL;
		m_Roto.m_Active = false;
//		ReleaseCapture();

//		RotoSetValue(RotoGetSteppedValue());
//		m_Dlg->RedrawWindow();
	}
}

void CTwBar::RotoOnMButtonDown(int _X, int _Y)
{
	if( !m_Roto.m_Active )
	{
		m_Roto.m_ActiveMiddle = true;
		RotoOnLButtonDown(_X, _Y);
	}
}

void CTwBar::RotoOnMButtonUp(int _X, int _Y)
{
	if( m_Roto.m_ActiveMiddle )
	{
		m_Roto.m_ActiveMiddle = false;
		RotoOnLButtonUp(_X, _Y);
	}
}


