//  ---------------------------------------------------------------------------
//
//  @file       TwBar.cpp
//  @author     Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//              For conditions of distribution and use, see License.txt
//
//  ---------------------------------------------------------------------------


#include "TwPrecomp.h"
#include <AntTweakBar.h>
#include "TwMgr.h"
#include "TwBar.h"
#include "TwColors.h"
  
using namespace std;

extern const char *g_ErrNotFound;
const char *g_ErrUnknownAttrib  = "Unknown parameter";
const char *g_ErrInvalidAttrib  = "Invalid parameter";
const char *g_ErrNotGroup       = "Value is not a group";
const char *g_ErrNoValue        = "Value required";
const char *g_ErrBadValue       = "Bad value";
const char *g_ErrUnknownType    = "Unknown type";
const char *g_ErrNotEnum        = "Must be of type Enum";

#undef PERF         // comment to print benchs
#define PERF(cmd)


PerfTimer g_BarTimer;

#define ANT_SET_CURSOR(_Name)       g_TwMgr->SetCursor(g_TwMgr->m_Cursor##_Name)
#define ANT_SET_ROTO_CURSOR(_Num)   g_TwMgr->SetCursor(g_TwMgr->m_RotoCursors[_Num])

#if !defined(ANT_WINDOWS)
#   define _stricmp strcasecmp
#   define _strdup  strdup
#endif  // defined(ANT_WINDOWS)

#if !defined(M_PI)
#   define M_PI 3.1415926535897932384626433832795
#endif  // !defined(M_PI)

const float  FLOAT_MAX  = 3.0e+38f;
const double DOUBLE_MAX = 1.0e+308;
const double DOUBLE_EPS = 1.0e-307;

bool IsCustomType(int _Type)
{
    return (g_TwMgr && _Type>=TW_TYPE_CUSTOM_BASE && _Type<TW_TYPE_CUSTOM_BASE+(int)g_TwMgr->m_Customs.size());
}

bool IsCSStringType(int _Type)
{
    return (_Type>TW_TYPE_CSSTRING_BASE && _Type<=TW_TYPE_CSSTRING_MAX);
}

bool IsEnumType(int _Type)
{
    return (g_TwMgr && _Type>=TW_TYPE_ENUM_BASE && _Type<TW_TYPE_ENUM_BASE+(int)g_TwMgr->m_Enums.size());
}

//  ---------------------------------------------------------------------------

CTwVar::CTwVar()
{ 
    m_IsRoot = false; 
    m_DontClip = false; 
    m_Visible = true; 
    m_LeftMargin = 0; 
    m_TopMargin = 0; 
    m_ColorPtr = &COLOR32_WHITE; 
    m_BgColorPtr = &COLOR32_ZERO;   // default
}

CTwVarAtom::CTwVarAtom()
{
    m_Type = TW_TYPE_UNDEF;
    m_Ptr = NULL;
    m_SetCallback = NULL;
    m_GetCallback = NULL;
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
    else if( m_Type==TW_TYPE_CDSTDSTRING && m_GetCallback==CTwMgr::CCDStdString::GetCB && m_ClientData!=NULL && g_TwMgr!=NULL )
    {
        // delete corresponding g_TwMgr->m_CDStdStrings element
        const CTwMgr::CCDStdString *CDStdString = (const CTwMgr::CCDStdString *)m_ClientData;
        //if( &(*CDStdString->m_This)==CDStdString )
        //  g_TwMgr->m_CDStdStrings.erase(CDStdString->m_This);
        for( list<CTwMgr::CCDStdString>::iterator it=g_TwMgr->m_CDStdStrings.begin(); it!=g_TwMgr->m_CDStdStrings.end(); ++it )
            if( &(*it)==CDStdString )
            {
                g_TwMgr->m_CDStdStrings.erase(it);
                break;
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

//  ---------------------------------------------------------------------------

void CTwVarAtom::ValueToString(string *_Str) const
{
    assert(_Str!=NULL);
    static const char *ErrStr = "unreachable";
    char Tmp[1024];
    if( m_Type==TW_TYPE_UNDEF || m_Type==TW_TYPE_HELP_ATOM || m_Type==TW_TYPE_HELP_GRP || m_Type==TW_TYPE_BUTTON )  // has no value
    {
        *_Str = "";
        return;
    }
    else if( m_Type==TW_TYPE_HELP_HEADER )
    {
        *_Str = "SHORTCUTS";
        return;
    }
    else if( m_Type==TW_TYPE_SHORTCUT ) // special case for help bar: display shortcut
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
                *_Str = (m_Val.m_Bool.m_TrueString!=NULL) ? m_Val.m_Bool.m_TrueString : "1";
            else
                *_Str = (m_Val.m_Bool.m_FalseString!=NULL) ? m_Val.m_Bool.m_FalseString : "0";
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
                *_Str = (m_Val.m_Bool.m_TrueString!=NULL) ? m_Val.m_Bool.m_TrueString : "1";
            else
                *_Str = (m_Val.m_Bool.m_FalseString!=NULL) ? m_Val.m_Bool.m_FalseString : "0";
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
                *_Str = (m_Val.m_Bool.m_TrueString!=NULL) ? m_Val.m_Bool.m_TrueString : "1";
            else
                *_Str = (m_Val.m_Bool.m_FalseString!=NULL) ? m_Val.m_Bool.m_FalseString : "0";
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
                *_Str = (m_Val.m_Bool.m_TrueString!=NULL) ? m_Val.m_Bool.m_TrueString : "1";
            else
                *_Str = (m_Val.m_Bool.m_FalseString!=NULL) ? m_Val.m_Bool.m_FalseString : "0";
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
            {
                *_Str = "  (0)";
                const_cast<char *>(_Str->c_str())[0] = '\0';
            }
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
                char Fmt[64];
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
                sprintf(Tmp, "%g", Val);
            else
            {
                char Fmt[128];
                sprintf(Fmt, "%%.%dlf", (int)m_Val.m_Float64.m_Precision);
                sprintf(Tmp, Fmt, Val);
            }
            *_Str = Tmp;
        }
        break;
    case TW_TYPE_STDSTRING:
        {
            if( UseGet )
                m_GetCallback(_Str, m_ClientData);
            else
                *_Str = *(std::string *)m_Ptr;
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
        if( IsEnumType(m_Type) )
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
        else if( IsCSStringType(m_Type) )
        {
            char *Val = NULL;
            if( UseGet )
            {
                int n = TW_CSSTRING_SIZE(m_Type);
                if( n+32>(int)g_TwMgr->m_CSStringBuffer.size() )
                    g_TwMgr->m_CSStringBuffer.resize(n+32);
                Val = &(g_TwMgr->m_CSStringBuffer[0]);
                m_GetCallback(Val , m_ClientData);
                Val[n] = '\0';
            }
            else
                Val = (char *)m_Ptr;
            if( Val!=NULL )
                *_Str = Val;
            else
                *_Str = "";
        }
        else if( m_Type==TW_TYPE_CDSTRING || m_Type==TW_TYPE_CDSTDSTRING )
        {
            char *Val = NULL;
            if( UseGet )
                m_GetCallback(&Val , m_ClientData);
            else
                Val = *(char **)m_Ptr;
            if( Val!=NULL )
                *_Str = Val;
            else
                *_Str = "";
        }
        else if( IsCustom() ) // m_Type>=TW_TYPE_CUSTOM_BASE && m_Type<TW_TYPE_CUSTOM_BASE+(int)g_TwMgr->m_Customs.size() )
        {
            *_Str = "";
        }
        else
        {
            *_Str = "unknown type";
            const_cast<CTwVarAtom *>(this)->m_ReadOnly = true;
        }
    }
}

//  ---------------------------------------------------------------------------

double CTwVarAtom::ValueToDouble() const
{
    if( m_Ptr==NULL && m_GetCallback==NULL )
        return 0;   // unreachable
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
        if( IsEnumType(m_Type) )
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

//  ---------------------------------------------------------------------------

void CTwVarAtom::ValueFromDouble(double _Val)
{
    if( m_Ptr==NULL && m_SetCallback==NULL )
        return; // unreachable
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
        if( IsEnumType(m_Type) )
        {
            unsigned int Val = (unsigned int)_Val;
            if( UseSet )
                m_SetCallback(&Val, m_ClientData);
            else
                *(unsigned int*)m_Ptr = Val;
        }
    }
}

//  ---------------------------------------------------------------------------

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
        {}  // nothing
    }

    if( _Min!=NULL )
        *_Min = min;
    if( _Max!=NULL )
        *_Max = max;
    if( _Step!=NULL )
        *_Step = step;
}

//  ---------------------------------------------------------------------------

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

//  ---------------------------------------------------------------------------

enum EVarAttribs
{
    V_LABEL = 1,
    V_HELP,
    V_GROUP,
    V_SHOW,
    V_HIDE,
    V_READONLY,
    V_READWRITE,
    V_ORDER,
    V_VISIBLE,
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
    else if( _stricmp(_Attrib, "order")==0 )
        return V_ORDER;
    else if( _stricmp(_Attrib, "visible")==0 )
        return V_VISIBLE;
    else if( _stricmp(_Attrib, "readonly")==0 )
        return V_READONLY;

    // for backward compatibility
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
                                // TODO: would have to be applied to other vars already created
                                if( _AttribID==V_LABEL )
                                {
                                    g_TwMgr->m_Structs[Idx].m_Members[im].m_Label = _Value;
//                                    m_Label = _Value;
                                }
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
                    Grp->m_ColorPtr = &(_Bar->m_ColGrpText);
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
    case V_SHOW: // for backward compatibility
        if( !m_Visible )
        {
            m_Visible = true;
            _Bar->NotUpToDate();
        }
        return 1;
    case V_HIDE: // for backward compatibility
        if( m_Visible )
        {
            m_Visible = false;
            _Bar->NotUpToDate();
        }
        return 1;
    /*
    case V_READONLY:
        SetReadOnly(true);
        _Bar->NotUpToDate();
        return 1;
    */
    case V_READWRITE: // for backward compatibility
        SetReadOnly(false);
        _Bar->NotUpToDate();
        return 1;
    case V_ORDER:
        // a special case for compatibility with deprecated command 'option=ogl/dx'
        if( IsGroup() && _Value!=NULL && static_cast<CTwVarGroup *>(this)->m_SummaryCallback==CColorExt::SummaryCB && static_cast<CTwVarGroup *>(this)->m_StructValuePtr!=NULL ) // is tw_type_color?
        {
            if( _stricmp(_Value, "ogl")==0 )
            {
                static_cast<CColorExt *>(static_cast<CTwVarGroup *>(this)->m_StructValuePtr)->m_OGL = true;
                return 1;
            }
            else if( _stricmp(_Value, "dx")==0 )
            {
                static_cast<CColorExt *>(static_cast<CTwVarGroup *>(this)->m_StructValuePtr)->m_OGL = false;
                return 1;
            }
        }
        // todo: general 'order' command (no else)
        return 0;
    case V_VISIBLE:
        if( _Value!=NULL && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "true")==0 || _stricmp(_Value, "1")==0 )
            {
                if( !m_Visible )
                {
                    m_Visible = true;
                    _Bar->NotUpToDate();
                }
                return 1;
            }
            else if( _stricmp(_Value, "false")==0 || _stricmp(_Value, "0")==0 )
            {
                if( m_Visible )
                {
                    m_Visible = false;
                    _Bar->NotUpToDate();
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
    case V_READONLY:
        if( _Value==NULL || strlen(_Value)==0 // no value is acceptable (for backward compatibility)
            || _stricmp(_Value, "true")==0 || _stricmp(_Value, "1")==0 )
        {
            if( !IsReadOnly() )
            {
                SetReadOnly(true);
                _Bar->NotUpToDate();
            }
            return 1;
        }
        else if( _stricmp(_Value, "false")==0 || _stricmp(_Value, "0")==0 )
        {
            if( IsReadOnly() )
            {
                SetReadOnly(false);
                _Bar->NotUpToDate();
            }
            return 1;
        }
        else
        {
            g_TwMgr->SetLastError(g_ErrBadValue);
            return 0;
        }
    default:
        g_TwMgr->SetLastError(g_ErrUnknownAttrib);
        return 0;
    }
}


ERetType CTwVar::GetAttrib(int _AttribID, TwBar * /*_Bar*/, CTwVarGroup * _VarParent, int /*_VarIndex*/, std::vector<double>& outDoubles, std::ostringstream& outString) const
{
    outDoubles.clear();
    outString.clear();

    switch( _AttribID )
    {
    case V_LABEL:
        outString << m_Label;
        return RET_STRING;
    case V_HELP:
        outString << m_Help;
        return RET_STRING;
    case V_GROUP:
        if( _VarParent!=NULL )
            outString << _VarParent->m_Name;
        return RET_STRING;
    case V_VISIBLE:
        outDoubles.push_back(m_Visible ? 1 : 0);
        return RET_DOUBLE;
    case V_READONLY:
        outDoubles.push_back(IsReadOnly() ? 1 : 0);
        return RET_DOUBLE;
    default:
        g_TwMgr->SetLastError(g_ErrUnknownAttrib);
        return RET_ERROR;
    }
}


//  ---------------------------------------------------------------------------

enum EVarAtomAttribs
{
    VA_KEY_INCR = V_ENDTAG+1,
    VA_KEY_DECR,
    VA_MIN,
    VA_MAX,
    VA_STEP,
    VA_PRECISION,
    VA_HEXA,
    VA_DECIMAL, // for backward compatibility
    VA_TRUE,
    VA_FALSE,
    VA_ENUM,
    VA_VALUE
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
        return VA_HEXA;
    else if( _stricmp(_Attrib, "decimal")==0 ) // for backward compatibility
    {
        *_HasValue = false;
        return VA_DECIMAL;
    }
    else if( _stricmp(_Attrib, "true")==0 )
        return VA_TRUE;
    else if( _stricmp(_Attrib, "false")==0 )
        return VA_FALSE;
    else if( _stricmp(_Attrib, "enum")==0 
             || _stricmp(_Attrib, "val")==0 ) // for backward compatibility
        return VA_ENUM;
    else if( _stricmp(_Attrib, "value")==0 )
        return VA_VALUE;

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
                    const double K_EPS = 1.0 - 1.0e-6;
                    if( Step>=1 )
                        *Precision = 0;
                    else if( Step>=0.1*K_EPS )
                        *Precision = 1;
                    else if( Step>=0.01*K_EPS )
                        *Precision = 2;
                    else if( Step>=0.001*K_EPS )
                        *Precision = 3;
                    else if( Step>=0.0001*K_EPS )
                        *Precision = 4;
                    else if( Step>=0.00001*K_EPS )
                        *Precision = 5;
                    else if( Step>=0.000001*K_EPS )
                        *Precision = 6;
                    else if( Step>=0.0000001*K_EPS )
                        *Precision = 7;
                    else if( Step>=0.00000001*K_EPS )
                        *Precision = 8;
                    else if( Step>=0.000000001*K_EPS )
                        *Precision = 9;
                    else if( Step>=0.0000000001*K_EPS )
                        *Precision = 10;
                    else if( Step>=0.00000000001*K_EPS )
                        *Precision = 11;
                    else if( Step>=0.000000000001*K_EPS )
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
            if( sscanf(_Value, "%d", &Precision)==1 && Precision>=-1 && Precision<=12 )
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
            bool hexa = false;
            if (_AttribID==VA_HEXA) 
            {
                if( _Value==NULL || strlen(_Value)==0 // no value is acceptable (for backward compatibility)
                    || _stricmp(_Value, "true")==0 || _stricmp(_Value, "1")==0 )
                    hexa = true;
            }

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
    case VA_ENUM:
        if( _Value && strlen(_Value)>0 && IsEnumType(m_Type) )
        {
            const char *s = _Value;
            int n = 0, i = 0;
            unsigned int u;
            bool Cont;
            g_TwMgr->m_Enums[m_Type-TW_TYPE_ENUM_BASE].m_Entries.clear(); // anyway reset entries
            do
            {
                Cont = false;
                i = 0;
                char Sep;
                n = sscanf(s, "%u %c%n", &u, &Sep, &i);
                if( n==2 && i>0 && ( Sep=='<' || Sep=='{' || Sep=='[' || Sep=='(' ) )
                {
                    if( Sep=='<' )  // Change to closing separator
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
                        //  m_Val.m_Enum.m_Entries = new UVal::CEnumVal::CEntries;
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
    case VA_VALUE:
        if( _Value!=NULL && strlen(_Value)>0 ) // do not check ReadOnly here.
        {
            if( !( m_Type==TW_TYPE_BUTTON || IsCustom() ) ) // || (m_Type>=TW_TYPE_CUSTOM_BASE && m_Type<TW_TYPE_CUSTOM_BASE+(int)g_TwMgr->m_Customs.size()) ) )
            {
                if( m_Type==TW_TYPE_CDSTRING || m_Type==TW_TYPE_CDSTDSTRING )
                {
                    if( m_SetCallback!=NULL )
                    {
                        m_SetCallback(&_Value, m_ClientData);
                        if( g_TwMgr!=NULL ) // Mgr might have been destroyed by the client inside a callback call
                            _Bar->NotUpToDate();
                        return 1;
                    }
                    else if( m_Type!=TW_TYPE_CDSTDSTRING )
                    {
                        char **StringPtr = (char **)m_Ptr;
                        if( StringPtr!=NULL && g_TwMgr->m_CopyCDStringToClient!=NULL )
                        {
                            g_TwMgr->m_CopyCDStringToClient(StringPtr, _Value);
                            _Bar->NotUpToDate();
                            return 1;
                        }
                    }
                }
                else if( IsCSStringType(m_Type) )
                {
                    int n = TW_CSSTRING_SIZE(m_Type);
                    if( n>0 )
                    {
                        string str = _Value;
                        if( (int)str.length()>n-1 )
                            str.resize(n-1);
                        if( m_SetCallback!=NULL )
                        {
                            m_SetCallback(str.c_str(), m_ClientData);
                            if( g_TwMgr!=NULL ) // Mgr might have been destroyed by the client inside a callback call
                                _Bar->NotUpToDate();
                            return 1;
                        }
                        else if( m_Ptr!=NULL )
                        {
                            if( n>1 )
                                strncpy((char *)m_Ptr, str.c_str(), n-1);
                            ((char *)m_Ptr)[n-1] = '\0';
                            _Bar->NotUpToDate();
                            return 1;
                        }
                    }
                }
                else
                {
                    double dbl;
                    if( sscanf(_Value, "%lf", &dbl)==1 )
                    {
                        ValueFromDouble(dbl);
                        if( g_TwMgr!=NULL ) // Mgr might have been destroyed by the client inside a callback call
                            _Bar->NotUpToDate();
                        return 1;
                    }
                }
            }
        }
        return 0;
    default:
        return CTwVar::SetAttrib(_AttribID, _Value, _Bar, _VarParent, _VarIndex);
    }
}

ERetType CTwVarAtom::GetAttrib(int _AttribID, TwBar *_Bar, CTwVarGroup *_VarParent, int _VarIndex, std::vector<double>& outDoubles, std::ostringstream& outString) const
{
    outDoubles.clear();
    outString.clear();
    std::string str;
    int num = 0;

    switch( _AttribID )
    {
    case VA_KEY_INCR:
        if( TwGetKeyString(&str, m_KeyIncr[0], m_KeyIncr[1]) )
            outString << str;
        return RET_STRING;
    case VA_KEY_DECR:
        if( TwGetKeyString(&str, m_KeyDecr[0], m_KeyDecr[1]) )
            outString << str;
        return RET_STRING;
    case VA_TRUE:
        if( m_Type==TW_TYPE_BOOL8 || m_Type==TW_TYPE_BOOL16 || m_Type==TW_TYPE_BOOL32 || m_Type==TW_TYPE_BOOLCPP )
        {
            outString << m_Val.m_Bool.m_TrueString;
            return RET_STRING;
        }
        else
        {
            g_TwMgr->SetLastError(g_ErrInvalidAttrib);
            return RET_ERROR;
        }
    case VA_FALSE:
        if( m_Type==TW_TYPE_BOOL8 || m_Type==TW_TYPE_BOOL16 || m_Type==TW_TYPE_BOOL32 || m_Type==TW_TYPE_BOOLCPP )
        {
            outString << m_Val.m_Bool.m_FalseString;
            return RET_STRING;
        }
        else
        {
            g_TwMgr->SetLastError(g_ErrInvalidAttrib);
            return RET_ERROR;
        }
    case VA_MIN:
    case VA_MAX:
    case VA_STEP:
        num = (_AttribID==VA_STEP) ? 2 : ((_AttribID==VA_MAX) ? 1 : 0);
        switch( m_Type )
        {
        case TW_TYPE_CHAR:
            outDoubles.push_back( *((&m_Val.m_Char.m_Min) + num) );
            return RET_DOUBLE;
        case TW_TYPE_INT8:
            outDoubles.push_back( *((&m_Val.m_Int8.m_Min) + num) );
            return RET_DOUBLE;
        case TW_TYPE_UINT8:
            outDoubles.push_back( *((&m_Val.m_UInt8.m_Min) + num) );
            return RET_DOUBLE;
        case TW_TYPE_INT16:
            outDoubles.push_back( *((&m_Val.m_Int16.m_Min) + num) );
            return RET_DOUBLE;
        case TW_TYPE_INT32:
            outDoubles.push_back( *((&m_Val.m_Int32.m_Min) + num) );
            return RET_DOUBLE;
        case TW_TYPE_UINT16:
            outDoubles.push_back( *((&m_Val.m_UInt16.m_Min) + num) );
            return RET_DOUBLE;
        case TW_TYPE_UINT32:
            outDoubles.push_back( *((&m_Val.m_UInt32.m_Min) + num) );
            return RET_DOUBLE;
        case TW_TYPE_FLOAT:
            outDoubles.push_back( *((&m_Val.m_Float32.m_Min) + num) );
            return RET_DOUBLE;
        case TW_TYPE_DOUBLE:
            outDoubles.push_back( *((&m_Val.m_Float64.m_Min) + num) );
            return RET_DOUBLE;
        default:
            g_TwMgr->SetLastError(g_ErrInvalidAttrib);
            return RET_ERROR;
        }
    case VA_PRECISION:
        if( m_Type==TW_TYPE_FLOAT )
        {
            outDoubles.push_back( m_Val.m_Float32.m_Precision );
            return RET_DOUBLE;
        }
        else if ( m_Type==TW_TYPE_DOUBLE )
        {
            outDoubles.push_back( m_Val.m_Float64.m_Precision );
            return RET_DOUBLE;
        }
        else
        {
            g_TwMgr->SetLastError(g_ErrInvalidAttrib);
            return RET_ERROR;
        }
    case VA_HEXA:
        switch( m_Type )
        {
        case TW_TYPE_CHAR:
            outDoubles.push_back( m_Val.m_Char.m_Hexa );
            return RET_DOUBLE;
        case TW_TYPE_INT8:
            outDoubles.push_back( m_Val.m_Int8.m_Hexa );
            return RET_DOUBLE;
        case TW_TYPE_INT16:
            outDoubles.push_back( m_Val.m_Int16.m_Hexa );
            return RET_DOUBLE;
        case TW_TYPE_INT32:
            outDoubles.push_back( m_Val.m_Int32.m_Hexa );
            return RET_DOUBLE;
        case TW_TYPE_UINT8:
            outDoubles.push_back( m_Val.m_UInt8.m_Hexa );
            return RET_DOUBLE;
        case TW_TYPE_UINT16:
            outDoubles.push_back( m_Val.m_UInt16.m_Hexa );
            return RET_DOUBLE;
        case TW_TYPE_UINT32:
            outDoubles.push_back( m_Val.m_UInt32.m_Hexa );
            return RET_DOUBLE;
        default:
            g_TwMgr->SetLastError(g_ErrInvalidAttrib);
            return RET_ERROR;
        }
    case VA_ENUM:
        if( IsEnumType(m_Type) )
        {
            CTwMgr::CEnum::CEntries::iterator it = g_TwMgr->m_Enums[m_Type-TW_TYPE_ENUM_BASE].m_Entries.begin();
            for( ; it != g_TwMgr->m_Enums[m_Type-TW_TYPE_ENUM_BASE].m_Entries.end(); ++it )
            {
                if( it != g_TwMgr->m_Enums[m_Type-TW_TYPE_ENUM_BASE].m_Entries.begin() )
                    outString << ',';
                outString << it->first << ' ';
                if( it->second.find_first_of("{}")==std::string::npos )
                    outString << '{' << it->second << '}';
                else if ( it->second.find_first_of("<>")==std::string::npos )
                    outString << '<' << it->second << '>';
                else if ( it->second.find_first_of("()")==std::string::npos )
                    outString << '(' << it->second << ')';
                else if ( it->second.find_first_of("[]")==std::string::npos )
                    outString << '[' << it->second << ']';
                else
                    outString << '{' << it->second << '}'; // should not occured (use braces)
            }
            return RET_STRING;
        }
        g_TwMgr->SetLastError(g_ErrInvalidAttrib);
        return RET_ERROR;
    case VA_VALUE:
        if( !( m_Type==TW_TYPE_BUTTON || IsCustom() ) ) // || (m_Type>=TW_TYPE_CUSTOM_BASE && m_Type<TW_TYPE_CUSTOM_BASE+(int)g_TwMgr->m_Customs.size()) ) )
        {
            if( m_Type==TW_TYPE_CDSTRING || m_Type==TW_TYPE_CDSTDSTRING || IsCSStringType(m_Type) )
            {
                string str;
                ValueToString(&str);
                outString << str;
                return RET_STRING;
            }
            else
            {
                outDoubles.push_back( ValueToDouble() );
                return RET_DOUBLE;
            }
        }
        g_TwMgr->SetLastError(g_ErrInvalidAttrib);
        return RET_ERROR;
    default:
        return CTwVar::GetAttrib(_AttribID, _Bar, _VarParent, _VarIndex, outDoubles, outString);
    }
}

//  ---------------------------------------------------------------------------

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
            if( m_Val.m_Button.m_Callback!=NULL )
            {
                m_Val.m_Button.m_Callback(m_ClientData);
                if( g_TwMgr==NULL ) // Mgr might have been destroyed by the client inside a callback call
                    return;
            }
        }
        else if( IsEnumType(m_Type) )
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

//  ---------------------------------------------------------------------------

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
    case TW_TYPE_CDSTRING:
    case TW_TYPE_STDSTRING:
        m_NoSlider = true;
        break;
    /*
    case TW_TYPE_ENUM8:
    case TW_TYPE_ENUM16:
    case TW_TYPE_ENUM32:
        m_NoSlider = true;
        break;
    */
    default:
        {} // nothing
    }

    // special types
    if(    m_Type==TW_TYPE_BUTTON 
        || IsEnumType(m_Type) // (m_Type>=TW_TYPE_ENUM_BASE && m_Type<TW_TYPE_ENUM_BASE+(int)g_TwMgr->m_Enums.size()) 
        || IsCSStringType(m_Type) // (m_Type>=TW_TYPE_CSSTRING_BASE && m_Type<=TW_TYPE_CSSTRING_MAX) 
        || m_Type==TW_TYPE_CDSTDSTRING 
        || IsCustom() ) // (m_Type>=TW_TYPE_CUSTOM_BASE && m_Type<TW_TYPE_CUSTOM_BASE+(int)g_TwMgr->m_Customs.size()) )
        m_NoSlider = true;
}

//  ---------------------------------------------------------------------------

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

//  ---------------------------------------------------------------------------

enum EVarGroupAttribs
{
    VG_OPEN = V_ENDTAG+1, // for backward compatibility
    VG_CLOSE,       // for backward compatibility
    VG_OPENED,
    VG_TYPEID,      // used internally for structs
    VG_VALPTR,      // used internally for structs
    VG_ALPHA,       // for backward compatibility
    VG_NOALPHA,     // for backward compatibility
    VG_COLORALPHA,  // tw_type_color* only
    VG_HLS,         // for backward compatibility
    VG_RGB,         // for backward compatibility
    VG_COLORMODE,   // tw_type_color* only
    VG_COLORORDER,  // tw_type_color* only
    VG_ARROW,       // tw_type_quat* only
    VG_ARROWCOLOR,  // tw_type_quat* only
    VG_AXISX,       // tw_type_quat* only
    VG_AXISY,       // tw_type_quat* only
    VG_AXISZ,       // tw_type_quat* only
    VG_SHOWVAL      // tw_type_quat* only
};

int CTwVarGroup::HasAttrib(const char *_Attrib, bool *_HasValue) const
{
    *_HasValue = false;
    if( _stricmp(_Attrib, "open")==0 ) // for backward compatibility
        return VG_OPEN;
    else if( _stricmp(_Attrib, "close")==0 ) // for backward compatibility
        return VG_CLOSE;
    else if( _stricmp(_Attrib, "opened")==0 )
    {
        *_HasValue = true;
        return VG_OPENED;
    }
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
    else if( _stricmp(_Attrib, "alpha")==0 ) // for backward compatibility
        return VG_ALPHA;
    else if( _stricmp(_Attrib, "noalpha")==0 ) // for backward compatibility
        return VG_NOALPHA;
    else if( _stricmp(_Attrib, "coloralpha")==0 )
    {
        *_HasValue = true;
        return VG_COLORALPHA;
    }
    else if( _stricmp(_Attrib, "hls")==0 ) // for backward compatibility
        return VG_HLS;
    else if( _stricmp(_Attrib, "rgb")==0 ) // for backward compatibility
        return VG_RGB;
    else if( _stricmp(_Attrib, "colormode")==0 )
    {
        *_HasValue = true;
        return VG_COLORMODE;
    }
    else if( _stricmp(_Attrib, "colororder")==0 )
    {
        *_HasValue = true;
        return VG_COLORORDER;
    }
    else if( _stricmp(_Attrib, "arrow")==0 )
    {
        *_HasValue = true;
        return VG_ARROW;
    }
    else if( _stricmp(_Attrib, "arrowcolor")==0 )
    {
        *_HasValue = true;
        return VG_ARROWCOLOR;
    }
    else if( _stricmp(_Attrib, "axisx")==0 )
    {
        *_HasValue = true;
        return VG_AXISX;
    }
    else if( _stricmp(_Attrib, "axisy")==0 )
    {
        *_HasValue = true;
        return VG_AXISY;
    }
    else if( _stricmp(_Attrib, "axisz")==0 )
    {
        *_HasValue = true;
        return VG_AXISZ;
    }
    else if( _stricmp(_Attrib, "showval")==0 )
    {
        *_HasValue = true;
        return VG_SHOWVAL;
    }

    return CTwVar::HasAttrib(_Attrib, _HasValue);
}

int CTwVarGroup::SetAttrib(int _AttribID, const char *_Value, TwBar *_Bar, struct CTwVarGroup *_VarParent, int _VarIndex)
{
    switch( _AttribID )
    {
    case VG_OPEN: // for backward compatibility
        if( !m_Open )
        {
            m_Open = true;
            _Bar->NotUpToDate();
        }
        return 1;
    case VG_CLOSE: // for backward compatibility
        if( m_Open )
        {
            m_Open = false;
            _Bar->NotUpToDate();
        }
        return 1;
    case VG_OPENED:
        if( _Value!=NULL && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "true")==0 || _stricmp(_Value, "1")==0 )
            {
                if( !m_Open )
                {
                    m_Open = true;
                    _Bar->NotUpToDate();
                }
                return 1;
            }
            else if( _stricmp(_Value, "false")==0 || _stricmp(_Value, "0")==0 )
            {
                if( m_Open )
                {
                    m_Open = false;
                    _Bar->NotUpToDate();
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
                m_ColorPtr = &(_Bar->m_ColStructText);
                return 1;
            }
            return 0;
        }
    case VG_ALPHA: // for backward compatibility
        if( m_SummaryCallback==CColorExt::SummaryCB && m_StructValuePtr!=NULL ) // is tw_type_color?
            if( static_cast<CColorExt *>(m_StructValuePtr)->m_CanHaveAlpha )
            {
                static_cast<CColorExt *>(m_StructValuePtr)->m_HasAlpha = true;
                _Bar->NotUpToDate();
                return 1;
            }
        return 0;
    case VG_NOALPHA: // for backward compatibility
        if( m_SummaryCallback==CColorExt::SummaryCB && m_StructValuePtr!=NULL ) // is tw_type_color?
        {
            static_cast<CColorExt *>(m_StructValuePtr)->m_HasAlpha = false;
            _Bar->NotUpToDate();
            return 1;
        }
        else
            return 0;
    case VG_COLORALPHA:
        if( _Value!=NULL && strlen(_Value)>0 )
        {
            if( m_SummaryCallback==CColorExt::SummaryCB && m_StructValuePtr!=NULL ) // is tw_type_color?
            {
                if( _stricmp(_Value, "true")==0 || _stricmp(_Value, "1")==0 )
                {
                    if( static_cast<CColorExt *>(m_StructValuePtr)->m_CanHaveAlpha )
                    {
                        if( !static_cast<CColorExt *>(m_StructValuePtr)->m_HasAlpha )
                        {
                            static_cast<CColorExt *>(m_StructValuePtr)->m_HasAlpha = true;
                            _Bar->NotUpToDate();
                        }
                        return 1;
                    }
                }
                else if( _stricmp(_Value, "false")==0 || _stricmp(_Value, "0")==0 )
                {
                    if( static_cast<CColorExt *>(m_StructValuePtr)->m_HasAlpha )
                    {
                        static_cast<CColorExt *>(m_StructValuePtr)->m_HasAlpha = false;
                        _Bar->NotUpToDate();
                    }
                    return 1;
                }
            }
        }
        return 0;
    case VG_HLS: // for backward compatibility
        if( m_SummaryCallback==CColorExt::SummaryCB && m_StructValuePtr!=NULL ) // is tw_type_color?
        {
            static_cast<CColorExt *>(m_StructValuePtr)->m_HLS = true;
            _Bar->NotUpToDate();
            return 1;
        }
        else
            return 0;
    case VG_RGB: // for backward compatibility
        if( m_SummaryCallback==CColorExt::SummaryCB && m_StructValuePtr!=NULL ) // is tw_type_color?
        {
            static_cast<CColorExt *>(m_StructValuePtr)->m_HLS = false;
            _Bar->NotUpToDate();
            return 1;
        }
        else
            return 0;
    case VG_COLORMODE:
        if( _Value!=NULL && strlen(_Value)>0 )
        {
            if( m_SummaryCallback==CColorExt::SummaryCB && m_StructValuePtr!=NULL ) // is tw_type_color?
            {
                if( _stricmp(_Value, "hls")==0 )
                {
                    if( !static_cast<CColorExt *>(m_StructValuePtr)->m_HLS )
                    {
                        static_cast<CColorExt *>(m_StructValuePtr)->m_HLS = true;
                        _Bar->NotUpToDate();
                    }
                    return 1;
                }
                else if( _stricmp(_Value, "rgb")==0 )
                {
                    if( static_cast<CColorExt *>(m_StructValuePtr)->m_HLS )
                    {
                        static_cast<CColorExt *>(m_StructValuePtr)->m_HLS = false;
                        _Bar->NotUpToDate();
                    }
                    return 1;
                }
            }
        }
        return 0;
    case VG_COLORORDER:
        if( m_SummaryCallback==CColorExt::SummaryCB && m_StructValuePtr!=NULL ) // is tw_type_color?
        {
            if( _Value!=NULL )
            {
                if( _stricmp(_Value, "rgba")==0 )
                    static_cast<CColorExt *>(m_StructValuePtr)->m_OGL = true;
                else if( _stricmp(_Value, "argb")==0 )
                    static_cast<CColorExt *>(m_StructValuePtr)->m_OGL = false;
                else
                    return 0;
                return 1;
            }
            return 0;
        }
        else
            return 0;
    case VG_ARROW:
        if( m_SummaryCallback==CQuaternionExt::SummaryCB && m_StructValuePtr!=NULL )    // is tw_type_quat?
        {
            if( _Value!=NULL )
            {
                double *dir = static_cast<CQuaternionExt *>(m_StructValuePtr)->m_Dir;
                double x, y, z;
                if( sscanf(_Value, "%lf %lf %lf", &x, &y, &z)==3 )
                {
                    dir[0] = x; 
                    dir[1] = y;
                    dir[2] = z;
                }
                else if( _stricmp(_Value, "off")==0 || _stricmp(_Value, "0")==0 )
                    dir[0] = dir[1] = dir[2] = 0;
                else
                    return 0;
                return 1;
            }
            return 0;
        }
        else
            return 0;
    case VG_ARROWCOLOR:
        if( m_SummaryCallback==CQuaternionExt::SummaryCB && m_StructValuePtr!=NULL )    // is tw_type_quat?
        {
            if( _Value!=NULL )
            {
                int r, g, b;
                if( sscanf(_Value, "%d %d %d", &r, &g, &b)==3 )
                    static_cast<CQuaternionExt *>(m_StructValuePtr)->m_DirColor = Color32FromARGBi(255, r, g, b);
                else
                    return 0;
                return 1;
            }
            return 0;
        }
        else
            return 0;
    case VG_AXISX:
    case VG_AXISY:
    case VG_AXISZ:
        if( m_SummaryCallback==CQuaternionExt::SummaryCB && m_StructValuePtr!=NULL )    // is tw_type_quat?
        {
            if( _Value!=NULL )
            {
                float x = 0, y = 0, z = 0;
                if( _stricmp(_Value, "x")==0 || _stricmp(_Value, "+x")==0 )
                    x = 1;
                else if( _stricmp(_Value, "-x")==0 )
                    x = -1;
                else if( _stricmp(_Value, "y")==0 || _stricmp(_Value, "+y")==0 )
                    y = 1;
                else if( _stricmp(_Value, "-y")==0 )
                    y = -1;
                else if( _stricmp(_Value, "z")==0 || _stricmp(_Value, "+z")==0 )
                    z = 1;
                else if( _stricmp(_Value, "-z")==0 )
                    z = -1;
                else
                    return 0;
                int i = (_AttribID==VG_AXISX) ? 0 : ((_AttribID==VG_AXISY) ? 1 : 2);
                static_cast<CQuaternionExt *>(m_StructValuePtr)->m_Permute[i][0] = x; 
                static_cast<CQuaternionExt *>(m_StructValuePtr)->m_Permute[i][1] = y; 
                static_cast<CQuaternionExt *>(m_StructValuePtr)->m_Permute[i][2] = z;
                return 1;
            }
            return 0;
        }
        else
            return 0;
    case VG_SHOWVAL:
        if( m_SummaryCallback==CQuaternionExt::SummaryCB && m_StructValuePtr!=NULL ) // is tw_type_quat?
        {
            if( _Value!=NULL )
            {
                if( _stricmp(_Value, "true")==0 || _stricmp(_Value, "on")==0 || _stricmp(_Value, "1")==0 )
                {
                    static_cast<CQuaternionExt *>(m_StructValuePtr)->m_ShowVal = true;
                    _Bar->NotUpToDate();
                    return 1;
                }
                else if( _stricmp(_Value, "false")==0 || _stricmp(_Value, "off")==0 || _stricmp(_Value, "0")==0 )
                {
                    static_cast<CQuaternionExt *>(m_StructValuePtr)->m_ShowVal = false;
                    _Bar->NotUpToDate();
                    return 1;
                }
            }
            return 0;
        }
        else
            return 0;
    default:
        return CTwVar::SetAttrib(_AttribID, _Value, _Bar, _VarParent, _VarIndex);
    }
}

ERetType CTwVarGroup::GetAttrib(int _AttribID, TwBar *_Bar, struct CTwVarGroup *_VarParent, int _VarIndex, std::vector<double>& outDoubles, std::ostringstream& outString) const
{
    outDoubles.clear();
    outString.clear();

    switch( _AttribID )
    {
    case VG_OPENED:
        outDoubles.push_back( m_Open );
        return RET_DOUBLE;
    case VG_COLORALPHA:
        if( m_SummaryCallback==CColorExt::SummaryCB && m_StructValuePtr!=NULL ) // is tw_type_color?
        {
            outDoubles.push_back( static_cast<CColorExt *>(m_StructValuePtr)->m_HasAlpha );
            return RET_DOUBLE;
        }
        g_TwMgr->SetLastError(g_ErrInvalidAttrib);
        return RET_ERROR;
    case VG_COLORMODE:
        if( m_SummaryCallback==CColorExt::SummaryCB && m_StructValuePtr!=NULL ) // is tw_type_color?
        {
            if( static_cast<CColorExt *>(m_StructValuePtr)->m_HLS ) 
                outString << "hls";
            else
                outString << "rgb";
            return RET_STRING;
        }
        g_TwMgr->SetLastError(g_ErrInvalidAttrib);
        return RET_ERROR;
    case VG_COLORORDER:
        if( m_SummaryCallback==CColorExt::SummaryCB && m_StructValuePtr!=NULL ) // is tw_type_color?
        {
            if( static_cast<CColorExt *>(m_StructValuePtr)->m_OGL )
                outString << "rgba";
            else 
                outString << "argb";
            return RET_STRING;
        }
        g_TwMgr->SetLastError(g_ErrInvalidAttrib);
        return RET_ERROR;
    case VG_ARROW:
        if( m_SummaryCallback==CQuaternionExt::SummaryCB && m_StructValuePtr!=NULL ) // is tw_type_quat?
        {
            double *dir = static_cast<CQuaternionExt *>(m_StructValuePtr)->m_Dir;
            outDoubles.push_back(dir[0]);
            outDoubles.push_back(dir[1]);
            outDoubles.push_back(dir[2]);
            return RET_DOUBLE;
        }
        g_TwMgr->SetLastError(g_ErrInvalidAttrib);
        return RET_ERROR;
    case VG_ARROWCOLOR:
        if( m_SummaryCallback==CQuaternionExt::SummaryCB && m_StructValuePtr!=NULL ) // is tw_type_quat?
        {
            int a, r, g, b;
            a = r = g = b = 0;
            Color32ToARGBi(static_cast<CQuaternionExt *>(m_StructValuePtr)->m_DirColor, &a, &r, &g, &b);
            outDoubles.push_back(r);
            outDoubles.push_back(g);
            outDoubles.push_back(b);
            return RET_DOUBLE;
        }
        g_TwMgr->SetLastError(g_ErrInvalidAttrib);
        return RET_ERROR;
    case VG_AXISX:
    case VG_AXISY:
    case VG_AXISZ:
        if( m_SummaryCallback==CQuaternionExt::SummaryCB && m_StructValuePtr!=NULL ) // is tw_type_quat?
        {
            int i = (_AttribID==VG_AXISX) ? 0 : ((_AttribID==VG_AXISY) ? 1 : 2);
            float x = static_cast<CQuaternionExt *>(m_StructValuePtr)->m_Permute[i][0]; 
            float y = static_cast<CQuaternionExt *>(m_StructValuePtr)->m_Permute[i][1]; 
            float z = static_cast<CQuaternionExt *>(m_StructValuePtr)->m_Permute[i][2]; 
            if( x>0 )
                outString << "+x";
            else if( x<0 )
                outString << "-x";
            else if( y>0 )
                outString << "+y";
            else if( y<0 )
                outString << "-y";
            else if( z>0 )
                outString << "+z";
            else if( z<0 )
                outString << "-z";
            else
                outString << "0"; // should not happened
            return RET_DOUBLE;
        }
        g_TwMgr->SetLastError(g_ErrInvalidAttrib);
        return RET_ERROR;
    case VG_SHOWVAL:
        if( m_SummaryCallback==CQuaternionExt::SummaryCB && m_StructValuePtr!=NULL ) // is tw_type_quat?
        {
            outDoubles.push_back( static_cast<CQuaternionExt *>(m_StructValuePtr)->m_ShowVal );
            return RET_DOUBLE;
        }
        g_TwMgr->SetLastError(g_ErrInvalidAttrib);
        return RET_ERROR;
    default:
        return CTwVar::GetAttrib(_AttribID, _Bar, _VarParent, _VarIndex, outDoubles, outString);
    }
}

//  ---------------------------------------------------------------------------

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

//  ---------------------------------------------------------------------------

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
    case TW_TYPE_CDSTRING:
        return sizeof(char *);
    case TW_TYPE_STDSTRING:
        return (g_TwMgr!=0) ? g_TwMgr->m_ClientStdStringStructSize : sizeof(std::string);
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
        else if( g_TwMgr && IsEnumType(_Type) )
            return 4;
        else if( IsCSStringType(_Type) )
            return TW_CSSTRING_SIZE(_Type);
        else if( _Type==TW_TYPE_CDSTDSTRING )
            return (g_TwMgr!=0) ? g_TwMgr->m_ClientStdStringStructSize : sizeof(std::string);
        else    // includes TW_TYPE_BUTTON
            return 0;
    }
}

//  ---------------------------------------------------------------------------

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
    if( g_TwMgr->m_UseOldColorScheme )
    {
        ColorHLSToRGBi(g_TwMgr->m_BarInitColorHue%256, 180, 200, &cr, &cg, &cb);
        m_Color = Color32FromARGBi(0xf0, cr, cg, cb);
        m_DarkText = true;
    }
    else
    {
        ColorHLSToRGBi(g_TwMgr->m_BarInitColorHue%256, 80, 200, &cr, &cg, &cb);
        m_Color = Color32FromARGBi(64, cr, cg, cb);
        m_DarkText = false;
    }
    g_TwMgr->m_BarInitColorHue -= 16;
    if( g_TwMgr->m_BarInitColorHue<0 )
        g_TwMgr->m_BarInitColorHue += 256;
    m_Font = g_TwMgr->m_CurrentFont;
    //m_Font = g_DefaultNormalFont;
    //m_Font = g_DefaultSmallFont;
    //m_Font = g_DefaultLargeFont;
    m_TitleWidth = 0;
    m_Sep = 1;
//#pragma warning "lineSep WIP"
    m_LineSep = 1;
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
    m_DrawRotoBtn = false;
    m_DrawClickBtn = false;
    m_DrawListBtn = false;
    m_DrawBoolBtn = false;
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
    m_ValuesWidthRatio = 0;
    m_VarHasBeenIncr = true;
    m_FirstLine0 = 0;
    m_HighlightedLine = -1;
    m_HighlightedLinePrev = -1;
    m_HighlightedLineLastValid = -1;
    m_HighlightIncrBtn = false;
    m_HighlightDecrBtn = false;
    m_HighlightRotoBtn = false;
    m_HighlightClickBtn = false;
    m_HighlightClickBtnAuto = 0;
    m_HighlightListBtn = false;
    m_HighlightBoolBtn = false;
    m_HighlightTitle = false;
    m_HighlightScroll = false;
    m_HighlightUpScroll = false;
    m_HighlightDnScroll = false;
    m_HighlightMinimize = false;
    m_HighlightFont = false;
    m_HighlightValWidth = false;
    m_HighlightLabelsHeader = false;
    m_HighlightValuesHeader = false;
    m_ButtonAlign = g_TwMgr->m_ButtonAlign;

    m_IsMinimized = false;
    m_MinNumber = 0;
    m_MinPosX = 0;
    m_MinPosY = 0;
    m_HighlightMaximize = false;
    m_IsHelpBar = false;
    m_IsPopupList = false;
    m_VarEnumLinkedToPopupList = NULL;
    m_BarLinkedToPopupList = NULL;

    m_Resizable = true;
    m_Movable = true;
    m_Iconifiable = true;
    m_Contained = g_TwMgr->m_Contained;

    m_TitleTextObj = g_TwMgr->m_Graph->NewTextObj();
    m_LabelsTextObj = g_TwMgr->m_Graph->NewTextObj();
    m_ValuesTextObj = g_TwMgr->m_Graph->NewTextObj();
    m_ShortcutTextObj = g_TwMgr->m_Graph->NewTextObj();
    m_HeadersTextObj = g_TwMgr->m_Graph->NewTextObj();
    m_ShortcutLine = -1;

    m_RotoMinRadius = 24;
    m_RotoNbSubdiv = 256;   // number of steps for one turn

    m_CustomActiveStructProxy = NULL;

    UpdateColors();
    NotUpToDate();
}

//  ---------------------------------------------------------------------------

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
    if( m_HeadersTextObj )
        g_TwMgr->m_Graph->DeleteTextObj(m_HeadersTextObj);
}

//  ---------------------------------------------------------------------------

const CTwVar *CTwBar::Find(const char *_Name, CTwVarGroup **_Parent, int *_Index) const
{
    return m_VarRoot.Find(_Name, _Parent, _Index);
}

CTwVar *CTwBar::Find(const char *_Name, CTwVarGroup **_Parent, int *_Index)
{
    return const_cast<CTwVar *>(const_cast<const CTwBar *>(this)->Find(_Name, _Parent, _Index));
}

//  ---------------------------------------------------------------------------

enum EBarAttribs
{
    BAR_LABEL = 1,
    BAR_HELP,
    BAR_COLOR,
    BAR_ALPHA,
    BAR_TEXT,
    BAR_SHOW,    // deprecated, used BAR_VISIBLE instead
    BAR_HIDE,    // deprecated, used BAR_VISIBLE instead
    BAR_ICONIFY, // deprecated, used BAR_ICONIFIED instead
    BAR_VISIBLE,
    BAR_ICONIFIED,
    BAR_SIZE,
    BAR_POSITION,
    BAR_REFRESH,
    BAR_FONT_SIZE,
    BAR_FONT_STYLE,
    BAR_VALUES_WIDTH,
    BAR_ICON_POS,
    BAR_ICON_ALIGN,
    BAR_ICON_MARGIN,
    BAR_RESIZABLE,
    BAR_MOVABLE,
    BAR_ICONIFIABLE,
    BAR_FONT_RESIZABLE,
    BAR_ALWAYS_TOP,
    BAR_ALWAYS_BOTTOM,
    BAR_COLOR_SCHEME,
    BAR_CONTAINED,
    BAR_BUTTON_ALIGN
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
    else if( _stricmp(_Attrib, "alpha")==0 )
        return BAR_ALPHA;
    else if( _stricmp(_Attrib, "text")==0 )
        return BAR_TEXT;
    else if( _stricmp(_Attrib, "size")==0 )
        return BAR_SIZE;
    else if( _stricmp(_Attrib, "position")==0 )
        return BAR_POSITION;
    else if( _stricmp(_Attrib, "refresh")==0 )
        return BAR_REFRESH;
    else if( _stricmp(_Attrib, "fontsize")==0 )
        return BAR_FONT_SIZE;
    else if( _stricmp(_Attrib, "fontstyle")==0 )
        return BAR_FONT_STYLE;
    else if( _stricmp(_Attrib, "valueswidth")==0 )
        return BAR_VALUES_WIDTH;
    else if( _stricmp(_Attrib, "iconpos")==0 )
        return BAR_ICON_POS;
    else if( _stricmp(_Attrib, "iconalign")==0 )
        return BAR_ICON_ALIGN;
    else if( _stricmp(_Attrib, "iconmargin")==0 )
        return BAR_ICON_MARGIN;
    else if( _stricmp(_Attrib, "resizable")==0 )
        return BAR_RESIZABLE;
    else if( _stricmp(_Attrib, "movable")==0 )
        return BAR_MOVABLE;
    else if( _stricmp(_Attrib, "iconifiable")==0 )
        return BAR_ICONIFIABLE;
    else if( _stricmp(_Attrib, "fontresizable")==0 )
        return BAR_FONT_RESIZABLE;
    else if( _stricmp(_Attrib, "alwaystop")==0 )
        return BAR_ALWAYS_TOP;
    else if( _stricmp(_Attrib, "alwaysbottom")==0 )
        return BAR_ALWAYS_BOTTOM;
    else if( _stricmp(_Attrib, "visible")==0 )
        return BAR_VISIBLE;
    else if( _stricmp(_Attrib, "iconified")==0 )
        return BAR_ICONIFIED;
    else if( _stricmp(_Attrib, "colorscheme")==0 )
        return BAR_COLOR_SCHEME;
    else if( _stricmp(_Attrib, "contained")==0 )
        return BAR_CONTAINED;
    else if( _stricmp(_Attrib, "buttonalign")==0 )
        return BAR_BUTTON_ALIGN;

    *_HasValue = false;
    if( _stricmp(_Attrib, "show")==0 ) // for backward compatibility
        return BAR_SHOW;
    else if( _stricmp(_Attrib, "hide")==0 ) // for backward compatibility
        return BAR_HIDE;
    else if( _stricmp(_Attrib, "iconify")==0 ) // for backward compatibility
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
            int alpha = (m_Color>>24) & 0xff;
            if( n==3 && v0>=0 && v0<=255 && v1>=0 && v1<=255 && v2>=0 && v2<=255 )
                c = Color32FromARGBi(alpha, v0, v1, v2);
            else if( n==4 && v0>=0 && v0<=255 && v1>=0 && v1<=255 && v2>=0 && v2<=255 && v3>=0 && v3<=255 )
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
    case BAR_ALPHA:
        if( _Value && strlen(_Value)>0 )
        {
            int alpha = 255;
            int n = sscanf(_Value, "%d", &alpha);
            if( n==1 && alpha>=0 && alpha<=255 )
                m_Color = (alpha<<24) | (m_Color & 0xffffff);
            else
            {
                g_TwMgr->SetLastError(g_ErrBadValue);
                return 0;
            }
            NotUpToDate();
            return 1;
        }
        else
        {
            g_TwMgr->SetLastError(g_ErrNoValue);
            return 0;
        }
    case BAR_TEXT:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "dark")==0 )
                m_DarkText = true;
            else if( _stricmp(_Value, "light")==0 )
                m_DarkText = false;
            else
            {
                g_TwMgr->SetLastError(g_ErrBadValue);
                return 0;
            }
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
    case BAR_VALUES_WIDTH:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "fit")==0 )
            {
                m_ValuesWidth = VALUES_WIDTH_FIT;
                NotUpToDate();
                return 1;
            } 
            else
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
        }
        else
        {
            g_TwMgr->SetLastError(g_ErrNoValue);
            return 0;
        }
    case BAR_FONT_SIZE:
        return g_TwMgr->SetAttrib(MGR_FONT_SIZE, _Value);
    case BAR_FONT_STYLE:
        return g_TwMgr->SetAttrib(MGR_FONT_STYLE, _Value);
    case BAR_ICON_POS:
        return g_TwMgr->SetAttrib(MGR_ICON_POS, _Value);
    case BAR_ICON_ALIGN:
        return g_TwMgr->SetAttrib(MGR_ICON_ALIGN, _Value);
    case BAR_ICON_MARGIN:
        return g_TwMgr->SetAttrib(MGR_ICON_MARGIN, _Value);
    case BAR_SHOW:    // deprecated
        TwSetBarState(this, TW_STATE_SHOWN);
        return 1;
    case BAR_HIDE:    // deprecated
        TwSetBarState(this, TW_STATE_HIDDEN);
        return 1;
    case BAR_ICONIFY: // deprecated
        TwSetBarState(this, TW_STATE_ICONIFIED);
        return 1;
    case BAR_RESIZABLE:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "1")==0 || _stricmp(_Value, "true")==0 )
            {
                m_Resizable = true;
                return 1;
            }
            else if( _stricmp(_Value, "0")==0 || _stricmp(_Value, "false")==0 )
            {
                m_Resizable = false;
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
    case BAR_MOVABLE:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "1")==0 || _stricmp(_Value, "true")==0 )
            {
                m_Movable = true;
                return 1;
            }
            else if( _stricmp(_Value, "0")==0 || _stricmp(_Value, "false")==0 )
            {
                m_Movable = false;
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
    case BAR_ICONIFIABLE:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "1")==0 || _stricmp(_Value, "true")==0 )
            {
                m_Iconifiable = true;
                return 1;
            }
            else if( _stricmp(_Value, "0")==0 || _stricmp(_Value, "false")==0 )
            {
                m_Iconifiable = false;
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
    case BAR_FONT_RESIZABLE:
        return g_TwMgr->SetAttrib(MGR_FONT_RESIZABLE, _Value);
    case BAR_ALWAYS_TOP:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "1")==0 || _stricmp(_Value, "true")==0 )
            {
                g_TwMgr->m_BarAlwaysOnTop = m_Name;
                if( g_TwMgr->m_BarAlwaysOnBottom.length()>0 && strcmp(g_TwMgr->m_BarAlwaysOnBottom.c_str(), m_Name.c_str())==0 )
                    g_TwMgr->m_BarAlwaysOnBottom.clear();
                TwSetTopBar(this);
                return 1;
            }
            else if( _stricmp(_Value, "0")==0 || _stricmp(_Value, "false")==0 )
            {
                if( g_TwMgr->m_BarAlwaysOnTop.length()>0 && strcmp(g_TwMgr->m_BarAlwaysOnTop.c_str(), m_Name.c_str())==0 )
                    g_TwMgr->m_BarAlwaysOnTop.clear();
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
    case BAR_ALWAYS_BOTTOM:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "1")==0 || _stricmp(_Value, "true")==0 )
            {
                g_TwMgr->m_BarAlwaysOnBottom = m_Name;
                if( g_TwMgr->m_BarAlwaysOnTop.length()>0 && strcmp(g_TwMgr->m_BarAlwaysOnTop.c_str(), m_Name.c_str())==0 )
                    g_TwMgr->m_BarAlwaysOnTop.clear();
                TwSetBottomBar(this);
                return 1;
            }
            else if( _stricmp(_Value, "0")==0 || _stricmp(_Value, "false")==0 )
            {
                if( g_TwMgr->m_BarAlwaysOnBottom.length()>0 && strcmp(g_TwMgr->m_BarAlwaysOnBottom.c_str(), m_Name.c_str())==0 )
                    g_TwMgr->m_BarAlwaysOnBottom.clear();
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
    case BAR_VISIBLE:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "1")==0 || _stricmp(_Value, "true")==0 )
            {
                TwSetBarState(this, TW_STATE_SHOWN);
                return 1;
            }
            else if( _stricmp(_Value, "0")==0 || _stricmp(_Value, "false")==0 )
            {
                TwSetBarState(this, TW_STATE_HIDDEN);
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
    case BAR_ICONIFIED:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "1")==0 || _stricmp(_Value, "true")==0 )
            {
                TwSetBarState(this, TW_STATE_ICONIFIED);
                return 1;
            }
            else if( _stricmp(_Value, "0")==0 || _stricmp(_Value, "false")==0 )
            {
                TwSetBarState(this, TW_STATE_UNICONIFIED);
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
    case BAR_COLOR_SCHEME:
        return g_TwMgr->SetAttrib(MGR_COLOR_SCHEME, _Value);
    case BAR_CONTAINED:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "1")==0 || _stricmp(_Value, "true")==0 )
            {
                m_Contained = true;
                return 1;
            }
            else if( _stricmp(_Value, "0")==0 || _stricmp(_Value, "false")==0 )
            {
                m_Contained = false;
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
    case BAR_BUTTON_ALIGN:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "left")==0 )
            {
                m_ButtonAlign = BUTTON_ALIGN_LEFT;
                return 1;
            }
            else if( _stricmp(_Value, "center")==0 )
            {
                m_ButtonAlign = BUTTON_ALIGN_CENTER;
                return 1;
            }
            if( _stricmp(_Value, "right")==0 )
            {
                m_ButtonAlign = BUTTON_ALIGN_RIGHT;
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
    default:
        g_TwMgr->SetLastError(g_ErrUnknownAttrib);
        return 0;
    }
}

ERetType CTwBar::GetAttrib(int _AttribID, std::vector<double>& outDoubles, std::ostringstream& outString) const
{
    outDoubles.clear();
    outString.clear();

    switch( _AttribID )
    {
    case BAR_LABEL:
        outString << m_Label;
        return RET_STRING;
    case BAR_HELP:
        outString << m_Help;
        return RET_STRING;
    case BAR_COLOR:
        {
            int a, r, g, b;
            a = r = g = b = 0;
            Color32ToARGBi(m_Color, &a, &r, &g, &b);
            outDoubles.push_back(r);
            outDoubles.push_back(g);
            outDoubles.push_back(b);
            return RET_DOUBLE;
        }
    case BAR_ALPHA:
        {
            int a, r, g, b;
            a = r = g = b = 0;
            Color32ToARGBi(m_Color, &a, &r, &g, &b);
            outDoubles.push_back(a);
            return RET_DOUBLE;
        }
    case BAR_TEXT:
        if( m_DarkText )
            outString << "dark";
        else 
            outString << "light";
        return RET_STRING;
    case BAR_SIZE:
        outDoubles.push_back(m_Width);
        outDoubles.push_back(m_Height);
        return RET_DOUBLE;
    case BAR_POSITION:
        outDoubles.push_back(m_PosX);
        outDoubles.push_back(m_PosY);
        return RET_DOUBLE;
    case BAR_REFRESH:
        outDoubles.push_back(m_UpdatePeriod);
        return RET_DOUBLE;
    case BAR_VALUES_WIDTH:
        outDoubles.push_back(m_ValuesWidth);
        return RET_DOUBLE;
    case BAR_FONT_SIZE:
        return g_TwMgr->GetAttrib(MGR_FONT_SIZE, outDoubles, outString);
    case BAR_FONT_STYLE:
        return g_TwMgr->GetAttrib(MGR_FONT_STYLE, outDoubles, outString);
    case BAR_ICON_POS:
        return g_TwMgr->GetAttrib(MGR_ICON_POS, outDoubles, outString);
    case BAR_ICON_ALIGN:
        return g_TwMgr->GetAttrib(MGR_ICON_ALIGN, outDoubles, outString);
    case BAR_ICON_MARGIN:
        return g_TwMgr->GetAttrib(MGR_ICON_MARGIN, outDoubles, outString);
    case BAR_RESIZABLE:
        outDoubles.push_back(m_Resizable);
        return RET_DOUBLE;
    case BAR_MOVABLE:
        outDoubles.push_back(m_Movable);
        return RET_DOUBLE;
    case BAR_ICONIFIABLE:
        outDoubles.push_back(m_Iconifiable);
        return RET_DOUBLE;
    case BAR_FONT_RESIZABLE:
        return g_TwMgr->GetAttrib(MGR_FONT_RESIZABLE, outDoubles, outString);
    case BAR_ALWAYS_TOP:
        outDoubles.push_back( g_TwMgr->m_BarAlwaysOnTop == m_Name );
        return RET_DOUBLE;
    case BAR_ALWAYS_BOTTOM:
        outDoubles.push_back( g_TwMgr->m_BarAlwaysOnBottom == m_Name );
        return RET_DOUBLE;
    case BAR_VISIBLE:
        outDoubles.push_back(m_Visible);
        return RET_DOUBLE;
    case BAR_ICONIFIED:
        outDoubles.push_back(m_IsMinimized);
        return RET_DOUBLE;
    case BAR_COLOR_SCHEME:
        return g_TwMgr->GetAttrib(MGR_COLOR_SCHEME, outDoubles, outString);
    case BAR_CONTAINED:
        outDoubles.push_back(m_Contained);
        return RET_DOUBLE;
    case BAR_BUTTON_ALIGN:
        if( m_ButtonAlign==BUTTON_ALIGN_LEFT )
            outString << "left";
        else if( m_ButtonAlign==BUTTON_ALIGN_CENTER )
            outString << "center";
        else
            outString << "right";
        return RET_STRING;
    default:
        g_TwMgr->SetLastError(g_ErrUnknownAttrib);
        return RET_ERROR;
    }
}

//  ---------------------------------------------------------------------------

void CTwBar::NotUpToDate()
{
    m_UpToDate = false;
}

//  ---------------------------------------------------------------------------

void CTwBar::UpdateColors()
{
    float a, r, g, b, h, l, s;
    Color32ToARGBf(m_Color, &a, &r, &g, &b);
    ColorRGBToHLSf(r, g, b, &h, &l, &s);
    bool lightText = !m_DarkText;

    // Colors independant of m_Color

    // Highlighted line background ramp
    m_ColHighBg0 = lightText ? Color32FromARGBf(0.4f, 0.9f, 0.9f, 0.9f) : Color32FromARGBf(0.4f, 1.0f, 1.0f, 1.0f);
    m_ColHighBg1 = lightText ? Color32FromARGBf(0.4f, 0.2f, 0.2f, 0.2f) : Color32FromARGBf(0.1f, 0.7f, 0.7f, 0.7f);

    // Text colors & background
    m_ColLabelText = lightText ? COLOR32_WHITE : COLOR32_BLACK;
    m_ColStructText = lightText ? 0xffefef00 : 0xff303000;

    m_ColValText = lightText ? 0xffc7d7ff : 0xff000080;
    m_ColValTextRO = lightText ? 0xffb7b7b7 : 0xff505050;
    m_ColValMin = lightText ? 0xff9797ff : 0xff0000f0;
    m_ColValMax = m_ColValMin;
    m_ColValTextNE = lightText ? 0xff97f797 : 0xff004000;

    m_ColValBg = lightText ? Color32FromARGBf(0.2f+0.3f*a, 0.1f, 0.1f, 0.1f) : Color32FromARGBf(0.2f+0.3f*a, 1, 1, 1);
    m_ColStructBg = lightText ? Color32FromARGBf(0.4f*a, 0, 0, 0) : Color32FromARGBf(0.4f*a, 1, 1, 1);

    m_ColLine = lightText ? Color32FromARGBf(0.6f, 1, 1, 1) : Color32FromARGBf(0.6f, 0.3f, 0.3f, 0.3f);
    m_ColLineShadow = lightText ? Color32FromARGBf(0.6f, 0, 0, 0) : Color32FromARGBf(0.6f, 0, 0, 0);
    m_ColUnderline = lightText ? 0xffd0d0d0 : 0xff202000;

    m_ColGrpBg = lightText ? Color32FromARGBf(0.1f+0.25f*a, 1, 1, 1) : Color32FromARGBf(0.1f+0.05f*a, 0, 0, 0);
    m_ColGrpText = lightText ? 0xffffff80 : 0xff000000;

    m_ColShortcutText = lightText ? 0xffffb060 : 0xff802000;
    m_ColShortcutBg = lightText ? Color32FromARGBf(0.4f*a, 0.2f, 0.2f, 0.2f) : Color32FromARGBf(0.4f*a, 0.8f, 0.8f, 0.8f);
    m_ColInfoText = lightText ? Color32FromARGBf(1.0f, 0.7f, 0.7f, 0.7f) : Color32FromARGBf(1.0f, 0.3f, 0.3f, 0.3f);

    m_ColRoto = lightText ? Color32FromARGBf(0.8f, 0.85f, 0.85f, 0.85f) : Color32FromARGBf(0.8f, 0.1f, 0.1f, 0.1f);
    m_ColRotoVal = Color32FromARGBf(1, 1.0f, 0.2f, 0.2f);
    m_ColRotoBound = lightText ? Color32FromARGBf(0.8f, 0.6f, 0.6f, 0.6f) : Color32FromARGBf(0.8f, 0.3f, 0.3f, 0.3f);

    m_ColEditText = lightText ? COLOR32_WHITE : COLOR32_BLACK;
    m_ColEditBg = lightText ? 0xff575757 : 0xffc7c7c7; // must be opaque
    m_ColEditSelText = lightText ? COLOR32_BLACK : COLOR32_WHITE;
    m_ColEditSelBg = lightText ? 0xffc7c7c7 : 0xff575757;

    // Colors dependant of m_Colors
    
    // Bar background
    ColorHLSToRGBf(h, l, s, &r, &g, &b);
    m_ColBg = Color32FromARGBf(a, r, g, b);
    ColorHLSToRGBf(h, l-0.05f, s, &r, &g, &b);
    m_ColBg1 = Color32FromARGBf(a, r, g, b);
    ColorHLSToRGBf(h, l-0.1f, s, &r, &g, &b);
    m_ColBg2 = Color32FromARGBf(a, r, g, b);
    
    ColorHLSToRGBf(h, l-0.15f, s, &r, &g, &b);
    m_ColTitleBg = Color32FromARGBf(a+0.9f, r, g, b);
    m_ColTitleText = lightText ? COLOR32_WHITE : COLOR32_BLACK;
    m_ColTitleShadow = lightText ? 0x40000000 : 0x00000000;
    ColorHLSToRGBf(h, l-0.25f, s, &r, &g, &b);
    m_ColTitleHighBg = Color32FromARGBf(a+0.8f, r, g, b);
    ColorHLSToRGBf(h, l-0.3f, s, &r, &g, &b);
    m_ColTitleUnactiveBg = Color32FromARGBf(a+0.2f, r, g, b);

    ColorHLSToRGBf(h, l-0.2f, s, &r, &g, &b);
    m_ColHierBg = Color32FromARGBf(a, r, g, b);

    ColorHLSToRGBf(h, l+0.1f, s, &r, &g, &b);
    m_ColBtn = Color32FromARGBf(0.2f+0.4f*a, r, g, b);
    ColorHLSToRGBf(h, l-0.35f, s, &r, &g, &b);
    m_ColHighBtn = Color32FromARGBf(0.4f+0.4f*a, r, g, b);
    ColorHLSToRGBf(h, l-0.25f, s, &r, &g, &b);
    m_ColFold = Color32FromARGBf(0.1f+0.4f*a, r, g, b);
    ColorHLSToRGBf(h, l-0.35f, s, &r, &g, &b);
    m_ColHighFold = Color32FromARGBf(0.3f+0.4f*a, r, g, b);

    ColorHLSToRGBf(h, 0.75f, s, &r, &g, &b);
    m_ColHelpBg = Color32FromARGBf(0.2f, 1, 1, 1);
    m_ColHelpText = lightText ? Color32FromARGBf(1, 0.2f, 1.0f, 0.2f) : Color32FromARGBf(1, 0, 0.4f, 0);
    m_ColSeparator = m_ColValTextRO;
    m_ColStaticText = m_ColHelpText;
}

/*
void CTwBar::UpdateColors()
{
    float a, r, g, b, h, l, s;
    Color32ToARGBf(m_Color, &a, &r, &g, &b);
    ColorRGBToHLSf(r, g, b, &h, &l, &s);
    bool lightText = !m_DarkText; // (l<=0.45f);
    l = 0.2f + 0.6f*l;
    
    ColorHLSToRGBf(h, l, s, &r, &g, &b);
    m_ColBg = Color32FromARGBf(a, r, g, b);
    ColorHLSToRGBf(h, l-0.1f, s, &r, &g, &b);
    m_ColBg1 = Color32FromARGBf(a, r, g, b);
    ColorHLSToRGBf(h, l-0.2f, s, &r, &g, &b);
    m_ColBg2 = Color32FromARGBf(a, r, g, b);

    ColorHLSToRGBf(h, l+0.1f, s, &r, &g, &b);
    m_ColHighBg = Color32FromARGBf(0.4f, r, g, b);
    //m_ColHighBg = Color32FromARGBf(a, 0.95f, 0.95f, 0.2f);
    
    m_ColLabelText = lightText ? COLOR32_WHITE : COLOR32_BLACK;
    m_ColStructText = lightText ? 0xffefef00 : 0xff505000;

    m_ColValText = lightText ? 0xffb7b7ff : 0xff000080;
    m_ColValTextRO = lightText ? 0xffb7b7b7 : 0xff505050;
    m_ColValMin = lightText ? 0xff9797ff : 0xff0000f0;
    m_ColValMax = m_ColValMin;
    m_ColValTextNE = lightText ? 0xff97f797 : 0xff006000;

    ColorHLSToRGBf(h, lightText ? (min(l+0.2f, 0.3f)) : (max(l-0.2f, 0.6f)), s, &r, &g, &b);
    m_ColValBg = Color32FromARGBf(0.4f*a, 0, 0, 0);
    m_ColStructBg = Color32FromARGBf(0.4f*a, 0, 0, 0);

    ColorHLSToRGBf(h, 0.4f, s, &r, &g, &b);
    m_ColTitleBg = Color32FromARGBf(a+0.4f, r, g, b);
    m_ColTitleText = lightText ? COLOR32_WHITE : COLOR32_BLACK;
    m_ColTitleShadow = lightText ? 0x80000000 : 0x80ffffff;
    ColorHLSToRGBf(h, 0.3f, s, &r, &g, &b);
    m_ColTitleHighBg = Color32FromARGBf(a+0.4f, r, g, b);
    ColorHLSToRGBf(h, 0.4f, s, &r, &g, &b);
    m_ColTitleUnactiveBg = Color32FromARGBf(a+0.2f, r, g, b);

    ColorHLSToRGBf(h, 0.8f, s, &r, &g, &b);
    m_ColLine = Color32FromARGBf(0.6f, r, g, b); // 0xfff0f0f0;
    m_ColLineShadow = Color32FromARGBf(0.6f, 0, 0, 0); //COLOR32_BLACK;
    m_ColUnderline = lightText ? 0xffd0d0d0 : 0xff202000;
    ColorHLSToRGBf(h, 0.7f, s, &r, &g, &b);
    m_ColBtn = Color32FromARGBf(0.6f, r, g, b);
    ColorHLSToRGBf(h, 0.4f, s, &r, &g, &b);
    m_ColHighBtn = Color32FromARGBf(0.6f, r, g, b);
    ColorHLSToRGBf(h, 0.6f, s, &r, &g, &b);
    m_ColFold = Color32FromARGBf(0.3f*a, r, g, b);
    ColorHLSToRGBf(h, 0.4f, s, &r, &g, &b);
    m_ColHighFold = Color32FromARGBf(0.3f, r, g, b);

    ColorHLSToRGBf(h, lightText ? l+0.2f : l-0.2f, s, &r, &g, &b);
    m_ColGrpBg = Color32FromARGBf(0.5f*a, r, g, b);
    m_ColGrpText = lightText ? 0xffffff80 : 0xff404000;

    ColorHLSToRGBf(h, 0.75f, s, &r, &g, &b);
    m_ColHelpBg = Color32FromARGBf(a, r, g, b);
    m_ColHelpText = Color32FromARGBf(1, 0, 0.4f, 0);

    ColorHLSToRGBf(h, 0.45f, s, &r, &g, &b);
    m_ColHierBg = Color32FromARGBf(0.75f*a, r, g, b);

    m_ColShortcutText = lightText ? 0xffff8040 : 0xff802000;  //0xfff0f0f0;
    m_ColShortcutBg = Color32FromARGBf(0.4f*a, 0.2f, 0.2f, 0.2f);
    m_ColInfoText = Color32FromARGBf(1.0f, 0.7f, 0.7f, 0.7f);

    m_ColRoto = Color32FromARGBf(1, 0.75f, 0.75f, 0.75f);
    m_ColRotoVal = Color32FromARGBf(1, 1.0f, 0.2f, 0.2f);
    m_ColRotoBound = Color32FromARGBf(1, 0.4f, 0.4f, 0.4f);

    m_ColEditText = lightText ? COLOR32_WHITE : COLOR32_BLACK;
    m_ColEditBg = lightText ? 0xb7575757 : 0xb7c7c7c7;
    m_ColEditSelText = lightText ? COLOR32_BLACK : COLOR32_WHITE;
    m_ColEditSelBg = lightText ? 0xffc7c7c7 : 0xff575757;

    m_ColSeparator = m_ColValTextRO;
    m_ColStaticText = m_ColHelpText;
}
*/

//  ---------------------------------------------------------------------------

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

//  ---------------------------------------------------------------------------

static inline int IncrBtnWidth(int _CharHeight) 
{ 
    return ((2*_CharHeight)/3+2)&0xfffe; // force even value 
}

//  ---------------------------------------------------------------------------

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

//  ---------------------------------------------------------------------------

void CTwBar::ListLabels(vector<string>& _Labels, vector<color32>& _Colors, vector<color32>& _BgColors, bool *_HasBgColors, const CTexFont *_Font, int _AtomWidthMax, int _GroupWidthMax)
{
    const int NbEtc = 2;
    string ValStr;
    int Len, i, x, Etc, s;
    const unsigned char *Text;
    unsigned char ch;
    int WidthMax;
    
    int Space = _Font->m_CharWidth[(int)' '];
    int LevelSpace = max(_Font->m_CharHeight-6, 4); // space used by DrawHierHandles

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
        _Labels.push_back("");  // add a new text line
        if( !m_HierTags[h].m_Var->IsGroup() && static_cast<const CTwVarAtom *>(m_HierTags[h].m_Var)->m_Type==TW_TYPE_BUTTON && static_cast<const CTwVarAtom *>(m_HierTags[h].m_Var)->m_ReadOnly && static_cast<const CTwVarAtom *>(m_HierTags[h].m_Var)->m_Val.m_Button.m_Callback!=NULL )
            _Colors.push_back(m_ColValTextRO); // special case for read-only buttons
        else
            _Colors.push_back(m_HierTags[h].m_Var->m_ColorPtr!=NULL ? *(m_HierTags[h].m_Var->m_ColorPtr) : COLOR32_WHITE);
        color32 bg = m_HierTags[h].m_Var->m_BgColorPtr!=NULL ? *(m_HierTags[h].m_Var->m_BgColorPtr) : 0;
        _BgColors.push_back(bg);
        if( _HasBgColors!=NULL && bg!=0 )
            *_HasBgColors = true;
        bool IsCustom = m_HierTags[h].m_Var->IsCustom(); // !m_HierTags[h].m_Var->IsGroup() && (static_cast<const CTwVarAtom *>(m_HierTags[h].m_Var)->m_Type>=TW_TYPE_CUSTOM_BASE && static_cast<const CTwVarAtom *>(m_HierTags[h].m_Var)->m_Type<TW_TYPE_CUSTOM_BASE+(int)g_TwMgr->m_Customs.size());
        if( !IsCustom )
        {
            string& CurrentLabel = _Labels[_Labels.size()-1];
            if( m_HierTags[h].m_Var->IsGroup() && static_cast<const CTwVarGroup *>(m_HierTags[h].m_Var)->m_SummaryCallback==NULL )
                WidthMax = _GroupWidthMax;
            else if( !m_HierTags[h].m_Var->IsGroup() && static_cast<const CTwVarAtom *>(m_HierTags[h].m_Var)->m_Type==TW_TYPE_BUTTON )
            {
                if( static_cast<const CTwVarAtom *>(m_HierTags[h].m_Var)->m_Val.m_Button.m_Callback==NULL )
                    WidthMax = _GroupWidthMax;
                else if( m_ButtonAlign == BUTTON_ALIGN_RIGHT )
                    WidthMax = _GroupWidthMax - 2*IncrBtnWidth(m_Font->m_CharHeight);
                else
                    WidthMax = _AtomWidthMax;
            }
            //else if( m_HighlightedLine==h && m_DrawRotoBtn )
            //  WidthMax = _AtomWidthMax - IncrBtnWidth(m_Font->m_CharHeight);
            else
                WidthMax = _AtomWidthMax;
            if( Space>0 )
                for( s=0; s<m_HierTags[h].m_Level*LevelSpace; s+=Space )
                {
                    CurrentLabel += ' ';
                    x += Space;
                }
            if( x+(NbEtc+2)*_Font->m_CharWidth[(int)'.']<WidthMax || m_HierTags[h].m_Var->m_DontClip)
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
}

//  ---------------------------------------------------------------------------

void CTwBar::ListValues(vector<string>& _Values, vector<color32>& _Colors, vector<color32>& _BgColors, const CTexFont *_Font, int _WidthMax)
{
    CTwFPU fpu; // force fpu precision

    const int NbEtc = 2;
    const CTwVarAtom *Atom = NULL;
    string ValStr;
    int Len, i, x, Etc;
    const unsigned char *Text;
    unsigned char ch;
    bool ReadOnly;
    bool IsMax;
    bool IsMin;
    bool IsROText;
    bool HasBgColor;
    bool AcceptEdit;
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
            IsROText = false;
            HasBgColor = true;
            AcceptEdit = false;
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
                if( Atom->m_Type==TW_TYPE_BOOLCPP || Atom->m_Type==TW_TYPE_BOOL8 || Atom->m_Type==TW_TYPE_BOOL16 || Atom->m_Type==TW_TYPE_BOOL32 )
                {
                    if (ValStr=="1")
                        ValStr = "\x7f"; // check sign
                    else if (ValStr=="0")
                        ValStr = " -"; //"\x97"; // uncheck sign
                }
                if(    (Atom->m_Type==TW_TYPE_CDSTRING && Atom->m_SetCallback==NULL && g_TwMgr->m_CopyCDStringToClient==NULL)
                    || (Atom->m_Type==TW_TYPE_CDSTDSTRING && Atom->m_SetCallback==NULL)
                    || (Atom->m_Type==TW_TYPE_STDSTRING && Atom->m_SetCallback==NULL && g_TwMgr->m_CopyStdStringToClient==NULL) )
                    IsROText = true;
                if( Atom->m_Type==TW_TYPE_HELP_ATOM || Atom->m_Type==TW_TYPE_HELP_GRP || Atom->m_Type==TW_TYPE_BUTTON || Atom->IsCustom() ) // (Atom->m_Type>=TW_TYPE_CUSTOM_BASE && Atom->m_Type<TW_TYPE_CUSTOM_BASE+(int)g_TwMgr->m_Customs.size()) )
                    HasBgColor = false;
                AcceptEdit = EditInPlaceAcceptVar(Atom) || (Atom->m_Type==TW_TYPE_SHORTCUT);
            }
            else if(m_HierTags[h].m_Var->IsGroup() && static_cast<const CTwVarGroup *>(m_HierTags[h].m_Var)->m_SummaryCallback!=NULL)
            {
                const CTwVarGroup *Grp = static_cast<const CTwVarGroup *>(m_HierTags[h].m_Var);
                // force internal value update
                for( size_t v=0; v<Grp->m_Vars.size(); v++ ) 
                    if( Grp->m_Vars[v]!=NULL && !Grp->m_Vars[v]->IsGroup() && Grp->m_Vars[v]->m_Visible )
                        static_cast<CTwVarAtom *>(Grp->m_Vars[v])->ValueToDouble();

                Summary[0] = '\0';
                if( Grp->m_SummaryCallback==CTwMgr::CStruct::DefaultSummary )
                    Grp->m_SummaryCallback(&Summary[0], SummaryMaxLength, Grp, Grp->m_SummaryClientData);
                else
                    Grp->m_SummaryCallback(&Summary[0], SummaryMaxLength, Grp->m_StructValuePtr, Grp->m_SummaryClientData);
                ValStr = (const char *)(&Summary[0]);
            }
            else
            {
                ValStr = "";    // is a group in the help bar
                HasBgColor = false;
            }
            Len = (int)ValStr.length();
            Text = (const unsigned char *)(ValStr.c_str());
            x = 0;
            Etc = 0;
            _Values.push_back("");  // add a new text line
            if( ReadOnly || (IsMin && IsMax) || IsROText )
                _Colors.push_back(m_ColValTextRO);
            else if( IsMin )
                _Colors.push_back(m_ColValMin);
            else if( IsMax )
                _Colors.push_back(m_ColValMax);
            else if( !AcceptEdit )
                _Colors.push_back(m_ColValTextNE);
            else
                _Colors.push_back(m_ColValText);
            if( !HasBgColor )
                _BgColors.push_back(0x00000000);
            else if( m_HierTags[h].m_Var->IsGroup() )
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
            if( m_HighlightedLine==h && m_DrawRotoBtn )
                wmax -= 3*IncrBtnWidth(m_Font->m_CharHeight);
            else if( m_HighlightedLine==h && m_DrawIncrDecrBtn )
                wmax -= 2*IncrBtnWidth(m_Font->m_CharHeight);
            else if( m_HighlightedLine==h && m_DrawListBtn )
                wmax -= 1*IncrBtnWidth(m_Font->m_CharHeight);
            else if( m_HighlightedLine==h && m_DrawBoolBtn )
                wmax -= 1*IncrBtnWidth(m_Font->m_CharHeight);
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
                else if( i<Len-2 && x+(NbEtc+2)*(_Font->m_CharWidth[(int)'.'])>=wmax )
                    Etc = 1;
            }
        }
        else
        {
            _Values.push_back("");  // add a new empty line
            _Colors.push_back(COLOR32_BLACK);
            _BgColors.push_back(0x00000000);
        }
}

//  ---------------------------------------------------------------------------

int CTwBar::ComputeLabelsWidth(const CTexFont *_Font)
{
    int Len, i, x, s;
    const unsigned char *Text;
    int LabelsWidth = 0;    
    int Space = _Font->m_CharWidth[(int)' '];
    int LevelSpace = max(_Font->m_CharHeight-6, 4); // space used by DrawHierHandles

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
        bool IsCustom = m_HierTags[h].m_Var->IsCustom(); // !m_HierTags[h].m_Var->IsGroup() && (static_cast<const CTwVarAtom *>(m_HierTags[h].m_Var)->m_Type>=TW_TYPE_CUSTOM_BASE && static_cast<const CTwVarAtom *>(m_HierTags[h].m_Var)->m_Type<TW_TYPE_CUSTOM_BASE+(int)g_TwMgr->m_Customs.size());
        if( !IsCustom )
        {
            if( Space>0 )
                for( s=0; s<m_HierTags[h].m_Level*LevelSpace; s+=Space )
                    x += Space;
            for( i=0; i<Len; ++i )
                x += _Font->m_CharWidth[(int)Text[i]];
            x += 3*Space; // add little margin
        }
        if (x > LabelsWidth)
            LabelsWidth = x;
    }

    return LabelsWidth;
}

int CTwBar::ComputeValuesWidth(const CTexFont *_Font)
{
    CTwFPU fpu; // force fpu precision

    const CTwVarAtom *Atom = NULL;
    string ValStr;
    int Len, i, x;
    int Space = _Font->m_CharWidth[(int)' '];
    const unsigned char *Text;
    int ValuesWidth = 0;

    int nh = (int)m_HierTags.size();
    for( int h=0; h<nh; ++h )
        if( !m_HierTags[h].m_Var->IsGroup() )
        {
            Atom = static_cast<const CTwVarAtom *>(m_HierTags[h].m_Var);
            Atom->ValueToString(&ValStr);

            Len = (int)ValStr.length();
            Text = (const unsigned char *)(ValStr.c_str());
            x = 0;
            for( i=0; i<Len; ++i )
                x += _Font->m_CharWidth[(int)Text[i]];
            x += 2*Space; // add little margin
            if (x > ValuesWidth)
                ValuesWidth = x;
        }

    return ValuesWidth;
}

//  ---------------------------------------------------------------------------

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

//  ---------------------------------------------------------------------------

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

    bool ValuesWidthFit = false;
    if( m_ValuesWidth==VALUES_WIDTH_FIT )
    {
        ValuesWidthFit = true;
        m_ValuesWidth = 0;
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
        if( m_Resizable )
        {
            if( m_Width>vpw && m_Contained )
            {
                m_Width = vpw;
                Modif = true;
            }
            if( m_Width<8*m_Font->m_CharHeight )
            {
                m_Width = 8*m_Font->m_CharHeight;
                Modif = true;
            }
            if( m_Height>vph && m_Contained )
            {
                m_Height = vph;
                Modif = true;
            }
            if( m_Height<5*m_Font->m_CharHeight )
            {
                m_Height = 5*m_Font->m_CharHeight;
                Modif = true;
            }
        }
        if( m_Movable && m_Contained )
        {
            if( m_PosX+m_Width>vpx+vpw )
                m_PosX = vpx+vpw-m_Width;
            if( m_PosX<vpx )
                m_PosX = vpx;
            if( m_PosY+m_Height>vpy+vph )
                m_PosY = vpy+vph-m_Height;
            if( m_PosY<vpy )
                m_PosY = vpy;
        }
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
        if (ValuesWidthFit)
            Modif = true;
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

    int NbLines = (m_VarY1-m_VarY0+1)/(m_Font->m_CharHeight+m_LineSep);
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

        if( ValuesWidthFit )
        {
            m_ValuesWidth = ComputeValuesWidth(m_Font);
            if( m_ValuesWidth<2*m_Font->m_CharHeight )
                m_ValuesWidth = 2*m_Font->m_CharHeight; // enough to draw buttons
            if( m_ValuesWidth>m_VarX2 - m_VarX0 )
                m_ValuesWidth = max(m_VarX2 - m_VarX0 - m_Font->m_CharHeight, 0);
            m_VarX1 = m_VarX2 - m_ValuesWidth;
            if( m_VarX1<m_VarX0+32 )
                m_VarX1 = m_VarX0+32;
            if( m_VarX1>m_VarX2 )
                m_VarX1 = m_VarX2;
            m_ValuesWidth = m_VarX2 - m_VarX1;
        }
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
        vector<color32> BgColors;
        bool HasBgColors = false;
        ListLabels(Labels, Colors, BgColors, &HasBgColors, m_Font, m_VarX1-m_VarX0, m_VarX2-m_VarX0);
        assert( Labels.size()==Colors.size() && Labels.size()==BgColors.size() );
        if( Labels.size()>0 )
            Gr->BuildText(m_LabelsTextObj, &(Labels[0]), &(Colors[0]), &(BgColors[0]), (int)Labels.size(), m_Font, m_LineSep, HasBgColors ? m_VarX1-m_VarX0-m_Font->m_CharHeight+2 : 0);
        else
            Gr->BuildText(m_LabelsTextObj, NULL, NULL, NULL, 0, m_Font, m_LineSep, 0);

        // Should draw click button?
        m_DrawClickBtn    = ( m_VarX2-m_VarX1>4*IncrBtnWidth(m_Font->m_CharHeight)
                              && m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size()
                              && m_HierTags[m_HighlightedLine].m_Var!=NULL 
                              && !m_HierTags[m_HighlightedLine].m_Var->IsGroup()
                              && !static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_ReadOnly
                              && (    static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type==TW_TYPE_BUTTON ));
                            //     || static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type==TW_TYPE_BOOLCPP
                            //     || static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type==TW_TYPE_BOOL8
                            //     || static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type==TW_TYPE_BOOL16
                            //     || static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type==TW_TYPE_BOOL32 ));

        // Should draw [-/+] button?
        m_DrawIncrDecrBtn = ( m_VarX2-m_VarX1>5*IncrBtnWidth(m_Font->m_CharHeight)
                              && m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size()
                              && m_HierTags[m_HighlightedLine].m_Var!=NULL 
                              && !m_HierTags[m_HighlightedLine].m_Var->IsGroup()
                              && static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type!=TW_TYPE_BUTTON
                              && !static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_ReadOnly
                              && !static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_NoSlider 
                              && !(m_EditInPlace.m_Active && m_EditInPlace.m_Var==m_HierTags[m_HighlightedLine].m_Var) );

        // Should draw [v] button (list)?
        m_DrawListBtn     = ( m_VarX2-m_VarX1>2*IncrBtnWidth(m_Font->m_CharHeight)
                              && m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size()
                              && m_HierTags[m_HighlightedLine].m_Var!=NULL 
                              && !m_HierTags[m_HighlightedLine].m_Var->IsGroup()
                              && IsEnumType(static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type)
                              && !static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_ReadOnly );

        // Should draw [<>] button (bool)?
        m_DrawBoolBtn     = ( m_VarX2-m_VarX1>4*IncrBtnWidth(m_Font->m_CharHeight)
                              && m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size()
                              && m_HierTags[m_HighlightedLine].m_Var!=NULL 
                              && !m_HierTags[m_HighlightedLine].m_Var->IsGroup()
                              && !static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_ReadOnly
                              && (    static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type==TW_TYPE_BOOLCPP
                                   || static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type==TW_TYPE_BOOL8
                                   || static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type==TW_TYPE_BOOL16
                                   || static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type==TW_TYPE_BOOL32 ));

        // Should draw [o] button?
        m_DrawRotoBtn     = m_DrawIncrDecrBtn;
        /*
        m_DrawRotoBtn     = ( m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size()
                              && m_HierTags[m_HighlightedLine].m_Var!=NULL 
                              && !m_HierTags[m_HighlightedLine].m_Var->IsGroup()
                              && static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type!=TW_TYPE_BUTTON
                              && !static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_ReadOnly
                              && !static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_NoSlider );
        */

        // Build values
        vector<string>& Values = Labels;    // reuse
        Values.resize(0);
        Colors.resize(0);
        BgColors.resize(0);
        ListValues(Values, Colors, BgColors, m_Font, m_VarX2-m_VarX1);
        assert( BgColors.size()==Values.size() && Colors.size()==Values.size() );
        if( Values.size()>0 )
            Gr->BuildText(m_ValuesTextObj, &(Values[0]), &(Colors[0]), &(BgColors[0]), (int)Values.size(), m_Font, m_LineSep, m_VarX2-m_VarX1);
        else
            Gr->BuildText(m_ValuesTextObj, NULL, NULL, NULL, 0, m_Font, m_LineSep, m_VarX2-m_VarX1);

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

        // build headers text
        if (m_HighlightLabelsHeader || m_HighlightValuesHeader) {
            std::string HeadersText = "Fit column content";
            ClampText(HeadersText, m_Font, m_Width-3*m_Font->m_CharHeight);
            Gr->BuildText(m_HeadersTextObj, &HeadersText, NULL, NULL, 1, m_Font, 0, 0);
        }
    }

    if( DoEndDraw )
        Gr->EndDraw();

    m_UpToDate = true;
    m_LastUpdateTime = float(g_BarTimer.GetTime());
}

//  ---------------------------------------------------------------------------

void CTwBar::DrawHierHandle()
{
    assert(m_Font);
    ITwGraph *Gr = g_TwMgr->m_Graph;

    //int x0 = m_PosX+m_Font->m_CharHeight+1;
    int x0 = m_PosX+3;
    //int x2 = m_PosX+m_VarX0-5;
    //int x2 = m_PosX+3*m_Font->m_CharWidth[(int)' ']-2;
    int x2 = m_PosX+m_Font->m_CharHeight-3;
    if( x2-x0<4 )
        x2 = x0+4;
    if( (x2-x0)&1 )
        --x2;
    int x1 = (x0+x2)/2;
    int w = x2-x0+1;
    int y0 = m_PosY+m_VarY0 +1;
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

            int dx = m_HierTags[h].m_Level * (x2-x0);

            if( Grp )
            {
                if( m_ColGrpBg!=0 && Grp->m_StructValuePtr==NULL )
                {
                    color32 cb = (Grp->m_StructType==TW_TYPE_HELP_STRUCT) ? m_ColStructBg : m_ColGrpBg;
                    //Gr->DrawRect(x0+dx-1, y0, m_PosX+m_VarX2, y0+m_Font->m_CharHeight-1, cb);
                    Gr->DrawRect(x2+dx+3, y0, m_PosX+m_VarX2, y0+m_Font->m_CharHeight-1+m_LineSep-1, cb);
                }

                if( m_DrawHandles )
                {
                    Gr->DrawLine(dx+x2+1,y0+dh0+1, dx+x2+1,y0+dh1+1, m_ColLineShadow);
                    Gr->DrawLine(dx+x0+1,y0+dh1+1, dx+x2+2,y0+dh1+1, m_ColLineShadow);
                }

                //Gr->DrawRect(x0+1,y0+dh0+1,x2-1,y0+dh1-1, (h==m_HighlightedLine) ? m_ColHighBtn : m_ColBtn);
                Gr->DrawRect(dx+x0, y0+dh0, dx+x2, y0+dh1, (h==m_HighlightedLine) ? m_ColHighFold : m_ColFold);
                if( m_DrawHandles )
                {
                    Gr->DrawLine(dx+x0,y0+dh0, dx+x2,y0+dh0, m_ColLine);
                    Gr->DrawLine(dx+x2,y0+dh0, dx+x2,y0+dh1+1, m_ColLine);
                    Gr->DrawLine(dx+x2,y0+dh1, dx+x0,y0+dh1, m_ColLine);
                    Gr->DrawLine(dx+x0,y0+dh1, dx+x0,y0+dh0, m_ColLine);
                }
                
                Gr->DrawLine(dx+x0+2,y0+dh0+w/2, dx+x2-1,y0+dh0+w/2, m_ColTitleText);
                if( !Grp->m_Open )
                    Gr->DrawLine(dx+x1,y0+dh0+2, dx+x1,y0+dh1-1, m_ColTitleText);

                /*
                if( m_ColGrpBg!=0 && Grp->m_StructValuePtr==NULL )
                {
                    color32 cb = (Grp->m_StructType==TW_TYPE_HELP_STRUCT) ? m_ColStructBg : m_ColGrpBg;
                    //int decal = m_Font->m_CharHeight/2-2+2*m_HierTags[h].m_Level;
                    //if( decal>m_Font->m_CharHeight-3 )
                    //  decal = m_Font->m_CharHeight-3;
                    int margin = dx; //m_Font->m_CharWidth[(int)' ']*m_HierTags[h].m_Level;
                    //Gr->DrawRect(m_PosX+m_VarX0+margin, y0+decal, m_PosX+m_VarX2, y0+m_Font->m_CharHeight-1, cb);
                    Gr->DrawRect(m_PosX+m_VarX0+margin-1, y0+1, m_PosX+m_VarX2, y0+m_Font->m_CharHeight, cb);// m_ColHierBg);
                    //Gr->DrawRect(m_PosX+m_VarX0-4, y0+m_Font->m_CharHeight/2-1, m_PosX+m_VarX0+margin-2, y0+m_Font->m_CharHeight/2, m_ColHierBg);
                }
                */
            }
            else if( static_cast<CTwVarAtom *>(m_HierTags[h].m_Var)->m_Type==TW_TYPE_HELP_GRP && m_ColHelpBg!=0 )
                Gr->DrawRect(m_PosX+m_VarX0+m_HierTags[h].m_Var->m_LeftMargin, y0+m_HierTags[h].m_Var->m_TopMargin, m_PosX+m_VarX2, y0+m_Font->m_CharHeight-1, m_ColHelpBg);
            //else if( static_cast<CTwVarAtom *>(m_HierTags[h].m_Var)->m_Type==TW_TYPE_HELP_HEADER && m_ColHelpBg!=0 )
            //  Gr->DrawRect(m_PosX+m_VarX0+m_HierTags[h].m_Var->m_LeftMargin, y0+m_HierTags[h].m_Var->m_TopMargin, m_PosX+m_VarX2, y0+m_Font->m_CharHeight-1, m_ColHelpBg);
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

            y0 = y1+m_LineSep;
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

        Gr->DrawRect(x0+2,y0+w, x1-2,y1-1-w, (m_ColBg&0xffffff)|0x11000000);
        if( m_DrawHandles || m_IsPopupList )
        {
            // scroll handle shadow lines
            Gr->DrawLine(x1-1,m_ScrollY0+1, x1-1,m_ScrollY1+1, m_ColLineShadow);
            Gr->DrawLine(x0+2,m_ScrollY1+1, x1,m_ScrollY1+1, m_ColLineShadow);
            
            // up & down arrow
            for( i=0; i<(x1-x0-2)/2; ++i )
            {
                Gr->DrawLine(x0+2+i,y0+w-2*i, x1-i,y0+w-2*i, m_ColLineShadow);
                Gr->DrawLine(x0+1+i,y0+w-1-2*i, x1-1-i,y0+w-1-2*i, m_HighlightUpScroll?((m_ColLine&0xffffff)|0x4f000000):m_ColLine);

                Gr->DrawLine(x0+2+i,y1-w+2+2*i, x1-i,y1-w+2+2*i, m_ColLineShadow);
                Gr->DrawLine(x0+1+i,y1-w+1+2*i, x1-1-i,y1-w+1+2*i, m_HighlightDnScroll?((m_ColLine&0xffffff)|0x4f000000):m_ColLine);
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
            Gr->DrawRect(x0+3,m_ScrollY0+1, x1-3,m_ScrollY1-1, m_ColBtn);
    }

    if( m_DrawHandles && !m_IsPopupList )
    {
        if( m_Resizable ) // Draw resize handles
        {
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
        }

        int xm = m_PosX+m_Width-2*m_Font->m_CharHeight, wm=m_Font->m_CharHeight-6;
        wm = (wm<6) ? 6 : wm;
        if( m_Iconifiable ) // Draw minimize button
        {
            Gr->DrawRect(xm+1, m_PosY+4, xm+wm-1, m_PosY+3+wm, m_HighlightMinimize?m_ColHighBtn:((m_ColBtn&0xffffff)|0x4f000000));
            Gr->DrawLine(xm, m_PosY+3, xm+wm, m_PosY+3, m_ColLine);
            Gr->DrawLine(xm+wm, m_PosY+3, xm+wm, m_PosY+3+wm, m_ColLine);
            Gr->DrawLine(xm+wm, m_PosY+3+wm, xm, m_PosY+3+wm, m_ColLine);
            Gr->DrawLine(xm, m_PosY+3+wm, xm, m_PosY+3, m_ColLine);
            Gr->DrawLine(xm+wm+1, m_PosY+4, xm+wm+1, m_PosY+4+wm, m_ColLineShadow);
            Gr->DrawLine(xm+wm+1, m_PosY+4+wm, xm, m_PosY+4+wm, m_ColLineShadow);
            Gr->DrawLine(xm+wm/3+((wm<9)?1:0)-1, m_PosY+4+wm/3-((wm<9)?0:1), xm+wm/2, m_PosY+2+wm-1, m_ColTitleText, true);
            Gr->DrawLine(xm+wm-wm/3+((wm<9)?0:1), m_PosY+4+wm/3-((wm<9)?0:1), xm+wm/2, m_PosY+2+wm-1, m_ColTitleText, true);
        }

        if( g_TwMgr->m_FontResizable ) // Draw font button
        {
            xm = m_PosX+m_Font->m_CharHeight+2;
            Gr->DrawRect(xm+1, m_PosY+4, xm+wm-1, m_PosY+3+wm, m_HighlightFont?m_ColHighBtn:((m_ColBtn&0xffffff)|0x4f000000));
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
}

//  ---------------------------------------------------------------------------

void CTwBar::Draw(int _DrawPart)
{
    PERF( PerfTimer Timer; double DT; )

    assert(m_Font);
    ITwGraph *Gr = g_TwMgr->m_Graph;

    m_CustomRecords.clear();

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
        int LevelSpace = max(m_Font->m_CharHeight-6, 4); // space used by DrawHierHandles

        color32 colBg = m_ColBg, colBg1 = m_ColBg1, colBg2 = m_ColBg2;
        if( m_DrawHandles || m_IsPopupList )
        {
            unsigned int alphaMin = 0x70;
            if( m_IsPopupList )
                alphaMin = 0xa0;
            if( (colBg>>24)<alphaMin )
                colBg = (colBg&0xffffff)|(alphaMin<<24);
            if( (colBg1>>24)<alphaMin )
                colBg1 = (colBg1&0xffffff)|(alphaMin<<24);
            if( (colBg2>>24)<alphaMin )
                colBg2 = (colBg2&0xffffff)|(alphaMin<<24);
        }

        // Draw title
        if( !m_IsPopupList )
        {
            PERF( Timer.Reset(); )
            if( _DrawPart&DRAW_BG )
            {
                //Gr->DrawRect(m_PosX, m_PosY, m_PosX+m_Width-1, m_PosY+m_Font->m_CharHeight+1, (m_HighlightTitle||m_MouseDragTitle) ? m_ColTitleHighBg : (m_DrawHandles ? m_ColTitleBg : m_ColTitleUnactiveBg));
                if( m_HighlightTitle || m_MouseDragTitle )
                    Gr->DrawRect(m_PosX, m_PosY, m_PosX+m_Width-1, m_PosY+m_Font->m_CharHeight+1, m_ColTitleHighBg);
                else if (m_DrawHandles)
                    Gr->DrawRect(m_PosX, m_PosY, m_PosX+m_Width-1, m_PosY+m_Font->m_CharHeight+1, m_ColTitleBg, m_ColTitleBg, colBg2, colBg1);
                else
                    Gr->DrawRect(m_PosX, m_PosY, m_PosX+m_Width-1, m_PosY+m_Font->m_CharHeight+1, m_ColTitleBg, m_ColTitleBg, colBg2, colBg1);
            }
            if( _DrawPart&DRAW_CONTENT )
            {
                const color32 COL0 = 0x50ffffff;
                const color32 COL1 = 0x501f1f1f;
                Gr->DrawRect(m_PosX, m_PosY, m_PosX+m_Width-1, y, COL0, COL0, COL1, COL1);
                if( m_ColTitleShadow!=0 )
                    Gr->DrawText(m_TitleTextObj, m_PosX+(m_Width-m_TitleWidth)/2+1, m_PosY+1, m_ColTitleShadow, 0);
                Gr->DrawText(m_TitleTextObj, m_PosX+(m_Width-m_TitleWidth)/2, m_PosY, m_ColTitleText, 0);
            }
            y = m_PosY+m_Font->m_CharHeight+1;
            if( _DrawPart&DRAW_CONTENT && m_DrawHandles )
                Gr->DrawLine(m_PosX, y, m_PosX+m_Width-1, y, 0x30ffffff); // 0x80afafaf);
            y++;
            PERF( DT = Timer.GetTime(); printf("Title=%.4fms ", 1000.0*DT); )
        }

        // Draw background
        PERF( Timer.Reset(); )
        if( _DrawPart&DRAW_BG )
        {
            Gr->DrawRect(m_PosX, y, m_PosX+m_Width-1, m_PosY+m_Height-1, colBg2, colBg1, colBg1, colBg);
            //Gr->DrawRect(m_PosX, y, m_PosX+m_VarX0-5, m_PosY+m_Height-1, m_ColHierBg);
            Gr->DrawRect(m_PosX+m_VarX2+3, y, m_PosX+m_Width-1, m_PosY+m_Height-1, m_ColHierBg);
        }

        if( _DrawPart&DRAW_CONTENT )
        {
            // Draw highlighted line
            if( m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size() && m_HierTags[m_HighlightedLine].m_Var!=NULL
                && (m_HierTags[m_HighlightedLine].m_Var->IsGroup() 
                    || (!static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_ReadOnly && !m_IsHelpBar 
                        && !m_HierTags[m_HighlightedLine].m_Var->IsCustom() ) ) ) // !(static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type>=TW_TYPE_CUSTOM_BASE && static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type<TW_TYPE_CUSTOM_BASE+(int)g_TwMgr->m_Customs.size()))) )
            {
                int y0 = m_PosY + m_VarY0 + m_HighlightedLine*(m_Font->m_CharHeight+m_LineSep);
                Gr->DrawRect(m_PosX+LevelSpace+6+LevelSpace*m_HierTags[m_HighlightedLine].m_Level, y0+1, m_PosX+m_VarX2, y0+m_Font->m_CharHeight-1+m_LineSep-1, m_ColHighBg0, m_ColHighBg0, m_ColHighBg1, m_ColHighBg1);
                int eps = (g_TwMgr->m_GraphAPI==TW_OPENGL || g_TwMgr->m_GraphAPI==TW_OPENGL_CORE) ? 1 : 0;
                if( !m_EditInPlace.m_Active )
                    Gr->DrawLine(m_PosX+LevelSpace+6+LevelSpace*m_HierTags[m_HighlightedLine].m_Level, y0+m_Font->m_CharHeight+m_LineSep-1+eps, m_PosX+m_VarX2, y0+m_Font->m_CharHeight+m_LineSep-1+eps, m_ColUnderline);
            }
            else if( m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size() && !m_HierTags[m_HighlightedLine].m_Var->IsGroup() )
            {
                int y0 = m_PosY + m_VarY0 + m_HighlightedLine*(m_Font->m_CharHeight+m_LineSep);
                color32 col = ColorBlend(m_ColHighBg0, m_ColHighBg1, 0.5f);
                CTwVarAtom *Atom = static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var);
                if( !Atom->IsCustom() // !(Atom->m_Type>=TW_TYPE_CUSTOM_BASE && Atom->m_Type<TW_TYPE_CUSTOM_BASE+(int)g_TwMgr->m_Customs.size()) 
                    && !(Atom->m_Type==TW_TYPE_BUTTON && Atom->m_Val.m_Button.m_Callback==NULL) )
                    Gr->DrawRect(m_PosX+LevelSpace+6+LevelSpace*m_HierTags[m_HighlightedLine].m_Level, y0+1, m_PosX+m_VarX2, y0+m_Font->m_CharHeight-1+m_LineSep-1, col);
                else
                    Gr->DrawRect(m_PosX+LevelSpace+6+LevelSpace*m_HierTags[m_HighlightedLine].m_Level, y0+1, m_PosX+LevelSpace+6+LevelSpace*m_HierTags[m_HighlightedLine].m_Level+4, y0+m_Font->m_CharHeight-1+m_LineSep-1, col);
            }
            color32 clight = 0x5FFFFFFF; // bar contour
            Gr->DrawLine(m_PosX, m_PosY, m_PosX, m_PosY+m_Height, clight);
            Gr->DrawLine(m_PosX, m_PosY, m_PosX+m_Width, m_PosY, clight);
            Gr->DrawLine(m_PosX+m_Width, m_PosY, m_PosX+m_Width, m_PosY+m_Height, clight);
            Gr->DrawLine(m_PosX, m_PosY+m_Height, m_PosX+m_Width, m_PosY+m_Height, clight);
            int dshad = 3;  // bar shadows
            color32 cshad = (((m_Color>>24)/2)<<24) & 0xFF000000;
            Gr->DrawRect(m_PosX, m_PosY+m_Height, m_PosX+dshad, m_PosY+m_Height+dshad, 0, cshad, 0, 0);
            Gr->DrawRect(m_PosX+dshad+1, m_PosY+m_Height, m_PosX+m_Width-1, m_PosY+m_Height+dshad, cshad, cshad, 0, 0);
            Gr->DrawRect(m_PosX+m_Width, m_PosY+m_Height, m_PosX+m_Width+dshad, m_PosY+m_Height+dshad, cshad, 0, 0, 0);
            Gr->DrawRect(m_PosX+m_Width, m_PosY, m_PosX+m_Width+dshad, m_PosY+dshad, 0, 0, cshad, 0);
            Gr->DrawRect(m_PosX+m_Width, m_PosY+dshad+1, m_PosX+m_Width+dshad, m_PosY+m_Height-1, cshad, 0, cshad, 0);
            PERF( DT = Timer.GetTime(); printf("Bg=%.4fms ", 1000.0*DT); )

            // Draw hierarchy handle
            PERF( Timer.Reset(); )
            DrawHierHandle();
            PERF( DT = Timer.GetTime(); printf("Handles=%.4fms ", 1000.0*DT); )

            // Draw labels
            PERF( Timer.Reset(); )
            Gr->DrawText(m_LabelsTextObj, m_PosX+LevelSpace+6, m_PosY+m_VarY0, 0 /*m_ColLabelText*/, 0);
            PERF( DT = Timer.GetTime(); printf("Labels=%.4fms ", 1000.0*DT); )

            // Draw values
            if( !m_IsPopupList )
            {
                PERF( Timer.Reset(); )
                Gr->DrawText(m_ValuesTextObj, m_PosX+m_VarX1, m_PosY+m_VarY0, 0 /*m_ColValText*/, 0 /*m_ColValBg*/);
                PERF( DT = Timer.GetTime(); printf("Values=%.4fms ", 1000.0*DT); )
            }

            // Draw preview for color values and draw buttons and custom types
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
                        if( Grp->m_Vars.size()>0 && Grp->m_Vars[0]!=NULL && !Grp->m_Vars[0]->IsGroup() )
                            static_cast<CTwVarAtom *>(Grp->m_Vars[0])->ValueToDouble(); // force ext update
                        int ydecal = (g_TwMgr->m_GraphAPI==TW_OPENGL || g_TwMgr->m_GraphAPI==TW_OPENGL_CORE) ? 1 : 0;
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
                    //else if( Grp->m_SummaryCallback==CustomTypeSummaryCB && Grp->m_StructValuePtr!=NULL )
                    //{
                    //}
                }
                else if( static_cast<CTwVarAtom *>(m_HierTags[h].m_Var)->m_Type==TW_TYPE_BUTTON && !m_IsPopupList )
                {
                    // draw button
                    int cbx0, cbx1;
                    if( m_ButtonAlign == BUTTON_ALIGN_LEFT )
                    {
                        cbx0 = m_PosX+m_VarX1+2;
                        cbx1 = m_PosX+m_VarX1+bw;
                    }
                    else if( m_ButtonAlign == BUTTON_ALIGN_CENTER )
                    {
                        cbx0 = m_PosX+(m_VarX1+m_VarX2)/2-bw/2+1;
                        cbx1 = m_PosX+(m_VarX1+m_VarX2)/2+bw/2-1;
                    }
                    else
                    {
                        cbx0 = m_PosX+m_VarX2-2*bw+bw/2;
                        cbx1 = m_PosX+m_VarX2-2-bw/2;
                    }
                    int cby0 = yh+3;
                    int cby1 = yh+m_Font->m_CharHeight-3;
                    if( !static_cast<CTwVarAtom *>(m_HierTags[h].m_Var)->m_ReadOnly )
                    {
                        double BtnAutoDelta = g_TwMgr->m_Timer.GetTime() - m_HighlightClickBtnAuto;
                        if( (m_HighlightClickBtn || (BtnAutoDelta>=0 && BtnAutoDelta<0.1)) && h==m_HighlightedLine )
                        {
                            cbx0--; cby0--; cbx1--; cby1--;
                            Gr->DrawRect(cbx0+2, cby0+2, cbx1+2, cby1+2, m_ColHighBtn);
                            Gr->DrawLine(cbx0+3, cby1+3, cbx1+4, cby1+3, 0xAF000000);
                            Gr->DrawLine(cbx1+3, cby0+3, cbx1+3, cby1+3, 0xAF000000);                       
                            Gr->DrawLine(cbx0+2, cby0+2, cbx0+2, cby1+2, m_ColLine);
                            Gr->DrawLine(cbx0+2, cby1+2, cbx1+2, cby1+2, m_ColLine);
                            Gr->DrawLine(cbx1+2, cby1+2, cbx1+2, cby0+2, m_ColLine);
                            Gr->DrawLine(cbx1+2, cby0+2, cbx0+2, cby0+2, m_ColLine);
                        }
                        else
                        {
                            Gr->DrawRect(cbx0+2, cby1+1, cbx1+2, cby1+2, (h==m_HighlightedLine)?0xAF000000:0x7F000000);
                            Gr->DrawRect(cbx1+1, cby0+2, cbx1+2, cby1, (h==m_HighlightedLine)?0xAF000000:0x7F000000);
                            Gr->DrawRect(cbx0, cby0, cbx1, cby1, (h==m_HighlightedLine)?m_ColHighBtn:m_ColBtn);
                            Gr->DrawLine(cbx0, cby0, cbx0, cby1, m_ColLine);
                            Gr->DrawLine(cbx0, cby1, cbx1, cby1, m_ColLine);
                            Gr->DrawLine(cbx1, cby1, cbx1, cby0, m_ColLine);
                            Gr->DrawLine(cbx1, cby0, cbx0, cby0, m_ColLine);
                        }
                    }
                    else if( static_cast<CTwVarAtom *>(m_HierTags[h].m_Var)->m_Val.m_Button.m_Callback!=NULL )
                    {
                        Gr->DrawRect(cbx0+1, cby0+1, cbx1+1, cby1+1, m_ColBtn);
                    }
                    else if( static_cast<CTwVarAtom *>(m_HierTags[h].m_Var)->m_Val.m_Button.m_Separator==1 )
                    {
                        int LevelSpace = max(m_Font->m_CharHeight-6, 4); // space used by DrawHierHandles
                        Gr->DrawLine(m_PosX+m_VarX0+m_HierTags[h].m_Level*LevelSpace, yh+m_Font->m_CharHeight/2, m_PosX+m_VarX2, yh+m_Font->m_CharHeight/2, m_ColSeparator );
                    }
                }
                else if( m_HierTags[h].m_Var->IsCustom() ) //static_cast<CTwVarAtom *>(m_HierTags[h].m_Var)->m_Type>=TW_TYPE_CUSTOM_BASE && static_cast<CTwVarAtom *>(m_HierTags[h].m_Var)->m_Type<TW_TYPE_CUSTOM_BASE+(int)g_TwMgr->m_Customs.size() )
                {   // record custom types
                    CTwMgr::CMemberProxy *mProxy = static_cast<CTwVarAtom *>(m_HierTags[h].m_Var)->m_Val.m_Custom.m_MemberProxy;
                    if( mProxy!=NULL && mProxy->m_StructProxy!=NULL )
                    {
                        CustomMap::iterator it = m_CustomRecords.find(mProxy->m_StructProxy);
                        int xMin = m_PosX + m_VarX0 + m_HierTags[h].m_Level*LevelSpace;
                        int xMax = m_PosX + m_VarX2 - 2;
                        int yMin = yh + 1;
                        int yMax = yh + m_Font->m_CharHeight;
                        if( it==m_CustomRecords.end() )
                        {
                            std::pair<CTwMgr::CStructProxy*, CCustomRecord> pr;
                            pr.first = mProxy->m_StructProxy;
                            pr.second.m_IndexMin = pr.second.m_IndexMax = mProxy->m_MemberIndex;
                            pr.second.m_XMin = xMin; 
                            pr.second.m_XMax = xMax;
                            pr.second.m_YMin = yMin;
                            pr.second.m_YMax = yMax;
                            pr.second.m_Y0 = 0; // will be filled by the draw loop below
                            pr.second.m_Y1 = 0; // will be filled by the draw loop below
                            pr.second.m_Var = mProxy->m_VarParent;
                            m_CustomRecords.insert(pr);
                        }
                        else
                        {
                            it->second.m_IndexMin = min(it->second.m_IndexMin, mProxy->m_MemberIndex);
                            it->second.m_IndexMax = min(it->second.m_IndexMax, mProxy->m_MemberIndex);
                            it->second.m_XMin = min(it->second.m_XMin, xMin);
                            it->second.m_XMax = max(it->second.m_XMax, xMax);
                            it->second.m_YMin = min(it->second.m_YMin, yMin);
                            it->second.m_YMax = max(it->second.m_YMax, yMax);
                            it->second.m_Y0 = 0;
                            it->second.m_Y1 = 0;
                            assert( it->second.m_Var==mProxy->m_VarParent );
                        }
                    }
                }

                yh += m_Font->m_CharHeight+m_LineSep;
            }

            // Draw custom types
            for( CustomMap::iterator it = m_CustomRecords.begin(); it!=m_CustomRecords.end(); ++it )
            {
                CTwMgr::CStructProxy *sProxy = it->first;
                assert( sProxy!=NULL );
                CCustomRecord& r = it->second;
                if( sProxy->m_CustomDrawCallback!=NULL )
                {
                    int y0 = r.m_YMin - max(r.m_IndexMin - sProxy->m_CustomIndexFirst, 0)*(m_Font->m_CharHeight + m_LineSep);
                    int y1 = y0 + max(sProxy->m_CustomIndexLast - sProxy->m_CustomIndexFirst + 1, 0)*(m_Font->m_CharHeight + m_LineSep) - 2;
                    if( y0<y1 )
                    {
                        r.m_Y0 = y0;
                        r.m_Y1 = y1;
                        Gr->ChangeViewport(r.m_XMin, r.m_YMin, r.m_XMax-r.m_XMin+1, r.m_YMax-r.m_YMin+1, 0, y0-r.m_YMin+1);
                        sProxy->m_CustomDrawCallback(r.m_XMax-r.m_XMin, y1-y0, sProxy->m_StructExtData, sProxy->m_StructClientData, this, r.m_Var);
                        Gr->RestoreViewport();
                    }
                }
            }

            if( m_DrawHandles && !m_IsPopupList )
            {
                // Draw -/+/o/click/v buttons
                if( (m_DrawIncrDecrBtn || m_DrawClickBtn || m_DrawListBtn || m_DrawBoolBtn || m_DrawRotoBtn) && m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size() )
                {
                    int y0 = m_PosY + m_VarY0 + m_HighlightedLine*(m_Font->m_CharHeight+m_LineSep);
                    if( m_DrawIncrDecrBtn )
                    {
                        bool IsMin = false;
                        bool IsMax = false;
                        if( !m_HierTags[m_HighlightedLine].m_Var->IsGroup() )
                        {
                            const CTwVarAtom *Atom = static_cast<const CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var);
                            double v, vmin, vmax;
                            v = Atom->ValueToDouble();
                            Atom->MinMaxStepToDouble(&vmin, &vmax, NULL);
                            IsMax = (v>=vmax);
                            IsMin = (v<=vmin);
                        }

                        /*
                        Gr->DrawRect(m_PosX+m_VarX2-2*bw+1, y0+1, m_PosX+m_VarX2-bw-1, y0+m_Font->m_CharHeight-2, (m_HighlightDecrBtn && !IsMin)?m_ColHighBtn:m_ColBtn);
                        Gr->DrawRect(m_PosX+m_VarX2-bw+1, y0+1, m_PosX+m_VarX2-1, y0+m_Font->m_CharHeight-2, (m_HighlightIncrBtn && !IsMax)?m_ColHighBtn:m_ColBtn);
                        // [-]
                        Gr->DrawLine(m_PosX+m_VarX2-2*bw+3+(bw>8?1:0), y0+m_Font->m_CharHeight/2, m_PosX+m_VarX2-bw-2-(bw>8?1:0), y0+m_Font->m_CharHeight/2, IsMin?m_ColValTextRO:m_ColTitleText);
                        // [+]
                        Gr->DrawLine(m_PosX+m_VarX2-bw+3, y0+m_Font->m_CharHeight/2, m_PosX+m_VarX2-2, y0+m_Font->m_CharHeight/2, IsMax?m_ColValTextRO:m_ColTitleText);
                        Gr->DrawLine(m_PosX+m_VarX2-bw/2, y0+m_Font->m_CharHeight/2-bw/2+2, m_PosX+m_VarX2-bw/2, y0+m_Font->m_CharHeight/2+bw/2-1, IsMax?m_ColValTextRO:m_ColTitleText);
                        */
                        Gr->DrawRect(m_PosX+m_VarX2-3*bw+1, y0+1, m_PosX+m_VarX2-2*bw-1, y0+m_Font->m_CharHeight-2, (m_HighlightDecrBtn && !IsMin)?m_ColHighBtn:m_ColBtn);
                        Gr->DrawRect(m_PosX+m_VarX2-2*bw+1, y0+1, m_PosX+m_VarX2-bw-1, y0+m_Font->m_CharHeight-2, (m_HighlightIncrBtn && !IsMax)?m_ColHighBtn:m_ColBtn);
                        // [-]
                        Gr->DrawLine(m_PosX+m_VarX2-3*bw+3+(bw>8?1:0), y0+m_Font->m_CharHeight/2, m_PosX+m_VarX2-2*bw-2-(bw>8?1:0), y0+m_Font->m_CharHeight/2, IsMin?m_ColValTextRO:m_ColTitleText);
                        // [+]
                        Gr->DrawLine(m_PosX+m_VarX2-2*bw+3, y0+m_Font->m_CharHeight/2, m_PosX+m_VarX2-bw-2, y0+m_Font->m_CharHeight/2, IsMax?m_ColValTextRO:m_ColTitleText);
                        Gr->DrawLine(m_PosX+m_VarX2-bw-bw/2, y0+m_Font->m_CharHeight/2-bw/2+2, m_PosX+m_VarX2-bw-bw/2, y0+m_Font->m_CharHeight/2+bw/2-1, IsMax?m_ColValTextRO:m_ColTitleText);
                    }
                    else if( m_DrawListBtn )
                    {
                        // [v]
                        int eps = 1;
                        int dx = -1;
                        Gr->DrawRect(m_PosX+m_VarX2-bw+1, y0+1, m_PosX+m_VarX2-1, y0+m_Font->m_CharHeight-2, m_HighlightListBtn?m_ColHighBtn:m_ColBtn);
                        Gr->DrawLine(m_PosX+m_VarX2-bw+4+dx, y0+m_Font->m_CharHeight/2-eps, m_PosX+m_VarX2-bw/2+1+dx, y0+m_Font->m_CharHeight-4, m_ColTitleText, true);
                        Gr->DrawLine(m_PosX+m_VarX2-bw/2+1+dx, y0+m_Font->m_CharHeight-4, m_PosX+m_VarX2-2+dx, y0+m_Font->m_CharHeight/2-1, m_ColTitleText, true);
                    }
                    else if( m_DrawBoolBtn )
                    {
                        Gr->DrawRect(m_PosX+m_VarX2-bw+1, y0+1, m_PosX+m_VarX2-1, y0+m_Font->m_CharHeight-2, m_HighlightBoolBtn?m_ColHighBtn:m_ColBtn);
                        // [x]
                        //Gr->DrawLine(m_PosX+m_VarX2-bw/2-bw/6, y0+m_Font->m_CharHeight/2-bw/6, m_PosX+m_VarX2-bw/2+bw/6, y0+m_Font->m_CharHeight/2+bw/6, m_ColTitleText, true);
                        //Gr->DrawLine(m_PosX+m_VarX2-bw/2-bw/6, y0+m_Font->m_CharHeight/2+bw/6, m_PosX+m_VarX2-bw/2+bw/6, y0+m_Font->m_CharHeight/2-bw/6, m_ColTitleText, true);
                        // [<>]
                        int s = bw/4;
                        int eps = 1;
                        Gr->DrawLine(m_PosX+m_VarX2-bw/2-1, y0+m_Font->m_CharHeight/2-s, m_PosX+m_VarX2-bw/2-s-1, y0+m_Font->m_CharHeight/2, m_ColTitleText, true);
                        Gr->DrawLine(m_PosX+m_VarX2-bw/2-s-1, y0+m_Font->m_CharHeight/2, m_PosX+m_VarX2-bw/2-eps, y0+m_Font->m_CharHeight/2+s+1-eps, m_ColTitleText, true);
                        //Gr->DrawLine(m_PosX+m_VarX2-bw/2+1, y0+m_Font->m_CharHeight/2+s, m_PosX+m_VarX2-bw/2+s+1, y0+m_Font->m_CharHeight/2, m_ColTitleText, true);
                        //Gr->DrawLine(m_PosX+m_VarX2-bw/2+s+1, y0+m_Font->m_CharHeight/2, m_PosX+m_VarX2-bw/2+1, y0+m_Font->m_CharHeight/2-s, m_ColTitleText, true);
                        Gr->DrawLine(m_PosX+m_VarX2-bw/2+2, y0+m_Font->m_CharHeight/2-s, m_PosX+m_VarX2-bw/2+s+2, y0+m_Font->m_CharHeight/2, m_ColTitleText, true);
                        Gr->DrawLine(m_PosX+m_VarX2-bw/2+s+2, y0+m_Font->m_CharHeight/2, m_PosX+m_VarX2-bw/2+1+eps, y0+m_Font->m_CharHeight/2+s+1-eps, m_ColTitleText, true);
                    }

                    if( m_DrawRotoBtn )
                    {
                        // [o] rotoslider button
                        /*
                        Gr->DrawRect(m_PosX+m_VarX1-bw-1, y0+1, m_PosX+m_VarX1-3, y0+m_Font->m_CharHeight-2, m_HighlightRotoBtn?m_ColHighBtn:m_ColBtn);
                        Gr->DrawLine(m_PosX+m_VarX1-bw+bw/2-2, y0+m_Font->m_CharHeight/2-1, m_PosX+m_VarX1-bw+bw/2-1, y0+m_Font->m_CharHeight/2-1, m_ColTitleText);
                        Gr->DrawLine(m_PosX+m_VarX1-bw+bw/2-3, y0+m_Font->m_CharHeight/2+0, m_PosX+m_VarX1-bw+bw/2+0, y0+m_Font->m_CharHeight/2+0, m_ColTitleText);
                        Gr->DrawLine(m_PosX+m_VarX1-bw+bw/2-3, y0+m_Font->m_CharHeight/2+1, m_PosX+m_VarX1-bw+bw/2+0, y0+m_Font->m_CharHeight/2+1, m_ColTitleText);
                        Gr->DrawLine(m_PosX+m_VarX1-bw+bw/2-2, y0+m_Font->m_CharHeight/2+2, m_PosX+m_VarX1-bw+bw/2-1, y0+m_Font->m_CharHeight/2+2, m_ColTitleText);
                        */
                        /*
                        Gr->DrawRect(m_PosX+m_VarX2-3*bw+1, y0+1, m_PosX+m_VarX2-2*bw-1, y0+m_Font->m_CharHeight-2, m_HighlightRotoBtn?m_ColHighBtn:m_ColBtn);
                        Gr->DrawLine(m_PosX+m_VarX2-3*bw+bw/2+0, y0+m_Font->m_CharHeight/2-1, m_PosX+m_VarX2-3*bw+bw/2+1, y0+m_Font->m_CharHeight/2-1, m_ColTitleText);
                        Gr->DrawLine(m_PosX+m_VarX2-3*bw+bw/2-1, y0+m_Font->m_CharHeight/2+0, m_PosX+m_VarX2-3*bw+bw/2+2, y0+m_Font->m_CharHeight/2+0, m_ColTitleText);
                        Gr->DrawLine(m_PosX+m_VarX2-3*bw+bw/2-1, y0+m_Font->m_CharHeight/2+1, m_PosX+m_VarX2-3*bw+bw/2+2, y0+m_Font->m_CharHeight/2+1, m_ColTitleText);
                        Gr->DrawLine(m_PosX+m_VarX2-3*bw+bw/2+0, y0+m_Font->m_CharHeight/2+2, m_PosX+m_VarX2-3*bw+bw/2+1, y0+m_Font->m_CharHeight/2+2, m_ColTitleText);
                        */
                        int dy = 0;
                        Gr->DrawRect(m_PosX+m_VarX2-bw+1, y0+1, m_PosX+m_VarX2-1, y0+m_Font->m_CharHeight-2, m_HighlightRotoBtn?m_ColHighBtn:m_ColBtn);
                        Gr->DrawLine(m_PosX+m_VarX2-bw+bw/2+0, y0+m_Font->m_CharHeight/2-1+dy, m_PosX+m_VarX2-bw+bw/2+1, y0+m_Font->m_CharHeight/2-1+dy, m_ColTitleText, true);
                        Gr->DrawLine(m_PosX+m_VarX2-bw+bw/2-1, y0+m_Font->m_CharHeight/2+0+dy, m_PosX+m_VarX2-bw+bw/2+2, y0+m_Font->m_CharHeight/2+0+dy, m_ColTitleText, true);
                        Gr->DrawLine(m_PosX+m_VarX2-bw+bw/2-1, y0+m_Font->m_CharHeight/2+1+dy, m_PosX+m_VarX2-bw+bw/2+2, y0+m_Font->m_CharHeight/2+1+dy, m_ColTitleText, true);
                        Gr->DrawLine(m_PosX+m_VarX2-bw+bw/2+0, y0+m_Font->m_CharHeight/2+2+dy, m_PosX+m_VarX2-bw+bw/2+1, y0+m_Font->m_CharHeight/2+2+dy, m_ColTitleText, true);
                    }
                }
                

                // Draw value width slider
                if( !m_HighlightValWidth )
                {
                    color32 col = m_DarkText ? COLOR32_WHITE : m_ColTitleText;
                    Gr->DrawRect(m_PosX+m_VarX1-2, m_PosY+m_VarY0-8, m_PosX+m_VarX1-1, m_PosY+m_VarY0-4, col);
                    Gr->DrawLine(m_PosX+m_VarX1-1, m_PosY+m_VarY0-3, m_PosX+m_VarX1, m_PosY+m_VarY0-3, m_ColLineShadow);
                    Gr->DrawLine(m_PosX+m_VarX1, m_PosY+m_VarY0-3, m_PosX+m_VarX1, m_PosY+m_VarY0-8, m_ColLineShadow);
                }
                else
                {
                    color32 col = m_DarkText ? COLOR32_WHITE : m_ColTitleText;
                    Gr->DrawRect(m_PosX+m_VarX1-2, m_PosY+m_VarY0-8, m_PosX+m_VarX1-1, m_PosY+m_VarY1, col);
                    Gr->DrawLine(m_PosX+m_VarX1-1, m_PosY+m_VarY1+1, m_PosX+m_VarX1, m_PosY+m_VarY1+1, m_ColLineShadow);
                    Gr->DrawLine(m_PosX+m_VarX1, m_PosY+m_VarY1+1, m_PosX+m_VarX1, m_PosY+m_VarY0-8, m_ColLineShadow);
                }

                // Draw labels & values headers
                if (m_HighlightLabelsHeader) 
                {
                    Gr->DrawRect(m_PosX+m_VarX0, m_PosY+m_Font->m_CharHeight+2, m_PosX+m_VarX1-4, m_PosY+m_VarY0-1, m_ColHighBg0, m_ColHighBg0, m_ColHighBg1, m_ColHighBg1);
                }
                if (m_HighlightValuesHeader) 
                {
                    Gr->DrawRect(m_PosX+m_VarX1+2, m_PosY+m_Font->m_CharHeight+2, m_PosX+m_VarX2, m_PosY+m_VarY0-1, m_ColHighBg0, m_ColHighBg0, m_ColHighBg1, m_ColHighBg1);
                }
            }

            // Draw key shortcut text
            if( m_HighlightedLine>=0 && m_HighlightedLine==m_ShortcutLine && !m_IsPopupList && !m_EditInPlace.m_Active )
            {
                PERF( Timer.Reset(); )  
                Gr->DrawRect(m_PosX+m_Font->m_CharHeight-2, m_PosY+m_VarY1+1, m_PosX+m_Width-m_Font->m_CharHeight-2, m_PosY+m_VarY1+1+m_Font->m_CharHeight, m_ColShortcutBg);
                Gr->DrawText(m_ShortcutTextObj, m_PosX+m_Font->m_CharHeight, m_PosY+m_VarY1+1, m_ColShortcutText, 0);
                PERF( DT = Timer.GetTime(); printf("Shortcut=%.4fms ", 1000.0*DT); )
            }
            else if( (m_HighlightLabelsHeader || m_HighlightValuesHeader) && !m_IsPopupList && !m_EditInPlace.m_Active )
            {
                Gr->DrawRect(m_PosX+m_Font->m_CharHeight-2, m_PosY+m_VarY1+1, m_PosX+m_Width-m_Font->m_CharHeight-2, m_PosY+m_VarY1+1+m_Font->m_CharHeight, m_ColShortcutBg);
                Gr->DrawText(m_HeadersTextObj, m_PosX+m_Font->m_CharHeight, m_PosY+m_VarY1+1, m_ColShortcutText, 0);
            }
            else if( m_IsHelpBar )
            {
                if( g_TwMgr->m_KeyPressedTextObj && g_TwMgr->m_KeyPressedStr.size()>0 ) // Draw key pressed
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

                // Draw EditInPlace
                EditInPlaceDraw();
            }

            if( g_TwMgr->m_PopupBar!=NULL && this!=g_TwMgr->m_PopupBar )
            {
                // darken bar if a popup bar is displayed
                Gr->DrawRect(m_PosX, m_PosY, m_PosX+m_Width-1, m_PosY+m_Height-1, 0x1F000000);
            }
        }
    }
    else // minimized
    {
        int vpx, vpy, vpw, vph;
        vpx = 0;
        vpy = 0;
        vpw = g_TwMgr->m_WndWidth;
        vph = g_TwMgr->m_WndHeight;
        if( g_TwMgr->m_IconMarginX>0 )
        {
            vpx = min(g_TwMgr->m_IconMarginX, vpw/3);
            vpw -= 2 * vpx;
        }
        if( g_TwMgr->m_IconMarginY>0 )
        {
            vpy = min(g_TwMgr->m_IconMarginY, vph/3);
            vph -= 2 * vpy;
        }

        int MinXOffset = 0, MinYOffset = 0;
        if( g_TwMgr->m_IconPos==3 )         // top-right
        {
            if( g_TwMgr->m_IconAlign==1 )   // horizontal
            {
                int n = max(1, vpw/m_Font->m_CharHeight-1);
                m_MinPosX = vpx + vpw-((m_MinNumber%n)+1)*m_Font->m_CharHeight;
                m_MinPosY = vpy + (m_MinNumber/n)*m_Font->m_CharHeight;
                MinYOffset = m_Font->m_CharHeight;
                MinXOffset = -m_TitleWidth;
            }
            else // vertical
            {
                int n = max(1, vph/m_Font->m_CharHeight-1);
                m_MinPosY = vpy + (m_MinNumber%n)*m_Font->m_CharHeight;
                m_MinPosX = vpx + vpw-((m_MinNumber/n)+1)*m_Font->m_CharHeight;
                MinXOffset = -m_TitleWidth-m_Font->m_CharHeight;
            }
        }
        else if( g_TwMgr->m_IconPos==2 )    // top-left
        {
            if( g_TwMgr->m_IconAlign==1 )   // horizontal
            {
                int n = max(1, vpw/m_Font->m_CharHeight-1);
                m_MinPosX = vpx + (m_MinNumber%n)*m_Font->m_CharHeight;
                m_MinPosY = vpy + (m_MinNumber/n)*m_Font->m_CharHeight;
                MinYOffset = m_Font->m_CharHeight;
            }
            else // vertical
            {
                int n = max(1, vph/m_Font->m_CharHeight-1);
                m_MinPosY = vpy + (m_MinNumber%n)*m_Font->m_CharHeight;
                m_MinPosX = vpx + (m_MinNumber/n)*m_Font->m_CharHeight;
                MinXOffset = m_Font->m_CharHeight;
            }
        }
        else if( g_TwMgr->m_IconPos==1 )    // bottom-right
        {
            if( g_TwMgr->m_IconAlign==1 )   // horizontal
            {
                int n = max(1, vpw/m_Font->m_CharHeight-1);
                m_MinPosX = vpx + vpw-((m_MinNumber%n)+1)*m_Font->m_CharHeight;
                m_MinPosY = vpy + vph-((m_MinNumber/n)+1)*m_Font->m_CharHeight;
                MinYOffset = -m_Font->m_CharHeight;
                MinXOffset = -m_TitleWidth;
            }
            else // vertical
            {
                int n = max(1, vph/m_Font->m_CharHeight-1);
                m_MinPosY = vpy + vph-((m_MinNumber%n)+1)*m_Font->m_CharHeight;
                m_MinPosX = vpx + vpw-((m_MinNumber/n)+1)*m_Font->m_CharHeight;
                MinXOffset = -m_TitleWidth-m_Font->m_CharHeight;
            }
        }
        else // bottom-left
        {
            if( g_TwMgr->m_IconAlign==1 )   // horizontal
            {
                int n = max(1, vpw/m_Font->m_CharHeight-1);
                m_MinPosX = vpx + (m_MinNumber%n)*m_Font->m_CharHeight;
                m_MinPosY = vpy + vph-((m_MinNumber/n)+1)*m_Font->m_CharHeight;
                MinYOffset = -m_Font->m_CharHeight;
            }
            else // vertical
            {
                int n = max(1, vph/m_Font->m_CharHeight-1);
                m_MinPosY = vpy + vph-((m_MinNumber%n)+1)*m_Font->m_CharHeight;
                m_MinPosX = vpx + (m_MinNumber/n)*m_Font->m_CharHeight;
                MinXOffset = m_Font->m_CharHeight;
            }
        }

        if( m_HighlightMaximize )
        {
            // Draw title
            if( _DrawPart&DRAW_BG )
            {
                Gr->DrawRect(m_MinPosX, m_MinPosY, m_MinPosX+m_Font->m_CharHeight, m_MinPosY+m_Font->m_CharHeight, m_ColTitleUnactiveBg);
                Gr->DrawRect(m_MinPosX+MinXOffset, m_MinPosY+MinYOffset, m_MinPosX+MinXOffset+m_TitleWidth+m_Font->m_CharHeight, m_MinPosY+MinYOffset+m_Font->m_CharHeight, m_ColTitleUnactiveBg);
            }
            if( _DrawPart&DRAW_CONTENT )
            {
                if( m_ColTitleShadow!=0 )
                    Gr->DrawText(m_TitleTextObj, m_MinPosX+MinXOffset+m_Font->m_CharHeight/2, m_MinPosY+1+MinYOffset, m_ColTitleShadow, 0);
                Gr->DrawText(m_TitleTextObj, m_MinPosX+MinXOffset+m_Font->m_CharHeight/2, m_MinPosY+MinYOffset, m_ColTitleText, 0);
            }
        }

        if( !m_IsHelpBar )
        {
            // Draw maximize button
            int xm = m_MinPosX+2, wm=m_Font->m_CharHeight-6;
            wm = (wm<6) ? 6 : wm;
            if( _DrawPart&DRAW_BG )
                Gr->DrawRect(xm+1, m_MinPosY+4, xm+wm-1, m_MinPosY+3+wm, m_HighlightMaximize?m_ColHighBtn:m_ColBtn);
            if( _DrawPart&DRAW_CONTENT )
            {
                Gr->DrawLine(xm, m_MinPosY+3, xm+wm, m_MinPosY+3, m_ColLine);
                Gr->DrawLine(xm+wm, m_MinPosY+3, xm+wm, m_MinPosY+3+wm, m_ColLine);
                Gr->DrawLine(xm+wm, m_MinPosY+3+wm, xm, m_MinPosY+3+wm, m_ColLine);
                Gr->DrawLine(xm, m_MinPosY+3+wm, xm, m_MinPosY+3, m_ColLine);
                Gr->DrawLine(xm+wm+1, m_MinPosY+4, xm+wm+1, m_MinPosY+4+wm, m_ColLineShadow);
                Gr->DrawLine(xm+wm+1, m_MinPosY+4+wm, xm, m_MinPosY+4+wm, m_ColLineShadow);
                Gr->DrawLine(xm+wm/3-1, m_MinPosY+3+wm-wm/3, xm+wm/2, m_MinPosY+6, m_ColTitleText, true);
                Gr->DrawLine(xm+wm-wm/3+1, m_MinPosY+3+wm-wm/3, xm+wm/2, m_MinPosY+6, m_ColTitleText, true);
            }
        }
        else
        {
            // Draw help button
            int xm = m_MinPosX+2, wm=m_Font->m_CharHeight-6;
            wm = (wm<6) ? 6 : wm;
            if( _DrawPart&DRAW_BG )
                Gr->DrawRect(xm+1, m_MinPosY+4, xm+wm-1, m_MinPosY+3+wm, m_HighlightMaximize?m_ColHighBtn:m_ColBtn);
            if( _DrawPart&DRAW_CONTENT )
            {
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
}

//  ---------------------------------------------------------------------------

bool CTwBar::MouseMotion(int _X, int _Y)
{
    assert(g_TwMgr->m_Graph && g_TwMgr->m_WndHeight>0 && g_TwMgr->m_WndWidth>0);
    if( !m_UpToDate )
        Update();
    
    bool Handled = false;
    bool CustomArea = false;
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
        const int ContainedMargin = 32;

        if( !m_MouseDrag )
        {   
            Handled = InBar;
            m_HighlightedLine = -1;
            m_HighlightIncrBtn = false;
            m_HighlightDecrBtn = false;
            m_HighlightRotoBtn = false;
            if( abs(m_MouseOriginX-_X)>6 || abs(m_MouseOriginY-_Y)>6 )
                m_HighlightClickBtn = false;
            m_HighlightListBtn = false;
            m_HighlightTitle = false;
            m_HighlightScroll = false;
            m_HighlightUpScroll = false;
            m_HighlightDnScroll = false;
            m_HighlightMinimize = false;
            m_HighlightFont = false;
            m_HighlightValWidth = false;
            m_HighlightLabelsHeader = false;
            m_HighlightValuesHeader = false;
            //if( InBar && _X>m_PosX+m_Font->m_CharHeight+1 && _X<m_PosX+m_VarX2 && _Y>=m_PosY+m_VarY0 && _Y<m_PosY+m_VarY1 )
            if( InBar && _X>m_PosX+2 && _X<m_PosX+m_VarX2 && _Y>=m_PosY+m_VarY0 && _Y<m_PosY+m_VarY1 )
            {   // mouse over var line
                m_HighlightedLine = (_Y-m_PosY-m_VarY0)/(m_Font->m_CharHeight+m_LineSep);
                if( m_HighlightedLine>=(int)m_HierTags.size() )
                    m_HighlightedLine = -1;
                else if(m_HighlightedLine>=0)
                    m_HighlightedLineLastValid = m_HighlightedLine;
                if( m_HighlightedLine<0 || m_HierTags[m_HighlightedLine].m_Var==NULL || m_HierTags[m_HighlightedLine].m_Var->IsGroup() )
                    ANT_SET_CURSOR(Arrow);
                else
                {
                    if( !m_HierTags[m_HighlightedLine].m_Var->IsGroup() && static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_NoSlider )
                    {
                        if( static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_ReadOnly && !m_IsHelpBar 
                            && !(static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Type==TW_TYPE_BUTTON && static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->m_Val.m_Button.m_Callback==NULL) )
                            ANT_SET_CURSOR(No); //(Arrow);
                        else
                        {
                            ANT_SET_CURSOR(Arrow);
                            CustomArea = true;
                        }

                        if( m_DrawListBtn )
                        {
                            m_HighlightListBtn = true;
                            CustomArea = false;
                        }
                        if( m_DrawBoolBtn )
                        {
                            m_HighlightBoolBtn = true;
                            CustomArea = false;
                        }
                    }
                    else if( m_DrawRotoBtn && ( _X>=m_PosX+m_VarX2-IncrBtnWidth(m_Font->m_CharHeight) || _X<m_PosX+m_VarX1 ) )  // [o] button
                    //else if( m_DrawRotoBtn && _X<m_PosX+m_VarX1 ) // [o] button
                    {
                        m_HighlightRotoBtn = true;
                        ANT_SET_CURSOR(Point);
                    }
                    else if( m_DrawIncrDecrBtn && _X>=m_PosX+m_VarX2-2*IncrBtnWidth(m_Font->m_CharHeight) ) // [+] button
                    {
                        m_HighlightIncrBtn = true;
                        ANT_SET_CURSOR(Arrow);
                    }
                    else if( m_DrawIncrDecrBtn && _X>=m_PosX+m_VarX2-3*IncrBtnWidth(m_Font->m_CharHeight) ) // [-] button
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
                        //ANT_SET_CURSOR(Point);
                        ANT_SET_CURSOR(IBeam);
                }
            }
            else if( InBar && m_Movable && !m_IsPopupList && _X>=m_PosX+2*m_Font->m_CharHeight && _X<m_PosX+m_Width-2*m_Font->m_CharHeight && _Y<m_PosY+m_Font->m_CharHeight )
            {   // mouse over title
                m_HighlightTitle = true;
                ANT_SET_CURSOR(Move);
            }
            else if ( InBar && !m_IsPopupList && _X>=m_PosX+m_VarX1-5 && _X<m_PosX+m_VarX1+5 && _Y>m_PosY+m_Font->m_CharHeight && _Y<m_PosY+m_VarY0 )
            {   // mouse over ValuesWidth handle
                m_HighlightValWidth = true;
                ANT_SET_CURSOR(WE);
            }
            else if ( InBar && !m_IsPopupList && !m_IsHelpBar && _X>=m_PosX+m_VarX0 && _X<m_PosX+m_VarX1-5 && _Y>m_PosY+m_Font->m_CharHeight && _Y<m_PosY+m_VarY0 )
            {   // mouse over left column header
                m_HighlightLabelsHeader = true;
                ANT_SET_CURSOR(Arrow);
            }
            else if ( InBar && !m_IsPopupList && _X>=m_PosX+m_VarX1+5 && _X<m_PosX+m_VarX2 && _Y>m_PosY+m_Font->m_CharHeight && _Y<m_PosY+m_VarY0 )
            {   // mouse over right column header
                m_HighlightValuesHeader = true;
                ANT_SET_CURSOR(Arrow);
            }
            //else if( InBar && m_NbDisplayedLines<m_NbHierLines && _X>=m_PosX && _X<m_PosX+m_Font->m_CharHeight && _Y>=m_ScrollY0 && _Y<m_ScrollY1 )
            else if( InBar && m_NbDisplayedLines<m_NbHierLines && _X>=m_PosX+m_VarX2+2 && _X<m_PosX+m_Width-2 && _Y>=m_ScrollY0 && _Y<m_ScrollY1 )
            {
                m_HighlightScroll = true;
              #ifdef ANT_WINDOWS
                ANT_SET_CURSOR(NS);
              #else
                ANT_SET_CURSOR(Arrow);
              #endif
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
            else if( InBar && m_Resizable && !m_IsPopupList && _X>=m_PosX && _X<m_PosX+m_Font->m_CharHeight && _Y>=m_PosY && _Y<m_PosY+m_Font->m_CharHeight )
                ANT_SET_CURSOR(TopLeft);
            else if( InBar && !m_IsPopupList && _X>=m_PosX && _X<m_PosX+m_Font->m_CharHeight && _Y>=m_PosY+m_Height-m_Font->m_CharHeight && _Y<m_PosY+m_Height )
                ANT_SET_CURSOR(BottomLeft);
            else if( InBar && m_Resizable && !m_IsPopupList && _X>=m_PosX+m_Width-m_Font->m_CharHeight && _X<m_PosX+m_Width && _Y>=m_PosY && _Y<m_PosY+m_Font->m_CharHeight )
                ANT_SET_CURSOR(TopRight);
            else if( InBar && m_Resizable && !m_IsPopupList && _X>=m_PosX+m_Width-m_Font->m_CharHeight && _X<m_PosX+m_Width && _Y>=m_PosY+m_Height-m_Font->m_CharHeight && _Y<m_PosY+m_Height )
                ANT_SET_CURSOR(BottomRight);
            else if( InBar && g_TwMgr->m_FontResizable && !m_IsPopupList && _X>=m_PosX+m_Font->m_CharHeight && _X<m_PosX+2*m_Font->m_CharHeight && _Y<m_PosY+m_Font->m_CharHeight )
            {
                m_HighlightFont = true;
                ANT_SET_CURSOR(Arrow);
            }
            else if( InBar && m_Iconifiable && !m_IsPopupList && _X>=m_PosX+m_Width-2*m_Font->m_CharHeight && _X<m_PosX+m_Width-m_Font->m_CharHeight && _Y<m_PosY+m_Font->m_CharHeight )
            {
                m_HighlightMinimize = true;
                ANT_SET_CURSOR(Arrow);
            }
            else if( m_IsHelpBar && InBar && _X>=m_PosX+m_VarX0 && _X<m_PosX+m_Width-m_Font->m_CharHeight && _Y>m_PosY+m_Height-m_Font->m_CharHeight && _Y<m_PosY+m_Height )
                ANT_SET_CURSOR(Arrow); //(Hand);   // web link
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
                {
                    ANT_SET_CURSOR(Arrow);
                    CustomArea = true;
                }
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
                if( m_Contained )
                {
                    if( m_PosX+m_Width>vpx+vpw )
                        m_PosX = vpx+vpw-m_Width;
                    if( m_PosX<vpx )
                        m_PosX = vpx;
                    if( m_PosY+m_Height>vpy+vph )
                        m_PosY = vpy+vph-m_Height;
                    if( m_PosY<vpy )
                        m_PosY = vpy;
                } 
                else 
                {
                    if( m_PosX+ContainedMargin>vpx+vpw )
                        m_PosX = vpx+vpw-ContainedMargin;
                    if( m_PosX+m_Width<vpx+ContainedMargin )
                        m_PosX = vpx+ContainedMargin-m_Width;
                    if( m_PosY+ContainedMargin>vpy+vph )
                        m_PosY = vpy+vph-ContainedMargin;
                    if( m_PosY+m_Height<vpy+ContainedMargin )
                        m_PosY = vpy+ContainedMargin-m_Height;
                }
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
              #ifdef ANT_WINDOWS
                ANT_SET_CURSOR(NS);
              #else
                ANT_SET_CURSOR(Arrow);
              #endif
                Handled = true;
                m_DrawHandles = true;
            }
            else if( m_MouseDragResizeUL )
            {
                int w = m_Width;
                int h = m_Height;
                m_PosX += _X-m_MouseOriginX;
                m_PosY += _Y-m_MouseOriginY;
                m_Width -= _X-m_MouseOriginX;
                m_Height -= _Y-m_MouseOriginY;
                m_MouseOriginX = _X;
                m_MouseOriginY = _Y;
                int vpx = 0, vpy = 0, vpw = g_TwMgr->m_WndWidth, vph = g_TwMgr->m_WndHeight;
                if( !m_Contained )
                {
                    if( m_PosX+ContainedMargin>vpx+vpw )
                        m_PosX = vpx+vpw-ContainedMargin;
                    if( m_PosX+m_Width<vpx+ContainedMargin )
                        m_PosX = vpx+ContainedMargin-m_Width;
                    if( m_PosY+ContainedMargin>vpy+vph )
                        m_PosY = vpy+vph-ContainedMargin;
                    if( m_PosY+m_Height<vpy+ContainedMargin )
                        m_PosY = vpy+ContainedMargin-m_Height;
                }
                else 
                {
                    if( m_PosX<vpx ) 
                    {
                        m_PosX = vpx;
                        m_Width = w;
                    }
                    if( m_PosY<vpy ) 
                    {
                        m_PosY = vpy;
                        m_Height = h;
                    }
                }
                if (m_ValuesWidthRatio > 0) 
                    m_ValuesWidth = int(m_ValuesWidthRatio * m_Width + 0.5);
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
                int h = m_Height;
                m_PosY += _Y-m_MouseOriginY;
                m_Width += _X-m_MouseOriginX;
                m_Height -= _Y-m_MouseOriginY;
                m_MouseOriginX = _X;
                m_MouseOriginY = _Y;
                int vpx = 0, vpy = 0, vpw = g_TwMgr->m_WndWidth, vph = g_TwMgr->m_WndHeight;
                if( !m_Contained )
                {
                    if( m_PosX+ContainedMargin>vpx+vpw )
                        m_PosX = vpx+vpw-ContainedMargin;
                    if( m_PosX+m_Width<vpx+ContainedMargin )
                        m_PosX = vpx+ContainedMargin-m_Width;
                    if( m_PosY+ContainedMargin>vpy+vph )
                        m_PosY = vpy+vph-ContainedMargin;
                    if( m_PosY+m_Height<vpy+ContainedMargin )
                        m_PosY = vpy+ContainedMargin-m_Height;
                }
                else
                {
                    if( m_PosX+m_Width>vpx+vpw )
                        m_Width = vpx+vpw-m_PosX;
                    if( m_PosY<vpy ) 
                    {
                        m_PosY = vpy;
                        m_Height = h;
                    }
                }
                if (m_ValuesWidthRatio > 0) 
                    m_ValuesWidth = int(m_ValuesWidthRatio * m_Width + 0.5);
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
                int w = m_Width;
                m_PosX += _X-m_MouseOriginX;
                m_Width -= _X-m_MouseOriginX;
                m_Height += _Y-m_MouseOriginY;
                m_MouseOriginX = _X;
                m_MouseOriginY = _Y;
                int vpx = 0, vpy = 0, vpw = g_TwMgr->m_WndWidth, vph = g_TwMgr->m_WndHeight;
                if( !m_Contained )
                {
                    if( m_PosX+ContainedMargin>vpx+vpw )
                        m_PosX = vpx+vpw-ContainedMargin;
                    if( m_PosX+m_Width<vpx+ContainedMargin )
                        m_PosX = vpx+ContainedMargin-m_Width;
                    if( m_PosY+ContainedMargin>vpy+vph )
                        m_PosY = vpy+vph-ContainedMargin;
                    if( m_PosY+m_Height<vpy+ContainedMargin )
                        m_PosY = vpy+ContainedMargin-m_Height;
                }
                else
                {
                    if( m_PosY+m_Height>vpy+vph )
                        m_Height = vpy+vph-m_PosY;
                    if( m_PosX<vpx ) 
                    {
                        m_PosX = vpx;
                        m_Width = w;
                    }
                }
                if (m_ValuesWidthRatio > 0) 
                    m_ValuesWidth = int(m_ValuesWidthRatio * m_Width + 0.5);
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
                int vpx = 0, vpy = 0, vpw = g_TwMgr->m_WndWidth, vph = g_TwMgr->m_WndHeight;
                if( !m_Contained )
                {
                    if( m_PosX+ContainedMargin>vpx+vpw )
                        m_PosX = vpx+vpw-ContainedMargin;
                    if( m_PosX+m_Width<vpx+ContainedMargin )
                        m_PosX = vpx+ContainedMargin-m_Width;
                    if( m_PosY+ContainedMargin>vpy+vph )
                        m_PosY = vpy+vph-ContainedMargin;
                    if( m_PosY+m_Height<vpy+ContainedMargin )
                        m_PosY = vpy+ContainedMargin-m_Height;
                } 
                else
                {
                    if( m_PosX+m_Width>vpx+vpw )
                        m_Width = vpx+vpw-m_PosX;
                    if( m_PosY+m_Height>vpy+vph )
                        m_Height = vpy+vph-m_PosY;
                }
                if (m_ValuesWidthRatio > 0) 
                    m_ValuesWidth = int(m_ValuesWidthRatio * m_Width + 0.5);
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
            else if( m_EditInPlace.m_Active )
            {
                EditInPlaceMouseMove(_X, _Y, true);
                ANT_SET_CURSOR(IBeam);
                Handled = true;
            }
            //else if( InBar )
            //  ANT_SET_CURSOR(Arrow);
        }
    }
    else // minimized
    {
        if( m_Iconifiable && _X>=m_MinPosX+2 && _X<m_MinPosX+m_Font->m_CharHeight && _Y>m_MinPosY && _Y<m_MinPosY+m_Font->m_CharHeight-2 )
        {
            m_HighlightMaximize = true;
            if( !m_IsHelpBar )
                ANT_SET_CURSOR(Arrow);
            else
              #ifdef ANT_WINDOWS
                ANT_SET_CURSOR(Help);
              #else
                ANT_SET_CURSOR(Arrow);
              #endif
            Handled = true;
        }
        else
            m_HighlightMaximize = false;
    }

    // Handled by a custom widget?
    CTwMgr::CStructProxy *currentCustomActiveStructProxy = NULL;
    if( g_TwMgr!=NULL && (!Handled || CustomArea) && !m_IsMinimized && m_CustomRecords.size()>0 )
    {
        bool CustomHandled = false;
        for( int s=0; s<2; ++s )    // 2 iterations: first for custom widget having focus, second for others if no focused widget.
            for( CustomMap::iterator it=m_CustomRecords.begin(); it!=m_CustomRecords.end(); ++it )
            {
                CTwMgr::CStructProxy *sProxy = it->first;
                const CCustomRecord& r = it->second;
                if( (s==1 || sProxy->m_CustomCaptureFocus) && !CustomHandled && sProxy!=NULL && sProxy->m_CustomMouseMotionCallback!=NULL && r.m_XMin<r.m_XMax && r.m_Y0<r.m_Y1 && r.m_YMin<=r.m_YMax && r.m_YMin>=r.m_Y0 && r.m_YMax<=r.m_Y1 )
                {
                    if( sProxy->m_CustomCaptureFocus || (_X>=r.m_XMin && _X<r.m_XMax && _Y>=r.m_YMin && _Y<r.m_YMax) )
                    {
                        CustomHandled = sProxy->m_CustomMouseMotionCallback(_X-r.m_XMin, _Y-r.m_Y0, r.m_XMax-r.m_XMin, r.m_Y1-r.m_Y0, sProxy->m_StructExtData, sProxy->m_StructClientData, this, r.m_Var);
                        currentCustomActiveStructProxy = sProxy;
                        s = 2; // force s-loop exit
                    }
                }
                else if( sProxy!=NULL )
                {
                    sProxy->m_CustomCaptureFocus = false;   // force free focus, just in case.
                    ANT_SET_CURSOR(Arrow);
                }
            }
        if( CustomHandled )
            Handled = true;
    }
    // If needed, send a 'MouseLeave' message to previously active custom struct
    if( g_TwMgr!=NULL && m_CustomActiveStructProxy!=NULL && m_CustomActiveStructProxy!=currentCustomActiveStructProxy )
    {
        bool found = false;
        for( list<CTwMgr::CStructProxy>::iterator it=g_TwMgr->m_StructProxies.begin(); it!=g_TwMgr->m_StructProxies.end() && !found; ++it )
            found = (&(*it)==m_CustomActiveStructProxy);
        if( found && m_CustomActiveStructProxy->m_CustomMouseLeaveCallback!=NULL )
            m_CustomActiveStructProxy->m_CustomMouseLeaveCallback(m_CustomActiveStructProxy->m_StructExtData, m_CustomActiveStructProxy->m_StructClientData, this);
    }
    m_CustomActiveStructProxy = currentCustomActiveStructProxy;

    return Handled;
}

//  ---------------------------------------------------------------------------

#ifdef ANT_WINDOWS
#   pragma optimize("", off)
//  disable optimizations because the conversion of Enum from unsigned int to double is not always exact if optimized and GraphAPI=DirectX !
#endif
static void ANT_CALL PopupCallback(void *_ClientData)
{
    CTwFPU fpu; // force fpu precision

    if( g_TwMgr!=NULL && g_TwMgr->m_PopupBar!=NULL )
    {
        unsigned int Enum = *(unsigned int *)&_ClientData;
        CTwVarAtom *Var = g_TwMgr->m_PopupBar->m_VarEnumLinkedToPopupList;
        CTwBar *Bar = g_TwMgr->m_PopupBar->m_BarLinkedToPopupList;
        if( Bar!=NULL && Var!=NULL && !Var->m_ReadOnly && IsEnumType(Var->m_Type) )
        {
            Var->ValueFromDouble(Enum);
            //Bar->UnHighlightLine();
            Bar->HaveFocus(true);
            Bar->NotUpToDate();
        }
        TwDeleteBar(g_TwMgr->m_PopupBar);
        g_TwMgr->m_PopupBar = NULL;
    }
}
#ifdef ANT_WINDOWS
#   pragma optimize("", on)
#endif

//  ---------------------------------------------------------------------------

bool CTwBar::MouseButton(ETwMouseButtonID _Button, bool _Pressed, int _X, int _Y)
{
    assert(g_TwMgr->m_Graph && g_TwMgr->m_WndHeight>0 && g_TwMgr->m_WndWidth>0);
    bool Handled = false;
    if( !m_UpToDate )
        Update();
    bool EditInPlaceActive = false;
    bool CustomArea = false;

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
                if( g_TwMgr==NULL ) // Mgr might have been destroyed by the client inside a callback call
                    return 1;
                NotUpToDate();
            }
            else if( _Pressed && m_HighlightDecrBtn )
            {
                static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var)->Increment(-1);
                if( g_TwMgr==NULL ) // Mgr might have been destroyed by the client inside a callback call
                    return 1;
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
                if( !Var->m_NoSlider && !Var->m_ReadOnly && m_HighlightRotoBtn )
                {
                    // begin rotoslider
                    if( _X>m_PosX+m_VarX1 )
                        RotoOnLButtonDown(m_PosX+m_VarX2-(1*IncrBtnWidth(m_Font->m_CharHeight))/2, _Y);
                    else
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
                else if( IsEnumType(Var->m_Type) && !Var->m_ReadOnly && !g_TwMgr->m_IsRepeatingMousePressed )
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
                    g_TwMgr->m_PopupBar->m_DarkText = m_DarkText;
                    g_TwMgr->m_PopupBar->m_PosX = m_PosX + m_VarX1 - 2;
                    g_TwMgr->m_PopupBar->m_PosY = m_PosY + m_VarY0 + (m_HighlightedLine+1)*(m_Font->m_CharHeight+m_LineSep);
                    g_TwMgr->m_PopupBar->m_Width = m_Width - 2*m_Font->m_CharHeight;
                    g_TwMgr->m_PopupBar->m_LineSep = g_TwMgr->m_PopupBar->m_Sep;
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
                    unsigned int CurrentEnumValue = (unsigned int)((int)Var->ValueToDouble());
                    for( CTwMgr::CEnum::CEntries::iterator It=e.m_Entries.begin(); It!=e.m_Entries.end(); ++It )
                    {
                        char ID[64];
                        sprintf(ID, "%u", It->first);
                        //ultoa(It->first, ID, 10);
                        TwAddButton(g_TwMgr->m_PopupBar, ID, PopupCallback, *(void**)&(It->first), NULL);
                        CTwVar *Btn = g_TwMgr->m_PopupBar->Find(ID);
                        if( Btn!=NULL )
                        {
                            Btn->m_Label = It->second.c_str();
                            if( It->first==CurrentEnumValue )
                            {
                                Btn->m_ColorPtr = &m_ColValTextNE;
                                Btn->m_BgColorPtr = &m_ColGrpBg;
                            }
                        }
                    }
                    g_TwMgr->m_HelpBarNotUpToDate = false;
                }
                else if( (Var->m_ReadOnly && (Var->m_Type==TW_TYPE_CDSTRING || Var->m_Type==TW_TYPE_CDSTDSTRING || Var->m_Type==TW_TYPE_STDSTRING || IsCSStringType(Var->m_Type)) && EditInPlaceAcceptVar(Var))
                         || (!Var->m_ReadOnly && EditInPlaceAcceptVar(Var)) )
                    {
                        int dw = 0;
                        //if( m_DrawIncrDecrBtn )
                        //  dw = 2*IncrBtnWidth(m_Font->m_CharHeight);
                        if( !m_EditInPlace.m_Active || m_EditInPlace.m_Var!=Var )
                        {
                            EditInPlaceStart(Var, m_VarX1, m_VarY0+(m_HighlightedLine)*(m_Font->m_CharHeight+m_LineSep), m_VarX2-m_VarX1-dw-1);
                            if( EditInPlaceIsReadOnly() )
                                EditInPlaceMouseMove(_X, _Y, false);
                            m_MouseDrag = false;
                            m_MouseDragVar = false;
                        }
                        else
                        {
                            EditInPlaceMouseMove(_X, _Y, false);
                            m_MouseDrag = true;
                            m_MouseDragVar = false;
                        }
                        EditInPlaceActive = m_EditInPlace.m_Active;
                        if( Var->m_ReadOnly )
                            ANT_SET_CURSOR(No);
                        else
                            ANT_SET_CURSOR(IBeam);
                    }
                else if( Var->m_ReadOnly )
                    ANT_SET_CURSOR(No);
                else
                {
                    ANT_SET_CURSOR(Arrow);
                    CustomArea = true;
                }
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
                {
                    ANT_SET_CURSOR(Arrow);
                    CustomArea = true;
                }
            }
            else if( !_Pressed && m_HighlightClickBtn ) // a button variable is activated
            {
                m_HighlightClickBtn = false;
                m_MouseDragVar = false;
                m_MouseDrag = false;
                Handled = true;
                NotUpToDate();
                if( !m_HierTags[m_HighlightedLine].m_Var->IsGroup() )
                {
                    CTwVarAtom * Var = static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var);
                    if( !Var->m_ReadOnly && Var->m_Type==TW_TYPE_BUTTON && Var->m_Val.m_Button.m_Callback!=NULL )
                    {
                        Var->m_Val.m_Button.m_Callback(Var->m_ClientData);
                        if( g_TwMgr==NULL ) // Mgr might have been destroyed by the client inside a callback call
                            return 1;
                    }
                }
            }
            else if( !_Pressed )
            {
                m_MouseDragVar = false;
                m_MouseDrag = false;
                CustomArea = true;
            }
        }
        else if( _Pressed && !m_MouseDrag && m_Movable && !m_IsPopupList 
                 && ( (_Button==TW_MOUSE_LEFT && _X>=m_PosX+2*m_Font->m_CharHeight && _X<m_PosX+m_Width-2*m_Font->m_CharHeight && _Y>=m_PosY && _Y<m_PosY+m_Font->m_CharHeight)
                      || (_Button==TW_MOUSE_MIDDLE && _X>=m_PosX && _X<m_PosX+m_Width && _Y>=m_PosY && _Y<m_PosY+m_Height) ) )
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
        else if( _Pressed && !m_MouseDrag && !m_IsPopupList && _Button==TW_MOUSE_LEFT && _X>=m_PosX+m_VarX1-3 && _X<m_PosX+m_VarX1+3 && _Y>m_PosY+m_Font->m_CharHeight && _Y<m_PosY+m_VarY0 )
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
          #ifdef ANT_WINDOWS
            ANT_SET_CURSOR(NS);
          #else
            ANT_SET_CURSOR(Arrow);
          #endif
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
        else if( _Pressed && !m_MouseDrag && m_Resizable && !m_IsPopupList && _Button==TW_MOUSE_LEFT && _X>=m_PosX && _X<m_PosX+m_Font->m_CharHeight && _Y>=m_PosY && _Y<m_PosY+m_Font->m_CharHeight )
        {
            m_MouseDrag = true;
            m_MouseDragResizeUL = true;
            m_MouseOriginX = _X;
            m_MouseOriginY = _Y;
            m_ValuesWidthRatio = (m_Width>0) ? (double)m_ValuesWidth/m_Width : 0;
            ANT_SET_CURSOR(TopLeft);
        }
        else if( !_Pressed && m_MouseDragResizeUL )
        {
            m_MouseDrag = false;
            m_MouseDragResizeUL = false;
            ANT_SET_CURSOR(Arrow);
        }
        else if( _Pressed && !m_MouseDrag && m_Resizable && !m_IsPopupList && _Button==TW_MOUSE_LEFT && _X>=m_PosX+m_Width-m_Font->m_CharHeight && _X<m_PosX+m_Width && _Y>=m_PosY && _Y<m_PosY+m_Font->m_CharHeight )
        {
            m_MouseDrag = true;
            m_MouseDragResizeUR = true;
            m_MouseOriginX = _X;
            m_MouseOriginY = _Y;
            m_ValuesWidthRatio = (m_Width>0) ? (double)m_ValuesWidth/m_Width : 0;
            ANT_SET_CURSOR(TopRight);
        }
        else if( !_Pressed && m_MouseDragResizeUR )
        {
            m_MouseDrag = false;
            m_MouseDragResizeUR = false;
            ANT_SET_CURSOR(Arrow);
        }
        else if( _Pressed && !m_MouseDrag && m_Resizable && !m_IsPopupList && _Button==TW_MOUSE_LEFT && _X>=m_PosX && _X<m_PosX+m_Font->m_CharHeight && _Y>=m_PosY+m_Height-m_Font->m_CharHeight && _Y<m_PosY+m_Height )
        {
            m_MouseDrag = true;
            m_MouseDragResizeLL = true;
            m_MouseOriginX = _X;
            m_MouseOriginY = _Y;
            m_ValuesWidthRatio = (m_Width>0) ? (double)m_ValuesWidth/m_Width : 0;
            ANT_SET_CURSOR(BottomLeft);
        }
        else if( !_Pressed && m_MouseDragResizeLL )
        {
            m_MouseDrag = false;
            m_MouseDragResizeLL = false;
            ANT_SET_CURSOR(Arrow);
        }
        else if( _Pressed && !m_MouseDrag && m_Resizable && !m_IsPopupList && _Button==TW_MOUSE_LEFT && _X>=m_PosX+m_Width-m_Font->m_CharHeight && _X<m_PosX+m_Width && _Y>=m_PosY+m_Height-m_Font->m_CharHeight && _Y<m_PosY+m_Height )
        {
            m_MouseDrag = true;
            m_MouseDragResizeLR = true;
            m_MouseOriginX = _X;
            m_MouseOriginY = _Y;
            m_ValuesWidthRatio = (m_Width>0) ? (double)m_ValuesWidth/m_Width : 0;
            ANT_SET_CURSOR(BottomRight);
        }
        else if( !_Pressed && m_MouseDragResizeLR )
        {
            m_MouseDrag = false;
            m_MouseDragResizeLR = false;
            ANT_SET_CURSOR(Arrow);
        }
        else if( _Pressed && !m_IsPopupList && _Button==TW_MOUSE_LEFT && m_HighlightLabelsHeader )
        {
            int w = ComputeLabelsWidth(m_Font);
            if( w<m_Font->m_CharHeight )
                w = m_Font->m_CharHeight;
            m_ValuesWidth = m_VarX2 - m_VarX0 - w;
            if( m_ValuesWidth<m_Font->m_CharHeight )
                m_ValuesWidth = m_Font->m_CharHeight;
            if( m_ValuesWidth>m_VarX2 - m_VarX0 )
                m_ValuesWidth = max(m_VarX2 - m_VarX0 - m_Font->m_CharHeight, 0);
            NotUpToDate();
            ANT_SET_CURSOR(Arrow);
        }
        else if( _Pressed && !m_IsPopupList && _Button==TW_MOUSE_LEFT && m_HighlightValuesHeader )
        {
            int w = ComputeValuesWidth(m_Font);
            if( w<2*m_Font->m_CharHeight )
                w = 2*m_Font->m_CharHeight; // enough to draw a button
            m_ValuesWidth = w;
            if( m_ValuesWidth>m_VarX2 - m_VarX0 )
                m_ValuesWidth = max(m_VarX2 - m_VarX0 - m_Font->m_CharHeight, 0);
            NotUpToDate();
            ANT_SET_CURSOR(Arrow);
        }
        else if( _Pressed && g_TwMgr->m_FontResizable && !m_IsPopupList && _X>=m_PosX+m_Font->m_CharHeight && _X<m_PosX+2*m_Font->m_CharHeight && _Y>m_PosY && _Y<m_PosY+m_Font->m_CharHeight )
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
        else if( _Pressed && m_Iconifiable && !m_IsPopupList && _Button==TW_MOUSE_LEFT && _X>=m_PosX+m_Width-2*m_Font->m_CharHeight && _X<m_PosX+m_Width-m_Font->m_CharHeight && _Y>m_PosY && _Y<m_PosY+m_Font->m_CharHeight )
        {
            // minimize
            g_TwMgr->Minimize(this);
            ANT_SET_CURSOR(Arrow);
        }
        else if( m_IsHelpBar && _Pressed && !g_TwMgr->m_IsRepeatingMousePressed && _X>=m_PosX+m_VarX0 && _X<m_PosX+m_Width-m_Font->m_CharHeight && _Y>m_PosY+m_Height-m_Font->m_CharHeight && _Y<m_PosY+m_Height )
        {
            /*
            const char *WebPage = "http://www.antisphere.com/Wiki/tools:anttweakbar";
            #if defined ANT_WINDOWS
                ShellExecute(NULL, "open", WebPage, NULL, NULL, SW_SHOWNORMAL);
            #elif defined ANT_UNIX
                // brute force: try all the possible browsers (I don't know how to find the default one; someone?)
                char DefaultBrowsers[] = "firefox,chrome,opera,mozilla,konqueror,galeon,dillo,netscape";
                char *browser = strtok(DefaultBrowsers, ",");
                char cmd[256];
                while(browser)
                {
                    snprintf(cmd, sizeof(cmd), "%s \"%s\" 1>& null &", browser, WebPage);
                    if( system(cmd) ) {} // avoiding warn_unused_result
                    browser = strtok(NULL, ","); // grab the next browser
                }
            #elif defined ANT_OSX
                char cmd[256];
                snprintf(cmd, sizeof(cmd), "open \"%s\" 1>& null &", WebPage);
                if( system(cmd) ) {} // avoiding warn_unused_result
            #endif
            ANT_SET_CURSOR(Hand);
            */
        }
        else
        {
            CustomArea = true;
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

    if( g_TwMgr!=NULL ) // Mgr might have been destroyed by the client inside a callback call
        if( _Pressed && !EditInPlaceActive && m_EditInPlace.m_Active )
            EditInPlaceEnd(true);
        
    // Handled by a custom widget?
    if( g_TwMgr!=NULL && (!Handled || CustomArea) && !m_IsMinimized && m_CustomRecords.size()>0 )
    {
        bool CustomHandled = false;
        for( int s=0; s<2; ++s )    // 2 iterations: first for custom widget having focus, second for others if no focused widget.
            for( CustomMap::iterator it=m_CustomRecords.begin(); it!=m_CustomRecords.end(); ++it )
            {
                CTwMgr::CStructProxy *sProxy = it->first;
                const CCustomRecord& r = it->second;
                if( (s==1 || sProxy->m_CustomCaptureFocus) && !CustomHandled && sProxy!=NULL && sProxy->m_CustomMouseButtonCallback!=NULL && r.m_XMin<r.m_XMax && r.m_Y0<r.m_Y1 && r.m_YMin<=r.m_YMax && r.m_YMin>=r.m_Y0 && r.m_YMax<=r.m_Y1 )
                {
                    if( sProxy->m_CustomCaptureFocus || (_X>=r.m_XMin && _X<r.m_XMax && _Y>=r.m_YMin && _Y<r.m_YMax) )
                    {
                        sProxy->m_CustomCaptureFocus = _Pressed;
                        CustomHandled = sProxy->m_CustomMouseButtonCallback(_Button, _Pressed, _X-r.m_XMin, _Y-r.m_Y0, r.m_XMax-r.m_XMin, r.m_Y1-r.m_Y0, sProxy->m_StructExtData, sProxy->m_StructClientData, this, r.m_Var);
                        s = 2; // force s-loop exit
                    }
                }
                else if( sProxy!=NULL )
                {
                    sProxy->m_CustomCaptureFocus = false;   // force free focus, just in case.
                    ANT_SET_CURSOR(Arrow);
                }
            }
        if( CustomHandled )
            Handled = true;
    }

    return Handled;
}


//  ---------------------------------------------------------------------------

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
        {
            Handled = true;
            if( m_EditInPlace.m_Active )
                EditInPlaceEnd(true);
        }
    }

    return Handled;
}

//  ---------------------------------------------------------------------------

CTwVarAtom *CTwVarGroup::FindShortcut(int _Key, int _Modifiers, bool *_DoIncr)
{
    CTwVarAtom *Atom;
    int Mask = 0xffffffff;
    if( _Key>' ' && _Key<256 ) // don't test SHIFT if _Key is a common key
        Mask &= ~TW_KMOD_SHIFT;

    // don't test KMOD_NUM and KMOD_CAPS modifiers coming from SDL
    Mask &= ~(0x1000);  // 0x1000 is the KMOD_NUM value defined in SDL_keysym.h
    Mask &= ~(0x2000);  // 0x2000 is the KMOD_CAPS value defined in SDL_keysym.h

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
            _Modifiers &= ~TW_KMOD_SHIFT;   // remove shift modifier
            if( _Key==266 )          // SDLK_KP_PERIOD
                _Key = Num ? '.' : TW_KEY_DELETE;
            else if( _Key==267 )     // SDLK_KP_DIVIDE
                _Key = '/';
            else if( _Key==268 )     // SDLK_KP_MULTIPLY
                _Key = '*';
            else if( _Key==269 )     // SDLK_KP_MINUS
                _Key = '-';
            else if( _Key==270 )     // SDLK_KP_PLUS
                _Key = '+';
            else if( _Key==271 )     // SDLK_KP_ENTER
                _Key = TW_KEY_RETURN;
            else if( _Key==272 )     // SDLK_KP_EQUALS
                _Key = '=';
            else if( Num )           // num SDLK_KP0..9
                _Key += '0' - 256;
            else if( _Key==256 )     // non-num SDLK_KP01
                _Key = TW_KEY_INSERT;
            else if( _Key==257 )     // non-num SDLK_KP1
                _Key = TW_KEY_END;
            else if( _Key==258 )     // non-num SDLK_KP2
                _Key = TW_KEY_DOWN;
            else if( _Key==259 )     // non-num SDLK_KP3
                _Key = TW_KEY_PAGE_DOWN;
            else if( _Key==260 )     // non-num SDLK_KP4
                _Key = TW_KEY_LEFT;
            else if( _Key==262 )     // non-num SDLK_KP6
                _Key = TW_KEY_RIGHT;
            else if( _Key==263 )     // non-num SDLK_KP7
                _Key = TW_KEY_HOME;
            else if( _Key==264 )     // non-num SDLK_KP8
                _Key = TW_KEY_UP;
            else if( _Key==265 )     // non-num SDLK_KP9
                _Key = TW_KEY_PAGE_UP;
        }
        */

        /*
        string Str;
        TwGetKeyString(&Str, _Key, _Modifiers);
        printf("key: %d 0x%04xd %s\n", _Key, _Modifiers, Str.c_str());
        */

        if( m_EditInPlace.m_Active )
        {
            Handled = EditInPlaceKeyPressed(_Key, _Modifiers);
        }
        else
        {
            bool BarActive = (m_DrawHandles || m_IsPopupList) && !m_IsMinimized;
            bool DoIncr = true;
            CTwVarAtom *Atom = m_VarRoot.FindShortcut(_Key, _Modifiers, &DoIncr);
            if( Atom!=NULL && Atom->m_Visible )
            {
                if( !Atom->m_ReadOnly )
                {
                    Atom->Increment( DoIncr ? +1 : -1 );
                    if( g_TwMgr==NULL ) // Mgr might have been destroyed by the client inside a callback call
                        return 1;
                    m_HighlightClickBtnAuto = g_TwMgr->m_Timer.GetTime();
                }
                NotUpToDate();
                Show(Atom);
                Handled = true;
            }
            else if( BarActive && m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size() && m_HierTags[m_HighlightedLine].m_Var )
            {
                if( _Key==TW_KEY_RIGHT )
                {
                    if( !m_HierTags[m_HighlightedLine].m_Var->IsGroup() ) 
                    {
                        CTwVarAtom *Atom = static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var);
                        bool Accept = !Atom->m_NoSlider || Atom->m_Type==TW_TYPE_BUTTON 
                                      || Atom->m_Type==TW_TYPE_BOOL8 || Atom->m_Type==TW_TYPE_BOOL16 || Atom->m_Type==TW_TYPE_BOOL32 || Atom->m_Type==TW_TYPE_BOOLCPP
                                      || IsEnumType(Atom->m_Type);
                        if( !Atom->IsReadOnly() && !m_IsPopupList && Accept )
                        {
                            Atom->Increment(+1);
                            if( g_TwMgr==NULL ) // Mgr might have been destroyed by the client inside a callback call
                                return 1;
                            m_HighlightClickBtnAuto = g_TwMgr->m_Timer.GetTime();
                            NotUpToDate();
                        }
                    } 
                    else 
                    {
                        CTwVarGroup *Grp = static_cast<CTwVarGroup *>(m_HierTags[m_HighlightedLine].m_Var);
                        if( !Grp->m_Open )
                        {
                            Grp->m_Open = true;
                            NotUpToDate();
                        }
                    }
                    Handled = true;
                }
                else if( _Key==TW_KEY_LEFT )
                {
                    if( !m_HierTags[m_HighlightedLine].m_Var->IsGroup() ) 
                    {
                        CTwVarAtom *Atom = static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var);
                        bool Accept = !Atom->m_NoSlider || Atom->m_Type==TW_TYPE_BUTTON 
                                      || Atom->m_Type==TW_TYPE_BOOL8 || Atom->m_Type==TW_TYPE_BOOL16 || Atom->m_Type==TW_TYPE_BOOL32 || Atom->m_Type==TW_TYPE_BOOLCPP
                                      || IsEnumType(Atom->m_Type);
                        if( !Atom->IsReadOnly() && Accept && !m_IsPopupList )
                        {
                            Atom->Increment(-1);
                            if( g_TwMgr==NULL ) // Mgr might have been destroyed by the client inside a callback call
                                return 1;
                            m_HighlightClickBtnAuto = g_TwMgr->m_Timer.GetTime();
                            NotUpToDate();
                        }
                    } 
                    else 
                    {
                        CTwVarGroup *Grp = static_cast<CTwVarGroup *>(m_HierTags[m_HighlightedLine].m_Var);
                        if( Grp->m_Open )
                        {
                            Grp->m_Open = false;
                            NotUpToDate();
                        }
                    }
                    Handled = true;
                }
                else if( _Key==TW_KEY_RETURN )
                {
                    if( !m_HierTags[m_HighlightedLine].m_Var->IsGroup() ) 
                    {
                        CTwVarAtom *Atom = static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var);
                        if( !Atom->IsReadOnly() )
                        {
                            if( Atom->m_Type==TW_TYPE_BUTTON || Atom->m_Type==TW_TYPE_BOOLCPP 
                                || Atom->m_Type==TW_TYPE_BOOL8 || Atom->m_Type==TW_TYPE_BOOL16 || Atom->m_Type==TW_TYPE_BOOL32 )
                            {
                                bool isPopup =  m_IsPopupList;
                                Atom->Increment(+1);
                                if( g_TwMgr==NULL // Mgr might have been destroyed by the client inside a callback call
                                    || isPopup )  // A popup destroys itself
                                    return 1;
                                m_HighlightClickBtnAuto = g_TwMgr->m_Timer.GetTime();
                                NotUpToDate();
                            } 
                            else // if( IsEnumType(Atom->m_Type) )
                            {
                                // simulate a mouse click
                                int y = m_PosY + m_VarY0 + m_HighlightedLine*(m_Font->m_CharHeight+m_LineSep) + m_Font->m_CharHeight/2;
                                int x = m_PosX + m_VarX1 + 2;
                                if( x>m_PosX+m_VarX2-2 ) 
                                    x = m_PosX + m_VarX2 - 2;
                                MouseMotion(x, y);
                                MouseButton(TW_MOUSE_LEFT, true, x, y);
                            }
                        }
                    } 
                    else 
                    {
                        CTwVarGroup *Grp = static_cast<CTwVarGroup *>(m_HierTags[m_HighlightedLine].m_Var);
                        Grp->m_Open = !Grp->m_Open;
                        NotUpToDate();
                    }
                    Handled = true;
                }
                else if( _Key==TW_KEY_UP )
                {
                    --m_HighlightedLine;
                    if( m_HighlightedLine<0 )
                    {
                        m_HighlightedLine = 0;
                        if( m_FirstLine>0 )
                        {
                            --m_FirstLine;
                            NotUpToDate();
                        }
                    }
                    m_HighlightedLineLastValid = m_HighlightedLine;
                    Handled = true;
                }
                else if( _Key==TW_KEY_DOWN )
                {
                    ++m_HighlightedLine;
                    if( m_HighlightedLine>=(int)m_HierTags.size() )
                    {
                        m_HighlightedLine = (int)m_HierTags.size() - 1;
                        if( m_FirstLine<m_NbHierLines-m_NbDisplayedLines )
                        {
                            ++m_FirstLine;
                            NotUpToDate();
                        }                    
                    }
                    m_HighlightedLineLastValid = m_HighlightedLine;
                    Handled = true;
                }
                else if( _Key==TW_KEY_ESCAPE && m_IsPopupList )
                {
                    Handled = true;
                    CTwBar *LinkedBar = m_BarLinkedToPopupList;
                    TwDeleteBar(this);
                    g_TwMgr->m_PopupBar = NULL;                
                    if( LinkedBar!=NULL )
                        LinkedBar->m_DrawHandles = true;
                    return true; // this bar has been destroyed
                }
            }
            else if( BarActive )
            {
                if( _Key==TW_KEY_UP || _Key==TW_KEY_DOWN || _Key==TW_KEY_LEFT || _Key==TW_KEY_RIGHT || _Key==TW_KEY_RETURN )
                {
                    if( m_HighlightedLineLastValid>=0 && m_HighlightedLineLastValid<(int)m_HierTags.size() )
                        m_HighlightedLine = m_HighlightedLineLastValid;
                    else if( m_HierTags.size()>0 )
                    {
                        if( _Key==TW_KEY_UP )
                            m_HighlightedLine = (int)m_HierTags.size()-1;
                        else
                            m_HighlightedLine = 0;
                    }
                    Handled = true;
                }
                else if( _Key==TW_KEY_ESCAPE && m_IsPopupList )
                {
                    Handled = true;
                    CTwBar *LinkedBar = m_BarLinkedToPopupList;
                    TwDeleteBar(this);
                    g_TwMgr->m_PopupBar = NULL;                
                    if( LinkedBar!=NULL )
                        LinkedBar->m_DrawHandles = true;
                    return true; // this bar has been destroyed
                }
            }
        }
    }
    return Handled;
}

//  ---------------------------------------------------------------------------

bool CTwBar::KeyTest(int _Key, int _Modifiers)
{
    assert(g_TwMgr->m_Graph && g_TwMgr->m_WndHeight>0 && g_TwMgr->m_WndWidth>0);
    bool Handled = false;
    if( !m_UpToDate )
        Update();

    if( _Key>0 && _Key<TW_KEY_LAST )
    {
        if( m_EditInPlace.m_Active )
            Handled = true;
        else
        {
            bool BarActive = (m_DrawHandles || m_IsPopupList) && !m_IsMinimized;
            bool DoIncr;
            CTwVarAtom *Atom = m_VarRoot.FindShortcut(_Key, _Modifiers, &DoIncr);
            if( Atom!=NULL && Atom->m_Visible )
                Handled = true;
            else if( BarActive && ( _Key==TW_KEY_RIGHT || _Key==TW_KEY_LEFT || _Key==TW_KEY_UP || _Key==TW_KEY_DOWN
                                    || _Key==TW_KEY_RETURN || (_Key==TW_KEY_ESCAPE && m_IsPopupList) ) )
                Handled = true;
        }
    }
    return Handled;
}

//  ---------------------------------------------------------------------------

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
            int NbLines = (m_VarY1-m_VarY0+1)/(m_Font->m_CharHeight+m_LineSep);
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

//  ---------------------------------------------------------------------------

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

//  ---------------------------------------------------------------------------

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

//  ---------------------------------------------------------------------------

void DrawArc(int _X, int _Y, int _Radius, float _StartAngleDeg, float _EndAngleDeg, color32 _Color) // angles in degree
{
    ITwGraph *Gr = g_TwMgr->m_Graph;
    if( Gr==NULL || !Gr->IsDrawing() || _Radius==0 || _StartAngleDeg==_EndAngleDeg )
        return;

    float startAngle = (float)M_PI*_StartAngleDeg/180;
    float endAngle = (float)M_PI*_EndAngleDeg/180;
    //float stepAngle = 8/(float)_Radius;   // segment length = 8 pixels
    float stepAngle = 4/(float)_Radius; // segment length = 4 pixels
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

//  ---------------------------------------------------------------------------

CTwBar::CRotoSlider::CRotoSlider()
{
    m_Var = NULL;
    m_Active = false;
    m_ActiveMiddle = false;
    m_Subdiv = 256; // will be recalculated in RotoOnLButtonDown
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
        float r = sqrtf(float(  (m_Roto.m_Current.x-m_Roto.m_Origin.x)*(m_Roto.m_Current.x-m_Roto.m_Origin.x) 
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
        if( m_HighlightRotoBtn )
            ANT_SET_CURSOR(Point);
        else
            ANT_SET_CURSOR(Arrow);
    }
}

void CTwBar::RotoOnLButtonDown(int _X, int _Y)
{
    CPoint p(_X, _Y);
    if( !m_Roto.m_Active && m_HighlightedLine>=0 && m_HighlightedLine<(int)m_HierTags.size() && m_HierTags[m_HighlightedLine].m_Var && !m_HierTags[m_HighlightedLine].m_Var->IsGroup() )
    {
        m_Roto.m_Var = static_cast<CTwVarAtom *>(m_HierTags[m_HighlightedLine].m_Var);
        int y = m_PosY + m_VarY0 + m_HighlightedLine*(m_Font->m_CharHeight+m_LineSep) + m_Font->m_CharHeight/2;
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
        //if( m_Roto.m_Var )
        //  RotoSetValue(RotoGetSteppedValue());

        m_Roto.m_Var = NULL;
        m_Roto.m_Active = false;
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


//  ---------------------------------------------------------------------------

CTwBar::CEditInPlace::CEditInPlace()
{
    assert( g_TwMgr!=NULL && g_TwMgr->m_Graph!=NULL );

    m_Var = NULL;
    m_Active = false;
    m_EditTextObj = g_TwMgr->m_Graph->NewTextObj();
    m_EditSelTextObj = g_TwMgr->m_Graph->NewTextObj();

    m_X = m_Y = m_Width = 0;
}

CTwBar::CEditInPlace::~CEditInPlace()
{
    assert( g_TwMgr!=NULL && g_TwMgr->m_Graph!=NULL );

    if( m_EditTextObj )
        g_TwMgr->m_Graph->DeleteTextObj(m_EditTextObj);
    if( m_EditSelTextObj )
        g_TwMgr->m_Graph->DeleteTextObj(m_EditSelTextObj);
}

bool CTwBar::EditInPlaceIsReadOnly()
{
    if( m_EditInPlace.m_Var==NULL )
        return true;
    else if( m_EditInPlace.m_Var->m_ReadOnly )
        return true;
    else if( m_EditInPlace.m_Var->m_Type==TW_TYPE_CDSTRING && ((m_EditInPlace.m_Var->m_Ptr==NULL && m_EditInPlace.m_Var->m_SetCallback==NULL) || (m_EditInPlace.m_Var->m_Ptr!=NULL && g_TwMgr->m_CopyCDStringToClient==NULL)) )
        return true;
    else if( m_EditInPlace.m_Var->m_Type==TW_TYPE_CDSTDSTRING && m_EditInPlace.m_Var->m_SetCallback==NULL )
        return true;
    else if( m_EditInPlace.m_Var->m_Type==TW_TYPE_STDSTRING && ((m_EditInPlace.m_Var->m_Ptr==NULL && m_EditInPlace.m_Var->m_SetCallback==NULL) || (m_EditInPlace.m_Var->m_Ptr!=NULL && g_TwMgr->m_CopyStdStringToClient==NULL)) )
        return true;
    else
        return false;
}

void CTwBar::EditInPlaceDraw()
{
    if( !m_EditInPlace.m_Active || m_EditInPlace.m_Var==NULL || m_EditInPlace.m_Width<=0 )
        return;

    // adjust m_FirstChar to see the caret, and extract the visible sub-string
    int i, StringLen = (int)m_EditInPlace.m_String.length();
    if( m_EditInPlace.m_FirstChar>m_EditInPlace.m_CaretPos )
        m_EditInPlace.m_FirstChar = m_EditInPlace.m_CaretPos;
    int SubstrWidth = 0;
    for( i=min(m_EditInPlace.m_CaretPos, StringLen-1); i>=0 && SubstrWidth<m_EditInPlace.m_Width; --i )
    {
        unsigned char u = m_EditInPlace.m_String.c_str()[i];
        SubstrWidth += m_Font->m_CharWidth[u];
    }
    int FirstChar = max(0, i);
    if( SubstrWidth>=m_EditInPlace.m_Width )
        FirstChar += 2;
    if( m_EditInPlace.m_FirstChar<FirstChar && FirstChar<StringLen )
        m_EditInPlace.m_FirstChar = FirstChar;
    if( m_EditInPlace.m_CaretPos==m_EditInPlace.m_FirstChar && m_EditInPlace.m_FirstChar>0 )
        --m_EditInPlace.m_FirstChar;
    SubstrWidth = 0;
    for( i=m_EditInPlace.m_FirstChar; i<StringLen && SubstrWidth<m_EditInPlace.m_Width; ++i )
    {
        unsigned char u = m_EditInPlace.m_String.c_str()[i];
        SubstrWidth += m_Font->m_CharWidth[u];
    }
    int LastChar = i;
    if( SubstrWidth>=m_EditInPlace.m_Width )
        --LastChar;
    string Substr = m_EditInPlace.m_String.substr( m_EditInPlace.m_FirstChar, LastChar-m_EditInPlace.m_FirstChar );

    // compute caret x pos
    int CaretX = m_PosX + m_EditInPlace.m_X;
    for( i=m_EditInPlace.m_FirstChar; i<m_EditInPlace.m_CaretPos && i<StringLen; ++i )
    {
        unsigned char u = m_EditInPlace.m_String.c_str()[i];
        CaretX += m_Font->m_CharWidth[u];
    }

    // draw edit text
    color32 ColText = EditInPlaceIsReadOnly() ? m_ColValTextRO : m_ColEditText;
    color32 ColBg = EditInPlaceIsReadOnly() ? m_ColValBg : m_ColEditBg;
    g_TwMgr->m_Graph->BuildText(m_EditInPlace.m_EditTextObj, &Substr, NULL, NULL, 1, m_Font, 0, m_EditInPlace.m_Width);
    g_TwMgr->m_Graph->DrawText(m_EditInPlace.m_EditTextObj, m_PosX+m_EditInPlace.m_X, m_PosY+m_EditInPlace.m_Y, ColText, ColBg);

    // draw selected text
    string StrSelected = "";
    if( m_EditInPlace.m_CaretPos>m_EditInPlace.m_SelectionStart )
    {
        int FirstSel = max(m_EditInPlace.m_SelectionStart, m_EditInPlace.m_FirstChar);
        int LastSel = min(m_EditInPlace.m_CaretPos, LastChar);
        StrSelected = m_EditInPlace.m_String.substr( FirstSel, LastSel-FirstSel );
    }
    else
    {
        int FirstSel = max(m_EditInPlace.m_CaretPos, m_EditInPlace.m_FirstChar);
        int LastSel = min(m_EditInPlace.m_SelectionStart, LastChar);
        StrSelected = m_EditInPlace.m_String.substr( FirstSel, LastSel-FirstSel );
    }
    int SelWidth = 0;
    for( i=0; i<(int)StrSelected.length(); ++i )
    {
        unsigned char u = StrSelected.c_str()[i];
        SelWidth += m_Font->m_CharWidth[u];
    }
    if( SelWidth>0 && StrSelected.length()>0 )
    {
        color32 ColSelBg = EditInPlaceIsReadOnly() ? m_ColValTextRO : m_ColEditSelBg;
        g_TwMgr->m_Graph->BuildText(m_EditInPlace.m_EditSelTextObj, &StrSelected, NULL, NULL, 1, m_Font, 0, SelWidth);
        if ( m_EditInPlace.m_CaretPos>m_EditInPlace.m_SelectionStart )
            g_TwMgr->m_Graph->DrawText(m_EditInPlace.m_EditSelTextObj, CaretX-SelWidth, m_PosY+m_EditInPlace.m_Y, m_ColEditSelText, ColSelBg);
        else
            g_TwMgr->m_Graph->DrawText(m_EditInPlace.m_EditSelTextObj, CaretX, m_PosY+m_EditInPlace.m_Y, m_ColEditSelText, ColSelBg);
    }

    // draw caret
    if( CaretX<=m_PosX+m_EditInPlace.m_X+m_EditInPlace.m_Width )
        g_TwMgr->m_Graph->DrawLine( CaretX, m_PosY+m_EditInPlace.m_Y+1, CaretX, m_PosY+m_EditInPlace.m_Y+m_Font->m_CharHeight, m_ColEditText );
}

bool CTwBar::EditInPlaceAcceptVar(const CTwVarAtom* _Var)
{
    if( _Var==NULL )
        return false;
    if( _Var->m_Type>=TW_TYPE_CHAR && _Var->m_Type<=TW_TYPE_DOUBLE )
        return true;
    if( _Var->m_Type==TW_TYPE_CDSTRING || _Var->m_Type==TW_TYPE_CDSTDSTRING || _Var->m_Type==TW_TYPE_STDSTRING )
        return true;
    if( IsCSStringType(_Var->m_Type) )
        return true;

    return false;
}

void CTwBar::EditInPlaceStart(CTwVarAtom* _Var, int _X, int _Y, int _Width)
{
    if( m_EditInPlace.m_Active )
        EditInPlaceEnd(true);

    m_EditInPlace.m_Active = true;
    m_EditInPlace.m_Var = _Var;
    m_EditInPlace.m_X = _X;
    m_EditInPlace.m_Y = _Y;
    m_EditInPlace.m_Width = _Width;
    m_EditInPlace.m_Var->ValueToString(&m_EditInPlace.m_String);
    if( m_EditInPlace.m_Var->m_Type==TW_TYPE_CHAR )
        m_EditInPlace.m_String = m_EditInPlace.m_String.substr(0, 1);
    m_EditInPlace.m_CaretPos = (int)m_EditInPlace.m_String.length();
    if( EditInPlaceIsReadOnly() )
        m_EditInPlace.m_SelectionStart = m_EditInPlace.m_CaretPos;
    else
        m_EditInPlace.m_SelectionStart = 0;
    m_EditInPlace.m_FirstChar = 0;
}

void CTwBar::EditInPlaceEnd(bool _Commit)
{
    if( _Commit && m_EditInPlace.m_Active && m_EditInPlace.m_Var!=NULL )
    {
        if( m_EditInPlace.m_Var->m_Type==TW_TYPE_CDSTRING || m_EditInPlace.m_Var->m_Type==TW_TYPE_CDSTDSTRING )
        {
            if( m_EditInPlace.m_Var->m_SetCallback!=NULL )
            {
                const char *String = m_EditInPlace.m_String.c_str();
                m_EditInPlace.m_Var->m_SetCallback(&String, m_EditInPlace.m_Var->m_ClientData);
            }
            else if( m_EditInPlace.m_Var->m_Type!=TW_TYPE_CDSTDSTRING )
            {
                char **StringPtr = (char **)m_EditInPlace.m_Var->m_Ptr;
                if( StringPtr!=NULL && g_TwMgr->m_CopyCDStringToClient!=NULL )
                    g_TwMgr->m_CopyCDStringToClient(StringPtr, m_EditInPlace.m_String.c_str());
            }
        }
        else if( m_EditInPlace.m_Var->m_Type==TW_TYPE_STDSTRING )
        {   
            // this case should never happened: TW_TYPE_STDSTRING are converted to TW_TYPE_CDSTDSTRING by TwAddVar
            if( m_EditInPlace.m_Var->m_SetCallback!=NULL )
                m_EditInPlace.m_Var->m_SetCallback(&(m_EditInPlace.m_String), m_EditInPlace.m_Var->m_ClientData);
            else
            {
                string *StringPtr = (string *)m_EditInPlace.m_Var->m_Ptr;
                if( StringPtr!=NULL && g_TwMgr->m_CopyStdStringToClient!=NULL )
                    g_TwMgr->m_CopyStdStringToClient(*StringPtr, m_EditInPlace.m_String);
            }
        }
        else if( IsCSStringType(m_EditInPlace.m_Var->m_Type) )
        {
            int n = TW_CSSTRING_SIZE(m_EditInPlace.m_Var->m_Type);
            if( n>0 )
            {
                if( (int)m_EditInPlace.m_String.length()>n-1 )
                    m_EditInPlace.m_String.resize(n-1);
                if( m_EditInPlace.m_Var->m_SetCallback!=NULL )
                    m_EditInPlace.m_Var->m_SetCallback(m_EditInPlace.m_String.c_str(), m_EditInPlace.m_Var->m_ClientData);
                else if( m_EditInPlace.m_Var->m_Ptr!=NULL )
                {
                    if( n>1 )
                        strncpy((char *)m_EditInPlace.m_Var->m_Ptr, m_EditInPlace.m_String.c_str(), n-1);
                    ((char *)m_EditInPlace.m_Var->m_Ptr)[n-1] = '\0';
                }
            }
        }
        else
        {
            double Val = 0, Min = 0, Max = 0, Step = 0;
            int n = 0;
            if( m_EditInPlace.m_Var->m_Type==TW_TYPE_CHAR )
            {
                unsigned char Char = 0;
                n = sscanf(m_EditInPlace.m_String.c_str(), "%c", &Char);
                Val = Char;
            }
            else
                n = sscanf(m_EditInPlace.m_String.c_str(), "%lf", &Val);
            if( n==1 )
            {
                m_EditInPlace.m_Var->MinMaxStepToDouble(&Min, &Max, &Step);
                if( Val<Min )
                    Val = Min;
                else if( Val>Max )
                    Val = Max;
                m_EditInPlace.m_Var->ValueFromDouble(Val);
            }
        }
        if( g_TwMgr!=NULL ) // Mgr might have been destroyed by the client inside a callback call
            NotUpToDate();
    }
    m_EditInPlace.m_Active = false;
    m_EditInPlace.m_Var = NULL;
}

bool CTwBar::EditInPlaceKeyPressed(int _Key, int _Modifiers)
{
    if( !m_EditInPlace.m_Active )
        return false;
    bool Handled = true; // if EditInPlace is active, it catches all key events
    bool DoCopy = false, DoPaste = false;

    switch( _Key )
    {
    case TW_KEY_ESCAPE:
        EditInPlaceEnd(false);
        break;
    case TW_KEY_RETURN:
        EditInPlaceEnd(true);
        break;
    case TW_KEY_LEFT:
        if( _Modifiers==TW_KMOD_SHIFT )
            m_EditInPlace.m_CaretPos = max(0, m_EditInPlace.m_CaretPos-1);
        else
        {
            if( m_EditInPlace.m_SelectionStart!=m_EditInPlace.m_CaretPos )
                m_EditInPlace.m_CaretPos = min(m_EditInPlace.m_SelectionStart, m_EditInPlace.m_CaretPos);
            else
                m_EditInPlace.m_CaretPos = max(0, m_EditInPlace.m_CaretPos-1);
            m_EditInPlace.m_SelectionStart = m_EditInPlace.m_CaretPos;
        }
        break;
    case TW_KEY_RIGHT:
        if( _Modifiers==TW_KMOD_SHIFT )
            m_EditInPlace.m_CaretPos = min((int)m_EditInPlace.m_String.length(), m_EditInPlace.m_CaretPos+1);
        else
        {
            if( m_EditInPlace.m_SelectionStart!=m_EditInPlace.m_CaretPos )
                m_EditInPlace.m_CaretPos = max(m_EditInPlace.m_SelectionStart, m_EditInPlace.m_CaretPos);
            else
                m_EditInPlace.m_CaretPos = min((int)m_EditInPlace.m_String.length(), m_EditInPlace.m_CaretPos+1);
            m_EditInPlace.m_SelectionStart = m_EditInPlace.m_CaretPos;
        }
        break;
    case TW_KEY_BACKSPACE:
        if( !EditInPlaceIsReadOnly() )
        {
            if( m_EditInPlace.m_SelectionStart==m_EditInPlace.m_CaretPos )
                m_EditInPlace.m_SelectionStart = max(0, m_EditInPlace.m_CaretPos-1);
            EditInPlaceEraseSelect();
        }
        break;
    case TW_KEY_DELETE:
        if( !EditInPlaceIsReadOnly() )
        {
            if( m_EditInPlace.m_SelectionStart==m_EditInPlace.m_CaretPos )
                m_EditInPlace.m_SelectionStart = min(m_EditInPlace.m_CaretPos+1, (int)m_EditInPlace.m_String.length());
            EditInPlaceEraseSelect();
        }
        break;
    case TW_KEY_HOME:
        m_EditInPlace.m_CaretPos = 0;
        if( _Modifiers!=TW_KMOD_SHIFT )
            m_EditInPlace.m_SelectionStart = m_EditInPlace.m_CaretPos;
        break;
    case TW_KEY_END:
        m_EditInPlace.m_CaretPos = (int)m_EditInPlace.m_String.length();
        if( _Modifiers!=TW_KMOD_SHIFT )
            m_EditInPlace.m_SelectionStart = m_EditInPlace.m_CaretPos;
        break;
    case TW_KEY_INSERT:
        if( _Modifiers==TW_KMOD_CTRL )
            DoCopy = true;
        else if( _Modifiers==TW_KMOD_SHIFT )
            DoPaste = true;
        break;
    default:
        if( _Modifiers==TW_KMOD_CTRL )
        {
            if( _Key=='c' || _Key=='C' )
                DoCopy = true;
            else if( _Key=='v' || _Key=='V' )
                DoPaste = true;
        }
        else if( _Key>=32 && _Key<=255 )
        {
            if( !EditInPlaceIsReadOnly() && m_EditInPlace.m_CaretPos>=0 && m_EditInPlace.m_CaretPos<=(int)m_EditInPlace.m_String.length() )
            {
                if( m_EditInPlace.m_SelectionStart!=m_EditInPlace.m_CaretPos )
                    EditInPlaceEraseSelect();
                string Str(1, (char)_Key);
                m_EditInPlace.m_String.insert(m_EditInPlace.m_CaretPos, Str);
                ++m_EditInPlace.m_CaretPos;
                m_EditInPlace.m_SelectionStart = m_EditInPlace.m_CaretPos;
            }
        }
    }

    if( DoPaste && !EditInPlaceIsReadOnly() )
    {
        if( m_EditInPlace.m_SelectionStart!=m_EditInPlace.m_CaretPos )
            EditInPlaceEraseSelect();
        string Str = "";
        if( EditInPlaceGetClipboard(&Str) && Str.length()>0 )
        {
            m_EditInPlace.m_String.insert(m_EditInPlace.m_CaretPos, Str);
            m_EditInPlace.m_CaretPos += (int)Str.length();
            m_EditInPlace.m_SelectionStart = m_EditInPlace.m_CaretPos;
        }
    }
    if( DoCopy )
    {
        string Str = "";
        if( m_EditInPlace.m_CaretPos>m_EditInPlace.m_SelectionStart )
            Str = m_EditInPlace.m_String.substr(m_EditInPlace.m_SelectionStart, m_EditInPlace.m_CaretPos-m_EditInPlace.m_SelectionStart);
        else if( m_EditInPlace.m_CaretPos<m_EditInPlace.m_SelectionStart )
            Str = m_EditInPlace.m_String.substr(m_EditInPlace.m_CaretPos, m_EditInPlace.m_SelectionStart-m_EditInPlace.m_CaretPos);
        EditInPlaceSetClipboard(Str);
    }

    return Handled;
}


bool CTwBar::EditInPlaceEraseSelect()
{
    assert(m_EditInPlace.m_Active);
    if( !EditInPlaceIsReadOnly() && m_EditInPlace.m_SelectionStart!=m_EditInPlace.m_CaretPos )
    {
        int PosMin = min( m_EditInPlace.m_CaretPos, m_EditInPlace.m_SelectionStart );
        m_EditInPlace.m_String.erase( PosMin, abs(m_EditInPlace.m_CaretPos - m_EditInPlace.m_SelectionStart) );
        m_EditInPlace.m_SelectionStart = m_EditInPlace.m_CaretPos = PosMin;
        if( m_EditInPlace.m_FirstChar>PosMin )
            m_EditInPlace.m_FirstChar = PosMin;
        return true;
    }
    else
        return false;
}


bool CTwBar::EditInPlaceMouseMove(int _X, int _Y, bool _Select)
{
    if ( !m_EditInPlace.m_Active || _Y<m_PosY+m_EditInPlace.m_Y || _Y>m_PosY+m_EditInPlace.m_Y+m_Font->m_CharHeight )
        return false;

    int i, CaretX = m_PosX+m_EditInPlace.m_X;
    for( i=m_EditInPlace.m_FirstChar; i<(int)m_EditInPlace.m_String.length() && CaretX<m_PosX+m_EditInPlace.m_X+m_EditInPlace.m_Width; ++i )
    {
        unsigned char u = m_EditInPlace.m_String.c_str()[i];
        int CharWidth = m_Font->m_CharWidth[u];
        if( _X < CaretX + CharWidth / 2 )
            break;
        CaretX += CharWidth;
    }
    if( CaretX>=m_PosX+m_EditInPlace.m_X+m_EditInPlace.m_Width )
        i = max(0, i-1);

    m_EditInPlace.m_CaretPos = i;
    if( !_Select )
        m_EditInPlace.m_SelectionStart = m_EditInPlace.m_CaretPos;
    return true;
}


bool CTwBar::EditInPlaceGetClipboard(std::string *_OutString)
{
    assert( _OutString!=NULL );
    *_OutString = m_EditInPlace.m_Clipboard; // default implementation

#if defined ANT_WINDOWS

    if( !IsClipboardFormatAvailable(CF_TEXT) )
        return false;
    if( !OpenClipboard(NULL) )
        return false;
    HGLOBAL TextHandle = GetClipboardData(CF_TEXT); 
    if( TextHandle!=NULL ) 
    { 
        const char *TextString = static_cast<char *>(GlobalLock(TextHandle));
        if( TextHandle!=NULL )
        {
            *_OutString = TextString;
            GlobalUnlock(TextHandle);
        } 
    }
    CloseClipboard(); 

#elif defined ANT_UNIX

    if( g_TwMgr->m_CurrentXDisplay!=NULL )
    {
        int NbBytes = 0;
        char *Buffer = XFetchBytes(g_TwMgr->m_CurrentXDisplay, &NbBytes);
        if( Buffer!=NULL )
        {
            if( NbBytes>0 )
            {
                char *Text = new char[NbBytes+1];
                memcpy(Text, Buffer, NbBytes);
                Text[NbBytes] = '\0';
                *_OutString = Text;
                delete[] Text;
            }
            XFree(Buffer);
        }
    }

#endif

    return true;
}


bool CTwBar::EditInPlaceSetClipboard(const std::string& _String)
{
    if( _String.length()<=0 )
        return false;   // keep last clipboard
    m_EditInPlace.m_Clipboard = _String; // default implementation

#if defined ANT_WINDOWS

    if( !OpenClipboard(NULL) )
        return false;
    EmptyClipboard();
    HGLOBAL TextHandle = GlobalAlloc(GMEM_MOVEABLE, _String.length()+1);
    if( TextHandle==NULL )
    { 
        CloseClipboard(); 
        return false; 
    }
    char *TextString = static_cast<char *>(GlobalLock(TextHandle));
    memcpy(TextString, _String.c_str(), _String.length());
    TextString[_String.length()] = '\0';
    GlobalUnlock(TextHandle); 
    SetClipboardData(CF_TEXT, TextHandle);
    CloseClipboard();

#elif defined ANT_UNIX

    if( g_TwMgr->m_CurrentXDisplay!=NULL )
    {
        XSetSelectionOwner(g_TwMgr->m_CurrentXDisplay, XA_PRIMARY, None, CurrentTime);
        char *Text = new char[_String.length()+1];
        memcpy(Text, _String.c_str(), _String.length());
        Text[_String.length()] = '\0';
        XStoreBytes(g_TwMgr->m_CurrentXDisplay, Text, _String.length());
        delete[] Text;
    }

#endif

    return true;
}


//  ---------------------------------------------------------------------------


