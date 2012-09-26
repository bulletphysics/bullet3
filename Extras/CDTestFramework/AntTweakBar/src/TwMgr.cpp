//  ---------------------------------------------------------------------------
//
//  @file       TwMgr.cpp
//  @author     Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//              For conditions of distribution and use, see License.txt
//
//  ---------------------------------------------------------------------------


#include "TwPrecomp.h"
#include <AntTweakBar.h>
#include "TwMgr.h"
#include "TwBar.h"
#include "TwFonts.h"
#include "TwOpenGL.h"
#include "TwOpenGLCore.h"
#ifdef ANT_WINDOWS
#   include "TwDirect3D9.h"
#   include "TwDirect3D10.h"
#   include "TwDirect3D11.h"
#   include "resource.h"
#   ifdef _DEBUG
#       include <crtdbg.h>
#   endif // _DEBUG
#endif // ANT_WINDOWS

#if !defined(ANT_WINDOWS)
#   define _snprintf snprintf
#endif  // defined(ANT_WINDOWS)


using namespace std;

CTwMgr *g_TwMgr = NULL; // current TwMgr
bool g_BreakOnError = false;
TwErrorHandler g_ErrorHandler = NULL;
int g_TabLength = 4;
CTwBar * const TW_GLOBAL_BAR = (CTwBar *)(-1);
int g_InitWndWidth = -1;
int g_InitWndHeight = -1;
TwCopyCDStringToClient  g_InitCopyCDStringToClient = NULL;
TwCopyStdStringToClient g_InitCopyStdStringToClient = NULL;

// multi-windows
const int TW_MASTER_WINDOW_ID = 0;
typedef map<int, CTwMgr *> CTwWndMap;
CTwWndMap g_Wnds;
CTwMgr *g_TwMasterMgr = NULL;

// error messages
extern const char *g_ErrUnknownAttrib;
extern const char *g_ErrNoValue;
extern const char *g_ErrBadValue;
const char *g_ErrInit       = "Already initialized";
const char *g_ErrShut       = "Already shutdown";
const char *g_ErrNotInit    = "Not initialized";
const char *g_ErrUnknownAPI = "Unsupported graph API";
const char *g_ErrBadDevice  = "Invalid graph device";
const char *g_ErrBadParam   = "Invalid parameter";
const char *g_ErrExist      = "Exists already";
const char *g_ErrNotFound   = "Not found";
const char *g_ErrNthToDo    = "Nothing to do";
const char *g_ErrBadSize    = "Bad size";
const char *g_ErrIsDrawing  = "Asynchronous drawing detected";
const char *g_ErrIsProcessing="Asynchronous processing detected";
const char *g_ErrOffset     = "Offset larger than StructSize";
const char *g_ErrDelStruct  = "Cannot delete a struct member";
const char *g_ErrNoBackQuote= "Name cannot include back-quote";
const char *g_ErrStdString  = "Debug/Release std::string mismatch";
const char *g_ErrCStrParam  = "Value count for TW_PARAM_CSTRING must be 1";
const char *g_ErrOutOfRange = "Index out of range";
const char *g_ErrHasNoValue = "Has no value";
const char *g_ErrBadType    = "Incompatible type";
const char *g_ErrDelHelp    = "Cannot delete help bar";
char g_ErrParse[512];

void ANT_CALL TwGlobalError(const char *_ErrorMessage);

#if defined(ANT_UNIX) || defined(ANT_OSX)
#define _stricmp strcasecmp
#define _strdup strdup
#endif

#ifdef ANT_WINDOWS
    bool g_UseCurRsc = true;    // use dll resources for rotoslider cursors
#endif

//  ---------------------------------------------------------------------------

const float  FLOAT_EPS     = 1.0e-7f;
const float  FLOAT_EPS_SQ  = 1.0e-14f;
const float  FLOAT_PI      = 3.14159265358979323846f;
const double DOUBLE_EPS    = 1.0e-14;
const double DOUBLE_EPS_SQ = 1.0e-28;
const double DOUBLE_PI     = 3.14159265358979323846;

inline double DegToRad(double degree) { return degree * (DOUBLE_PI/180.0); }
inline double RadToDeg(double radian) { return radian * (180.0/DOUBLE_PI); }

//  ---------------------------------------------------------------------------

//  a static global object to verify that Tweakbar module has been properly terminated (in debug mode only)
#ifdef _DEBUG
static struct CTwVerif
{
    ~CTwVerif() 
    { 
        if( g_TwMgr!=NULL )
            g_TwMgr->SetLastError("Tweak bar module has not been terminated properly: call TwTerminate()\n");
    }
} s_Verif;
#endif // _DEBUG

//  ---------------------------------------------------------------------------
//  Color ext type
//  ---------------------------------------------------------------------------

void CColorExt::RGB2HLS()
{
    float fH = 0, fL = 0, fS = 0;
    ColorRGBToHLSf((float)R/255.0f, (float)G/255.0f, (float)B/255.0f, &fH, &fL, &fS);
    H = (int)fH;
    if( H>=360 ) 
        H -= 360;
    else if( H<0 )
        H += 360;
    L = (int)(255.0f*fL + 0.5f);
    if( L<0 )
        L = 0;
    else if( L>255 )
        L = 255;
    S = (int)(255.0f*fS + 0.5f);
    if( S<0 ) 
        S = 0;
    else if( S>255 )
        S = 255;
}

void CColorExt::HLS2RGB()
{
    float fR = 0, fG = 0, fB = 0;
    ColorHLSToRGBf((float)H, (float)L/255.0f, (float)S/255.0f, &fR, &fG, &fB);
    R = (int)(255.0f*fR + 0.5f);
    if( R<0 ) 
        R = 0;
    else if( R>255 )
        R = 255;
    G = (int)(255.0f*fG + 0.5f);
    if( G<0 ) 
        G = 0;
    else if( G>255 )
        G = 255;
    B = (int)(255.0f*fB + 0.5f);
    if( B<0 ) 
        B = 0;
    else if( B>255 )
        B = 255;
}

void ANT_CALL CColorExt::InitColor32CB(void *_ExtValue, void *_ClientData)
{
    CColorExt *ext = static_cast<CColorExt *>(_ExtValue);
    if( ext )
    {
        ext->m_IsColorF = false;
        ext->R = 0;
        ext->G = 0;
        ext->B = 0;
        ext->H = 0;
        ext->L = 0;
        ext->S = 0;
        ext->A = 255;
        ext->m_HLS = false;
        ext->m_HasAlpha = false;
        ext->m_CanHaveAlpha = true;
        if( g_TwMgr && g_TwMgr->m_GraphAPI==TW_DIRECT3D9 ) // D3D10 now use OGL rgba order!
            ext->m_OGL = false;
        else
            ext->m_OGL = true;
        ext->m_PrevConvertedColor = Color32FromARGBi(ext->A, ext->R, ext->G, ext->B);
        ext->m_StructProxy = (CTwMgr::CStructProxy *)_ClientData;
    }
}

void ANT_CALL CColorExt::InitColor3FCB(void *_ExtValue, void *_ClientData)
{
    InitColor32CB(_ExtValue, _ClientData);
    CColorExt *ext = static_cast<CColorExt *>(_ExtValue);
    if( ext )
    {
        ext->m_IsColorF = true;
        ext->m_HasAlpha = false;
        ext->m_CanHaveAlpha = false;
    }
}

void ANT_CALL CColorExt::InitColor4FCB(void *_ExtValue, void *_ClientData)
{
    InitColor32CB(_ExtValue, _ClientData);
    CColorExt *ext = static_cast<CColorExt *>(_ExtValue);
    if( ext )
    {
        ext->m_IsColorF = true;
        ext->m_HasAlpha = true;
        ext->m_CanHaveAlpha = true;
    }
}

void ANT_CALL CColorExt::CopyVarFromExtCB(void *_VarValue, const void *_ExtValue, unsigned int _ExtMemberIndex, void *_ClientData)
{
    unsigned int *var32 = static_cast<unsigned int *>(_VarValue);
    float *varF = static_cast<float *>(_VarValue);
    CColorExt *ext = (CColorExt *)(_ExtValue);
    CTwMgr::CMemberProxy *mProxy = static_cast<CTwMgr::CMemberProxy *>(_ClientData);
    if( _VarValue && ext )
    {
        if( ext->m_HasAlpha && mProxy && mProxy->m_StructProxy && mProxy->m_StructProxy->m_Type==g_TwMgr->m_TypeColor3F )
            ext->m_HasAlpha = false;

        // Synchronize HLS and RGB
        if( _ExtMemberIndex>=0 && _ExtMemberIndex<=2 )
            ext->RGB2HLS();
        else if( _ExtMemberIndex>=3 && _ExtMemberIndex<=5 )
            ext->HLS2RGB();
        else if( mProxy && _ExtMemberIndex==7 && mProxy->m_VarParent )
        {
            assert( mProxy->m_VarParent->m_Vars.size()==8 );
            if(    mProxy->m_VarParent->m_Vars[0]->m_Visible != !ext->m_HLS
                || mProxy->m_VarParent->m_Vars[1]->m_Visible != !ext->m_HLS
                || mProxy->m_VarParent->m_Vars[2]->m_Visible != !ext->m_HLS
                || mProxy->m_VarParent->m_Vars[3]->m_Visible != ext->m_HLS
                || mProxy->m_VarParent->m_Vars[4]->m_Visible != ext->m_HLS
                || mProxy->m_VarParent->m_Vars[5]->m_Visible != ext->m_HLS )
            {
                mProxy->m_VarParent->m_Vars[0]->m_Visible = !ext->m_HLS;
                mProxy->m_VarParent->m_Vars[1]->m_Visible = !ext->m_HLS;
                mProxy->m_VarParent->m_Vars[2]->m_Visible = !ext->m_HLS;
                mProxy->m_VarParent->m_Vars[3]->m_Visible = ext->m_HLS;
                mProxy->m_VarParent->m_Vars[4]->m_Visible = ext->m_HLS;
                mProxy->m_VarParent->m_Vars[5]->m_Visible = ext->m_HLS;
                mProxy->m_Bar->NotUpToDate();
            }
            if( mProxy->m_VarParent->m_Vars[6]->m_Visible != ext->m_HasAlpha )
            {
                mProxy->m_VarParent->m_Vars[6]->m_Visible = ext->m_HasAlpha;
                mProxy->m_Bar->NotUpToDate();
            }
            if( static_cast<CTwVarAtom *>(mProxy->m_VarParent->m_Vars[7])->m_ReadOnly )
            {
                static_cast<CTwVarAtom *>(mProxy->m_VarParent->m_Vars[7])->m_ReadOnly = false;
                mProxy->m_Bar->NotUpToDate();
            }
        }
        // Convert to color32
        color32 col = Color32FromARGBi((ext->m_HasAlpha ? ext->A : 255), ext->R, ext->G, ext->B);
        if( ext->m_OGL && !ext->m_IsColorF )
            col = (col&0xff00ff00) | (unsigned char)(col>>16) | (((unsigned char)(col))<<16);
        if( ext->m_IsColorF )
            Color32ToARGBf(col, (ext->m_HasAlpha ? varF+3 : NULL), varF+0, varF+1, varF+2);
        else
        {
            if( ext->m_HasAlpha )
                *var32 = col;
            else
                *var32 = ((*var32)&0xff000000) | (col&0x00ffffff);
        }
        ext->m_PrevConvertedColor = col;
    }
}

void ANT_CALL CColorExt::CopyVarToExtCB(const void *_VarValue, void *_ExtValue, unsigned int _ExtMemberIndex, void *_ClientData)
{
    const unsigned int *var32 = static_cast<const unsigned int *>(_VarValue);
    const float *varF = static_cast<const float *>(_VarValue);
    CColorExt *ext = static_cast<CColorExt *>(_ExtValue);
    CTwMgr::CMemberProxy *mProxy = static_cast<CTwMgr::CMemberProxy *>(_ClientData);
    if( _VarValue && ext )
    {
        if( ext->m_HasAlpha && mProxy && mProxy->m_StructProxy && mProxy->m_StructProxy->m_Type==g_TwMgr->m_TypeColor3F )
            ext->m_HasAlpha = false;

        if( mProxy && _ExtMemberIndex==7 && mProxy->m_VarParent )
        {
            assert( mProxy->m_VarParent->m_Vars.size()==8 );
            if(    mProxy->m_VarParent->m_Vars[0]->m_Visible != !ext->m_HLS
                || mProxy->m_VarParent->m_Vars[1]->m_Visible != !ext->m_HLS
                || mProxy->m_VarParent->m_Vars[2]->m_Visible != !ext->m_HLS
                || mProxy->m_VarParent->m_Vars[3]->m_Visible != ext->m_HLS
                || mProxy->m_VarParent->m_Vars[4]->m_Visible != ext->m_HLS
                || mProxy->m_VarParent->m_Vars[5]->m_Visible != ext->m_HLS )
            {
                mProxy->m_VarParent->m_Vars[0]->m_Visible = !ext->m_HLS;
                mProxy->m_VarParent->m_Vars[1]->m_Visible = !ext->m_HLS;
                mProxy->m_VarParent->m_Vars[2]->m_Visible = !ext->m_HLS;
                mProxy->m_VarParent->m_Vars[3]->m_Visible = ext->m_HLS;
                mProxy->m_VarParent->m_Vars[4]->m_Visible = ext->m_HLS;
                mProxy->m_VarParent->m_Vars[5]->m_Visible = ext->m_HLS;
                mProxy->m_Bar->NotUpToDate();
            }
            if( mProxy->m_VarParent->m_Vars[6]->m_Visible != ext->m_HasAlpha )
            {
                mProxy->m_VarParent->m_Vars[6]->m_Visible = ext->m_HasAlpha;
                mProxy->m_Bar->NotUpToDate();
            }
            if( static_cast<CTwVarAtom *>(mProxy->m_VarParent->m_Vars[7])->m_ReadOnly )
            {
                static_cast<CTwVarAtom *>(mProxy->m_VarParent->m_Vars[7])->m_ReadOnly = false;
                mProxy->m_Bar->NotUpToDate();
            }
        }
        color32 col;
        if( ext->m_IsColorF )
            col = Color32FromARGBf((ext->m_HasAlpha ? varF[3] : 1), varF[0], varF[1], varF[2]);
        else
            col = *var32;
        if( ext->m_OGL && !ext->m_IsColorF )
            col = (col&0xff00ff00) | (unsigned char)(col>>16) | (((unsigned char)(col))<<16);
        Color32ToARGBi(col, (ext->m_HasAlpha ? &ext->A : NULL), &ext->R, &ext->G, &ext->B);
        if( (col & 0x00ffffff)!=(ext->m_PrevConvertedColor & 0x00ffffff) )
            ext->RGB2HLS();
        ext->m_PrevConvertedColor = col;
    }
}

void ANT_CALL CColorExt::SummaryCB(char *_SummaryString, size_t /*_SummaryMaxLength*/, const void *_ExtValue, void * /*_ClientData*/)
{
    // copy var 
    CColorExt *ext = (CColorExt *)(_ExtValue);
    if( ext && ext->m_StructProxy && ext->m_StructProxy->m_StructData )
    {
        if( ext->m_StructProxy->m_StructGetCallback )
            ext->m_StructProxy->m_StructGetCallback(ext->m_StructProxy->m_StructData, ext->m_StructProxy->m_StructClientData);
        //if( *(unsigned int *)(ext->m_StructProxy->m_StructData)!=ext->m_PrevConvertedColor )
        CopyVarToExtCB(ext->m_StructProxy->m_StructData, ext, 99, NULL);
    }

    //unsigned int col = 0;
    //CopyVar32FromExtCB(&col, _ExtValue, 99, _ClientData);
    //_snprintf(_SummaryString, _SummaryMaxLength, "0x%.8X", col);
    //(void) _SummaryMaxLength, _ExtValue, _ClientData;
    _SummaryString[0] = ' ';    // required to force background color for this value
    _SummaryString[1] = '\0';
}

void CColorExt::CreateTypes()
{
    if( g_TwMgr==NULL )
        return;
    TwStructMember ColorExtMembers[] = { { "Red", TW_TYPE_INT32, offsetof(CColorExt, R), "min=0 max=255" },
                                         { "Green", TW_TYPE_INT32, offsetof(CColorExt, G), "min=0 max=255" },
                                         { "Blue", TW_TYPE_INT32, offsetof(CColorExt, B), "min=0 max=255" },
                                         { "Hue", TW_TYPE_INT32, offsetof(CColorExt, H), "hide min=0 max=359" },
                                         { "Lightness", TW_TYPE_INT32, offsetof(CColorExt, L), "hide min=0 max=255" },
                                         { "Saturation", TW_TYPE_INT32, offsetof(CColorExt, S), "hide min=0 max=255" },
                                         { "Alpha", TW_TYPE_INT32, offsetof(CColorExt, A), "hide min=0 max=255" },
                                         { "Mode", TW_TYPE_BOOLCPP, offsetof(CColorExt, m_HLS), "true='HLS' false='RGB' readwrite" } };
    g_TwMgr->m_TypeColor32 = TwDefineStructExt("COLOR32", ColorExtMembers, 8, sizeof(unsigned int), sizeof(CColorExt), CColorExt::InitColor32CB, CColorExt::CopyVarFromExtCB, CColorExt::CopyVarToExtCB, CColorExt::SummaryCB, CTwMgr::CStruct::s_PassProxyAsClientData, "A 32-bit-encoded color.");
    g_TwMgr->m_TypeColor3F = TwDefineStructExt("COLOR3F", ColorExtMembers, 8, 3*sizeof(float), sizeof(CColorExt), CColorExt::InitColor3FCB, CColorExt::CopyVarFromExtCB, CColorExt::CopyVarToExtCB, CColorExt::SummaryCB, CTwMgr::CStruct::s_PassProxyAsClientData, "A 3-floats-encoded RGB color.");
    g_TwMgr->m_TypeColor4F = TwDefineStructExt("COLOR4F", ColorExtMembers, 8, 4*sizeof(float), sizeof(CColorExt), CColorExt::InitColor4FCB, CColorExt::CopyVarFromExtCB, CColorExt::CopyVarToExtCB, CColorExt::SummaryCB, CTwMgr::CStruct::s_PassProxyAsClientData, "A 4-floats-encoded RGBA color.");
    // Do not name them "TW_COLOR*" because the name is displayed in the help bar.
}

//  ---------------------------------------------------------------------------
//  Quaternion ext type
//  ---------------------------------------------------------------------------

void ANT_CALL CQuaternionExt::InitQuat4FCB(void *_ExtValue, void *_ClientData)
{
    CQuaternionExt *ext = static_cast<CQuaternionExt *>(_ExtValue);
    if( ext )
    {
        ext->Qx = ext->Qy = ext->Qz = 0;
        ext->Qs = 1;
        ext->Vx = 1;
        ext->Vy = ext->Vz = 0;
        ext->Angle = 0;
        ext->Dx = ext->Dy = ext->Dz = 0;
        ext->m_AAMode = false; // Axis & angle mode hidden
        ext->m_ShowVal = false;
        ext->m_IsFloat = true;
        ext->m_IsDir = false;
        ext->m_Dir[0] = ext->m_Dir[1] = ext->m_Dir[2] = 0;
        ext->m_DirColor = 0xffffff00;
        int i, j;
        for(i=0; i<3; ++i)
            for(j=0; j<3; ++j)
                ext->m_Permute[i][j] = (i==j) ? 1.0f : 0.0f;
        ext->m_StructProxy = (CTwMgr::CStructProxy *)_ClientData;
        ext->ConvertToAxisAngle();
        ext->m_Highlighted = false;
        ext->m_Rotating = false;
        if( ext->m_StructProxy!=NULL )
        {
            ext->m_StructProxy->m_CustomDrawCallback = CQuaternionExt::DrawCB;
            ext->m_StructProxy->m_CustomMouseButtonCallback = CQuaternionExt::MouseButtonCB;
            ext->m_StructProxy->m_CustomMouseMotionCallback = CQuaternionExt::MouseMotionCB;
            ext->m_StructProxy->m_CustomMouseLeaveCallback = CQuaternionExt::MouseLeaveCB;
        }
    }
}

void ANT_CALL CQuaternionExt::InitQuat4DCB(void *_ExtValue, void *_ClientData)
{
    CQuaternionExt *ext = static_cast<CQuaternionExt *>(_ExtValue);
    if( ext )
    {
        ext->Qx = ext->Qy = ext->Qz = 0;
        ext->Qs = 1;
        ext->Vx = 1;
        ext->Vy = ext->Vz = 0;
        ext->Angle = 0;
        ext->Dx = ext->Dy = ext->Dz = 0;
        ext->m_AAMode = false; // Axis & angle mode hidden
        ext->m_ShowVal = false;
        ext->m_IsFloat = false;
        ext->m_IsDir = false;
        ext->m_Dir[0] = ext->m_Dir[1] = ext->m_Dir[2] = 0;
        ext->m_DirColor = 0xffffff00;
        int i, j;
        for(i=0; i<3; ++i)
            for(j=0; j<3; ++j)
                ext->m_Permute[i][j] = (i==j) ? 1.0f : 0.0f;
        ext->m_StructProxy = (CTwMgr::CStructProxy *)_ClientData;
        ext->ConvertToAxisAngle();
        ext->m_Highlighted = false;
        ext->m_Rotating = false;
        if( ext->m_StructProxy!=NULL )
        {
            ext->m_StructProxy->m_CustomDrawCallback = CQuaternionExt::DrawCB;
            ext->m_StructProxy->m_CustomMouseButtonCallback = CQuaternionExt::MouseButtonCB;
            ext->m_StructProxy->m_CustomMouseMotionCallback = CQuaternionExt::MouseMotionCB;
            ext->m_StructProxy->m_CustomMouseLeaveCallback = CQuaternionExt::MouseLeaveCB;
        }
    }
}

void ANT_CALL CQuaternionExt::InitDir3FCB(void *_ExtValue, void *_ClientData)
{
    CQuaternionExt *ext = static_cast<CQuaternionExt *>(_ExtValue);
    if( ext )
    {
        ext->Qx = ext->Qy = ext->Qz = 0;
        ext->Qs = 1;
        ext->Vx = 1;
        ext->Vy = ext->Vz = 0;
        ext->Angle = 0;
        ext->Dx = 1;
        ext->Dy = ext->Dz = 0;
        ext->m_AAMode = false; // Axis & angle mode hidden
        ext->m_ShowVal = true;
        ext->m_IsFloat = true;
        ext->m_IsDir = true;
        ext->m_Dir[0] = ext->m_Dir[1] = ext->m_Dir[2] = 0;
        ext->m_DirColor = 0xffffff00;
        int i, j;
        for(i=0; i<3; ++i)
            for(j=0; j<3; ++j)
                ext->m_Permute[i][j] = (i==j) ? 1.0f : 0.0f;
        ext->m_StructProxy = (CTwMgr::CStructProxy *)_ClientData;
        ext->ConvertToAxisAngle();
        ext->m_Highlighted = false;
        ext->m_Rotating = false;
        if( ext->m_StructProxy!=NULL )
        {
            ext->m_StructProxy->m_CustomDrawCallback = CQuaternionExt::DrawCB;
            ext->m_StructProxy->m_CustomMouseButtonCallback = CQuaternionExt::MouseButtonCB;
            ext->m_StructProxy->m_CustomMouseMotionCallback = CQuaternionExt::MouseMotionCB;
            ext->m_StructProxy->m_CustomMouseLeaveCallback = CQuaternionExt::MouseLeaveCB;
        }
    }
}

void ANT_CALL CQuaternionExt::InitDir3DCB(void *_ExtValue, void *_ClientData)
{
    CQuaternionExt *ext = static_cast<CQuaternionExt *>(_ExtValue);
    if( ext )
    {
        ext->Qx = ext->Qy = ext->Qz = 0;
        ext->Qs = 1;
        ext->Vx = 1;
        ext->Vy = ext->Vz = 0;
        ext->Angle = 0;
        ext->Dx = 1;
        ext->Dy = ext->Dz = 0;
        ext->m_AAMode = false; // Axis & angle mode hidden
        ext->m_ShowVal = true;
        ext->m_IsFloat = false;
        ext->m_IsDir = true;
        ext->m_Dir[0] = ext->m_Dir[1] = ext->m_Dir[2] = 0;
        ext->m_DirColor = 0xffffff00;
        int i, j;
        for(i=0; i<3; ++i)
            for(j=0; j<3; ++j)
                ext->m_Permute[i][j] = (i==j) ? 1.0f : 0.0f;
        ext->m_StructProxy = (CTwMgr::CStructProxy *)_ClientData;
        ext->ConvertToAxisAngle();
        ext->m_Highlighted = false;
        ext->m_Rotating = false;
        if( ext->m_StructProxy!=NULL )
        {
            ext->m_StructProxy->m_CustomDrawCallback = CQuaternionExt::DrawCB;
            ext->m_StructProxy->m_CustomMouseButtonCallback = CQuaternionExt::MouseButtonCB;
            ext->m_StructProxy->m_CustomMouseMotionCallback = CQuaternionExt::MouseMotionCB;
            ext->m_StructProxy->m_CustomMouseLeaveCallback = CQuaternionExt::MouseLeaveCB;
        }
    }
}

void ANT_CALL CQuaternionExt::CopyVarFromExtCB(void *_VarValue, const void *_ExtValue, unsigned int _ExtMemberIndex, void *_ClientData)
{
    CQuaternionExt *ext = (CQuaternionExt *)(_ExtValue);
    CTwMgr::CMemberProxy *mProxy = static_cast<CTwMgr::CMemberProxy *>(_ClientData);
    if( _VarValue && ext )
    {
        // Synchronize Quat and AxisAngle
        if( _ExtMemberIndex>=4 && _ExtMemberIndex<=7 )
        {
            ext->ConvertToAxisAngle();
            // show/hide quat values
            if( _ExtMemberIndex==4 && mProxy && mProxy->m_VarParent )
            {
                assert( mProxy->m_VarParent->m_Vars.size()==16 );
                bool visible = ext->m_ShowVal;
                if( ext->m_IsDir )
                {
                    if(    mProxy->m_VarParent->m_Vars[13]->m_Visible != visible
                        || mProxy->m_VarParent->m_Vars[14]->m_Visible != visible
                        || mProxy->m_VarParent->m_Vars[15]->m_Visible != visible )
                    {
                        mProxy->m_VarParent->m_Vars[13]->m_Visible = visible;
                        mProxy->m_VarParent->m_Vars[14]->m_Visible = visible;
                        mProxy->m_VarParent->m_Vars[15]->m_Visible = visible;
                        mProxy->m_Bar->NotUpToDate();
                    }
                }
                else
                {
                    if(    mProxy->m_VarParent->m_Vars[4]->m_Visible != visible
                        || mProxy->m_VarParent->m_Vars[5]->m_Visible != visible
                        || mProxy->m_VarParent->m_Vars[6]->m_Visible != visible
                        || mProxy->m_VarParent->m_Vars[7]->m_Visible != visible )
                    {
                        mProxy->m_VarParent->m_Vars[4]->m_Visible = visible;
                        mProxy->m_VarParent->m_Vars[5]->m_Visible = visible;
                        mProxy->m_VarParent->m_Vars[6]->m_Visible = visible;
                        mProxy->m_VarParent->m_Vars[7]->m_Visible = visible;
                        mProxy->m_Bar->NotUpToDate();
                    }
                }
            }
        }
        else if( _ExtMemberIndex>=8 && _ExtMemberIndex<=11 )
            ext->ConvertFromAxisAngle();
        else if( mProxy && _ExtMemberIndex==12 && mProxy->m_VarParent && !ext->m_IsDir )
        {
            assert( mProxy->m_VarParent->m_Vars.size()==16 );
            bool aa = ext->m_AAMode;
            if(    mProxy->m_VarParent->m_Vars[4]->m_Visible != !aa
                || mProxy->m_VarParent->m_Vars[5]->m_Visible != !aa
                || mProxy->m_VarParent->m_Vars[6]->m_Visible != !aa
                || mProxy->m_VarParent->m_Vars[7]->m_Visible != !aa
                || mProxy->m_VarParent->m_Vars[8 ]->m_Visible != aa
                || mProxy->m_VarParent->m_Vars[9 ]->m_Visible != aa
                || mProxy->m_VarParent->m_Vars[10]->m_Visible != aa
                || mProxy->m_VarParent->m_Vars[11]->m_Visible != aa )
            {
                mProxy->m_VarParent->m_Vars[4]->m_Visible = !aa;
                mProxy->m_VarParent->m_Vars[5]->m_Visible = !aa;
                mProxy->m_VarParent->m_Vars[6]->m_Visible = !aa;
                mProxy->m_VarParent->m_Vars[7]->m_Visible = !aa;
                mProxy->m_VarParent->m_Vars[8 ]->m_Visible = aa;
                mProxy->m_VarParent->m_Vars[9 ]->m_Visible = aa;
                mProxy->m_VarParent->m_Vars[10]->m_Visible = aa;
                mProxy->m_VarParent->m_Vars[11]->m_Visible = aa;
                mProxy->m_Bar->NotUpToDate();
            }
            if( static_cast<CTwVarAtom *>(mProxy->m_VarParent->m_Vars[12])->m_ReadOnly )
            {
                static_cast<CTwVarAtom *>(mProxy->m_VarParent->m_Vars[12])->m_ReadOnly = false;
                mProxy->m_Bar->NotUpToDate();
            }
        }

        if( ext->m_IsFloat )
        {
            float *var = static_cast<float *>(_VarValue);
            if( ext->m_IsDir )
            {
                var[0] = (float)ext->Dx;
                var[1] = (float)ext->Dy;
                var[2] = (float)ext->Dz;
            }
            else // quat
            {
                var[0] = (float)ext->Qx;
                var[1] = (float)ext->Qy;
                var[2] = (float)ext->Qz;
                var[3] = (float)ext->Qs;
            }
        }
        else
        {
            double *var = static_cast<double *>(_VarValue);
            if( ext->m_IsDir )
            {
                var[0] = ext->Dx;
                var[1] = ext->Dy;
                var[2] = ext->Dz;
            }
            else // quat
            {
                var[0] = ext->Qx;
                var[1] = ext->Qy;
                var[2] = ext->Qz;
                var[3] = ext->Qs;
            }
        }
    }
}

void ANT_CALL CQuaternionExt::CopyVarToExtCB(const void *_VarValue, void *_ExtValue, unsigned int _ExtMemberIndex, void *_ClientData)
{
    CQuaternionExt *ext = static_cast<CQuaternionExt *>(_ExtValue);
    CTwMgr::CMemberProxy *mProxy = static_cast<CTwMgr::CMemberProxy *>(_ClientData);
    (void)mProxy;
    if( _VarValue && ext )
    {
        if( mProxy && _ExtMemberIndex==12 && mProxy->m_VarParent && !ext->m_IsDir )
        {
            assert( mProxy->m_VarParent->m_Vars.size()==16 );
            bool aa = ext->m_AAMode;
            if(    mProxy->m_VarParent->m_Vars[4]->m_Visible != !aa
                || mProxy->m_VarParent->m_Vars[5]->m_Visible != !aa
                || mProxy->m_VarParent->m_Vars[6]->m_Visible != !aa
                || mProxy->m_VarParent->m_Vars[7]->m_Visible != !aa
                || mProxy->m_VarParent->m_Vars[8 ]->m_Visible != aa
                || mProxy->m_VarParent->m_Vars[9 ]->m_Visible != aa
                || mProxy->m_VarParent->m_Vars[10]->m_Visible != aa
                || mProxy->m_VarParent->m_Vars[11]->m_Visible != aa )
            {
                mProxy->m_VarParent->m_Vars[4]->m_Visible = !aa;
                mProxy->m_VarParent->m_Vars[5]->m_Visible = !aa;
                mProxy->m_VarParent->m_Vars[6]->m_Visible = !aa;
                mProxy->m_VarParent->m_Vars[7]->m_Visible = !aa;
                mProxy->m_VarParent->m_Vars[8 ]->m_Visible = aa;
                mProxy->m_VarParent->m_Vars[9 ]->m_Visible = aa;
                mProxy->m_VarParent->m_Vars[10]->m_Visible = aa;
                mProxy->m_VarParent->m_Vars[11]->m_Visible = aa;
                mProxy->m_Bar->NotUpToDate();
            }
            if( static_cast<CTwVarAtom *>(mProxy->m_VarParent->m_Vars[12])->m_ReadOnly )
            {
                static_cast<CTwVarAtom *>(mProxy->m_VarParent->m_Vars[12])->m_ReadOnly = false;
                mProxy->m_Bar->NotUpToDate();
            }
        }
        else if( mProxy && _ExtMemberIndex==4 && mProxy->m_VarParent )
        {
            assert( mProxy->m_VarParent->m_Vars.size()==16 );
            bool visible = ext->m_ShowVal;
            if( ext->m_IsDir )
            {
                if(    mProxy->m_VarParent->m_Vars[13]->m_Visible != visible
                    || mProxy->m_VarParent->m_Vars[14]->m_Visible != visible
                    || mProxy->m_VarParent->m_Vars[15]->m_Visible != visible )
                {
                    mProxy->m_VarParent->m_Vars[13]->m_Visible = visible;
                    mProxy->m_VarParent->m_Vars[14]->m_Visible = visible;
                    mProxy->m_VarParent->m_Vars[15]->m_Visible = visible;
                    mProxy->m_Bar->NotUpToDate();
                }
            }
            else
            {
                if(    mProxy->m_VarParent->m_Vars[4]->m_Visible != visible
                    || mProxy->m_VarParent->m_Vars[5]->m_Visible != visible
                    || mProxy->m_VarParent->m_Vars[6]->m_Visible != visible
                    || mProxy->m_VarParent->m_Vars[7]->m_Visible != visible )
                {
                    mProxy->m_VarParent->m_Vars[4]->m_Visible = visible;
                    mProxy->m_VarParent->m_Vars[5]->m_Visible = visible;
                    mProxy->m_VarParent->m_Vars[6]->m_Visible = visible;
                    mProxy->m_VarParent->m_Vars[7]->m_Visible = visible;
                    mProxy->m_Bar->NotUpToDate();
                }
            }
        }

        if( ext->m_IsFloat )
        {
            const float *var = static_cast<const float *>(_VarValue);
            if( ext->m_IsDir )
            {
                ext->Dx = var[0];
                ext->Dy = var[1];
                ext->Dz = var[2];
                QuatFromDir(&ext->Qx, &ext->Qy, &ext->Qz, &ext->Qs, var[0], var[1], var[2]);
            }
            else
            {
                ext->Qx = var[0];
                ext->Qy = var[1];
                ext->Qz = var[2];
                ext->Qs = var[3];
            }

        }
        else
        {
            const double *var = static_cast<const double *>(_VarValue);
            if( ext->m_IsDir )
            {
                ext->Dx = var[0];
                ext->Dy = var[1];
                ext->Dz = var[2];
                QuatFromDir(&ext->Qx, &ext->Qy, &ext->Qz, &ext->Qs, var[0], var[1], var[2]);
            }
            else
            {
                ext->Qx = var[0];
                ext->Qy = var[1];
                ext->Qz = var[2];
                ext->Qs = var[3];
            }
        }
        ext->ConvertToAxisAngle();
    }
}

void ANT_CALL CQuaternionExt::SummaryCB(char *_SummaryString, size_t _SummaryMaxLength, const void *_ExtValue, void * /*_ClientData*/)
{
    const CQuaternionExt *ext = static_cast<const CQuaternionExt *>(_ExtValue);
    if( ext )
    {
        if( ext->m_AAMode )
            _snprintf(_SummaryString, _SummaryMaxLength, "V={%.2f,%.2f,%.2f} A=%.0f°", ext->Vx, ext->Vy, ext->Vz, ext->Angle);
        else if( ext->m_IsDir )
        {
            //float d[] = {1, 0, 0};
            //ApplyQuat(d+0, d+1, d+2, 1, 0, 0, (float)ext->Qx, (float)ext->Qy, (float)ext->Qz, (float)ext->Qs);
            _snprintf(_SummaryString, _SummaryMaxLength, "V={%.2f,%.2f,%.2f}", ext->Dx, ext->Dy, ext->Dz);
        }
        else
            _snprintf(_SummaryString, _SummaryMaxLength, "Q={x:%.2f,y:%.2f,z:%.2f,s:%.2f}", ext->Qx, ext->Qy, ext->Qz, ext->Qs);
    }
    else
    {
        _SummaryString[0] = ' ';    // required to force background color for this value
        _SummaryString[1] = '\0';
    }
}

TwType CQuaternionExt::s_CustomType = TW_TYPE_UNDEF;
vector<float>   CQuaternionExt::s_SphTri;
vector<color32> CQuaternionExt::s_SphCol;
vector<int>     CQuaternionExt::s_SphTriProj;
vector<color32> CQuaternionExt::s_SphColLight;
vector<float>   CQuaternionExt::s_ArrowTri[4];
vector<float>   CQuaternionExt::s_ArrowNorm[4];
vector<int>     CQuaternionExt::s_ArrowTriProj[4];
vector<color32> CQuaternionExt::s_ArrowColLight[4];

void CQuaternionExt::CreateTypes()
{
    if( g_TwMgr==NULL )
        return;
    s_CustomType = (TwType)(TW_TYPE_CUSTOM_BASE + (int)g_TwMgr->m_Customs.size());
    g_TwMgr->m_Customs.push_back(NULL); // increment custom type number

    for(int pass=0; pass<2; pass++) // pass 0: create quat types; pass 1: create dir types
    {
        const char *quatDefPass0 = "step=0.01 hide";
        const char *quatDefPass1 = "step=0.01 hide";
        const char *quatSDefPass0 = "step=0.01 min=-1 max=1 hide";
        const char *quatSDefPass1 = "step=0.01 min=-1 max=1 hide";
        const char *dirDefPass0 = "step=0.01 hide";
        const char *dirDefPass1 = "step=0.01";
        const char *quatDef = (pass==0) ? quatDefPass0 : quatDefPass1;
        const char *quatSDef = (pass==0) ? quatSDefPass0 : quatSDefPass1;
        const char *dirDef = (pass==0) ? dirDefPass0 : dirDefPass1;

        TwStructMember QuatExtMembers[] = { { "0", s_CustomType, 0, "" },
                                            { "1", s_CustomType, 0, "" },
                                            { "2", s_CustomType, 0, "" }, 
                                            { "3", s_CustomType, 0, "" }, 
                                            { "Quat X", TW_TYPE_DOUBLE, offsetof(CQuaternionExt, Qx), quatDef }, // copy of the source quaternion
                                            { "Quat Y", TW_TYPE_DOUBLE, offsetof(CQuaternionExt, Qy), quatDef },
                                            { "Quat Z", TW_TYPE_DOUBLE, offsetof(CQuaternionExt, Qz), quatDef },
                                            { "Quat S", TW_TYPE_DOUBLE, offsetof(CQuaternionExt, Qs), quatSDef },
                                            { "Axis X", TW_TYPE_DOUBLE, offsetof(CQuaternionExt, Vx), "step=0.01 hide" }, // axis and angle conversion -> Mode hidden because it is not equivalent to a quat (would have required vector renormalization)
                                            { "Axis Y", TW_TYPE_DOUBLE, offsetof(CQuaternionExt, Vy), "step=0.01 hide" },
                                            { "Axis Z", TW_TYPE_DOUBLE, offsetof(CQuaternionExt, Vz), "step=0.01 hide" },
                                            { "Angle (degree)",  TW_TYPE_DOUBLE, offsetof(CQuaternionExt, Angle), "step=1 min=-360 max=360 hide" },
                                            { "Mode", TW_TYPE_BOOLCPP, offsetof(CQuaternionExt, m_AAMode), "true='Axis Angle' false='Quaternion' readwrite hide" },
                                            { "Dir X", TW_TYPE_DOUBLE, offsetof(CQuaternionExt, Dx), dirDef },      // copy of the source direction
                                            { "Dir Y", TW_TYPE_DOUBLE, offsetof(CQuaternionExt, Dy), dirDef },
                                            { "Dir Z", TW_TYPE_DOUBLE, offsetof(CQuaternionExt, Dz), dirDef } };
        if( pass==0 ) 
        {
            g_TwMgr->m_TypeQuat4F = TwDefineStructExt("QUAT4F", QuatExtMembers, sizeof(QuatExtMembers)/sizeof(QuatExtMembers[0]), 4*sizeof(float), sizeof(CQuaternionExt), CQuaternionExt::InitQuat4FCB, CQuaternionExt::CopyVarFromExtCB, CQuaternionExt::CopyVarToExtCB, CQuaternionExt::SummaryCB, CTwMgr::CStruct::s_PassProxyAsClientData, "A 4-floats-encoded quaternion");
            g_TwMgr->m_TypeQuat4D = TwDefineStructExt("QUAT4D", QuatExtMembers, sizeof(QuatExtMembers)/sizeof(QuatExtMembers[0]), 4*sizeof(double), sizeof(CQuaternionExt), CQuaternionExt::InitQuat4DCB, CQuaternionExt::CopyVarFromExtCB, CQuaternionExt::CopyVarToExtCB, CQuaternionExt::SummaryCB, CTwMgr::CStruct::s_PassProxyAsClientData, "A 4-doubles-encoded quaternion");
        }
        else if( pass==1 )
        {
            g_TwMgr->m_TypeDir3F = TwDefineStructExt("DIR4F", QuatExtMembers, sizeof(QuatExtMembers)/sizeof(QuatExtMembers[0]), 3*sizeof(float), sizeof(CQuaternionExt), CQuaternionExt::InitDir3FCB, CQuaternionExt::CopyVarFromExtCB, CQuaternionExt::CopyVarToExtCB, CQuaternionExt::SummaryCB, CTwMgr::CStruct::s_PassProxyAsClientData, "A 3-floats-encoded direction");
            g_TwMgr->m_TypeDir3D = TwDefineStructExt("DIR4D", QuatExtMembers, sizeof(QuatExtMembers)/sizeof(QuatExtMembers[0]), 3*sizeof(double), sizeof(CQuaternionExt), CQuaternionExt::InitDir3DCB, CQuaternionExt::CopyVarFromExtCB, CQuaternionExt::CopyVarToExtCB, CQuaternionExt::SummaryCB, CTwMgr::CStruct::s_PassProxyAsClientData, "A 3-doubles-encoded direction");
        }
    }

    CreateSphere();
    CreateArrow();
}

void CQuaternionExt::ConvertToAxisAngle()
{
    if( fabs(Qs)>(1.0 + FLOAT_EPS) )
    {
        //Vx = Vy = Vz = 0; // no, keep the previous value
        Angle = 0;
    }
    else
    {
        double a;
        if( Qs>=1.0f )
            a = 0; // and keep V
        else if( Qs<=-1.0f )
            a = DOUBLE_PI; // and keep V
        else if( fabs(Qx*Qx+Qy*Qy+Qz*Qz+Qs*Qs)<FLOAT_EPS_SQ )
            a = 0;
        else
        {
            a = acos(Qs);
            if( a*Angle<0 ) // Preserve the sign of Angle
                a = -a;
            double f = 1.0f / sin(a);
            Vx = Qx * f;
            Vy = Qy * f;
            Vz = Qz * f;
        }
        Angle = 2.0*a;
    }

    //  if( Angle>FLOAT_PI )
    //      Angle -= 2.0f*FLOAT_PI;
    //  else if( Angle<-FLOAT_PI )
    //      Angle += 2.0f*FLOAT_PI;
    Angle = RadToDeg(Angle);

    if( fabs(Angle)<FLOAT_EPS && fabs(Vx*Vx+Vy*Vy+Vz*Vz)<FLOAT_EPS_SQ )
        Vx = 1.0e-7;    // all components cannot be null
}

void CQuaternionExt::ConvertFromAxisAngle()
{
    double n = Vx*Vx + Vy*Vy + Vz*Vz;
    if( fabs(n)>FLOAT_EPS_SQ )
    {
        double f = 0.5*DegToRad(Angle);
        Qs = cos(f);
        //do not normalize
        //if( fabs(n - 1.0)>FLOAT_EPS_SQ )
        //  f = sin(f) * (1.0/sqrt(n)) ;
        //else
        //  f = sin(f);
        f = sin(f);

        Qx = Vx * f;
        Qy = Vy * f;
        Qz = Vz * f;
    }
    else
    {
        Qs = 1.0;
        Qx = Qy = Qz = 0.0;
    }
}

void CQuaternionExt::CopyToVar()
{
    if( m_StructProxy!=NULL )
    {
        if( m_StructProxy->m_StructSetCallback!=NULL )
        {
            if( m_IsFloat )
            {
                if( m_IsDir )
                {
                    float d[] = {1, 0, 0};
                    ApplyQuat(d+0, d+1, d+2, 1, 0, 0, (float)Qx, (float)Qy, (float)Qz, (float)Qs);
                    float l = (float)sqrt(Dx*Dx + Dy*Dy + Dz*Dz);
                    d[0] *= l; d[1] *= l; d[2] *= l;
                    Dx = d[0]; Dy = d[1]; Dz = d[2]; // update also Dx,Dy,Dz
                    m_StructProxy->m_StructSetCallback(d, m_StructProxy->m_StructClientData);
                }
                else
                {
                    float q[] = { (float)Qx, (float)Qy, (float)Qz, (float)Qs };
                    m_StructProxy->m_StructSetCallback(q, m_StructProxy->m_StructClientData);
                }
            }
            else
            {
                if( m_IsDir )
                {
                    float d[] = {1, 0, 0};
                    ApplyQuat(d+0, d+1, d+2, 1, 0, 0, (float)Qx, (float)Qy, (float)Qz, (float)Qs);
                    double l = sqrt(Dx*Dx + Dy*Dy + Dz*Dz);
                    double dd[] = {l*d[0], l*d[1], l*d[2]};
                    Dx = dd[0]; Dy = dd[1]; Dz = dd[2]; // update also Dx,Dy,Dz
                    m_StructProxy->m_StructSetCallback(dd, m_StructProxy->m_StructClientData);
                }
                else
                {
                    double q[] = { Qx, Qy, Qz, Qs };
                    m_StructProxy->m_StructSetCallback(q, m_StructProxy->m_StructClientData);
                }
            }
        }
        else if( m_StructProxy->m_StructData!=NULL )
        {
            if( m_IsFloat )
            {
                if( m_IsDir )
                {
                    float *d = static_cast<float *>(m_StructProxy->m_StructData);
                    ApplyQuat(d+0, d+1, d+2, 1, 0, 0, (float)Qx, (float)Qy, (float)Qz, (float)Qs);
                    float l = (float)sqrt(Dx*Dx + Dy*Dy + Dz*Dz);
                    d[0] *= l; d[1] *= l; d[2] *= l;
                    Dx = d[0]; Dy = d[1]; Dz = d[2]; // update also Dx,Dy,Dz
                }
                else
                {
                    float *q = static_cast<float *>(m_StructProxy->m_StructData);
                    q[0] = (float)Qx; q[1] = (float)Qy; q[2] = (float)Qz; q[3] = (float)Qs;
                }
            }
            else 
            {
                if( m_IsDir )
                {
                    double *dd = static_cast<double *>(m_StructProxy->m_StructData);
                    float d[] = {1, 0, 0};
                    ApplyQuat(d+0, d+1, d+2, 1, 0, 0, (float)Qx, (float)Qy, (float)Qz, (float)Qs);
                    double l = sqrt(Dx*Dx + Dy*Dy + Dz*Dz);
                    dd[0] = l*d[0]; dd[1] = l*d[1]; dd[2] = l*d[2];
                    Dx = dd[0]; Dy = dd[1]; Dz = dd[2]; // update also Dx,Dy,Dz
                }
                else
                {
                    double *q = static_cast<double *>(m_StructProxy->m_StructData);
                    q[0] = Qx; q[1] = Qy; q[2] = Qz; q[3] = Qs;
                }
            }
        }
    }
}

void CQuaternionExt::CreateSphere()
{
    const int SUBDIV = 7;
    s_SphTri.clear();
    s_SphCol.clear();

    const float A[8*3] = { 1,0,0, 0,0,-1, -1,0,0, 0,0,1,   0,0,1,  1,0,0,  0,0,-1, -1,0,0 };
    const float B[8*3] = { 0,1,0, 0,1,0,  0,1,0,  0,1,0,   0,-1,0, 0,-1,0, 0,-1,0, 0,-1,0 };
    const float C[8*3] = { 0,0,1, 1,0,0,  0,0,-1, -1,0,0,  1,0,0,  0,0,-1, -1,0,0, 0,0,1  };
    //const color32 COL_A[8] = { 0xffff8080, 0xff000080, 0xff800000, 0xff8080ff,  0xff8080ff, 0xffff8080, 0xff000080, 0xff800000 };
    //const color32 COL_B[8] = { 0xff80ff80, 0xff80ff80, 0xff80ff80, 0xff80ff80,  0xff008000, 0xff008000, 0xff008000, 0xff008000 };
    //const color32 COL_C[8] = { 0xff8080ff, 0xffff8080, 0xff000080, 0xff800000,  0xffff8080, 0xff000080, 0xff800000, 0xff8080ff };
    const color32 COL_A[8] = { 0xffffffff, 0xffffff40, 0xff40ff40, 0xff40ffff,  0xffff40ff, 0xffff4040, 0xff404040, 0xff4040ff };
    const color32 COL_B[8] = { 0xffffffff, 0xffffff40, 0xff40ff40, 0xff40ffff,  0xffff40ff, 0xffff4040, 0xff404040, 0xff4040ff };
    const color32 COL_C[8] = { 0xffffffff, 0xffffff40, 0xff40ff40, 0xff40ffff,  0xffff40ff, 0xffff4040, 0xff404040, 0xff4040ff };

    int i, j, k, l;
    float xa, ya, za, xb, yb, zb, xc, yc, zc, x, y, z, norm, u[3], v[3];
    color32 col;
    for( i=0; i<8; ++i )
    {
        xa = A[3*i+0]; ya = A[3*i+1]; za = A[3*i+2];
        xb = B[3*i+0]; yb = B[3*i+1]; zb = B[3*i+2];
        xc = C[3*i+0]; yc = C[3*i+1]; zc = C[3*i+2];
        for( j=0; j<=SUBDIV; ++j )
            for( k=0; k<=2*(SUBDIV-j); ++k )
            {
                if( k%2==0 )
                {
                    u[0] = ((float)j)/(SUBDIV+1);
                    v[0] = ((float)(k/2))/(SUBDIV+1);
                    u[1] = ((float)(j+1))/(SUBDIV+1);
                    v[1] = ((float)(k/2))/(SUBDIV+1);
                    u[2] = ((float)j)/(SUBDIV+1);
                    v[2] = ((float)(k/2+1))/(SUBDIV+1);
                }
                else
                {
                    u[0] = ((float)j)/(SUBDIV+1);
                    v[0] = ((float)(k/2+1))/(SUBDIV+1);
                    u[1] = ((float)(j+1))/(SUBDIV+1);
                    v[1] = ((float)(k/2))/(SUBDIV+1);
                    u[2] = ((float)(j+1))/(SUBDIV+1);
                    v[2] = ((float)(k/2+1))/(SUBDIV+1);
                }

                for( l=0; l<3; ++l )
                {
                    x = (1.0f-u[l]-v[l])*xa + u[l]*xb + v[l]*xc;
                    y = (1.0f-u[l]-v[l])*ya + u[l]*yb + v[l]*yc;
                    z = (1.0f-u[l]-v[l])*za + u[l]*zb + v[l]*zc;
                    norm = sqrtf(x*x+y*y+z*z);
                    x /= norm; y /= norm; z /= norm;
                    s_SphTri.push_back(x); s_SphTri.push_back(y); s_SphTri.push_back(z);
                    if( u[l]+v[l]>FLOAT_EPS )
                        col = ColorBlend(COL_A[i], ColorBlend(COL_B[i], COL_C[i], v[l]/(u[l]+v[l])), u[l]+v[l]);
                    else
                        col = COL_A[i];
                    //if( (j==0 && k==0) || (j==0 && k==2*SUBDIV) || (j==SUBDIV && k==0) )
                    //  col = 0xffff0000;
                    s_SphCol.push_back(col);
                }
            }
    }
    s_SphTriProj.clear();
    s_SphTriProj.resize(2*s_SphCol.size(), 0);
    s_SphColLight.clear();
    s_SphColLight.resize(s_SphCol.size(), 0);
}

void CQuaternionExt::CreateArrow()
{
    const int   SUBDIV  = 15;
    const float CYL_RADIUS  = 0.08f;
    const float CONE_RADIUS = 0.16f;
    const float CONE_LENGTH = 0.25f;
    const float ARROW_BGN = -1.1f;
    const float ARROW_END = 1.15f;
    int i;
    for(i=0; i<4; ++i)
    {
        s_ArrowTri[i].clear();
        s_ArrowNorm[i].clear();
    }
    
    float x0, x1, y0, y1, z0, z1, a0, a1, nx, nn;
    for(i=0; i<SUBDIV; ++i)
    {
        a0 = 2.0f*FLOAT_PI*(float(i))/SUBDIV;
        a1 = 2.0f*FLOAT_PI*(float(i+1))/SUBDIV;
        x0 = ARROW_BGN;
        x1 = ARROW_END-CONE_LENGTH;
        y0 = cosf(a0);
        z0 = sinf(a0);
        y1 = cosf(a1);
        z1 = sinf(a1);
        s_ArrowTri[ARROW_CYL].push_back(x1); s_ArrowTri[ARROW_CYL].push_back(CYL_RADIUS*y0); s_ArrowTri[ARROW_CYL].push_back(CYL_RADIUS*z0);
        s_ArrowTri[ARROW_CYL].push_back(x0); s_ArrowTri[ARROW_CYL].push_back(CYL_RADIUS*y0); s_ArrowTri[ARROW_CYL].push_back(CYL_RADIUS*z0);
        s_ArrowTri[ARROW_CYL].push_back(x0); s_ArrowTri[ARROW_CYL].push_back(CYL_RADIUS*y1); s_ArrowTri[ARROW_CYL].push_back(CYL_RADIUS*z1);
        s_ArrowTri[ARROW_CYL].push_back(x1); s_ArrowTri[ARROW_CYL].push_back(CYL_RADIUS*y0); s_ArrowTri[ARROW_CYL].push_back(CYL_RADIUS*z0);
        s_ArrowTri[ARROW_CYL].push_back(x0); s_ArrowTri[ARROW_CYL].push_back(CYL_RADIUS*y1); s_ArrowTri[ARROW_CYL].push_back(CYL_RADIUS*z1);
        s_ArrowTri[ARROW_CYL].push_back(x1); s_ArrowTri[ARROW_CYL].push_back(CYL_RADIUS*y1); s_ArrowTri[ARROW_CYL].push_back(CYL_RADIUS*z1);
        s_ArrowNorm[ARROW_CYL].push_back(0); s_ArrowNorm[ARROW_CYL].push_back(y0); s_ArrowNorm[ARROW_CYL].push_back(z0);
        s_ArrowNorm[ARROW_CYL].push_back(0); s_ArrowNorm[ARROW_CYL].push_back(y0); s_ArrowNorm[ARROW_CYL].push_back(z0);
        s_ArrowNorm[ARROW_CYL].push_back(0); s_ArrowNorm[ARROW_CYL].push_back(y1); s_ArrowNorm[ARROW_CYL].push_back(z1);
        s_ArrowNorm[ARROW_CYL].push_back(0); s_ArrowNorm[ARROW_CYL].push_back(y0); s_ArrowNorm[ARROW_CYL].push_back(z0);
        s_ArrowNorm[ARROW_CYL].push_back(0); s_ArrowNorm[ARROW_CYL].push_back(y1); s_ArrowNorm[ARROW_CYL].push_back(z1);
        s_ArrowNorm[ARROW_CYL].push_back(0); s_ArrowNorm[ARROW_CYL].push_back(y1); s_ArrowNorm[ARROW_CYL].push_back(z1);
        s_ArrowTri[ARROW_CYL_CAP].push_back(x0); s_ArrowTri[ARROW_CYL_CAP].push_back(0); s_ArrowTri[ARROW_CYL_CAP].push_back(0);
        s_ArrowTri[ARROW_CYL_CAP].push_back(x0); s_ArrowTri[ARROW_CYL_CAP].push_back(CYL_RADIUS*y1); s_ArrowTri[ARROW_CYL_CAP].push_back(CYL_RADIUS*z1);
        s_ArrowTri[ARROW_CYL_CAP].push_back(x0); s_ArrowTri[ARROW_CYL_CAP].push_back(CYL_RADIUS*y0); s_ArrowTri[ARROW_CYL_CAP].push_back(CYL_RADIUS*z0);
        s_ArrowNorm[ARROW_CYL_CAP].push_back(-1); s_ArrowNorm[ARROW_CYL_CAP].push_back(0); s_ArrowNorm[ARROW_CYL_CAP].push_back(0);
        s_ArrowNorm[ARROW_CYL_CAP].push_back(-1); s_ArrowNorm[ARROW_CYL_CAP].push_back(0); s_ArrowNorm[ARROW_CYL_CAP].push_back(0);
        s_ArrowNorm[ARROW_CYL_CAP].push_back(-1); s_ArrowNorm[ARROW_CYL_CAP].push_back(0); s_ArrowNorm[ARROW_CYL_CAP].push_back(0);
        x0 = ARROW_END-CONE_LENGTH;
        x1 = ARROW_END;
        nx = CONE_RADIUS/(x1-x0);
        nn = 1.0f/sqrtf(nx*nx+1);
        s_ArrowTri[ARROW_CONE].push_back(x1); s_ArrowTri[ARROW_CONE].push_back(0); s_ArrowTri[ARROW_CONE].push_back(0);
        s_ArrowTri[ARROW_CONE].push_back(x0); s_ArrowTri[ARROW_CONE].push_back(CONE_RADIUS*y0); s_ArrowTri[ARROW_CONE].push_back(CONE_RADIUS*z0);
        s_ArrowTri[ARROW_CONE].push_back(x0); s_ArrowTri[ARROW_CONE].push_back(CONE_RADIUS*y1); s_ArrowTri[ARROW_CONE].push_back(CONE_RADIUS*z1);
        s_ArrowTri[ARROW_CONE].push_back(x1); s_ArrowTri[ARROW_CONE].push_back(0); s_ArrowTri[ARROW_CONE].push_back(0);
        s_ArrowTri[ARROW_CONE].push_back(x0); s_ArrowTri[ARROW_CONE].push_back(CONE_RADIUS*y1); s_ArrowTri[ARROW_CONE].push_back(CONE_RADIUS*z1);
        s_ArrowTri[ARROW_CONE].push_back(x1); s_ArrowTri[ARROW_CONE].push_back(0); s_ArrowTri[ARROW_CONE].push_back(0);
        s_ArrowNorm[ARROW_CONE].push_back(nn*nx); s_ArrowNorm[ARROW_CONE].push_back(nn*y0); s_ArrowNorm[ARROW_CONE].push_back(nn*z0);
        s_ArrowNorm[ARROW_CONE].push_back(nn*nx); s_ArrowNorm[ARROW_CONE].push_back(nn*y0); s_ArrowNorm[ARROW_CONE].push_back(nn*z0);
        s_ArrowNorm[ARROW_CONE].push_back(nn*nx); s_ArrowNorm[ARROW_CONE].push_back(nn*y1); s_ArrowNorm[ARROW_CONE].push_back(nn*z1);
        s_ArrowNorm[ARROW_CONE].push_back(nn*nx); s_ArrowNorm[ARROW_CONE].push_back(nn*y0); s_ArrowNorm[ARROW_CONE].push_back(nn*z0);
        s_ArrowNorm[ARROW_CONE].push_back(nn*nx); s_ArrowNorm[ARROW_CONE].push_back(nn*y1); s_ArrowNorm[ARROW_CONE].push_back(nn*z1);
        s_ArrowNorm[ARROW_CONE].push_back(nn*nx); s_ArrowNorm[ARROW_CONE].push_back(nn*y1); s_ArrowNorm[ARROW_CONE].push_back(nn*z1);
        s_ArrowTri[ARROW_CONE_CAP].push_back(x0); s_ArrowTri[ARROW_CONE_CAP].push_back(0); s_ArrowTri[ARROW_CONE_CAP].push_back(0);
        s_ArrowTri[ARROW_CONE_CAP].push_back(x0); s_ArrowTri[ARROW_CONE_CAP].push_back(CONE_RADIUS*y1); s_ArrowTri[ARROW_CONE_CAP].push_back(CONE_RADIUS*z1);
        s_ArrowTri[ARROW_CONE_CAP].push_back(x0); s_ArrowTri[ARROW_CONE_CAP].push_back(CONE_RADIUS*y0); s_ArrowTri[ARROW_CONE_CAP].push_back(CONE_RADIUS*z0);
        s_ArrowNorm[ARROW_CONE_CAP].push_back(-1); s_ArrowNorm[ARROW_CONE_CAP].push_back(0); s_ArrowNorm[ARROW_CONE_CAP].push_back(0);
        s_ArrowNorm[ARROW_CONE_CAP].push_back(-1); s_ArrowNorm[ARROW_CONE_CAP].push_back(0); s_ArrowNorm[ARROW_CONE_CAP].push_back(0);
        s_ArrowNorm[ARROW_CONE_CAP].push_back(-1); s_ArrowNorm[ARROW_CONE_CAP].push_back(0); s_ArrowNorm[ARROW_CONE_CAP].push_back(0);
    }

    for(i=0; i<4; ++i)
    {
        s_ArrowTriProj[i].clear();
        s_ArrowTriProj[i].resize(2*(s_ArrowTri[i].size()/3), 0);
        s_ArrowColLight[i].clear();
        s_ArrowColLight[i].resize(s_ArrowTri[i].size()/3, 0);
    }
}

static inline void QuatMult(double *out, const double *q1, const double *q2)
{
    out[0] = q1[3]*q2[0] + q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1];
    out[1] = q1[3]*q2[1] + q1[1]*q2[3] + q1[2]*q2[0] - q1[0]*q2[2];
    out[2] = q1[3]*q2[2] + q1[2]*q2[3] + q1[0]*q2[1] - q1[1]*q2[0];
    out[3] = q1[3]*q2[3] - (q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2]);
}

static inline void QuatFromAxisAngle(double *out, const double *axis, double angle)
{
    double n = axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2];
    if( fabs(n)>DOUBLE_EPS )
    {
        double f = 0.5*angle;
        out[3] = cos(f);
        f = sin(f)/sqrt(n);
        out[0] = axis[0]*f;
        out[1] = axis[1]*f;
        out[2] = axis[2]*f;
    }
    else
    {
        out[3] = 1.0;
        out[0] = out[1] = out[2] = 0.0;
    }
}

static inline void Vec3Cross(double *out, const double *a, const double *b)
{
    out[0] = a[1]*b[2]-a[2]*b[1];
    out[1] = a[2]*b[0]-a[0]*b[2];
    out[2] = a[0]*b[1]-a[1]*b[0];
}

static inline double Vec3Dot(const double *a, const double *b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static inline void Vec3RotY(float *x, float *y, float *z)
{
    (void)y;
    float tmp = *x;
    *x = - *z;
    *z = tmp;
}

static inline void Vec3RotZ(float *x, float *y, float *z)
{
    (void)z;
    float tmp = *x;
    *x = - *y;
    *y = tmp;
}

void CQuaternionExt::ApplyQuat(float *outX, float *outY, float *outZ, float x, float y, float z, float qx, float qy, float qz, float qs)
{
    float ps = - qx * x - qy * y - qz * z;
    float px =   qs * x + qy * z - qz * y;
    float py =   qs * y + qz * x - qx * z;
    float pz =   qs * z + qx * y - qy * x;
    *outX = - ps * qx + px * qs - py * qz + pz * qy;
    *outY = - ps * qy + py * qs - pz * qx + px * qz;
    *outZ = - ps * qz + pz * qs - px * qy + py * qx;
}

void CQuaternionExt::QuatFromDir(double *outQx, double *outQy, double *outQz, double *outQs, double dx, double dy, double dz)
{
    // compute a quaternion that rotates (1,0,0) to (dx,dy,dz)

    double dn = sqrt(dx*dx + dy*dy + dz*dz);
    if( dn<DOUBLE_EPS_SQ )
    {
        *outQx = *outQy = *outQz = 0;
        *outQs = 1;
    }
    else
    {
        double rotAxis[3] = { 0, -dz, dy };
        if( rotAxis[0]*rotAxis[0] + rotAxis[1]*rotAxis[1] + rotAxis[2]*rotAxis[2]<DOUBLE_EPS_SQ )
        {
            rotAxis[0] = rotAxis[1] = 0;
            rotAxis[2] = 1;
        }
        double rotAngle = acos(dx/dn);
        double rotQuat[4];
        QuatFromAxisAngle(rotQuat, rotAxis, rotAngle);
        *outQx = rotQuat[0];
        *outQy = rotQuat[1];
        *outQz = rotQuat[2];
        *outQs = rotQuat[3];
    }
}

void CQuaternionExt::Permute(float *outX, float *outY, float *outZ, float x, float y, float z)
{
    float px = x, py = y, pz = z;
    *outX = m_Permute[0][0]*px + m_Permute[1][0]*py + m_Permute[2][0]*pz;
    *outY = m_Permute[0][1]*px + m_Permute[1][1]*py + m_Permute[2][1]*pz;
    *outZ = m_Permute[0][2]*px + m_Permute[1][2]*py + m_Permute[2][2]*pz;
}

void CQuaternionExt::PermuteInv(float *outX, float *outY, float *outZ, float x, float y, float z)
{
    float px = x, py = y, pz = z;
    *outX = m_Permute[0][0]*px + m_Permute[0][1]*py + m_Permute[0][2]*pz;
    *outY = m_Permute[1][0]*px + m_Permute[1][1]*py + m_Permute[1][2]*pz;
    *outZ = m_Permute[2][0]*px + m_Permute[2][1]*py + m_Permute[2][2]*pz;
}

void CQuaternionExt::Permute(double *outX, double *outY, double *outZ, double x, double y, double z)
{
    double px = x, py = y, pz = z;
    *outX = m_Permute[0][0]*px + m_Permute[1][0]*py + m_Permute[2][0]*pz;
    *outY = m_Permute[0][1]*px + m_Permute[1][1]*py + m_Permute[2][1]*pz;
    *outZ = m_Permute[0][2]*px + m_Permute[1][2]*py + m_Permute[2][2]*pz;
}

void CQuaternionExt::PermuteInv(double *outX, double *outY, double *outZ, double x, double y, double z)
{
    double px = x, py = y, pz = z;
    *outX = m_Permute[0][0]*px + m_Permute[0][1]*py + m_Permute[0][2]*pz;
    *outY = m_Permute[1][0]*px + m_Permute[1][1]*py + m_Permute[1][2]*pz;
    *outZ = m_Permute[2][0]*px + m_Permute[2][1]*py + m_Permute[2][2]*pz;
}

static inline float QuatD(int w, int h)
{
    return (float)min(abs(w), abs(h)) - 4;
}

static inline int QuatPX(float x, int w, int h)
{
    return (int)(x*0.5f*QuatD(w, h) + (float)w*0.5f + 0.5f);
}

static inline int QuatPY(float y, int w, int h)
{
    return (int)(-y*0.5f*QuatD(w, h) + (float)h*0.5f - 0.5f);
}

static inline float QuatIX(int x, int w, int h)
{
    return (2.0f*(float)x - (float)w - 1.0f)/QuatD(w, h);
}

static inline float QuatIY(int y, int w, int h)
{
    return (-2.0f*(float)y + (float)h - 1.0f)/QuatD(w, h);
}

void CQuaternionExt::DrawCB(int w, int h, void *_ExtValue, void *_ClientData, TwBar *_Bar, CTwVarGroup *varGrp)
{
    if( g_TwMgr==NULL || g_TwMgr->m_Graph==NULL )
        return;
    assert( g_TwMgr->m_Graph->IsDrawing() );
    CQuaternionExt *ext = static_cast<CQuaternionExt *>(_ExtValue);
    assert( ext!=NULL );
    (void)_ClientData; (void)_Bar;

    // show/hide quat values
    assert( varGrp->m_Vars.size()==16 );
    bool visible = ext->m_ShowVal;
    if( ext->m_IsDir )
    {
        if(    varGrp->m_Vars[13]->m_Visible != visible
            || varGrp->m_Vars[14]->m_Visible != visible
            || varGrp->m_Vars[15]->m_Visible != visible )
        {
            varGrp->m_Vars[13]->m_Visible = visible;
            varGrp->m_Vars[14]->m_Visible = visible;
            varGrp->m_Vars[15]->m_Visible = visible;
            _Bar->NotUpToDate();
        }
    }
    else
    {
        if(    varGrp->m_Vars[4]->m_Visible != visible
            || varGrp->m_Vars[5]->m_Visible != visible
            || varGrp->m_Vars[6]->m_Visible != visible
            || varGrp->m_Vars[7]->m_Visible != visible )
        {
            varGrp->m_Vars[4]->m_Visible = visible;
            varGrp->m_Vars[5]->m_Visible = visible;
            varGrp->m_Vars[6]->m_Visible = visible;
            varGrp->m_Vars[7]->m_Visible = visible;
            _Bar->NotUpToDate();
        }
    }

    // force ext update
    static_cast<CTwVarAtom *>(varGrp->m_Vars[4])->ValueToDouble();

    assert( s_SphTri.size()>0 );
    assert( s_SphTri.size()==3*s_SphCol.size() );
    assert( s_SphTriProj.size()==2*s_SphCol.size() );
    assert( s_SphColLight.size()==s_SphCol.size() );

    if( QuatD(w, h)<=2 )
        return;
    float x, y, z, nx, ny, nz, kx, ky, kz, qx, qy, qz, qs;
    int i, j, k, l, m;

    // normalize quaternion
    float qn = (float)sqrt(ext->Qs*ext->Qs+ext->Qx*ext->Qx+ext->Qy*ext->Qy+ext->Qz*ext->Qz);
    if( qn>FLOAT_EPS )
    {
        qx = (float)ext->Qx/qn;
        qy = (float)ext->Qy/qn;
        qz = (float)ext->Qz/qn;
        qs = (float)ext->Qs/qn;
    }
    else
    {
        qx = qy = qz = 0;
        qs = 1;
    }

    double normDir = sqrt(ext->m_Dir[0]*ext->m_Dir[0] + ext->m_Dir[1]*ext->m_Dir[1] + ext->m_Dir[2]*ext->m_Dir[2]);
    bool drawDir = ext->m_IsDir || (normDir>DOUBLE_EPS);
    color32 alpha = ext->m_Highlighted ? 0xffffffff : 0xb0ffffff;
    
    // check if frame is right-handed
    ext->Permute(&kx, &ky, &kz, 1, 0, 0);
    double px[3] = { (double)kx, (double)ky, (double)kz };
    ext->Permute(&kx, &ky, &kz, 0, 1, 0);
    double py[3] = { (double)kx, (double)ky, (double)kz };
    ext->Permute(&kx, &ky, &kz, 0, 0, 1);
    double pz[3] = { (double)kx, (double)ky, (double)kz };
    double ez[3];
    Vec3Cross(ez, px, py);
    bool frameRightHanded = (ez[0]*pz[0]+ez[1]*pz[1]+ez[2]*pz[2] >= 0);
    ITwGraph::Cull cull = frameRightHanded ? ITwGraph::CULL_CW : ITwGraph::CULL_CCW;

    if( drawDir )
    {
        float dir[] = {(float)ext->m_Dir[0], (float)ext->m_Dir[1], (float)ext->m_Dir[2]};
        if( normDir<DOUBLE_EPS )
        {
            normDir = 1;
            dir[0] = 1;
        }
        kx = dir[0]; ky = dir[1]; kz = dir[2];
        double rotDirAxis[3] = { 0, -kz, ky };
        if( rotDirAxis[0]*rotDirAxis[0] + rotDirAxis[1]*rotDirAxis[1] + rotDirAxis[2]*rotDirAxis[2]<DOUBLE_EPS_SQ )
        {
            rotDirAxis[0] = rotDirAxis[1] = 0;
            rotDirAxis[2] = 1;
        }
        double rotDirAngle = acos(kx/normDir);
        double rotDirQuat[4];
        QuatFromAxisAngle(rotDirQuat, rotDirAxis, rotDirAngle);

        kx = 1; ky = 0; kz = 0;
        ApplyQuat(&kx, &ky, &kz, kx, ky, kz, (float)rotDirQuat[0], (float)rotDirQuat[1], (float)rotDirQuat[2], (float)rotDirQuat[3]);
        ApplyQuat(&kx, &ky, &kz, kx, ky, kz, qx, qy, qz, qs);
        for(k=0; k<4; ++k) // 4 parts of the arrow
        {
            // draw order
            ext->Permute(&x, &y, &z, kx, ky, kz);
            j = (z>0) ? 3-k : k;

            assert( s_ArrowTriProj[j].size()==2*(s_ArrowTri[j].size()/3) && s_ArrowColLight[j].size()==s_ArrowTri[j].size()/3 && s_ArrowNorm[j].size()==s_ArrowTri[j].size() ); 
            const int ntri = (int)s_ArrowTri[j].size()/3;
            const float *tri = &(s_ArrowTri[j][0]);
            const float *norm = &(s_ArrowNorm[j][0]);
            int *triProj = &(s_ArrowTriProj[j][0]);
            color32 *colLight = &(s_ArrowColLight[j][0]);
            for(i=0; i<ntri; ++i)
            {
                x = tri[3*i+0]; y = tri[3*i+1]; z = tri[3*i+2];
                nx = norm[3*i+0]; ny = norm[3*i+1]; nz = norm[3*i+2];
                if( x>0 )
                    x = 2.5f*x - 2.0f;
                else
                    x += 0.2f;
                y *= 1.5f;
                z *= 1.5f;
                ApplyQuat(&x, &y, &z, x, y, z, (float)rotDirQuat[0], (float)rotDirQuat[1], (float)rotDirQuat[2], (float)rotDirQuat[3]);
                ApplyQuat(&x, &y, &z, x, y, z, qx, qy, qz, qs);
                ext->Permute(&x, &y, &z, x, y, z);
                ApplyQuat(&nx, &ny, &nz, nx, ny, nz, (float)rotDirQuat[0], (float)rotDirQuat[1], (float)rotDirQuat[2], (float)rotDirQuat[3]);
                ApplyQuat(&nx, &ny, &nz, nx, ny, nz, qx, qy, qz, qs);
                ext->Permute(&nx, &ny, &nz, nx, ny, nz);
                triProj[2*i+0] = QuatPX(x, w, h);
                triProj[2*i+1] = QuatPY(y, w, h);
                color32 col = (ext->m_DirColor|0xff000000) & alpha;
                colLight[i] = ColorBlend(0xff000000, col, fabsf(TClamp(nz, -1.0f, 1.0f)));
            }
            if( s_ArrowTri[j].size()>=9 ) // 1 tri = 9 floats
                g_TwMgr->m_Graph->DrawTriangles((int)s_ArrowTri[j].size()/9, triProj, colLight, cull);
        }
    }
    else
    {
        /*
        int px0 = QuatPX(0, w, h)-1, py0 = QuatPY(0, w, h), r0 = (int)(0.5f*QuatD(w, h)-0.5f);
        color32 col0 = 0x80000000;
        DrawArc(px0-1, py0, r0, 0, 360, col0);
        DrawArc(px0+1, py0, r0, 0, 360, col0);
        DrawArc(px0, py0-1, r0, 0, 360, col0);
        DrawArc(px0, py0+1, r0, 0, 360, col0);
        */
        // draw arrows & sphere
        const float SPH_RADIUS = 0.75f;
        for(m=0; m<2; ++m)  // m=0: back, m=1: front
        {
            for(l=0; l<3; ++l)  // draw 3 arrows
            {
                kx = 1; ky = 0; kz = 0;
                if( l==1 )
                    Vec3RotZ(&kx, &ky, &kz); 
                else if( l==2 )
                    Vec3RotY(&kx, &ky, &kz);
                ApplyQuat(&kx, &ky, &kz, kx, ky, kz, qx, qy, qz, qs);
                for(k=0; k<4; ++k) // 4 parts of the arrow
                {
                    // draw order
                    ext->Permute(&x, &y, &z, kx, ky, kz);
                    j = (z>0) ? 3-k : k;

                    bool cone = true;
                    if( (m==0 && z>0) || (m==1 && z<=0) )
                    {
                        if( j==ARROW_CONE || j==ARROW_CONE_CAP ) // do not draw cone
                            continue;
                        else
                            cone = false;
                    }
                    assert( s_ArrowTriProj[j].size()==2*(s_ArrowTri[j].size()/3) && s_ArrowColLight[j].size()==s_ArrowTri[j].size()/3 && s_ArrowNorm[j].size()==s_ArrowTri[j].size() ); 
                    const int ntri = (int)s_ArrowTri[j].size()/3;
                    const float *tri = &(s_ArrowTri[j][0]);
                    const float *norm = &(s_ArrowNorm[j][0]);
                    int *triProj = &(s_ArrowTriProj[j][0]);
                    color32 *colLight = &(s_ArrowColLight[j][0]);
                    for(i=0; i<ntri; ++i)
                    {
                        x = tri[3*i+0]; y = tri[3*i+1]; z = tri[3*i+2];
                        if( cone && x<=0 )
                            x = SPH_RADIUS;
                        else if( !cone && x>0 )
                            x = -SPH_RADIUS;
                        nx = norm[3*i+0]; ny = norm[3*i+1]; nz = norm[3*i+2];
                        if( l==1 )
                        {
                            Vec3RotZ(&x, &y, &z); 
                            Vec3RotZ(&nx, &ny, &nz); 
                        }
                        else if( l==2 )
                        {
                            Vec3RotY(&x, &y, &z);
                            Vec3RotY(&nx, &ny, &nz);
                        }
                        ApplyQuat(&x, &y, &z, x, y, z, qx, qy, qz, qs);
                        ext->Permute(&x, &y, &z, x, y, z);
                        ApplyQuat(&nx, &ny, &nz, nx, ny, nz, qx, qy, qz, qs);
                        ext->Permute(&nx, &ny, &nz, nx, ny, nz);
                        triProj[2*i+0] = QuatPX(x, w, h);
                        triProj[2*i+1] = QuatPY(y, w, h);
                        float fade = ( m==0 && z<0 ) ? TClamp(2.0f*z*z, 0.0f, 1.0f) : 0;
                        float alphaFade = 1.0f;
                        Color32ToARGBf(alpha, &alphaFade, NULL, NULL, NULL);
                        alphaFade *= (1.0f-fade);
                        color32 alphaFadeCol = Color32FromARGBf(alphaFade, 1, 1, 1);
                        color32 col = (l==0) ? 0xffff0000 : ( (l==1) ? 0xff00ff00 : 0xff0000ff );
                        colLight[i] = ColorBlend(0xff000000, col, fabsf(TClamp(nz, -1.0f, 1.0f))) & alphaFadeCol;
                    }
                    if( s_ArrowTri[j].size()>=9 ) // 1 tri = 9 floats
                        g_TwMgr->m_Graph->DrawTriangles((int)s_ArrowTri[j].size()/9, triProj, colLight, cull);
                }
            }

            if( m==0 )
            {
                const float *tri = &(s_SphTri[0]);
                int *triProj = &(s_SphTriProj[0]);
                const color32 *col = &(s_SphCol[0]);
                color32 *colLight = &(s_SphColLight[0]);
                const int ntri = (int)s_SphTri.size()/3;
                for(i=0; i<ntri; ++i)   // draw sphere
                {
                    x = SPH_RADIUS*tri[3*i+0]; y = SPH_RADIUS*tri[3*i+1]; z = SPH_RADIUS*tri[3*i+2];
                    ApplyQuat(&x, &y, &z, x, y, z, qx, qy, qz, qs);
                    ext->Permute(&x, &y, &z, x, y, z);
                    triProj[2*i+0] = QuatPX(x, w, h);
                    triProj[2*i+1] = QuatPY(y, w, h);
                    colLight[i] = ColorBlend(0xff000000, col[i], fabsf(TClamp(z/SPH_RADIUS, -1.0f, 1.0f))) & alpha;
                }
                g_TwMgr->m_Graph->DrawTriangles((int)s_SphTri.size()/9, triProj, colLight, cull);
            }
        }

        // draw x
        g_TwMgr->m_Graph->DrawLine(w-12, h-36, w-12+5, h-36+5, 0xffc00000, true);
        g_TwMgr->m_Graph->DrawLine(w-12+5, h-36, w-12, h-36+5, 0xffc00000, true);
        // draw y
        g_TwMgr->m_Graph->DrawLine(w-12, h-25, w-12+3, h-25+4, 0xff00c000, true);
        g_TwMgr->m_Graph->DrawLine(w-12+5, h-25, w-12, h-25+7, 0xff00c000, true);
        // draw z
        g_TwMgr->m_Graph->DrawLine(w-12, h-12, w-12+5, h-12, 0xff0000c0, true);
        g_TwMgr->m_Graph->DrawLine(w-12, h-12+5, w-12+5, h-12+5, 0xff0000c0, true);
        g_TwMgr->m_Graph->DrawLine(w-12, h-12+5, w-12+5, h-12, 0xff0000c0, true);
    }

    // draw borders
    g_TwMgr->m_Graph->DrawLine(1, 0, w-1, 0, 0x40000000);
    g_TwMgr->m_Graph->DrawLine(w-1, 0, w-1, h-1, 0x40000000);
    g_TwMgr->m_Graph->DrawLine(w-1, h-1, 1, h-1, 0x40000000);
    g_TwMgr->m_Graph->DrawLine(1, h-1, 1, 0, 0x40000000);
}

bool CQuaternionExt::MouseMotionCB(int mouseX, int mouseY, int w, int h, void *structExtValue, void *clientData, TwBar *bar, CTwVarGroup *varGrp)
{
    CQuaternionExt *ext = static_cast<CQuaternionExt *>(structExtValue);
    if( ext==NULL )
        return false;
    (void)clientData, (void)varGrp;

    if( mouseX>0 && mouseX<w && mouseY>0 && mouseY<h )
        ext->m_Highlighted = true;

    if( ext->m_Rotating )
    {
        double x = QuatIX(mouseX, w, h);
        double y = QuatIY(mouseY, w, h);
        double z = 1;
        double px, py, pz, ox, oy, oz;
        ext->PermuteInv(&px, &py, &pz, x, y, z);
        ext->PermuteInv(&ox, &oy, &oz, ext->m_OrigX, ext->m_OrigY, 1);
        double n0 = sqrt(ox*ox + oy*oy + oz*oz);
        double n1 = sqrt(px*px + py*py + pz*pz);
        if( n0>DOUBLE_EPS && n1>DOUBLE_EPS )
        {
            double v0[] = { ox/n0, oy/n0, oz/n0 };
            double v1[] = { px/n1, py/n1, pz/n1 };
            double axis[3];
            Vec3Cross(axis, v0, v1);
            double sa = sqrt(Vec3Dot(axis, axis));
            double ca = Vec3Dot(v0, v1);
            double angle = atan2(sa, ca);
            if( x*x+y*y>1.0 )
                angle *= 1.0 + 0.2f*(sqrt(x*x+y*y)-1.0);
            double qrot[4], qres[4], qorig[4];
            QuatFromAxisAngle(qrot, axis, angle);
            double nqorig = sqrt(ext->m_OrigQuat[0]*ext->m_OrigQuat[0]+ext->m_OrigQuat[1]*ext->m_OrigQuat[1]+ext->m_OrigQuat[2]*ext->m_OrigQuat[2]+ext->m_OrigQuat[3]*ext->m_OrigQuat[3]);
            if( fabs(nqorig)>DOUBLE_EPS_SQ )
            {
                qorig[0] = ext->m_OrigQuat[0]/nqorig;
                qorig[1] = ext->m_OrigQuat[1]/nqorig;
                qorig[2] = ext->m_OrigQuat[2]/nqorig;
                qorig[3] = ext->m_OrigQuat[3]/nqorig;
                QuatMult(qres, qrot, qorig);
                ext->Qx = qres[0];
                ext->Qy = qres[1];
                ext->Qz = qres[2];
                ext->Qs = qres[3];
            }
            else
            {
                ext->Qx = qrot[0];
                ext->Qy = qrot[1];
                ext->Qz = qrot[2];
                ext->Qs = qrot[3];
            }
            ext->CopyToVar();
            if( bar!=NULL )
                bar->NotUpToDate();

            ext->m_PrevX = x;
            ext->m_PrevY = y;
        }
    }

    return true;
}

bool CQuaternionExt::MouseButtonCB(TwMouseButtonID button, bool pressed, int mouseX, int mouseY, int w, int h, void *structExtValue, void *clientData, TwBar *bar, CTwVarGroup *varGrp)
{
    CQuaternionExt *ext = static_cast<CQuaternionExt *>(structExtValue);
    if( ext==NULL )
        return false;
    (void)clientData; (void)bar, (void)varGrp;

    if( button==TW_MOUSE_LEFT )
    {
        if( pressed )
        {
            ext->m_OrigQuat[0] = ext->Qx;
            ext->m_OrigQuat[1] = ext->Qy;
            ext->m_OrigQuat[2] = ext->Qz;
            ext->m_OrigQuat[3] = ext->Qs;
            ext->m_OrigX = QuatIX(mouseX, w, h);
            ext->m_OrigY = QuatIY(mouseY, w, h);
            ext->m_PrevX = ext->m_OrigX;
            ext->m_PrevY = ext->m_OrigY;
            ext->m_Rotating = true;
        }
        else
            ext->m_Rotating = false;
    }

    //printf("Click %x\n", structExtValue);
    return true;
}

void CQuaternionExt::MouseLeaveCB(void *structExtValue, void *clientData, TwBar *bar)
{
    CQuaternionExt *ext = static_cast<CQuaternionExt *>(structExtValue);
    if( ext==NULL )
        return;
    (void)clientData; (void)bar;

    //printf("Leave %x\n", structExtValue);
    ext->m_Highlighted = false;
    ext->m_Rotating = false;
}


//  ---------------------------------------------------------------------------
//  Convertion between VC++ Debug/Release std::string
//  (Needed because VC++ adds some extra info to std::string in Debug mode!)
//  And resolve binary std::string incompatibility between VS2008- and VS2010+
//  ---------------------------------------------------------------------------

#ifdef _MSC_VER
// VS2008 and lower store the string allocator pointer at the beginning
// VS2010 and higher store the string allocator pointer at the end
static void FixVS2010StdStringLibToClient(void *strPtr)
{
    char *ptr = (char *)strPtr;
    const size_t SizeOfUndecoratedString = 16 + 2*sizeof(size_t) + sizeof(void *); // size of a VS std::string without extra debug iterator and info.
    assert(SizeOfUndecoratedString <= sizeof(std::string));
    TwType LibStdStringBaseType = (TwType)(TW_TYPE_STDSTRING&0xffff0000);
    void **allocAddress2008 = (void **)(ptr + sizeof(std::string) - SizeOfUndecoratedString);
    void **allocAddress2010 = (void **)(ptr + sizeof(std::string) - sizeof(void *));
    if (LibStdStringBaseType == TW_TYPE_STDSTRING_VS2008 && g_TwMgr->m_ClientStdStringBaseType == TW_TYPE_STDSTRING_VS2010)
    {
        void *allocator = *allocAddress2008;
        memmove(allocAddress2008, allocAddress2008 + 1, SizeOfUndecoratedString - sizeof(void *));
        *allocAddress2010 = allocator;
    }
    else if (LibStdStringBaseType == TW_TYPE_STDSTRING_VS2010 && g_TwMgr->m_ClientStdStringBaseType == TW_TYPE_STDSTRING_VS2008)
    {
        void *allocator = *allocAddress2010;
        memmove(allocAddress2008 + 1, allocAddress2008, SizeOfUndecoratedString - sizeof(void *));
        *allocAddress2008 = allocator;
    }
}

static void FixVS2010StdStringClientToLib(void *strPtr)
{
    char *ptr = (char *)strPtr;
    const size_t SizeOfUndecoratedString = 16 + 2*sizeof(size_t) + sizeof(void *); // size of a VS std::string without extra debug iterator and info.
    assert(SizeOfUndecoratedString <= sizeof(std::string));
    TwType LibStdStringBaseType = (TwType)(TW_TYPE_STDSTRING&0xffff0000);
    void **allocAddress2008 = (void **)(ptr + sizeof(std::string) - SizeOfUndecoratedString);
    void **allocAddress2010 = (void **)(ptr + sizeof(std::string) - sizeof(void *));
    if (LibStdStringBaseType == TW_TYPE_STDSTRING_VS2008 && g_TwMgr->m_ClientStdStringBaseType == TW_TYPE_STDSTRING_VS2010)
    {
        void *allocator = *allocAddress2010;
        memmove(allocAddress2008 + 1, allocAddress2008, SizeOfUndecoratedString - sizeof(void *));
        *allocAddress2008 = allocator;
    }
    else if (LibStdStringBaseType == TW_TYPE_STDSTRING_VS2010 && g_TwMgr->m_ClientStdStringBaseType == TW_TYPE_STDSTRING_VS2008)
    {
        void *allocator = *allocAddress2008;
        memmove(allocAddress2008, allocAddress2008 + 1, SizeOfUndecoratedString - sizeof(void *));
        *allocAddress2010 = allocator;
    }
}
#endif // _MSC_VER

CTwMgr::CClientStdString::CClientStdString()
{
    memset(m_Data, 0, sizeof(m_Data));
}

void CTwMgr::CClientStdString::FromLib(const char *libStr)
{
    m_LibStr = libStr; // it is ok to have a local copy here
    memcpy(m_Data + sizeof(void *), &m_LibStr, sizeof(std::string));
#ifdef _MSC_VER
    FixVS2010StdStringLibToClient(m_Data + sizeof(void *));
#endif
}

std::string& CTwMgr::CClientStdString::ToClient() 
{
    assert( g_TwMgr!=NULL );
    if( g_TwMgr->m_ClientStdStringStructSize==sizeof(std::string)+sizeof(void *) )
        return *(std::string *)(m_Data);
    else if( g_TwMgr->m_ClientStdStringStructSize+sizeof(void *)==sizeof(std::string) )
        return *(std::string *)(m_Data + 2*sizeof(void *));
    else
    {
        assert( g_TwMgr->m_ClientStdStringStructSize==sizeof(std::string) );
        return *(std::string *)(m_Data + sizeof(void *));
    }
}


CTwMgr::CLibStdString::CLibStdString()
{
    memset(m_Data, 0, sizeof(m_Data));
}

void CTwMgr::CLibStdString::FromClient(const std::string& clientStr)
{
    assert( g_TwMgr!=NULL );
    memcpy(m_Data + sizeof(void *), &clientStr, g_TwMgr->m_ClientStdStringStructSize);
#ifdef _MSC_VER
    FixVS2010StdStringClientToLib(m_Data + sizeof(void *));
#endif
}

std::string& CTwMgr::CLibStdString::ToLib()
{
    assert( g_TwMgr!=NULL );
    if( g_TwMgr->m_ClientStdStringStructSize==sizeof(std::string)+sizeof(void *) )
        return *(std::string *)(m_Data + 2*sizeof(void *));
    else if( g_TwMgr->m_ClientStdStringStructSize+sizeof(void *)==sizeof(std::string) )
        return *(std::string *)(m_Data);
    else
    {
        assert( g_TwMgr->m_ClientStdStringStructSize==sizeof(std::string) );
        return *(std::string *)(m_Data + sizeof(void *));
    }
}


//  ---------------------------------------------------------------------------
//  Management functions
//  ---------------------------------------------------------------------------


static int TwCreateGraph(ETwGraphAPI _GraphAPI)
{
    assert( g_TwMgr!=NULL && g_TwMgr->m_Graph==NULL );

    switch( _GraphAPI )
    {
    case TW_OPENGL:
        g_TwMgr->m_Graph = new CTwGraphOpenGL;
        break;
    case TW_OPENGL_CORE:
        g_TwMgr->m_Graph = new CTwGraphOpenGLCore;
        break;
    case TW_DIRECT3D9:
        #ifdef ANT_WINDOWS_DX9
            if( g_TwMgr->m_Device!=NULL )
                g_TwMgr->m_Graph = new CTwGraphDirect3D9;
            else
            {
                g_TwMgr->SetLastError(g_ErrBadDevice);
                return 0;
            }
        #endif // ANT_WINDOWS
        break;
    case TW_DIRECT3D10:
        #ifdef ANT_WINDOWS_DX10
            if( g_TwMgr->m_Device!=NULL )
                g_TwMgr->m_Graph = new CTwGraphDirect3D10;
            else
            {
                g_TwMgr->SetLastError(g_ErrBadDevice);
                return 0;
            }
        #endif // ANT_WINDOWS
        break;
    case TW_DIRECT3D11:
        #ifdef ANT_WINDOWS_DX11
            if( g_TwMgr->m_Device!=NULL )
                g_TwMgr->m_Graph = new CTwGraphDirect3D11;
            else
            {
                g_TwMgr->SetLastError(g_ErrBadDevice);
                return 0;
            }
        #endif // ANT_WINDOWS
        break;
    }

    if( g_TwMgr->m_Graph==NULL )
    {
        g_TwMgr->SetLastError(g_ErrUnknownAPI);
        return 0;
    }
    else
        return g_TwMgr->m_Graph->Init();
}

//  ---------------------------------------------------------------------------

static inline int TwFreeAsyncDrawing()
{
    if( g_TwMgr && g_TwMgr->m_Graph && g_TwMgr->m_Graph->IsDrawing() )
    {
        const double SLEEP_MAX = 0.25; // wait at most 1/4 second
        PerfTimer timer;
        while( g_TwMgr->m_Graph->IsDrawing() && timer.GetTime()<SLEEP_MAX )
        {
            #if defined(ANT_WINDOWS)
                Sleep(1); // milliseconds
            #elif defined(ANT_UNIX) || defined(ANT_OSX)
                usleep(1000); // microseconds
            #endif
        }
        if( g_TwMgr->m_Graph->IsDrawing() )
        {
            g_TwMgr->SetLastError(g_ErrIsDrawing);
            return 0;
        }
    }
    return 1;
}

//  ---------------------------------------------------------------------------

/*
static inline int TwFreeAsyncProcessing()
{
    if( g_TwMgr && g_TwMgr->IsProcessing() )
    {
        const double SLEEP_MAX = 0.25; // wait at most 1/4 second
        PerfTimer timer;
        while( g_TwMgr->IsProcessing() && timer.GetTime()<SLEEP_MAX )
        {
            #if defined(ANT_WINDOWS)
                Sleep(1); // milliseconds
            #elif defined(ANT_UNIX) 
                usleep(1000); // microseconds
            #endif
        }
        if( g_TwMgr->IsProcessing() )
        {
            g_TwMgr->SetLastError(g_ErrIsProcessing);
            return 0;
        }
    }
    return 1;
}

static inline int TwBeginProcessing()
{
    if( !TwFreeAsyncProcessing() )
        return 0;
    if( g_TwMgr )
        g_TwMgr->SetProcessing(true);
}

static inline int TwEndProcessing()
{
    if( g_TwMgr )
        g_TwMgr->SetProcessing(false);
}
*/

//  ---------------------------------------------------------------------------

static int TwInitMgr()
{
    assert( g_TwMasterMgr!=NULL );
    assert( g_TwMgr!=NULL );

    g_TwMgr->m_CurrentFont = g_DefaultNormalFont;
    g_TwMgr->m_Graph = g_TwMasterMgr->m_Graph;

    g_TwMgr->m_KeyPressedTextObj = g_TwMgr->m_Graph->NewTextObj();
    g_TwMgr->m_InfoTextObj = g_TwMgr->m_Graph->NewTextObj();

    g_TwMgr->m_HelpBar = TwNewBar("TW_HELP");
    if( g_TwMgr->m_HelpBar )
    {
        g_TwMgr->m_HelpBar->m_Label = "~ Help & Shortcuts ~";
        g_TwMgr->m_HelpBar->m_PosX = 32;
        g_TwMgr->m_HelpBar->m_PosY = 32;
        g_TwMgr->m_HelpBar->m_Width = 400;
        g_TwMgr->m_HelpBar->m_Height = 200;
        g_TwMgr->m_HelpBar->m_ValuesWidth = 12*(g_TwMgr->m_HelpBar->m_Font->m_CharHeight/2);
        g_TwMgr->m_HelpBar->m_Color = 0xa05f5f5f; //0xd75f5f5f;
        g_TwMgr->m_HelpBar->m_DarkText = false;
        g_TwMgr->m_HelpBar->m_IsHelpBar = true;
        g_TwMgr->Minimize(g_TwMgr->m_HelpBar);
    }
    else
        return 0;

    CColorExt::CreateTypes();
    CQuaternionExt::CreateTypes();

    return 1;
}


int ANT_CALL TwInit(ETwGraphAPI _GraphAPI, void *_Device)
{
#if defined(_DEBUG) && defined(ANT_WINDOWS)
    _CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF|_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF));
#endif

    if( g_TwMasterMgr!=NULL )
    {
        g_TwMasterMgr->SetLastError(g_ErrInit);
        return 0;
    }
    assert( g_TwMgr==0 );
    assert( g_Wnds.empty() );

    g_TwMasterMgr = new CTwMgr(_GraphAPI, _Device, TW_MASTER_WINDOW_ID);
    g_Wnds[TW_MASTER_WINDOW_ID] = g_TwMasterMgr;
    g_TwMgr = g_TwMasterMgr;

    TwGenerateDefaultFonts();
    g_TwMgr->m_CurrentFont = g_DefaultNormalFont;

    int Res = TwCreateGraph(_GraphAPI);
    if( Res )
        Res = TwInitMgr();
    
    if( !Res )
        TwTerminate();

    return Res;
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwSetLastError(const char *_StaticErrorMessage)
{
    if( g_TwMasterMgr!=0 ) 
    {
        g_TwMasterMgr->SetLastError(_StaticErrorMessage);
        return 1;
    }
    else 
        return 0;
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwTerminate()
{
    if( g_TwMgr==NULL )
    {
        //TwGlobalError(g_ErrShut); -> not an error
        return 0;  // already shutdown
    }

    // For multi-thread safety
    if( !TwFreeAsyncDrawing() )
        return 0;

    CTwWndMap::iterator it;
    for( it=g_Wnds.begin(); it!=g_Wnds.end(); it++ )
    {
        g_TwMgr = it->second;

        g_TwMgr->m_Terminating = true;
        TwDeleteAllBars();
        if( g_TwMgr->m_CursorsCreated )
            g_TwMgr->FreeCursors();

        if( g_TwMgr->m_Graph )
        {
            if( g_TwMgr->m_KeyPressedTextObj )
            {
                g_TwMgr->m_Graph->DeleteTextObj(g_TwMgr->m_KeyPressedTextObj);
                g_TwMgr->m_KeyPressedTextObj = NULL;
            }
            if( g_TwMgr->m_InfoTextObj )
            {
                g_TwMgr->m_Graph->DeleteTextObj(g_TwMgr->m_InfoTextObj);
                g_TwMgr->m_InfoTextObj = NULL;
            }
            if (g_TwMgr != g_TwMasterMgr)
                g_TwMgr->m_Graph = NULL;
        }

        if (g_TwMgr != g_TwMasterMgr) 
        {
            delete g_TwMgr;
            g_TwMgr = NULL;
        }
    }

    // delete g_TwMasterMgr
    int Res = 1;
    g_TwMgr = g_TwMasterMgr;
    if( g_TwMasterMgr->m_Graph )
    {
        Res = g_TwMasterMgr->m_Graph->Shut();
        delete g_TwMasterMgr->m_Graph;
        g_TwMasterMgr->m_Graph = NULL;
    }
    TwDeleteDefaultFonts();
    delete g_TwMasterMgr;
    g_TwMasterMgr = NULL;
    g_TwMgr = NULL;
    g_Wnds.clear();

    return Res;
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwGetCurrentWindow()
{
    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return 0; // not initialized
    }

    return g_TwMgr->m_WndID;
}

int ANT_CALL TwSetCurrentWindow(int wndID)
{
    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return 0; // not initialized
    }

    if (wndID != g_TwMgr->m_WndID)
    { 
        CTwWndMap::iterator foundWnd = g_Wnds.find(wndID);
        if (foundWnd == g_Wnds.end())
        {
            // create a new CTwMgr
            g_TwMgr = new CTwMgr(g_TwMasterMgr->m_GraphAPI, g_TwMasterMgr->m_Device, wndID);
            g_Wnds[wndID] = g_TwMgr;
            return TwInitMgr();
        }
        else 
        {
            g_TwMgr = foundWnd->second;
            return 1;
        }
    }
    else 
        return 1;
}

int ANT_CALL TwWindowExists(int wndID)
{
    CTwWndMap::iterator foundWnd = g_Wnds.find(wndID);
    if (foundWnd == g_Wnds.end())
        return 0;
    else
        return 1;
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwDraw()
{
    PERF( PerfTimer Timer; double DT; )
    //CTwFPU fpu;   // fpu precision only forced in update (do not modif dx draw calls)

    if( g_TwMgr==NULL || g_TwMgr->m_Graph==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return 0; // not initialized
    }

    assert(g_TwMgr->m_Bars.size()==g_TwMgr->m_Order.size());

    // For multi-thread savety
    if( !TwFreeAsyncDrawing() )
        return 0;

    // Create cursors
    #if defined(ANT_WINDOWS) || defined(ANT_OSX)
        if( !g_TwMgr->m_CursorsCreated )
            g_TwMgr->CreateCursors();
    #elif defined(ANT_UNIX)
        if( !g_TwMgr->m_CurrentXDisplay )
            g_TwMgr->m_CurrentXDisplay = glXGetCurrentDisplay();
        if( !g_TwMgr->m_CurrentXWindow )
            g_TwMgr->m_CurrentXWindow = glXGetCurrentDrawable();
        if( g_TwMgr->m_CurrentXDisplay && !g_TwMgr->m_CursorsCreated )
            g_TwMgr->CreateCursors();
    #endif

    // Autorepeat TW_MOUSE_PRESSED
    double CurrTime = g_TwMgr->m_Timer.GetTime();
    double RepeatDT = CurrTime - g_TwMgr->m_LastMousePressedTime;
    double DrawDT = CurrTime - g_TwMgr->m_LastDrawTime;
    if(    RepeatDT>2.0*g_TwMgr->m_RepeatMousePressedDelay 
        || DrawDT>2.0*g_TwMgr->m_RepeatMousePressedDelay 
        || abs(g_TwMgr->m_LastMousePressedPosition[0]-g_TwMgr->m_LastMouseX)>4
        || abs(g_TwMgr->m_LastMousePressedPosition[1]-g_TwMgr->m_LastMouseY)>4 )
    {
        g_TwMgr->m_CanRepeatMousePressed = false;
        g_TwMgr->m_IsRepeatingMousePressed = false;
    }
    if( g_TwMgr->m_CanRepeatMousePressed )
    {
        if(    (!g_TwMgr->m_IsRepeatingMousePressed && RepeatDT>g_TwMgr->m_RepeatMousePressedDelay)
            || (g_TwMgr->m_IsRepeatingMousePressed && RepeatDT>g_TwMgr->m_RepeatMousePressedPeriod) )
        {
            g_TwMgr->m_IsRepeatingMousePressed = true;
            g_TwMgr->m_LastMousePressedTime = g_TwMgr->m_Timer.GetTime();
            TwMouseButton(TW_MOUSE_PRESSED, g_TwMgr->m_LastMousePressedButtonID);
        }
    }
    g_TwMgr->m_LastDrawTime = CurrTime;

    if( g_TwMgr->m_WndWidth<0 || g_TwMgr->m_WndHeight<0 )
    {
        g_TwMgr->SetLastError(g_ErrBadSize);
        return 0;
    }
    else if( g_TwMgr->m_WndWidth==0 || g_TwMgr->m_WndHeight==0 )    // probably iconified
        return 1;   // nothing to do

    // count number of bars to draw
    size_t i, j;
    int Nb = 0;
    for( i=0; i<g_TwMgr->m_Bars.size(); ++i )
        if( g_TwMgr->m_Bars[i]!=NULL && g_TwMgr->m_Bars[i]->m_Visible )
            ++Nb;

    if( Nb>0 )
    {
        PERF( Timer.Reset(); )
        g_TwMgr->m_Graph->BeginDraw(g_TwMgr->m_WndWidth, g_TwMgr->m_WndHeight);
        PERF( DT = Timer.GetTime(); printf("\nBegin=%.4fms ", 1000.0*DT); )

        PERF( Timer.Reset(); )
        vector<CRect> TopBarsRects, ClippedBarRects;
        for( i=0; i<g_TwMgr->m_Bars.size(); ++i )
        {
            CTwBar *Bar = g_TwMgr->m_Bars[ g_TwMgr->m_Order[i] ];
            if( Bar->m_Visible )
            {
                if( g_TwMgr->m_OverlapContent || Bar->IsMinimized() )
                    Bar->Draw();
                else
                {
                    // Clip overlapped transparent bars to make them more readable
                    const int Margin = 4;
                    CRect BarRect(Bar->m_PosX - Margin, Bar->m_PosY - Margin, Bar->m_Width + 2*Margin, Bar->m_Height + 2*Margin);
                    TopBarsRects.clear();
                    for( j=i+1; j<g_TwMgr->m_Bars.size(); ++j )
                    {
                        CTwBar *TopBar = g_TwMgr->m_Bars[g_TwMgr->m_Order[j]];
                        if( TopBar->m_Visible && !TopBar->IsMinimized() )
                            TopBarsRects.push_back(CRect(TopBar->m_PosX, TopBar->m_PosY, TopBar->m_Width, TopBar->m_Height));
                    }
                    ClippedBarRects.clear();
                    BarRect.Subtract(TopBarsRects, ClippedBarRects);

                    if( ClippedBarRects.size()==1 && ClippedBarRects[0]==BarRect )
                        //g_TwMgr->m_Graph->DrawRect(Bar->m_PosX, Bar->m_PosY, Bar->m_PosX+Bar->m_Width-1, Bar->m_PosY+Bar->m_Height-1, 0x70ffffff); // Clipping test
                        Bar->Draw(); // unclipped
                    else
                    {
                        Bar->Draw(CTwBar::DRAW_BG); // draw background only

                        // draw content for each clipped rectangle
                        for( j=0; j<ClippedBarRects.size(); j++ )
                            if (ClippedBarRects[j].W>1 && ClippedBarRects[j].H>1)
                            {
                                g_TwMgr->m_Graph->SetScissor(ClippedBarRects[j].X+1, ClippedBarRects[j].Y, ClippedBarRects[j].W, ClippedBarRects[j].H-1);
                                //g_TwMgr->m_Graph->DrawRect(0, 0, 1000, 1000, 0x70ffffff); // Clipping test
                                Bar->Draw(CTwBar::DRAW_CONTENT);
                            }
                        g_TwMgr->m_Graph->SetScissor(0, 0, 0, 0);
                    }
                }
            }
        }
        PERF( DT = Timer.GetTime(); printf("Draw=%.4fms ", 1000.0*DT); )

        PERF( Timer.Reset(); )
        g_TwMgr->m_Graph->EndDraw();
        PERF( DT = Timer.GetTime(); printf("End=%.4fms\n", 1000.0*DT); )
    }

    return 1;
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwWindowSize(int _Width, int _Height)
{
    g_InitWndWidth = _Width;
    g_InitWndHeight = _Height;

    if( g_TwMgr==NULL || g_TwMgr->m_Graph==NULL )
    {
        //TwGlobalError(g_ErrNotInit);  -> not an error here
        return 0;  // not initialized
    }

    if( _Width<0 || _Height<0 )
    {
        g_TwMgr->SetLastError(g_ErrBadSize);
        return 0;
    }

    // For multi-thread savety
    if( !TwFreeAsyncDrawing() )
        return 0;

    // Delete the extra text objects
    if( g_TwMgr->m_KeyPressedTextObj )
    {
        g_TwMgr->m_Graph->DeleteTextObj(g_TwMgr->m_KeyPressedTextObj);
        g_TwMgr->m_KeyPressedTextObj = NULL;
    }
    if( g_TwMgr->m_InfoTextObj )
    {
        g_TwMgr->m_Graph->DeleteTextObj(g_TwMgr->m_InfoTextObj);
        g_TwMgr->m_InfoTextObj = NULL;
    }

    g_TwMgr->m_WndWidth = _Width;
    g_TwMgr->m_WndHeight = _Height;
    g_TwMgr->m_Graph->Restore();

    // Recreate extra text objects
    if( g_TwMgr->m_WndWidth!=0 && g_TwMgr->m_WndHeight!=0 )
    {
        if( g_TwMgr->m_KeyPressedTextObj==NULL )
        {
            g_TwMgr->m_KeyPressedTextObj = g_TwMgr->m_Graph->NewTextObj();
            g_TwMgr->m_KeyPressedBuildText = true;
        }
        if( g_TwMgr->m_InfoTextObj==NULL )
        {
            g_TwMgr->m_InfoTextObj = g_TwMgr->m_Graph->NewTextObj();
            g_TwMgr->m_InfoBuildText = true;
        }
    }

    for( std::vector<TwBar*>::iterator it=g_TwMgr->m_Bars.begin(); it!=g_TwMgr->m_Bars.end(); ++it )
        (*it)->NotUpToDate();
    
    return 1;
}

//  ---------------------------------------------------------------------------

CTwMgr::CTwMgr(ETwGraphAPI _GraphAPI, void *_Device, int _WndID)
{
    m_GraphAPI = _GraphAPI;
    m_Device = _Device;
    m_WndID = _WndID;
    m_LastError = NULL;
    m_CurrentDbgFile = "";
    m_CurrentDbgLine = 0;
    //m_Processing = false;
    m_Graph = NULL;
    m_WndWidth = g_InitWndWidth;
    m_WndHeight = g_InitWndHeight;
    m_CurrentFont = NULL;   // set after by TwIntialize
    m_NbMinimizedBars = 0;
    m_HelpBar = NULL;
    m_HelpBarNotUpToDate = true;
    m_HelpBarUpdateNow = false;
    m_LastHelpUpdateTime = 0;
    m_LastMouseX = -1;
    m_LastMouseY = -1;
    m_LastMouseWheelPos = 0;
    m_IconPos = 0;
    m_IconAlign = 0;
    m_IconMarginX = m_IconMarginY = 8;
    m_FontResizable = true;
    m_KeyPressedTextObj = NULL;
    m_KeyPressedBuildText = false;
    m_KeyPressedTime = 0;
    m_InfoTextObj = NULL;
    m_InfoBuildText = true;
    m_BarInitColorHue = 155;
    m_PopupBar = NULL;
    m_TypeColor32 = TW_TYPE_UNDEF;
    m_TypeColor3F = TW_TYPE_UNDEF;
    m_TypeColor4F = TW_TYPE_UNDEF;
    m_LastMousePressedTime = 0;
    m_LastMousePressedButtonID = TW_MOUSE_MIDDLE;
    m_LastMousePressedPosition[0] = -1000;
    m_LastMousePressedPosition[1] = -1000;
    m_RepeatMousePressedDelay = 0.5;
    m_RepeatMousePressedPeriod = 0.1;
    m_CanRepeatMousePressed = false;
    m_IsRepeatingMousePressed = false;
    m_LastDrawTime = 0;
    m_UseOldColorScheme = false;
    m_Contained = false;
    m_ButtonAlign = BUTTON_ALIGN_RIGHT;
    m_OverlapContent = false;
    m_Terminating = false;
    
    m_CursorsCreated = false;   
    #if defined(ANT_UNIX)
        m_CurrentXDisplay = NULL;
        m_CurrentXWindow = 0;
    #endif  // defined(ANT_UNIX)

    m_CopyCDStringToClient = g_InitCopyCDStringToClient;
    m_CopyStdStringToClient = g_InitCopyStdStringToClient;
    m_ClientStdStringStructSize = 0;
    m_ClientStdStringBaseType = (TwType)0;
}

//  ---------------------------------------------------------------------------

CTwMgr::~CTwMgr()
{
}

//  ---------------------------------------------------------------------------

int CTwMgr::FindBar(const char *_Name) const
{
    if( _Name==NULL || strlen(_Name)<=0 )
        return -1;
    int i;
    for( i=0; i<(int)m_Bars.size(); ++i )
        if( m_Bars[i]!=NULL && strcmp(_Name, m_Bars[i]->m_Name.c_str())==0 )
            return i;
    return -1;
}


//  ---------------------------------------------------------------------------

int CTwMgr::HasAttrib(const char *_Attrib, bool *_HasValue) const
{
    *_HasValue = true;
    if( _stricmp(_Attrib, "help")==0 )
        return MGR_HELP;
    else if( _stricmp(_Attrib, "fontsize")==0 )
        return MGR_FONT_SIZE;
    else if( _stricmp(_Attrib, "fontstyle")==0 )
        return MGR_FONT_STYLE;
    else if( _stricmp(_Attrib, "iconpos")==0 )
        return MGR_ICON_POS;
    else if( _stricmp(_Attrib, "iconalign")==0 )
        return MGR_ICON_ALIGN;
    else if( _stricmp(_Attrib, "iconmargin")==0 )
        return MGR_ICON_MARGIN;
    else if( _stricmp(_Attrib, "fontresizable")==0 )
        return MGR_FONT_RESIZABLE;
    else if( _stricmp(_Attrib, "colorscheme")==0 )
        return MGR_COLOR_SCHEME;
    else if( _stricmp(_Attrib, "contained")==0 )
        return MGR_CONTAINED;
    else if( _stricmp(_Attrib, "buttonalign")==0 )
        return MGR_BUTTON_ALIGN;
    else if( _stricmp(_Attrib, "overlap")==0 )
        return MGR_OVERLAP;

    *_HasValue = false;
    return 0; // not found
}

int CTwMgr::SetAttrib(int _AttribID, const char *_Value)
{
    switch( _AttribID )
    {
    case MGR_HELP:
        if( _Value && strlen(_Value)>0 )
        {
            m_Help = _Value;
            m_HelpBarNotUpToDate = true;
            return 1;
        }
        else
        {
            SetLastError(g_ErrNoValue);
            return 0;
        }
    case MGR_FONT_SIZE:
        if( _Value && strlen(_Value)>0 )
        {
            int s;
            int n = sscanf(_Value, "%d", &s);
            if( n==1 && s>=1 && s<=3 )
            {
                if( s==1 )
                    SetFont(g_DefaultSmallFont, true);
                else if( s==2 )
                    SetFont(g_DefaultNormalFont, true);
                else if( s==3 )
                    SetFont(g_DefaultLargeFont, true);
                return 1;
            }
            else
            {
                SetLastError(g_ErrBadValue);
                return 0;
            }
        }
        else
        {
            SetLastError(g_ErrNoValue);
            return 0;
        }
    case MGR_FONT_STYLE:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "fixed")==0 )
            {
                if( m_CurrentFont!=g_DefaultFixed1Font )
                {
                    SetFont(g_DefaultFixed1Font, true);
                    m_FontResizable = false; // for now fixed font is not resizable
                }
                return 1;
            } 
            else if( _stricmp(_Value, "default")==0 )
            {
                if( m_CurrentFont!=g_DefaultSmallFont && m_CurrentFont!=g_DefaultNormalFont && m_CurrentFont!=g_DefaultLargeFont )
                {
                    if( m_CurrentFont == g_DefaultFixed1Font )
                        m_FontResizable = true;
                    SetFont(g_DefaultNormalFont, true);
                }
                return 1;
            }
            else
            {
                SetLastError(g_ErrBadValue);
                return 0;
            }
        }
        else
        {
            SetLastError(g_ErrNoValue);
            return 0;
        }
    case MGR_ICON_POS:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "bl")==0 || _stricmp(_Value, "lb")==0 || _stricmp(_Value, "bottomleft")==0 || _stricmp(_Value, "leftbottom")==0 )
            {
                m_IconPos = 0;
                return 1;
            }
            else if( _stricmp(_Value, "br")==0 || _stricmp(_Value, "rb")==0 || _stricmp(_Value, "bottomright")==0 || _stricmp(_Value, "rightbottom")==0 )
            {
                m_IconPos = 1;
                return 1;
            }
            else if( _stricmp(_Value, "tl")==0 || _stricmp(_Value, "lt")==0 || _stricmp(_Value, "topleft")==0 || _stricmp(_Value, "lefttop")==0 )
            {
                m_IconPos = 2;
                return 1;
            }
            else if( _stricmp(_Value, "tr")==0 || _stricmp(_Value, "rt")==0 || _stricmp(_Value, "topright")==0 || _stricmp(_Value, "righttop")==0 )
            {
                m_IconPos = 3;
                return 1;
            }
            else
            {
                SetLastError(g_ErrBadValue);
                return 0;
            }
        }
        else
        {
            SetLastError(g_ErrNoValue);
            return 0;
        }
    case MGR_ICON_ALIGN:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "vert")==0 || _stricmp(_Value, "vertical")==0  )
            {
                m_IconAlign = 0;
                return 1;
            }
            else if( _stricmp(_Value, "horiz")==0 || _stricmp(_Value, "horizontal")==0  )
            {
                m_IconAlign = 1;
                return 1;
            }
            else
            {
                SetLastError(g_ErrBadValue);
                return 0;
            }
        }
        else
        {
            SetLastError(g_ErrNoValue);
            return 0;
        }
    case MGR_ICON_MARGIN:
        if( _Value && strlen(_Value)>0 )
        {
            int x, y;
            int n = sscanf(_Value, "%d%d", &x, &y);
            if( n==2 && x>=0 && y>=0 )
            {
                m_IconMarginX = x;
                m_IconMarginY = y;
                return 1;
            }
            else
            {
                SetLastError(g_ErrBadValue);
                return 0;
            }
        }
        else
        {
            SetLastError(g_ErrNoValue);
            return 0;
        }
    case MGR_FONT_RESIZABLE:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "1")==0 || _stricmp(_Value, "true")==0 )
            {
                m_FontResizable = true;
                return 1;
            }
            else if( _stricmp(_Value, "0")==0 || _stricmp(_Value, "false")==0 )
            {
                m_FontResizable = false;
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
    case MGR_COLOR_SCHEME:
        if( _Value && strlen(_Value)>0 )
        {
            int s;
            int n = sscanf(_Value, "%d", &s);
            if( n==1 && s>=0 && s<=1 )
            {
                if( s==0 )
                    m_UseOldColorScheme = true;
                else
                    m_UseOldColorScheme = false;
                return 1;
            }
            else
            {
                SetLastError(g_ErrBadValue);
                return 0;
            }
        }
        else
        {
            g_TwMgr->SetLastError(g_ErrNoValue);
            return 0;
        }
    case MGR_CONTAINED:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "1")==0 || _stricmp(_Value, "true")==0 )
                m_Contained = true;
            else if( _stricmp(_Value, "0")==0 || _stricmp(_Value, "false")==0 )
                m_Contained = false;
            else
            {
                g_TwMgr->SetLastError(g_ErrBadValue);
                return 0;
            }
            vector<TwBar*>::iterator barIt;
            for( barIt=g_TwMgr->m_Bars.begin(); barIt!=g_TwMgr->m_Bars.end(); ++barIt )
                if( (*barIt)!=NULL )
                    (*barIt)->m_Contained = m_Contained;
            return 1;
        }
        else
        {
            g_TwMgr->SetLastError(g_ErrNoValue);
            return 0;
        }
    case MGR_BUTTON_ALIGN:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "left")==0 )
                m_ButtonAlign = BUTTON_ALIGN_LEFT;
            else if( _stricmp(_Value, "center")==0 )
                m_ButtonAlign = BUTTON_ALIGN_CENTER;
            else if( _stricmp(_Value, "right")==0 )
                m_ButtonAlign = BUTTON_ALIGN_RIGHT;
            else
            {
                g_TwMgr->SetLastError(g_ErrBadValue);
                return 0;
            }
            vector<TwBar*>::iterator barIt;
            for( barIt=g_TwMgr->m_Bars.begin(); barIt!=g_TwMgr->m_Bars.end(); ++barIt )
                if( (*barIt)!=NULL )
                    (*barIt)->m_ButtonAlign = m_ButtonAlign;
            return 1;
        }
        else
        {
            g_TwMgr->SetLastError(g_ErrNoValue);
            return 0;
        }
    case MGR_OVERLAP:
        if( _Value && strlen(_Value)>0 )
        {
            if( _stricmp(_Value, "1")==0 || _stricmp(_Value, "true")==0 )
            {
                m_OverlapContent = true;
                return 1;
            }
            else if( _stricmp(_Value, "0")==0 || _stricmp(_Value, "false")==0 )
            {
                m_OverlapContent = false;
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

ERetType CTwMgr::GetAttrib(int _AttribID, std::vector<double>& outDoubles, std::ostringstream& outString) const
{
    outDoubles.clear();
    outString.clear();

    switch( _AttribID )
    {
    case MGR_HELP:
        outString << m_Help;
        return RET_STRING;
    case MGR_FONT_SIZE:
        if( m_CurrentFont==g_DefaultSmallFont )
            outDoubles.push_back(1);
        else if( m_CurrentFont==g_DefaultNormalFont )
            outDoubles.push_back(2);
        else if( m_CurrentFont==g_DefaultLargeFont )
            outDoubles.push_back(3);
        else
            outDoubles.push_back(0); // should not happened
        return RET_DOUBLE;
    case MGR_FONT_STYLE:
        if( m_CurrentFont==g_DefaultFixed1Font )
            outString << "fixed";
        else 
            outString << "default";
        return RET_STRING;
    case MGR_ICON_POS:
        if( m_IconPos==0 )
            outString << "bottomleft";
        else if( m_IconPos==1 )
            outString << "bottomright";
        else if( m_IconPos==2 )
            outString << "topleft";
        else if( m_IconPos==3 )
            outString << "topright";
        else
            outString << "undefined"; // should not happened
        return RET_STRING;
    case MGR_ICON_ALIGN:
        if( m_IconAlign==0 )
            outString << "vertical";
        else if( m_IconAlign==1 )
            outString << "horizontal";
        else
            outString << "undefined"; // should not happened
        return RET_STRING;
    case MGR_ICON_MARGIN:
        outDoubles.push_back(m_IconMarginX);
        outDoubles.push_back(m_IconMarginY);
        return RET_DOUBLE;
    case MGR_FONT_RESIZABLE:
        outDoubles.push_back(m_FontResizable);
        return RET_DOUBLE;
    case MGR_COLOR_SCHEME:
        outDoubles.push_back(m_UseOldColorScheme ? 0 : 1);
        return RET_DOUBLE;
    case MGR_CONTAINED:
        {
            bool contained = m_Contained;
            /*
            if( contained ) 
            {
                vector<TwBar*>::iterator barIt;
                for( barIt=g_TwMgr->m_Bars.begin(); barIt!=g_TwMgr->m_Bars.end(); ++barIt )
                    if( (*barIt)!=NULL && !(*barIt)->m_Contained )
                    {
                        contained = false;
                        break;
                    }
            }
            */
            outDoubles.push_back(contained);
            return RET_DOUBLE;
        }
    case MGR_BUTTON_ALIGN:
        if( m_ButtonAlign==BUTTON_ALIGN_LEFT )
            outString << "left";
        else if( m_ButtonAlign==BUTTON_ALIGN_CENTER )
            outString << "center";
        else
            outString << "right";
        return RET_STRING;
    case MGR_OVERLAP:
        outDoubles.push_back(m_OverlapContent);
        return RET_DOUBLE;
    default:
        g_TwMgr->SetLastError(g_ErrUnknownAttrib);
        return RET_ERROR;
    }
}

//  ---------------------------------------------------------------------------

void CTwMgr::Minimize(TwBar *_Bar)
{
    assert(m_Graph!=NULL && _Bar!=NULL);
    assert(m_Bars.size()==m_MinOccupied.size());
    if( _Bar->m_IsMinimized )
        return;
    if( _Bar->m_Visible )
    {
        size_t i = m_NbMinimizedBars;
        m_NbMinimizedBars++;
        for( i=0; i<m_MinOccupied.size(); ++i )
            if( !m_MinOccupied[i] )
                break;
        if( i<m_MinOccupied.size() )
            m_MinOccupied[i] = true;
        _Bar->m_MinNumber = (int)i;
    }
    else
        _Bar->m_MinNumber = -1;
    _Bar->m_IsMinimized = true;
    _Bar->NotUpToDate();
}

//  ---------------------------------------------------------------------------

void CTwMgr::Maximize(TwBar *_Bar)
{
    assert(m_Graph!=NULL && _Bar!=NULL);
    assert(m_Bars.size()==m_MinOccupied.size());
    if( !_Bar->m_IsMinimized )
        return;
    if( _Bar->m_Visible )
    {
        --m_NbMinimizedBars;
        if( m_NbMinimizedBars<0 )
            m_NbMinimizedBars = 0;
        if( _Bar->m_MinNumber>=0 && _Bar->m_MinNumber<(int)m_MinOccupied.size() )
            m_MinOccupied[_Bar->m_MinNumber] = false;
    }
    _Bar->m_IsMinimized = false;
    _Bar->NotUpToDate();
    if( _Bar->m_IsHelpBar )
        m_HelpBarNotUpToDate = true;
}

//  ---------------------------------------------------------------------------

void CTwMgr::Hide(TwBar *_Bar)
{
    assert(m_Graph!=NULL && _Bar!=NULL);
    if( !_Bar->m_Visible )
        return;
    if( _Bar->IsMinimized() )
    {
        Maximize(_Bar);
        _Bar->m_Visible = false;
        Minimize(_Bar);
    }
    else
        _Bar->m_Visible = false;
    if( !_Bar->m_IsHelpBar )
        m_HelpBarNotUpToDate = true;
}

//  ---------------------------------------------------------------------------

void CTwMgr::Unhide(TwBar *_Bar)
{
    assert(m_Graph!=NULL && _Bar!=NULL);
    if( _Bar->m_Visible )
        return;
    if( _Bar->IsMinimized() )
    {
        Maximize(_Bar);
        _Bar->m_Visible = true;
        Minimize(_Bar);
    }
    else
        _Bar->m_Visible = true;
    _Bar->NotUpToDate();
    if( !_Bar->m_IsHelpBar )
        m_HelpBarNotUpToDate = true;
}

//  ---------------------------------------------------------------------------

void CTwMgr::SetFont(const CTexFont *_Font, bool _ResizeBars)
{
    assert(m_Graph!=NULL);
    assert(_Font!=NULL);

    m_CurrentFont = _Font;

    for( int i=0; i<(int)m_Bars.size(); ++i )
        if( m_Bars[i]!=NULL )
        {
            int fh = m_Bars[i]->m_Font->m_CharHeight;
            m_Bars[i]->m_Font = _Font;
            if( _ResizeBars )
            {
                if( m_Bars[i]->m_Movable )
                {
                    m_Bars[i]->m_PosX += (3*(fh-_Font->m_CharHeight))/2;
                    m_Bars[i]->m_PosY += (fh-_Font->m_CharHeight)/2;
                }
                if( m_Bars[i]->m_Resizable ) 
                {
                    m_Bars[i]->m_Width = (m_Bars[i]->m_Width*_Font->m_CharHeight)/fh;
                    m_Bars[i]->m_Height = (m_Bars[i]->m_Height*_Font->m_CharHeight)/fh;
                    m_Bars[i]->m_ValuesWidth = (m_Bars[i]->m_ValuesWidth*_Font->m_CharHeight)/fh;
                }
            }
            m_Bars[i]->NotUpToDate();
        }

    if( g_TwMgr->m_HelpBar!=NULL )
        g_TwMgr->m_HelpBar->Update();
    g_TwMgr->m_InfoBuildText = true;
    g_TwMgr->m_KeyPressedBuildText = true;
    m_HelpBarNotUpToDate = true;
}

//  ---------------------------------------------------------------------------

void ANT_CALL TwGlobalError(const char *_ErrorMessage)  // to be called when g_TwMasterMgr is not created
{
    if( g_ErrorHandler==NULL )
    {
        fprintf(stderr, "ERROR(AntTweakBar) >> %s\n", _ErrorMessage);
    #ifdef ANT_WINDOWS
        OutputDebugString("ERROR(AntTweakBar) >> ");
        OutputDebugString(_ErrorMessage);
        OutputDebugString("\n");
    #endif // ANT_WINDOWS
    }
    else
        g_ErrorHandler(_ErrorMessage);

    if( g_BreakOnError )
        abort();
}

//  ---------------------------------------------------------------------------

void CTwMgr::SetLastError(const char *_ErrorMessage)    // _ErrorMessage must be a static string
{
    if (this != g_TwMasterMgr)
    {
        // route to master
        g_TwMasterMgr->SetLastError(_ErrorMessage);
        return;
    }

    m_LastError = _ErrorMessage;

    if( g_ErrorHandler==NULL )
    {
        if( m_CurrentDbgFile!=NULL && strlen(m_CurrentDbgFile)>0 && m_CurrentDbgLine>0 )
            fprintf(stderr, "%s(%d): ", m_CurrentDbgFile, m_CurrentDbgLine);
        fprintf(stderr, "ERROR(AntTweakBar) >> %s\n", m_LastError);
    #ifdef ANT_WINDOWS
        if( m_CurrentDbgFile!=NULL && strlen(m_CurrentDbgFile)>0 && m_CurrentDbgLine>0 )
        {
            OutputDebugString(m_CurrentDbgFile);
            char sl[32];
            sprintf(sl, "(%d): ", m_CurrentDbgLine);
            OutputDebugString(sl);
        }
        OutputDebugString("ERROR(AntTweakBar) >> ");
        OutputDebugString(m_LastError);
        OutputDebugString("\n");
    #endif // ANT_WINDOWS
    }
    else
        g_ErrorHandler(_ErrorMessage);

    if( g_BreakOnError )
        abort();
}

//  ---------------------------------------------------------------------------

const char *CTwMgr::GetLastError()
{
    if (this != g_TwMasterMgr) 
    {
        // route to master
        return g_TwMasterMgr->GetLastError();
    }

    const char *Err = m_LastError;
    m_LastError = NULL;
    return Err;
}

//  ---------------------------------------------------------------------------

const char *CTwMgr::CheckLastError() const
{
    return m_LastError;
}

//  ---------------------------------------------------------------------------

void CTwMgr::SetCurrentDbgParams(const char *dbgFile, int dbgLine)
{
    m_CurrentDbgFile = dbgFile;
    m_CurrentDbgLine = dbgLine;
}

//  ---------------------------------------------------------------------------

int ANT_CALL __TwDbg(const char *dbgFile, int dbgLine)
{
    if( g_TwMgr!=NULL )
        g_TwMgr->SetCurrentDbgParams(dbgFile, dbgLine);
    return 0;   // always returns zero
}

//  ---------------------------------------------------------------------------

void ANT_CALL TwHandleErrors(TwErrorHandler _ErrorHandler, int _BreakOnError)
{
    g_ErrorHandler = _ErrorHandler;
    g_BreakOnError = (_BreakOnError) ? true : false;
}

void ANT_CALL TwHandleErrors(TwErrorHandler _ErrorHandler)
{
    TwHandleErrors(_ErrorHandler, false);
}

//  ---------------------------------------------------------------------------

const char *ANT_CALL TwGetLastError()
{
    if( g_TwMasterMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return g_ErrNotInit;
    }
    else
        return g_TwMasterMgr->GetLastError();
}

//  ---------------------------------------------------------------------------

TwBar *ANT_CALL TwNewBar(const char *_Name)
{
    if( g_TwMgr==NULL || g_TwMgr->m_Graph==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return NULL; // not initialized
    }

    TwFreeAsyncDrawing(); // For multi-thread savety

    if( _Name==NULL || strlen(_Name)<=0 )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return NULL;
    }
    if( g_TwMgr->FindBar(_Name)>=0 )
    {
        g_TwMgr->SetLastError(g_ErrExist);
        return NULL;
    }

    if( strstr(_Name, "`")!=NULL )
    {
        g_TwMgr->SetLastError(g_ErrNoBackQuote);
        return NULL;
    }

    if( g_TwMgr->m_PopupBar!=NULL ) // delete popup bar if it exists
    {
        TwDeleteBar(g_TwMgr->m_PopupBar);
        g_TwMgr->m_PopupBar = NULL;
    }

    TwBar *Bar = new CTwBar(_Name);
    g_TwMgr->m_Bars.push_back(Bar);
    g_TwMgr->m_Order.push_back((int)g_TwMgr->m_Bars.size()-1);
    g_TwMgr->m_MinOccupied.push_back(false);
    g_TwMgr->m_HelpBarNotUpToDate = true;

    return Bar;
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwDeleteBar(TwBar *_Bar)
{
    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return 0; // not initialized
    }
    if( _Bar==NULL )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return 0;
    }
    if( _Bar==g_TwMgr->m_HelpBar )
    {
        g_TwMgr->SetLastError(g_ErrDelHelp);
        return 0;
    }

    TwFreeAsyncDrawing(); // For multi-thread savety

    vector<TwBar*>::iterator BarIt;
    int i = 0;
    for( BarIt=g_TwMgr->m_Bars.begin(); BarIt!=g_TwMgr->m_Bars.end(); ++BarIt, ++i )
        if( (*BarIt)==_Bar )
            break;
    if( BarIt==g_TwMgr->m_Bars.end() )
    {
        g_TwMgr->SetLastError(g_ErrNotFound);
        return 0;
    }

    if( g_TwMgr->m_PopupBar!=NULL && _Bar!=g_TwMgr->m_PopupBar )    // delete popup bar first if it exists
    {
        TwDeleteBar(g_TwMgr->m_PopupBar);
        g_TwMgr->m_PopupBar = NULL;
    }

    // force bar to un-minimize
    g_TwMgr->Maximize(_Bar);
    // find an empty MinOccupied
    vector<bool>::iterator itm;
    int j = 0;
    for( itm=g_TwMgr->m_MinOccupied.begin(); itm!=g_TwMgr->m_MinOccupied.end(); ++itm, ++j)
        if( (*itm)==false )
            break;
    assert( itm!=g_TwMgr->m_MinOccupied.end() );
    // shift MinNumbers and erase the empty MinOccupied
    for( size_t k=0; k<g_TwMgr->m_Bars.size(); ++k )
        if( g_TwMgr->m_Bars[k]!=NULL && g_TwMgr->m_Bars[k]->m_MinNumber>j )
            g_TwMgr->m_Bars[k]->m_MinNumber -= 1;
    g_TwMgr->m_MinOccupied.erase(itm);
    // erase _Bar order
    vector<int>::iterator BarOrderIt = g_TwMgr->m_Order.end();
    for(vector<int>::iterator it=g_TwMgr->m_Order.begin(); it!=g_TwMgr->m_Order.end(); ++it ) 
        if( (*it)==i )
            BarOrderIt = it;
        else if( (*it)>i )
            (*it) -= 1;
    assert( BarOrderIt!=g_TwMgr->m_Order.end() );
    g_TwMgr->m_Order.erase(BarOrderIt);

    // erase & delete _Bar
    g_TwMgr->m_Bars.erase(BarIt);
    delete _Bar;

    g_TwMgr->m_HelpBarNotUpToDate = true;
    return 1;
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwDeleteAllBars()
{
    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return 0; // not initialized
    }

    TwFreeAsyncDrawing(); // For multi-thread savety

    int n = 0;
    if( g_TwMgr->m_Terminating || g_TwMgr->m_HelpBar==NULL ) 
    {
        for( size_t i=0; i<g_TwMgr->m_Bars.size(); ++i )
            if( g_TwMgr->m_Bars[i]!=NULL )
            {
                ++n;
                delete g_TwMgr->m_Bars[i];
                g_TwMgr->m_Bars[i] = NULL;
            }
        g_TwMgr->m_Bars.clear();
        g_TwMgr->m_Order.clear();
        g_TwMgr->m_MinOccupied.clear();
        g_TwMgr->m_HelpBarNotUpToDate = true;
    }
    else
    {
        vector<CTwBar *> bars = g_TwMgr->m_Bars;
        for( size_t i = 0; i < bars.size(); ++i )
            if( bars[i]!=0 && bars[i]!=g_TwMgr->m_HelpBar)
            {
                ++n;
                TwDeleteBar(bars[i]);
            }
        g_TwMgr->m_HelpBarNotUpToDate = true;
    }

    if( n==0 )
    {
        //g_TwMgr->SetLastError(g_ErrNthToDo);
        return 0;
    }
    else
        return 1;
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwSetTopBar(const TwBar *_Bar)
{
    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return 0; // not initialized
    }
    if( _Bar==NULL )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return 0;
    }

    TwFreeAsyncDrawing(); // For multi-thread savety

    if( _Bar!=g_TwMgr->m_PopupBar && g_TwMgr->m_BarAlwaysOnBottom.length()>0 )
    {
        if( strcmp(_Bar->m_Name.c_str(), g_TwMgr->m_BarAlwaysOnBottom.c_str())==0 )
            return TwSetBottomBar(_Bar);
    }

    int i = -1, iOrder;
    for( iOrder=0; iOrder<(int)g_TwMgr->m_Bars.size(); ++iOrder )
    {
        i = g_TwMgr->m_Order[iOrder];
        assert( i>=0 && i<(int)g_TwMgr->m_Bars.size() );
        if( g_TwMgr->m_Bars[i]==_Bar )
            break;
    }
    if( i<0 || iOrder>=(int)g_TwMgr->m_Bars.size() )    // bar not found
    {
        g_TwMgr->SetLastError(g_ErrNotFound);
        return 0;
    }

    for( int j=iOrder; j<(int)g_TwMgr->m_Bars.size()-1; ++j )
        g_TwMgr->m_Order[j] = g_TwMgr->m_Order[j+1];
    g_TwMgr->m_Order[(int)g_TwMgr->m_Bars.size()-1] = i;

    if( _Bar!=g_TwMgr->m_PopupBar && g_TwMgr->m_BarAlwaysOnTop.length()>0 )
    {
        int topIdx = g_TwMgr->FindBar(g_TwMgr->m_BarAlwaysOnTop.c_str());
        TwBar *top = (topIdx>=0 && topIdx<(int)g_TwMgr->m_Bars.size()) ? g_TwMgr->m_Bars[topIdx] : NULL;
        if( top!=NULL && top!=_Bar )
            TwSetTopBar(top);
    }

    if( g_TwMgr->m_PopupBar!=NULL && _Bar!=g_TwMgr->m_PopupBar )
        TwSetTopBar(g_TwMgr->m_PopupBar);

    return 1;
}

//  ---------------------------------------------------------------------------

TwBar * ANT_CALL TwGetTopBar()
{
    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return NULL; // not initialized
    }

    if( g_TwMgr->m_Bars.size()>0 && g_TwMgr->m_PopupBar==NULL )
        return g_TwMgr->m_Bars[g_TwMgr->m_Order[ g_TwMgr->m_Bars.size()-1 ]];
    else if( g_TwMgr->m_Bars.size()>1 && g_TwMgr->m_PopupBar!=NULL )
        return g_TwMgr->m_Bars[g_TwMgr->m_Order[ g_TwMgr->m_Bars.size()-2 ]];
    else
        return NULL;
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwSetBottomBar(const TwBar *_Bar)
{
    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return 0; // not initialized
    }
    if( _Bar==NULL )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return 0;
    }

    TwFreeAsyncDrawing(); // For multi-thread savety

    if( _Bar!=g_TwMgr->m_PopupBar && g_TwMgr->m_BarAlwaysOnTop.length()>0 )
    {
        if( strcmp(_Bar->m_Name.c_str(), g_TwMgr->m_BarAlwaysOnTop.c_str())==0 )
            return TwSetTopBar(_Bar);
    }

    int i = -1, iOrder;
    for( iOrder=0; iOrder<(int)g_TwMgr->m_Bars.size(); ++iOrder )
    {
        i = g_TwMgr->m_Order[iOrder];
        assert( i>=0 && i<(int)g_TwMgr->m_Bars.size() );
        if( g_TwMgr->m_Bars[i]==_Bar )
            break;
    }
    if( i<0 || iOrder>=(int)g_TwMgr->m_Bars.size() )    // bar not found
    {
        g_TwMgr->SetLastError(g_ErrNotFound);
        return 0;
    }

    if( iOrder>0 )
        for( int j=iOrder-1; j>=0; --j )
            g_TwMgr->m_Order[j+1] = g_TwMgr->m_Order[j];
    g_TwMgr->m_Order[0] = i;

    if( _Bar!=g_TwMgr->m_PopupBar && g_TwMgr->m_BarAlwaysOnBottom.length()>0 )
    {
        int btmIdx = g_TwMgr->FindBar(g_TwMgr->m_BarAlwaysOnBottom.c_str());
        TwBar *btm = (btmIdx>=0 && btmIdx<(int)g_TwMgr->m_Bars.size()) ? g_TwMgr->m_Bars[btmIdx] : NULL;
        if( btm!=NULL && btm!=_Bar )
            TwSetBottomBar(btm);
    }

    return 1;
}

//  ---------------------------------------------------------------------------

TwBar* ANT_CALL TwGetBottomBar()
{
    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return NULL; // not initialized
    }

    if( g_TwMgr->m_Bars.size()>0 )
        return g_TwMgr->m_Bars[g_TwMgr->m_Order[0]];
    else
        return NULL;
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwSetBarState(TwBar *_Bar, TwState _State)
{
    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return 0;  // not initialized
    }
    if( _Bar==NULL )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return 0;
    }

    TwFreeAsyncDrawing(); // For multi-thread savety

    switch( _State )
    {
    case TW_STATE_SHOWN:
        g_TwMgr->Unhide(_Bar);
        return 1;
    case TW_STATE_ICONIFIED:
        //g_TwMgr->Unhide(_Bar);
        g_TwMgr->Minimize(_Bar);
        return 1;
    case TW_STATE_HIDDEN:
        //g_TwMgr->Maximize(_Bar);
        g_TwMgr->Hide(_Bar);
        return 1;
    case TW_STATE_UNICONIFIED:
        //g_TwMgr->Unhide(_Bar);
        g_TwMgr->Maximize(_Bar);
        return 1;
    default:
        g_TwMgr->SetLastError(g_ErrBadParam);
        return 0;
    }
}

//  ---------------------------------------------------------------------------

/*
TwState ANT_CALL TwGetBarState(const TwBar *_Bar)
{
    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return TW_STATE_ERROR;  // not initialized
    }
    if( _Bar==NULL )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return TW_STATE_ERROR;
    }

    if( !_Bar->m_Visible )
        return TW_STATE_HIDDEN;
    else if( _Bar->IsMinimized() )
        return TW_STATE_ICONIFIED;
    else
        return TW_STATE_SHOWN;
}
*/

//  ---------------------------------------------------------------------------

const char * ANT_CALL TwGetBarName(TwBar *_Bar)
{
    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return NULL;  // not initialized
    }
    if( _Bar==NULL )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return NULL;
    }
    vector<TwBar*>::iterator BarIt;
    int i = 0;
    for( BarIt=g_TwMgr->m_Bars.begin(); BarIt!=g_TwMgr->m_Bars.end(); ++BarIt, ++i )
        if( (*BarIt)==_Bar )
            break;
    if( BarIt==g_TwMgr->m_Bars.end() )
    {
        g_TwMgr->SetLastError(g_ErrNotFound);
        return NULL;
    }

    return _Bar->m_Name.c_str();
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwGetBarCount()
{
    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return 0;  // not initialized
    }

    return (int)g_TwMgr->m_Bars.size();
}


//  ---------------------------------------------------------------------------

TwBar * ANT_CALL TwGetBarByIndex(int index)
{
    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return NULL;  // not initialized
    }

    if( index>=0 && index<(int)g_TwMgr->m_Bars.size() ) 
        return g_TwMgr->m_Bars[index];
    else 
    {
        g_TwMgr->SetLastError(g_ErrOutOfRange);
        return NULL;
    }
}

//  ---------------------------------------------------------------------------

TwBar * ANT_CALL TwGetBarByName(const char *name)
{
    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return NULL; // not initialized
    }

    int idx = g_TwMgr->FindBar(name);
    if ( idx>=0 && idx<(int)g_TwMgr->m_Bars.size() )
        return g_TwMgr->m_Bars[idx];
    else
        return NULL;
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwRefreshBar(TwBar *bar)
{
    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return 0;  // not initialized
    }
    if( bar==NULL )
    {
        vector<TwBar*>::iterator BarIt;
        for( BarIt=g_TwMgr->m_Bars.begin(); BarIt!=g_TwMgr->m_Bars.end(); ++BarIt )
            if( *BarIt!=NULL )
                (*BarIt)->NotUpToDate();
    }
    else
    {
        vector<TwBar*>::iterator BarIt;
        int i = 0;
        for( BarIt=g_TwMgr->m_Bars.begin(); BarIt!=g_TwMgr->m_Bars.end(); ++BarIt, ++i )
            if( (*BarIt)==bar )
                break;
        if( BarIt==g_TwMgr->m_Bars.end() )
        {
            g_TwMgr->SetLastError(g_ErrNotFound);
            return 0;
        }

        bar->NotUpToDate();
    }
    return 1;
}

//  ---------------------------------------------------------------------------

int BarVarHasAttrib(CTwBar *_Bar, CTwVar *_Var, const char *_Attrib, bool *_HasValue);
int BarVarSetAttrib(CTwBar *_Bar, CTwVar *_Var, CTwVarGroup *_VarParent, int _VarIndex, int _AttribID, const char *_Value);
ERetType BarVarGetAttrib(CTwBar *_Bar, CTwVar *_Var, CTwVarGroup *_VarParent, int _VarIndex, int _AttribID, std::vector<double>& outDouble, std::ostringstream& outString);


int ANT_CALL TwGetParam(TwBar *bar, const char *varName, const char *paramName, TwParamValueType paramValueType, unsigned int outValueMaxCount, void *outValues)
{
    CTwFPU fpu; // force fpu precision

    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return 0; // not initialized
    }
    if( paramName==NULL || strlen(paramName)<=0 )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return 0;
    }
    if( outValueMaxCount<=0 || outValues==NULL )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return 0;
    }

    if( bar==NULL ) 
        bar = TW_GLOBAL_BAR;
    else 
    {
        vector<TwBar*>::iterator barIt;
        int i = 0;
        for( barIt=g_TwMgr->m_Bars.begin(); barIt!=g_TwMgr->m_Bars.end(); ++barIt, ++i )
            if( (*barIt)==bar )
                break;
        if( barIt==g_TwMgr->m_Bars.end() )
        {
            g_TwMgr->SetLastError(g_ErrNotFound);
            return 0;
        }
    }
    CTwVarGroup *varParent = NULL;
    int varIndex = -1;
    CTwVar *var = NULL;
    if( varName!=NULL && strlen(varName)>0 )
    {
        var = bar->Find(varName, &varParent, &varIndex);
        if( var==NULL )
        {
            _snprintf(g_ErrParse, sizeof(g_ErrParse), "Unknown var '%s/%s'", 
                      (bar==TW_GLOBAL_BAR) ? "GLOBAL" : bar->m_Name.c_str(), varName);
            g_ErrParse[sizeof(g_ErrParse)-1] = '\0';
            g_TwMgr->SetLastError(g_ErrParse);
            return 0;
        }
    }

    bool hasValue = false;
    int paramID = BarVarHasAttrib(bar, var, paramName, &hasValue);
    if( paramID>0 )
    {
        std::ostringstream valStr;
        std::vector<double> valDbl;
        const char *PrevLastErrorPtr = g_TwMgr->CheckLastError();

        ERetType retType = BarVarGetAttrib(bar, var, varParent, varIndex, paramID, valDbl, valStr);
        unsigned int i, valDblCount = (unsigned int)valDbl.size();
        if( valDblCount > outValueMaxCount )
            valDblCount = outValueMaxCount;
        if( retType==RET_DOUBLE && valDblCount==0 )
        {
            g_TwMgr->SetLastError(g_ErrHasNoValue);
            retType = RET_ERROR;
        }

        if( retType==RET_DOUBLE ) 
        {
            switch( paramValueType ) 
            {
            case TW_PARAM_INT32:
                for( i=0; i<valDblCount; i++ )
                    (static_cast<int *>(outValues))[i] = (int)valDbl[i];
                return valDblCount;
            case TW_PARAM_FLOAT:
                for( i=0; i<valDblCount; i++ )
                    (static_cast<float *>(outValues))[i] = (float)valDbl[i];
                return valDblCount;
            case TW_PARAM_DOUBLE:
                for( i=0; i<valDblCount; i++ )
                    (static_cast<double *>(outValues))[i] = valDbl[i];
                return valDblCount;
            case TW_PARAM_CSTRING:
                valStr.clear();
                for( i=0; i<(unsigned int)valDbl.size(); i++ ) // not valDblCount here
                    valStr << ((i>0) ? " " : "") << valDbl[i];
                strncpy(static_cast<char *>(outValues), valStr.str().c_str(), outValueMaxCount);
                i = (unsigned int)valStr.str().size();
                if( i>outValueMaxCount-1 )
                    i = outValueMaxCount-1;
                (static_cast<char *>(outValues))[i] = '\0';
                return 1; // always returns 1 for CSTRING 
            default:
                g_TwMgr->SetLastError(g_ErrBadParam); // Unknown param value type
                retType = RET_ERROR;
            }
        }
        else if( retType==RET_STRING ) 
        {
            if( paramValueType == TW_PARAM_CSTRING )
            {
                strncpy(static_cast<char *>(outValues), valStr.str().c_str(), outValueMaxCount);
                i = (unsigned int)valStr.str().size();
                if( i>outValueMaxCount-1 )
                    i = outValueMaxCount-1;
                (static_cast<char *>(outValues))[i] = '\0';
                return 1; // always returns 1 for CSTRING 
            }
            else 
            {
                g_TwMgr->SetLastError(g_ErrBadType); // string cannot be converted to int or double
                retType = RET_ERROR;
            }
        }

        if( retType==RET_ERROR )
        {
            bool errMsg = (g_TwMgr->CheckLastError()!=NULL && strlen(g_TwMgr->CheckLastError())>0 && PrevLastErrorPtr!=g_TwMgr->CheckLastError());
            _snprintf(g_ErrParse, sizeof(g_ErrParse), "Unable to get param '%s%s%s %s' %s%s",
                      (bar==TW_GLOBAL_BAR) ? "GLOBAL" : bar->m_Name.c_str(), (var!=NULL) ? "/" : "", 
                      (var!=NULL) ? varName : "", paramName, errMsg ? " : " : "", 
                      errMsg ? g_TwMgr->CheckLastError() : "");
            g_ErrParse[sizeof(g_ErrParse)-1] = '\0';
            g_TwMgr->SetLastError(g_ErrParse);
        }
        return retType;
    }
    else
    {
        _snprintf(g_ErrParse, sizeof(g_ErrParse), "Unknown param '%s%s%s %s'", 
                  (bar==TW_GLOBAL_BAR) ? "GLOBAL" : bar->m_Name.c_str(), 
                  (var!=NULL) ? "/" : "", (var!=NULL) ? varName : "", paramName);
        g_ErrParse[sizeof(g_ErrParse)-1] = '\0';
        g_TwMgr->SetLastError(g_ErrParse);
        return 0;
    }
}


int ANT_CALL TwSetParam(TwBar *bar, const char *varName, const char *paramName, TwParamValueType paramValueType, unsigned int inValueCount, const void *inValues)
{
    CTwFPU fpu; // force fpu precision

    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return 0; // not initialized
    }
    if( paramName==NULL || strlen(paramName)<=0 )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return 0;
    }
    if( inValueCount>0 && inValues==NULL )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return 0;
    }

    TwFreeAsyncDrawing(); // For multi-thread savety

    if( bar==NULL ) 
        bar = TW_GLOBAL_BAR;
    else
    {
        vector<TwBar*>::iterator barIt;
        int i = 0;
        for( barIt=g_TwMgr->m_Bars.begin(); barIt!=g_TwMgr->m_Bars.end(); ++barIt, ++i )
            if( (*barIt)==bar )
                break;
        if( barIt==g_TwMgr->m_Bars.end() )
        {
            g_TwMgr->SetLastError(g_ErrNotFound);
            return 0;
        }
    }
    CTwVarGroup *varParent = NULL;
    int varIndex = -1;
    CTwVar *var = NULL;
    if( varName!=NULL && strlen(varName)>0 )
    {
        var = bar->Find(varName, &varParent, &varIndex);
        if( var==NULL )
        {
            _snprintf(g_ErrParse, sizeof(g_ErrParse), "Unknown var '%s/%s'", 
                      (bar==TW_GLOBAL_BAR) ? "GLOBAL" : bar->m_Name.c_str(), varName);
            g_ErrParse[sizeof(g_ErrParse)-1] = '\0';
            g_TwMgr->SetLastError(g_ErrParse);
            return 0;
        }
    }

    bool hasValue = false;
    int paramID = BarVarHasAttrib(bar, var, paramName, &hasValue);
    if( paramID>0 )
    {
        int ret = 0;
        const char *PrevLastErrorPtr = g_TwMgr->CheckLastError();
        if( hasValue ) 
        {
            std::ostringstream valuesStr;
            unsigned int i;
            switch( paramValueType ) 
            {
            case TW_PARAM_INT32:
                for( i=0; i<inValueCount; i++ )
                    valuesStr << (static_cast<const int *>(inValues))[i] << ((i<inValueCount-1) ? " " : "");
                break;
            case TW_PARAM_FLOAT:
                for( i=0; i<inValueCount; i++ )
                    valuesStr << (static_cast<const float *>(inValues))[i] << ((i<inValueCount-1) ? " " : "");
                break;
            case TW_PARAM_DOUBLE:
                for( i=0; i<inValueCount; i++ )
                    valuesStr << (static_cast<const double *>(inValues))[i] << ((i<inValueCount-1) ? " " : "");
                break;
            case TW_PARAM_CSTRING:
                /*
                for( i=0; i<inValueCount; i++ )
                {
                    valuesStr << '`';
                    const char *str = (static_cast<char * const *>(inValues))[i];
                    for( const char *ch = str; *ch!=0; ch++ )
                        if( *ch=='`' )
                            valuesStr << "`'`'`";
                        else
                            valuesStr << *ch;
                    valuesStr << "` ";
                }
                */
                if( inValueCount!=1 )
                {
                    g_TwMgr->SetLastError(g_ErrCStrParam); // count for CString param must be 1
                    return 0;
                }
                else
                    valuesStr << static_cast<const char *>(inValues);
                break;
            default:
                g_TwMgr->SetLastError(g_ErrBadParam); // Unknown param value type
                return 0;
            }
            ret = BarVarSetAttrib(bar, var, varParent, varIndex, paramID, valuesStr.str().c_str());
        }
        else
            ret = BarVarSetAttrib(bar, var, varParent, varIndex, paramID, NULL);
        if( ret==0 )
        {
            bool errMsg = (g_TwMgr->CheckLastError()!=NULL && strlen(g_TwMgr->CheckLastError())>0 && PrevLastErrorPtr!=g_TwMgr->CheckLastError());
            _snprintf(g_ErrParse, sizeof(g_ErrParse), "Unable to set param '%s%s%s %s' %s%s",
                      (bar==TW_GLOBAL_BAR) ? "GLOBAL" : bar->m_Name.c_str(), (var!=NULL) ? "/" : "", 
                      (var!=NULL) ? varName : "", paramName, errMsg ? " : " : "", 
                      errMsg ? g_TwMgr->CheckLastError() : "");
            g_ErrParse[sizeof(g_ErrParse)-1] = '\0';
            g_TwMgr->SetLastError(g_ErrParse);
        }
        return ret;
    } 
    else
    {
        _snprintf(g_ErrParse, sizeof(g_ErrParse), "Unknown param '%s%s%s %s'", 
                  (bar==TW_GLOBAL_BAR) ? "GLOBAL" : bar->m_Name.c_str(), 
                  (var!=NULL) ? "/" : "", (var!=NULL) ? varName : "", paramName);
        g_ErrParse[sizeof(g_ErrParse)-1] = '\0';
        g_TwMgr->SetLastError(g_ErrParse);
        return 0;
    }
}

//  ---------------------------------------------------------------------------

static int s_PassProxy = 0;
void *CTwMgr::CStruct::s_PassProxyAsClientData = &s_PassProxy;  // special tag

CTwMgr::CStructProxy::CStructProxy()
{ 
    memset(this, 0, sizeof(*this)); 
}

CTwMgr::CStructProxy::~CStructProxy() 
{ 
    if( m_StructData!=NULL && m_DeleteStructData )
    {
        //if( m_StructExtData==NULL && g_TwMgr!=NULL && m_Type>=TW_TYPE_STRUCT_BASE && m_Type<TW_TYPE_STRUCT_BASE+(int)g_TwMgr->m_Structs.size() )
        //  g_TwMgr->UninitVarData(m_Type, m_StructData, g_TwMgr->m_Structs[m_Type-TW_TYPE_STRUCT_BASE].m_Size);
        delete[] (char*)m_StructData;
    }
    if( m_StructExtData!=NULL )
    {
        //if( g_TwMgr!=NULL && m_Type>=TW_TYPE_STRUCT_BASE && m_Type<TW_TYPE_STRUCT_BASE+(int)g_TwMgr->m_Structs.size() )
        //  g_TwMgr->UninitVarData(m_Type, m_StructExtData, g_TwMgr->m_Structs[m_Type-TW_TYPE_STRUCT_BASE].m_Size);
        delete[] (char*)m_StructExtData;
    }
    memset(this, 0, sizeof(*this));
}

/*
void CTwMgr::InitVarData(TwType _Type, void *_Data, size_t _Size)
{
    if( _Data!=NULL )
    {
        if( _Type>=TW_TYPE_STRUCT_BASE && _Type<TW_TYPE_STRUCT_BASE+(int)m_Structs.size() )
        {
            CTwMgr::CStruct& s = m_Structs[_Type-TW_TYPE_STRUCT_BASE];
            for( size_t i=0; i<s.m_Members.size(); ++i )
            {
                CTwMgr::CStructMember& sm = s.m_Members[i];
                assert( sm.m_Offset+sm.m_Size<=_Size );
                InitVarData(sm.m_Type, (char *)_Data + sm.m_Offset, sm.m_Size);
            }
        }
        else if( _Type==TW_TYPE_STDSTRING )
            ::new(_Data) std::string;
        else
            memset(_Data, 0, _Size);
    }
}

void CTwMgr::UninitVarData(TwType _Type, void *_Data, size_t _Size)
{
    if( _Data!=NULL )
    {
        if( _Type>=TW_TYPE_STRUCT_BASE && _Type<TW_TYPE_STRUCT_BASE+(int)m_Structs.size() )
        {
            CTwMgr::CStruct& s = m_Structs[_Type-TW_TYPE_STRUCT_BASE];
            for( size_t i=0; i<s.m_Members.size(); ++i )
            {
                CTwMgr::CStructMember& sm = s.m_Members[i];
                assert( sm.m_Offset+sm.m_Size<=_Size );
                UninitVarData(sm.m_Type, (char *)_Data + sm.m_Offset, sm.m_Size);
            }
        }
        else if( _Type==TW_TYPE_STDSTRING )
        {
            std::string *Str = (std::string *)_Data;
            Str->~string();
            memset(_Data, 0, _Size);
        }
        else
            memset(_Data, 0, _Size);
    }
}
*/

void CTwMgr::UnrollCDStdString(std::vector<CCDStdStringRecord>& _Records, TwType _Type, void *_Data)
{
    if( _Data!=NULL )
    {
        if( _Type>=TW_TYPE_STRUCT_BASE && _Type<TW_TYPE_STRUCT_BASE+(int)m_Structs.size() )
        {
            CTwMgr::CStruct& s = m_Structs[_Type-TW_TYPE_STRUCT_BASE];
            if( !s.m_IsExt )
                for( size_t i=0; i<s.m_Members.size(); ++i )
                {
                    CTwMgr::CStructMember& sm = s.m_Members[i];
                    UnrollCDStdString(_Records, sm.m_Type, (char *)_Data + sm.m_Offset);
                }
            else
            {
                // nothing:
                // Ext struct cannot have var of type TW_TYPE_CDSTDSTRING (converted from TW_TYPE_STDSTRING)
            }
        }
        else if( _Type==TW_TYPE_STDSTRING || _Type==TW_TYPE_CDSTDSTRING )
        {
            _Records.push_back(CCDStdStringRecord());
            CCDStdStringRecord& Rec = _Records.back();
            Rec.m_DataPtr = _Data;
            memcpy(Rec.m_PrevValue, _Data, m_ClientStdStringStructSize);
            const char *Str = *(const char **)_Data;
            if( Str!=NULL )
                // Rec.m_StdString = Str;
                Rec.m_ClientStdString.FromLib(Str);
            memcpy(Rec.m_DataPtr, &(Rec.m_ClientStdString.ToClient()), sizeof(std::string));
        }
    }
}

void CTwMgr::RestoreCDStdString(const std::vector<CCDStdStringRecord>& _Records)
{
    for( size_t i=0; i<_Records.size(); ++i )
        memcpy(_Records[i].m_DataPtr, _Records[i].m_PrevValue, m_ClientStdStringStructSize);
}

CTwMgr::CMemberProxy::CMemberProxy() 
{ 
    memset(this, 0, sizeof(*this)); 
}

CTwMgr::CMemberProxy::~CMemberProxy() 
{ 
    memset(this, 0, sizeof(*this)); 
}

void ANT_CALL CTwMgr::CMemberProxy::SetCB(const void *_Value, void *_ClientData)
{
    if( _ClientData && _Value )
    {
        const CMemberProxy *mProxy = static_cast<const CMemberProxy *>(_ClientData);
        if( g_TwMgr && mProxy )
        {
            const CStructProxy *sProxy = mProxy->m_StructProxy;
            if( sProxy && sProxy->m_StructData && sProxy->m_Type>=TW_TYPE_STRUCT_BASE && sProxy->m_Type<TW_TYPE_STRUCT_BASE+(int)g_TwMgr->m_Structs.size() )
            {
                CTwMgr::CStruct& s = g_TwMgr->m_Structs[sProxy->m_Type-TW_TYPE_STRUCT_BASE];
                if( mProxy->m_MemberIndex>=0 && mProxy->m_MemberIndex<(int)s.m_Members.size() )
                {
                    CTwMgr::CStructMember& m = s.m_Members[mProxy->m_MemberIndex];
                    if( m.m_Size>0 && m.m_Type!=TW_TYPE_BUTTON )
                    {
                        if( s.m_IsExt )
                        {
                            memcpy((char *)sProxy->m_StructExtData + m.m_Offset, _Value, m.m_Size);
                            if( s.m_CopyVarFromExtCallback && sProxy->m_StructExtData )
                                s.m_CopyVarFromExtCallback(sProxy->m_StructData, sProxy->m_StructExtData, mProxy->m_MemberIndex, (s.m_ExtClientData==s.s_PassProxyAsClientData) ? _ClientData : s.m_ExtClientData);
                        }
                        else
                            memcpy((char *)sProxy->m_StructData + m.m_Offset, _Value, m.m_Size);
                        if( sProxy->m_StructSetCallback )
                        {
                            g_TwMgr->m_CDStdStringRecords.resize(0);
                            g_TwMgr->UnrollCDStdString(g_TwMgr->m_CDStdStringRecords, sProxy->m_Type, sProxy->m_StructData);
                            sProxy->m_StructSetCallback(sProxy->m_StructData, sProxy->m_StructClientData);
                            g_TwMgr->RestoreCDStdString(g_TwMgr->m_CDStdStringRecords);
                        }
                    }
                }
            }
        }
    }
}

void ANT_CALL CTwMgr::CMemberProxy::GetCB(void *_Value, void *_ClientData)
{
    if( _ClientData && _Value )
    {
        const CMemberProxy *mProxy = static_cast<const CMemberProxy *>(_ClientData);
        if( g_TwMgr && mProxy )
        {
            const CStructProxy *sProxy = mProxy->m_StructProxy;
            if( sProxy && sProxy->m_StructData && sProxy->m_Type>=TW_TYPE_STRUCT_BASE && sProxy->m_Type<TW_TYPE_STRUCT_BASE+(int)g_TwMgr->m_Structs.size() )
            {
                CTwMgr::CStruct& s = g_TwMgr->m_Structs[sProxy->m_Type-TW_TYPE_STRUCT_BASE];
                if( mProxy->m_MemberIndex>=0 && mProxy->m_MemberIndex<(int)s.m_Members.size() )
                {
                    CTwMgr::CStructMember& m = s.m_Members[mProxy->m_MemberIndex];
                    if( m.m_Size>0 && m.m_Type!=TW_TYPE_BUTTON )
                    {
                        if( sProxy->m_StructGetCallback )
                            sProxy->m_StructGetCallback(sProxy->m_StructData, sProxy->m_StructClientData);
                        if( s.m_IsExt )
                        {
                            if( s.m_CopyVarToExtCallback && sProxy->m_StructExtData )
                                s.m_CopyVarToExtCallback(sProxy->m_StructData, sProxy->m_StructExtData, mProxy->m_MemberIndex,  (s.m_ExtClientData==s.s_PassProxyAsClientData) ? _ClientData : s.m_ExtClientData);
                            memcpy(_Value, (char *)sProxy->m_StructExtData + m.m_Offset, m.m_Size);
                        }
                        else
                            memcpy(_Value, (char *)sProxy->m_StructData + m.m_Offset, m.m_Size);
                    }
                }
            }
        }
    }
}

//  ---------------------------------------------------------------------------

void ANT_CALL CTwMgr::CCDStdString::SetCB(const void *_Value, void *_ClientData)
{
    if( _Value==NULL || _ClientData==NULL || g_TwMgr==NULL )
        return;
    CTwMgr::CCDStdString *CDStdString = (CTwMgr::CCDStdString *)_ClientData;
    const char *SrcStr = *(const char **)_Value;
    if( SrcStr==NULL )
    {
        static char s_EmptyString[] = "";
        SrcStr = s_EmptyString;
    }
    if( CDStdString->m_ClientSetCallback==NULL )
    {
        if( g_TwMgr->m_CopyStdStringToClient && CDStdString->m_ClientStdStringPtr!=NULL ) 
        {
            CTwMgr::CClientStdString clientSrcStr; // convert VC++ Release/Debug std::string
            clientSrcStr.FromLib(SrcStr);
            g_TwMgr->m_CopyStdStringToClient(*(CDStdString->m_ClientStdStringPtr), clientSrcStr.ToClient());
        }
    }
    else
    {
        if( CDStdString->m_ClientSetCallback==CMemberProxy::SetCB )
            CDStdString->m_ClientSetCallback(&SrcStr, CDStdString->m_ClientData);
        else
        {
            CTwMgr::CClientStdString clientSrcStr; // convert VC++ Release/Debug std::string
            clientSrcStr.FromLib(SrcStr);
            std::string& ValStr = clientSrcStr.ToClient();
            CDStdString->m_ClientSetCallback(&ValStr, CDStdString->m_ClientData);
        }
    }
}

void ANT_CALL CTwMgr::CCDStdString::GetCB(void *_Value, void *_ClientData)
{
    if( _Value==NULL || _ClientData==NULL || g_TwMgr==NULL )
        return;
    CTwMgr::CCDStdString *CDStdString = (CTwMgr::CCDStdString *)_ClientData;
    char **DstStrPtr = (char **)_Value;
    if( CDStdString->m_ClientGetCallback==NULL )
    {
        if( CDStdString->m_ClientStdStringPtr!=NULL )
        {
            //*DstStrPtr = const_cast<char *>(CDStdString->m_ClientStdStringPtr->c_str());
            static CTwMgr::CLibStdString s_LibStr; // static because it will be used as a returned value
            s_LibStr.FromClient(*CDStdString->m_ClientStdStringPtr);
            *DstStrPtr = const_cast<char *>(s_LibStr.ToLib().c_str());
        }
        else
        {
            static char s_EmptyString[] = "";
            *DstStrPtr = s_EmptyString;
        }
    }
    else
    {
        // m_ClientGetCallback uses TwCopyStdStringToLibrary to copy string
        // and TwCopyStdStringToLibrary does the VC++ Debug/Release std::string conversion.
        CDStdString->m_ClientGetCallback(&(CDStdString->m_LocalString[0]), CDStdString->m_ClientData);
        //*DstStrPtr = const_cast<char *>(CDStdString->m_LocalString.c_str());
        char **StrPtr = (char **)&(CDStdString->m_LocalString[0]);
        *DstStrPtr = *StrPtr;
    }
}

//  ---------------------------------------------------------------------------

static int s_SeparatorTag = 0;

//  ---------------------------------------------------------------------------

static int AddVar(TwBar *_Bar, const char *_Name, ETwType _Type, void *_VarPtr, bool _ReadOnly, TwSetVarCallback _SetCallback, TwGetVarCallback _GetCallback, TwButtonCallback _ButtonCallback, void *_ClientData, const char *_Def)
{
    CTwFPU fpu; // force fpu precision

    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return 0; // not initialized
    }

    char unnamedVarName[64];
    if( _Name==NULL || strlen(_Name)==0 ) // create a name automatically
    {
        static unsigned int s_UnnamedVarCount = 0;
        _snprintf(unnamedVarName, sizeof(unnamedVarName), "TW_UNNAMED_%04X", s_UnnamedVarCount);
        _Name = unnamedVarName;
        ++s_UnnamedVarCount;
    }

    if( _Bar==NULL || _Name==NULL || strlen(_Name)==0 || (_VarPtr==NULL && _GetCallback==NULL && _Type!=TW_TYPE_BUTTON) )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return 0;
    }
    if( _Bar->Find(_Name)!=NULL )
    {
        g_TwMgr->SetLastError(g_ErrExist);
        return 0;
    }

    if( strstr(_Name, "`")!=NULL )
    {
        g_TwMgr->SetLastError(g_ErrNoBackQuote);
        return 0;
    }

    if( _VarPtr==NULL && _Type!=TW_TYPE_BUTTON && _GetCallback!=NULL && _SetCallback==NULL )
        _ReadOnly = true;   // force readonly in this case

    // Convert color types
    if( _Type==TW_TYPE_COLOR32 )
        _Type = g_TwMgr->m_TypeColor32;
    else if( _Type==TW_TYPE_COLOR3F )
        _Type = g_TwMgr->m_TypeColor3F;
    else if( _Type==TW_TYPE_COLOR4F )
        _Type = g_TwMgr->m_TypeColor4F;

    // Convert rotation types
    if( _Type==TW_TYPE_QUAT4F )
        _Type = g_TwMgr->m_TypeQuat4F;
    else if( _Type==TW_TYPE_QUAT4D )
        _Type = g_TwMgr->m_TypeQuat4D;
    else if( _Type==TW_TYPE_DIR3F )
        _Type = g_TwMgr->m_TypeDir3F;
    else if( _Type==TW_TYPE_DIR3D )
        _Type = g_TwMgr->m_TypeDir3D;

    // VC++ uses a different definition of std::string in Debug and Release modes.
    // sizeof(std::string) is encoded in TW_TYPE_STDSTRING to overcome this issue.
    // With VS2010 the binary representation of std::string has changed too. This is
    // also detected here.
    if( (_Type&0xffff0000)==(TW_TYPE_STDSTRING&0xffff0000) || (_Type&0xffff0000)==TW_TYPE_STDSTRING_VS2010 || (_Type&0xffff0000)==TW_TYPE_STDSTRING_VS2008 )
    {
        if( g_TwMgr->m_ClientStdStringBaseType==0 )
            g_TwMgr->m_ClientStdStringBaseType = (TwType)(_Type&0xffff0000);

        size_t clientStdStringStructSize = (_Type&0xffff);
        if( g_TwMgr->m_ClientStdStringStructSize==0 )
            g_TwMgr->m_ClientStdStringStructSize = clientStdStringStructSize;
        int diff = abs((int)g_TwMgr->m_ClientStdStringStructSize - (int)sizeof(std::string));
        if( g_TwMgr->m_ClientStdStringStructSize!=clientStdStringStructSize || g_TwMgr->m_ClientStdStringStructSize==0 
            || (diff!=0 && diff!=sizeof(void*)))
        {
            g_TwMgr->SetLastError(g_ErrStdString);
            return 0;
        }

        _Type = TW_TYPE_STDSTRING; // force type to be our TW_TYPE_STDSTRING
    }

    if( _Type==TW_TYPE_STDSTRING )
    {
        g_TwMgr->m_CDStdStrings.push_back(CTwMgr::CCDStdString());
        CTwMgr::CCDStdString& CDStdString = g_TwMgr->m_CDStdStrings.back();
        CDStdString.m_ClientStdStringPtr = (std::string *)_VarPtr;
        CDStdString.m_ClientSetCallback = _SetCallback;
        CDStdString.m_ClientGetCallback = _GetCallback;
        CDStdString.m_ClientData = _ClientData;
        //CDStdString.m_This = g_TwMgr->m_CDStdStrings.end();
        //--CDStdString.m_This;
        TwGetVarCallback GetCB = CTwMgr::CCDStdString::GetCB;
        TwSetVarCallback SetCB = CTwMgr::CCDStdString::SetCB;
        if( _VarPtr==NULL && _SetCallback==NULL )
            SetCB = NULL;
        if( _VarPtr==NULL && _GetCallback==NULL )
            GetCB = NULL;
        return AddVar(_Bar, _Name, TW_TYPE_CDSTDSTRING, NULL, _ReadOnly, SetCB, GetCB, NULL, &CDStdString, _Def);
    }
    else if(    (_Type>TW_TYPE_UNDEF && _Type<TW_TYPE_STRUCT_BASE)
             || (_Type>=TW_TYPE_ENUM_BASE && _Type<TW_TYPE_ENUM_BASE+(int)g_TwMgr->m_Enums.size()) 
             || (_Type>TW_TYPE_CSSTRING_BASE && _Type<=TW_TYPE_CSSTRING_MAX)
             || _Type==TW_TYPE_CDSTDSTRING 
             || IsCustomType(_Type) ) // (_Type>=TW_TYPE_CUSTOM_BASE && _Type<TW_TYPE_CUSTOM_BASE+(int)g_TwMgr->m_Customs.size()) )
    {
        CTwVarAtom *Var = new CTwVarAtom;
        Var->m_Name = _Name;
        Var->m_Ptr = _VarPtr;
        Var->m_Type = _Type;
        Var->m_ColorPtr = &(_Bar->m_ColLabelText);
        if( _VarPtr!=NULL )
        {
            assert( _GetCallback==NULL && _SetCallback==NULL && _ButtonCallback==NULL );

            Var->m_ReadOnly = _ReadOnly;
            Var->m_GetCallback = NULL;
            Var->m_SetCallback = NULL;
            Var->m_ClientData = NULL;
        }
        else
        {
            assert( _GetCallback!=NULL || _Type==TW_TYPE_BUTTON );

            Var->m_GetCallback = _GetCallback;
            Var->m_SetCallback = _SetCallback;
            Var->m_ClientData = _ClientData;
            if( _Type==TW_TYPE_BUTTON )
            {
                Var->m_Val.m_Button.m_Callback = _ButtonCallback;
                if( _ButtonCallback==NULL && _ClientData==&s_SeparatorTag )
                {
                    Var->m_Val.m_Button.m_Separator = 1;
                    Var->m_Label = " ";
                }
                else if( _ButtonCallback==NULL )
                    Var->m_ColorPtr = &(_Bar->m_ColStaticText);
            }
            if( _Type!=TW_TYPE_BUTTON )
                Var->m_ReadOnly = (_SetCallback==NULL || _ReadOnly);
            else
                Var->m_ReadOnly = (_ButtonCallback==NULL);
        }
        Var->SetDefaults();

        if( IsCustomType(_Type) ) // _Type>=TW_TYPE_CUSTOM_BASE && _Type<TW_TYPE_CUSTOM_BASE+(int)g_TwMgr->m_Customs.size() )
        {
            if( Var->m_GetCallback==CTwMgr::CMemberProxy::GetCB && Var->m_SetCallback==CTwMgr::CMemberProxy::SetCB )
                Var->m_Val.m_Custom.m_MemberProxy = static_cast<CTwMgr::CMemberProxy *>(Var->m_ClientData);
            else
                Var->m_Val.m_Custom.m_MemberProxy = NULL;
        }

        _Bar->m_VarRoot.m_Vars.push_back(Var);
        _Bar->NotUpToDate();
        g_TwMgr->m_HelpBarNotUpToDate = true;

        if( _Def!=NULL && strlen(_Def)>0 )
        {
            string d = '`' + _Bar->m_Name + "`/`" + _Name + "` " + _Def;
            return TwDefine(d.c_str());
        }
        else
            return 1;
    }
    else if(_Type>=TW_TYPE_STRUCT_BASE && _Type<TW_TYPE_STRUCT_BASE+(TwType)g_TwMgr->m_Structs.size())
    {
        CTwMgr::CStruct& s = g_TwMgr->m_Structs[_Type-TW_TYPE_STRUCT_BASE];
        CTwMgr::CStructProxy *sProxy = NULL;
        void *vPtr;
        if( !s.m_IsExt )
        {
            if( _VarPtr!=NULL )
                vPtr = _VarPtr;
            else
            {
                assert( _GetCallback!=NULL || _SetCallback!=NULL );
                assert( s.m_Size>0 );
                vPtr = new char[s.m_Size];
                memset(vPtr, 0, s.m_Size);
                // create a new StructProxy
                g_TwMgr->m_StructProxies.push_back(CTwMgr::CStructProxy());
                sProxy = &(g_TwMgr->m_StructProxies.back());
                sProxy->m_Type = _Type;
                sProxy->m_StructData = vPtr;
                sProxy->m_DeleteStructData = true;
                sProxy->m_StructSetCallback = _SetCallback;
                sProxy->m_StructGetCallback = _GetCallback;
                sProxy->m_StructClientData = _ClientData;
                sProxy->m_CustomDrawCallback = NULL;
                sProxy->m_CustomMouseButtonCallback = NULL;
                sProxy->m_CustomMouseMotionCallback = NULL;
                sProxy->m_CustomMouseLeaveCallback = NULL;
                sProxy->m_CustomCaptureFocus = false;
                sProxy->m_CustomIndexFirst = -1;
                sProxy->m_CustomIndexLast = -1;
                //g_TwMgr->InitVarData(sProxy->m_Type, sProxy->m_StructData, s.m_Size);
            }
        }
        else // s.m_IsExt
        {
            assert( s.m_Size>0 && s.m_ClientStructSize>0 );
            vPtr = new char[s.m_Size];  // will be m_StructExtData
            memset(vPtr, 0, s.m_Size);
            // create a new StructProxy
            g_TwMgr->m_StructProxies.push_back(CTwMgr::CStructProxy());
            sProxy = &(g_TwMgr->m_StructProxies.back());
            sProxy->m_Type = _Type;
            sProxy->m_StructExtData = vPtr;
            sProxy->m_StructSetCallback = _SetCallback;
            sProxy->m_StructGetCallback = _GetCallback;
            sProxy->m_StructClientData = _ClientData;
            sProxy->m_CustomDrawCallback = NULL;
            sProxy->m_CustomMouseButtonCallback = NULL;
            sProxy->m_CustomMouseMotionCallback = NULL;
            sProxy->m_CustomMouseLeaveCallback = NULL;
            sProxy->m_CustomCaptureFocus = false;
            sProxy->m_CustomIndexFirst = -1;
            sProxy->m_CustomIndexLast = -1;
            //g_TwMgr->InitVarData(sProxy->m_Type, sProxy->m_StructExtData, s.m_Size);
            if( _VarPtr!=NULL )
            {
                sProxy->m_StructData = _VarPtr;
                sProxy->m_DeleteStructData = false;
            }
            else
            {
                sProxy->m_StructData = new char[s.m_ClientStructSize];
                memset(sProxy->m_StructData, 0, s.m_ClientStructSize);
                sProxy->m_DeleteStructData = true;
                //g_TwMgr->InitVarData(ClientStructType, sProxy->m_StructData, s.m_ClientStructSize); //ClientStructType is unknown
            }
            _VarPtr = NULL; // force use of TwAddVarCB for members

            // init m_StructExtdata
            if( s.m_ExtClientData==CTwMgr::CStruct::s_PassProxyAsClientData )
                s.m_StructExtInitCallback(sProxy->m_StructExtData, sProxy);
            else
                s.m_StructExtInitCallback(sProxy->m_StructExtData, s.m_ExtClientData);
        }

        for( int i=0; i<(int)s.m_Members.size(); ++i )
        {
            CTwMgr::CStructMember& m = s.m_Members[i];
            string name = string(_Name) + '.' + m.m_Name;
            const char *access = "";
            if( _ReadOnly )
                access = "readonly ";
            string def  = "label=`" + m.m_Name + "` group=`" + _Name + "` " + access; // + m.m_DefString;  // member def must be done after group def
            if( _VarPtr!=NULL )
            {
                if( TwAddVarRW(_Bar, name.c_str(), m.m_Type, (char*)vPtr+m.m_Offset, def.c_str())==0 )
                    return 0;
            }
            else
            {
                assert( sProxy!=NULL );
                // create a new MemberProxy
                g_TwMgr->m_MemberProxies.push_back(CTwMgr::CMemberProxy());
                CTwMgr::CMemberProxy& mProxy = g_TwMgr->m_MemberProxies.back();
                mProxy.m_StructProxy = sProxy;
                mProxy.m_MemberIndex = i;
                assert( !(s.m_IsExt && (m.m_Type==TW_TYPE_STDSTRING || m.m_Type==TW_TYPE_CDSTDSTRING)) );   // forbidden because this case is not handled by UnrollCDStdString
                if( TwAddVarCB(_Bar, name.c_str(), m.m_Type, CTwMgr::CMemberProxy::SetCB, CTwMgr::CMemberProxy::GetCB, &mProxy, def.c_str())==0 )
                    return 0;
                mProxy.m_Var = _Bar->Find(name.c_str(), &mProxy.m_VarParent, NULL);
                mProxy.m_Bar = _Bar;
            }

            if( sProxy!=NULL && IsCustomType(m.m_Type) ) // m.m_Type>=TW_TYPE_CUSTOM_BASE && m.m_Type<TW_TYPE_CUSTOM_BASE+(int)g_TwMgr->m_Customs.size() )
            {
                if( sProxy->m_CustomIndexFirst<0 )
                    sProxy->m_CustomIndexFirst = sProxy->m_CustomIndexLast = i;
                else
                    sProxy->m_CustomIndexLast = i;
            }
        }
        char structInfo[64];
        sprintf(structInfo, "typeid=%d valptr=%p close ", _Type, vPtr);
        string grpDef = '`' + _Bar->m_Name + "`/`" + _Name + "` " + structInfo;
        if( _Def!=NULL && strlen(_Def)>0 )
            grpDef += _Def;
        int ret = TwDefine(grpDef.c_str());
        for( int i=0; i<(int)s.m_Members.size(); ++i ) // members must be defined even if grpDef has error
        {
            CTwMgr::CStructMember& m = s.m_Members[i];
            if( m.m_DefString.length()>0 )
            {
                string memberDef = '`' + _Bar->m_Name + "`/`" + _Name + '.' + m.m_Name + "` " + m.m_DefString;
                if( !TwDefine(memberDef.c_str()) ) // all members must be defined even if memberDef has error
                    ret = 0;
            }
        }
        return ret;
    }
    else
    {
        if( _Type==TW_TYPE_CSSTRING_BASE )
            g_TwMgr->SetLastError(g_ErrBadSize); // static string of size null
        else
            g_TwMgr->SetLastError(g_ErrNotFound);
        return 0;
    }
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwAddVarRW(TwBar *_Bar, const char *_Name, ETwType _Type, void *_Var, const char *_Def)
{
    return AddVar(_Bar, _Name, _Type, _Var, false, NULL, NULL, NULL, NULL, _Def);
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwAddVarRO(TwBar *_Bar, const char *_Name, ETwType _Type, const void *_Var, const char *_Def)
{
    return AddVar(_Bar, _Name, _Type, const_cast<void *>(_Var), true, NULL, NULL, NULL, NULL, _Def);
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwAddVarCB(TwBar *_Bar, const char *_Name, ETwType _Type, TwSetVarCallback _SetCallback, TwGetVarCallback _GetCallback, void *_ClientData, const char *_Def)
{
    return AddVar(_Bar, _Name, _Type, NULL, false, _SetCallback, _GetCallback, NULL, _ClientData, _Def);
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwAddButton(TwBar *_Bar, const char *_Name, TwButtonCallback _Callback, void *_ClientData, const char *_Def)
{
    return AddVar(_Bar, _Name, TW_TYPE_BUTTON, NULL, false, NULL, NULL, _Callback, _ClientData, _Def);
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwAddSeparator(TwBar *_Bar, const char *_Name, const char *_Def)
{
    return AddVar(_Bar, _Name, TW_TYPE_BUTTON, NULL, true, NULL, NULL, NULL, &s_SeparatorTag, _Def);
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwRemoveVar(TwBar *_Bar, const char *_Name)
{
    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return 0; // not initialized
    }
    if( _Bar==NULL || _Name==NULL || strlen(_Name)==0 )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return 0;
    }

    if( g_TwMgr->m_PopupBar!=NULL && _Bar!=g_TwMgr->m_PopupBar )    // delete popup bar first if it exists
    {
        TwDeleteBar(g_TwMgr->m_PopupBar);
        g_TwMgr->m_PopupBar = NULL;
    }

    _Bar->StopEditInPlace();    // desactivate EditInPlace

    CTwVarGroup *Parent = NULL;
    int Index = -1;
    CTwVar *Var = _Bar->Find(_Name, &Parent, &Index);
    if( Var!=NULL && Parent!=NULL && Index>=0 )
    {
        if( Parent->m_StructValuePtr!=NULL )
        {
            g_TwMgr->SetLastError(g_ErrDelStruct);
            return 0;
        }

        delete Var;
        Parent->m_Vars.erase(Parent->m_Vars.begin()+Index);
        if( Parent!=&(_Bar->m_VarRoot) && Parent->m_Vars.size()<=0 )
            TwRemoveVar(_Bar, Parent->m_Name.c_str());
        _Bar->NotUpToDate();
        if( _Bar!=g_TwMgr->m_HelpBar )
            g_TwMgr->m_HelpBarNotUpToDate = true;
        return 1;
    }

    g_TwMgr->SetLastError(g_ErrNotFound);
    return 0;
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwRemoveAllVars(TwBar *_Bar)
{
    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return 0; // not initialized
    }
    if( _Bar==NULL )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return 0;
    }

    if( g_TwMgr->m_PopupBar!=NULL && _Bar!=g_TwMgr->m_PopupBar && _Bar!=g_TwMgr->m_HelpBar )    // delete popup bar first if it exists
    {
        TwDeleteBar(g_TwMgr->m_PopupBar);
        g_TwMgr->m_PopupBar = NULL;
    }

    _Bar->StopEditInPlace();    // desactivate EditInPlace

    for( vector<CTwVar*>::iterator it=_Bar->m_VarRoot.m_Vars.begin(); it!=_Bar->m_VarRoot.m_Vars.end(); ++it )
        if( *it != NULL )
        {
            delete *it;
            *it = NULL;
        }
    _Bar->m_VarRoot.m_Vars.resize(0);
    _Bar->NotUpToDate();
    g_TwMgr->m_HelpBarNotUpToDate = true;
    return 1;
}

//  ---------------------------------------------------------------------------

int ParseToken(string& _Token, const char *_Def, int& Line, int& Column, bool _KeepQuotes, bool _EndCR, char _Sep1='\0', char _Sep2='\0')
{
    const char *Cur = _Def;
    _Token = "";
    // skip spaces
    while( *Cur==' ' || *Cur=='\t' || *Cur=='\r' || *Cur=='\n' )
    {
        if( *Cur=='\n' && _EndCR )
            return (int)(Cur-_Def); // a CR has been found
        ++Cur;
        if( *Cur=='\n' )
        {
            ++Line;
            Column = 1;
        }
        else if( *Cur=='\t' )
            Column += g_TabLength;
        else if( *Cur!='\r' )
            ++Column;
    }
    // read token
    int QuoteLine=0, QuoteColumn=0;
    const char *QuoteCur;
    char Quote = 0;
    bool AddChar;
    bool LineJustIncremented = false;
    while(    (Quote==0 && (*Cur!='\0' && *Cur!=' ' && *Cur!='\t' && *Cur!='\r' && *Cur!='\n' && *Cur!=_Sep1 && *Cur!=_Sep2))
           || (Quote!=0 && (*Cur!='\0' /* && *Cur!='\r' && *Cur!='\n' */)) ) // allow multi-line strings
    {
        LineJustIncremented = false;
        AddChar = true;
        if( Quote==0 && (*Cur=='\'' || *Cur=='\"' || *Cur=='`') )
        {
            Quote = *Cur;
            QuoteLine = Line;
            QuoteColumn = Column;
            QuoteCur = Cur;
            AddChar = _KeepQuotes;
        }
        else if ( Quote!=0 && *Cur==Quote )
        {
            Quote = 0;
            AddChar = _KeepQuotes;
        }

        if( AddChar )
            _Token += *Cur;
        ++Cur;
        if( *Cur=='\t' )
            Column += g_TabLength;
        else if( *Cur=='\n' )
        {
            ++Line;
            LineJustIncremented = true;
            Column = 1;
        }
        else
            ++Column;
    }

    if( Quote!=0 )
    {
        Line = QuoteLine;
        Column = QuoteColumn;
        return -(int)(Cur-_Def);    // unclosed quote
    }
    else
    {
        if( *Cur=='\n' )
        {
            if( !LineJustIncremented )
                ++Line;
            Column = 1;
        }
        else if( *Cur=='\t' )
            Column += g_TabLength;
        else if( *Cur!='\r' && *Cur!='\0' )
            ++Column;
        return (int)(Cur-_Def);
    }
}

//  ---------------------------------------------------------------------------

int GetBarVarFromString(CTwBar **_Bar, CTwVar **_Var, CTwVarGroup **_VarParent, int *_VarIndex, const char *_Str)
{
    *_Bar = NULL;
    *_Var = NULL;
    *_VarParent = NULL;
    *_VarIndex = -1;
    vector<string> Names;
    string Token;
    const char *Cur =_Str;
    int l=1, c=1, p=1;
    while( *Cur!='\0' && p>0 && Names.size()<=3 )
    {
        p = ParseToken(Token, Cur, l, c, false, true, '/', '\\');
        if( p>0 && Token.size()>0 )
        {
            Names.push_back(Token);
            Cur += p + ((Cur[p]!='\0')?1:0);
        }
    }
    if( p<=0 || (Names.size()!=1 && Names.size()!=2) )
        return 0;   // parse error
    int BarIdx = g_TwMgr->FindBar(Names[0].c_str());
    if( BarIdx<0 )
    {
        if( Names.size()==1 && strcmp(Names[0].c_str(), "GLOBAL")==0 )
        {
            *_Bar = TW_GLOBAL_BAR;
            return +3;  // 'GLOBAL' found
        }
        else
            return -1;  // bar not found
    }
    *_Bar = g_TwMgr->m_Bars[BarIdx];
    if( Names.size()==1 )
        return 1;   // bar found, no var name parsed
    *_Var = (*_Bar)->Find(Names[1].c_str(), _VarParent, _VarIndex);
    if( *_Var==NULL )
        return -2;  // var not found
    return 2;       // bar and var found
}


int BarVarHasAttrib(CTwBar *_Bar, CTwVar *_Var, const char *_Attrib, bool *_HasValue)
{
    assert(_Bar!=NULL && _HasValue!=NULL && _Attrib!=NULL && strlen(_Attrib)>0);
    *_HasValue = false;
    if( _Bar==TW_GLOBAL_BAR )
    {
        assert( _Var==NULL );
        return g_TwMgr->HasAttrib(_Attrib, _HasValue);
    }
    else if( _Var==NULL )
        return _Bar->HasAttrib(_Attrib, _HasValue);
    else
        return _Var->HasAttrib(_Attrib, _HasValue);
}


int BarVarSetAttrib(CTwBar *_Bar, CTwVar *_Var, CTwVarGroup *_VarParent, int _VarIndex, int _AttribID, const char *_Value)
{
    assert(_Bar!=NULL && _AttribID>0);

    /* don't delete popupbar here: if any attrib is changed every frame by the app, popup will not work anymore.
    if( g_TwMgr->m_PopupBar!=NULL && _Bar!=g_TwMgr->m_PopupBar && g_TwMgr->m_PopupBar->m_BarLinkedToPopupList==_Bar )   // delete popup bar first if it exists
    {
        TwDeleteBar(g_TwMgr->m_PopupBar);
        g_TwMgr->m_PopupBar = NULL;
    }
    */

    if( _Bar==TW_GLOBAL_BAR )
    {
        assert( _Var==NULL );
        return g_TwMgr->SetAttrib(_AttribID, _Value);
    }
    else if( _Var==NULL )
        return _Bar->SetAttrib(_AttribID, _Value);
    else
        return _Var->SetAttrib(_AttribID, _Value, _Bar, _VarParent, _VarIndex);
    // don't make _Bar not-up-to-date here, should be done in SetAttrib if needed to avoid too frequent refreshs
}
 

ERetType BarVarGetAttrib(CTwBar *_Bar, CTwVar *_Var, CTwVarGroup *_VarParent, int _VarIndex, int _AttribID, std::vector<double>& outDoubles, std::ostringstream& outString)
{
    assert(_Bar!=NULL && _AttribID>0);

    if( _Bar==TW_GLOBAL_BAR )
    {
        assert( _Var==NULL );
        return g_TwMgr->GetAttrib(_AttribID, outDoubles, outString);
    }
    else if( _Var==NULL )
        return _Bar->GetAttrib(_AttribID, outDoubles, outString);
    else
        return _Var->GetAttrib(_AttribID, _Bar, _VarParent, _VarIndex, outDoubles, outString);
}

//  ---------------------------------------------------------------------------

static inline std::string ErrorPosition(bool _MultiLine, int _Line, int _Column)
{
    if( !_MultiLine )
        return "";
    else
    {
        char pos[32];
        //_snprintf(pos, sizeof(pos)-1, " line %d column %d", _Line, _Column);
        _snprintf(pos, sizeof(pos)-1, " line %d", _Line); (void)_Column;
        pos[sizeof(pos)-1] = '\0';
        return pos;
    }
}

//  ---------------------------------------------------------------------------

int ANT_CALL TwDefine(const char *_Def)
{
    CTwFPU fpu; // force fpu precision

    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return 0; // not initialized
    }
    if( _Def==NULL )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return 0;
    }

    bool MultiLine = false;
    const char *Cur = _Def;
    while( *Cur!='\0' )
    {
        if( *Cur=='\n' )
        {
            MultiLine = true;
            break;
        }
        ++Cur;
    }

    int Line = 1;
    int Column = 1;
    enum EState { PARSE_NAME, PARSE_ATTRIB };
    EState State = PARSE_NAME;
    string Token;
    string Value;
    CTwBar *Bar = NULL;
    CTwVar *Var = NULL;
    CTwVarGroup *VarParent = NULL;
    int VarIndex = -1;
    int p; 

    Cur = _Def;
    while( *Cur!='\0' )
    {
        const char *PrevCur = Cur;
        p = ParseToken(Token, Cur, Line, Column, (State==PARSE_NAME), (State==PARSE_ATTRIB), (State==PARSE_ATTRIB)?'=':'\0');
        if( p<=0 || Token.size()<=0 )
        {
            if( p>0 && Cur[p]=='\0' )
            {
                Cur += p;
                continue;
            }
            _snprintf(g_ErrParse, sizeof(g_ErrParse), "Parsing error in def string%s [%-16s...]", ErrorPosition(MultiLine, Line, Column).c_str(), (p<0)?(Cur-p):PrevCur);
            g_ErrParse[sizeof(g_ErrParse)-1] = '\0';
            g_TwMgr->SetLastError(g_ErrParse);
            return 0;
        }
        char CurSep = Cur[p];
        Cur += p + ((CurSep!='\0')?1:0);

        if( State==PARSE_NAME )
        {
            int Err = GetBarVarFromString(&Bar, &Var, &VarParent, &VarIndex, Token.c_str());
            if( Err<=0 )
            {
                if( Err==-1 )
                    _snprintf(g_ErrParse, sizeof(g_ErrParse), "Parsing error in def string: Bar not found%s [%-16s...]", ErrorPosition(MultiLine, Line, Column).c_str(), Token.c_str());
                else if( Err==-2 )
                    _snprintf(g_ErrParse, sizeof(g_ErrParse), "Parsing error in def string: Variable not found%s [%-16s...]", ErrorPosition(MultiLine, Line, Column).c_str(), Token.c_str());
                else
                    _snprintf(g_ErrParse, sizeof(g_ErrParse), "Parsing error in def string%s [%-16s...]", ErrorPosition(MultiLine, Line, Column).c_str(), Token.c_str());
                g_ErrParse[sizeof(g_ErrParse)-1] = '\0';
                g_TwMgr->SetLastError(g_ErrParse);
                return 0;
            }
            State = PARSE_ATTRIB;
        }
        else // State==PARSE_ATTRIB
        {
            assert(State==PARSE_ATTRIB);
            assert(Bar!=NULL);

            bool HasValue = false;
            Value = "";
            int AttribID = BarVarHasAttrib(Bar, Var, Token.c_str(), &HasValue);
            if( AttribID<=0 )
            {
                _snprintf(g_ErrParse, sizeof(g_ErrParse), "Parsing error in def string: Unknown attribute%s [%-16s...]", ErrorPosition(MultiLine, Line, Column).c_str(), Token.c_str());
                g_ErrParse[sizeof(g_ErrParse)-1] = '\0';    
                g_TwMgr->SetLastError(g_ErrParse);
                return 0;
            }

            // special case for backward compatibility
            if( HasValue && ( _stricmp(Token.c_str(), "readonly")==0 || _stricmp(Token.c_str(), "hexa")==0 ) )
            {
                if( CurSep==' ' || CurSep=='\t' )
                {
                    const char *ch = Cur;
                    while( *ch==' ' || *ch=='\t' ) // find next non-space character
                        ++ch;
                    if( *ch!='=' ) // if this is not '=' the param has no value
                        HasValue = false;
                }
            }

            if( HasValue )
            {
                if( CurSep!='=' )
                {
                    string EqualStr;
                    p = ParseToken(EqualStr, Cur, Line, Column, true, true, '=');
                    CurSep = Cur[p];
                    if( p<0 || EqualStr.size()>0 || CurSep!='=' )
                    {
                        _snprintf(g_ErrParse, sizeof(g_ErrParse), "Parsing error in def string: '=' not found while reading attribute value%s [%-16s...]", ErrorPosition(MultiLine, Line, Column).c_str(), Token.c_str());
                        g_ErrParse[sizeof(g_ErrParse)-1] = '\0';
                        g_TwMgr->SetLastError(g_ErrParse);
                        return 0;
                    }
                    Cur += p + 1;
                }
                p = ParseToken(Value, Cur, Line, Column, false, true);
                if( p<=0 )
                {
                    _snprintf(g_ErrParse, sizeof(g_ErrParse), "Parsing error in def string: can't read attribute value%s [%-16s...]", ErrorPosition(MultiLine, Line, Column).c_str(), Token.c_str());
                    g_ErrParse[sizeof(g_ErrParse)-1] = '\0';
                    g_TwMgr->SetLastError(g_ErrParse);
                    return 0;
                }
                CurSep = Cur[p];
                Cur += p + ((CurSep!='\0')?1:0);
            }
            const char *PrevLastErrorPtr = g_TwMgr->CheckLastError();
            if( BarVarSetAttrib(Bar, Var, VarParent, VarIndex, AttribID, HasValue?Value.c_str():NULL)==0 )
            {
                if( g_TwMgr->CheckLastError()==NULL || strlen(g_TwMgr->CheckLastError())<=0 || g_TwMgr->CheckLastError()==PrevLastErrorPtr )
                    _snprintf(g_ErrParse, sizeof(g_ErrParse), "Parsing error in def string: wrong attribute value%s [%-16s...]", ErrorPosition(MultiLine, Line, Column).c_str(), Token.c_str());
                else
                    _snprintf(g_ErrParse, sizeof(g_ErrParse), "%s%s [%-16s...]", g_TwMgr->CheckLastError(), ErrorPosition(MultiLine, Line, Column).c_str(), Token.c_str());
                g_ErrParse[sizeof(g_ErrParse)-1] = '\0';
                g_TwMgr->SetLastError(g_ErrParse);
                return 0;
            }
            // sweep spaces to detect next attrib
            while( *Cur==' ' || *Cur=='\t' || *Cur=='\r' )
            {
                ++Cur;
                if( *Cur=='\t' )
                    Column += g_TabLength;
                else if( *Cur!='\r' )
                    ++Column;
            }
            if( *Cur=='\n' )    // new line detected
            {
                ++Line;
                Column = 1;
                State = PARSE_NAME;
            }
        }
    }

    g_TwMgr->m_HelpBarNotUpToDate = true;
    return 1;
}

//  ---------------------------------------------------------------------------

TwType ANT_CALL TwDefineEnum(const char *_Name, const TwEnumVal *_EnumValues, unsigned int _NbValues)
{
    CTwFPU fpu; // force fpu precision

    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return TW_TYPE_UNDEF; // not initialized
    }
    if( _EnumValues==NULL && _NbValues!=0 )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return TW_TYPE_UNDEF;
    }

    if( g_TwMgr->m_PopupBar!=NULL ) // delete popup bar first if it exists
    {
        TwDeleteBar(g_TwMgr->m_PopupBar);
        g_TwMgr->m_PopupBar = NULL;
    }

    size_t enumIndex = g_TwMgr->m_Enums.size();
    if( _Name!=NULL && strlen(_Name)>0 )
        for( size_t j=0; j<g_TwMgr->m_Enums.size(); ++j )
            if( strcmp(_Name, g_TwMgr->m_Enums[j].m_Name.c_str())==0 )
            {
                enumIndex = j;
                break;
            }
    if( enumIndex==g_TwMgr->m_Enums.size() )
        g_TwMgr->m_Enums.push_back(CTwMgr::CEnum());
    assert( enumIndex>=0 && enumIndex<g_TwMgr->m_Enums.size() );
    CTwMgr::CEnum& e = g_TwMgr->m_Enums[enumIndex];
    if( _Name!=NULL && strlen(_Name)>0 )
        e.m_Name = _Name;
    else
        e.m_Name = "";
    e.m_Entries.clear();
    for(unsigned int i=0; i<_NbValues; ++i)
    {
        CTwMgr::CEnum::CEntries::value_type Entry(_EnumValues[i].Value, (_EnumValues[i].Label!=NULL)?_EnumValues[i].Label:"");
        pair<CTwMgr::CEnum::CEntries::iterator, bool> Result = e.m_Entries.insert(Entry);
        if( !Result.second )
            (Result.first)->second = Entry.second;
    }

    return TwType( TW_TYPE_ENUM_BASE + enumIndex );
}

//  ---------------------------------------------------------------------------

TwType TW_CALL TwDefineEnumFromString(const char *_Name, const char *_EnumString)
{
    if (_EnumString == NULL) 
        return TwDefineEnum(_Name, NULL, 0);

    // split enumString
    stringstream EnumStream(_EnumString);
    string Label;
    vector<string> Labels;
    while( getline(EnumStream, Label, ',') ) {
        // trim Label
        size_t Start = Label.find_first_not_of(" \n\r\t");
        size_t End = Label.find_last_not_of(" \n\r\t");
        if( Start==string::npos || End==string::npos )
            Label = "";
        else
            Label = Label.substr(Start, (End-Start)+1);
        // store Label
        Labels.push_back(Label);
    }
    // create TwEnumVal array
    vector<TwEnumVal> Vals(Labels.size());
    for( int i=0; i<(int)Labels.size(); i++ )
    {
        Vals[i].Value = i;
        Vals[i].Label = Labels[i].c_str();
    }

    return TwDefineEnum(_Name, Vals.empty() ? NULL : &(Vals[0]), (unsigned int)Vals.size());
}

//  ---------------------------------------------------------------------------

void ANT_CALL CTwMgr::CStruct::DefaultSummary(char *_SummaryString, size_t _SummaryMaxLength, const void *_Value, void *_ClientData)
{
    const CTwVarGroup *varGroup = static_cast<const CTwVarGroup *>(_Value); // special case
    if( _SummaryString && _SummaryMaxLength>0 )
        _SummaryString[0] = '\0';
    size_t structIndex = (size_t)(_ClientData);
    if(    g_TwMgr && _SummaryString && _SummaryMaxLength>2
        && varGroup && static_cast<const CTwVar *>(varGroup)->IsGroup()
        && structIndex>=0 && structIndex<=g_TwMgr->m_Structs.size() )
    {
        // return g_TwMgr->m_Structs[structIndex].m_Name.c_str();
        CTwMgr::CStruct& s = g_TwMgr->m_Structs[structIndex];
        _SummaryString[0] = '{';
        _SummaryString[1] = '\0';
        bool separator = false;
        for( size_t i=0; i<s.m_Members.size(); ++i )
        {
            string varName = varGroup->m_Name + '.' + s.m_Members[i].m_Name;
            const CTwVar *var = varGroup->Find(varName.c_str(), NULL, NULL);
            if( var )
            {
                if( var->IsGroup() )
                {
                    const CTwVarGroup *grp = static_cast<const CTwVarGroup *>(var);
                    if( grp->m_SummaryCallback!=NULL )
                    {
                        size_t l = strlen(_SummaryString);
                        if( separator )
                        {
                            _SummaryString[l++] = ',';
                            _SummaryString[l++] = '\0';
                        }
                        if( grp->m_SummaryCallback==CTwMgr::CStruct::DefaultSummary )
                            grp->m_SummaryCallback(_SummaryString+l, _SummaryMaxLength-l, grp, grp->m_SummaryClientData);
                        else
                            grp->m_SummaryCallback(_SummaryString+l, _SummaryMaxLength-l, grp->m_StructValuePtr, grp->m_SummaryClientData);
                        separator = true;
                    }
                }
                else
                {
                    size_t l = strlen(_SummaryString);
                    if( separator )
                    {
                        _SummaryString[l++] = ',';
                        _SummaryString[l++] = '\0';
                    }
                    string valString;
                    const CTwVarAtom *atom = static_cast<const CTwVarAtom *>(var);
                    atom->ValueToString(&valString);
                    if( atom->m_Type==TW_TYPE_BOOLCPP || atom->m_Type==TW_TYPE_BOOL8 || atom->m_Type==TW_TYPE_BOOL16 || atom->m_Type==TW_TYPE_BOOL32 )
                    {
                        if (valString == "0")
                            valString = "-";
                        else if (valString == "1")
                            valString = "\x7f"; // check sign
                    }
                    strncat(_SummaryString, valString.c_str(), _SummaryMaxLength-l);
                    separator = true;
                }
                if( strlen(_SummaryString)>_SummaryMaxLength-2 )
                    break;
            }
        }
        size_t l = strlen(_SummaryString);
        if( l>_SummaryMaxLength-2 )
        {
            _SummaryString[_SummaryMaxLength-2] = '.';
            _SummaryString[_SummaryMaxLength-1] = '.';
            _SummaryString[_SummaryMaxLength+0] = '\0';
        }
        else
        {
            _SummaryString[l+0] = '}';
            _SummaryString[l+1] = '\0';
        }
    }
}

//  ---------------------------------------------------------------------------

TwType ANT_CALL TwDefineStruct(const char *_StructName, const TwStructMember *_StructMembers, unsigned int _NbMembers, size_t _StructSize, TwSummaryCallback _SummaryCallback, void *_SummaryClientData)
{
    CTwFPU fpu; // force fpu precision

    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return TW_TYPE_UNDEF; // not initialized
    }
    if( _StructMembers==NULL || _NbMembers==0 || _StructSize==0 )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return TW_TYPE_UNDEF;
    }

    if( _StructName!=NULL && strlen(_StructName)>0 )
        for( size_t j=0; j<g_TwMgr->m_Structs.size(); ++j )
            if( strcmp(_StructName, g_TwMgr->m_Structs[j].m_Name.c_str())==0 )
            {
                g_TwMgr->SetLastError(g_ErrExist);
                return TW_TYPE_UNDEF;
            }

    size_t structIndex = g_TwMgr->m_Structs.size();
    CTwMgr::CStruct s;
    s.m_Size = _StructSize;
    if( _StructName!=NULL && strlen(_StructName)>0 )
        s.m_Name = _StructName;
    else
        s.m_Name = "";
    s.m_Members.resize(_NbMembers);
    if( _SummaryCallback!=NULL )
    {
        s.m_SummaryCallback = _SummaryCallback;
        s.m_SummaryClientData = _SummaryClientData;
    }
    else
    {
        s.m_SummaryCallback = CTwMgr::CStruct::DefaultSummary;
        s.m_SummaryClientData = (void *)(structIndex);
    }
    for( unsigned int i=0; i<_NbMembers; ++i )
    {
        CTwMgr::CStructMember& m = s.m_Members[i];
        if( _StructMembers[i].Name!=NULL )
            m.m_Name = _StructMembers[i].Name;
        else
        {
            char name[16];
            sprintf(name, "%u", i);
            m.m_Name = name;
        }
        m.m_Type = _StructMembers[i].Type;
        m.m_Size = 0;   // to avoid endless recursivity in GetDataSize
        m.m_Size = CTwVar::GetDataSize(m.m_Type);
        if( _StructMembers[i].Offset<_StructSize )
            m.m_Offset = _StructMembers[i].Offset;
        else
        {
            g_TwMgr->SetLastError(g_ErrOffset);
            return TW_TYPE_UNDEF;
        }
        if( _StructMembers[i].DefString!=NULL && strlen(_StructMembers[i].DefString)>0 )
            m.m_DefString = _StructMembers[i].DefString;
        else
            m.m_DefString = "";
    }

    g_TwMgr->m_Structs.push_back(s);
    assert( g_TwMgr->m_Structs.size()==structIndex+1 );
    return TwType( TW_TYPE_STRUCT_BASE + structIndex );
}

//  ---------------------------------------------------------------------------

TwType ANT_CALL TwDefineStructExt(const char *_StructName, const TwStructMember *_StructExtMembers, unsigned int _NbExtMembers, size_t _StructSize, size_t _StructExtSize, TwStructExtInitCallback _StructExtInitCallback, TwCopyVarFromExtCallback _CopyVarFromExtCallback, TwCopyVarToExtCallback _CopyVarToExtCallback, TwSummaryCallback _SummaryCallback, void *_ClientData, const char *_Help)
{
    CTwFPU fpu; // force fpu precision

    if( g_TwMgr==NULL )
    {
        TwGlobalError(g_ErrNotInit);
        return TW_TYPE_UNDEF; // not initialized
    }
    if( _StructSize==0 || _StructExtInitCallback==NULL || _CopyVarFromExtCallback==NULL || _CopyVarToExtCallback==NULL )
    {
        g_TwMgr->SetLastError(g_ErrBadParam);
        return TW_TYPE_UNDEF;
    }
    TwType type = TwDefineStruct(_StructName, _StructExtMembers, _NbExtMembers, _StructExtSize, _SummaryCallback, _ClientData);
    if( type>=TW_TYPE_STRUCT_BASE && type<TW_TYPE_STRUCT_BASE+(int)g_TwMgr->m_Structs.size() )
    {
        CTwMgr::CStruct& s = g_TwMgr->m_Structs[type-TW_TYPE_STRUCT_BASE];
        s.m_IsExt = true;
        s.m_ClientStructSize = _StructSize;
        s.m_StructExtInitCallback = _StructExtInitCallback;
        s.m_CopyVarFromExtCallback = _CopyVarFromExtCallback;   
        s.m_CopyVarToExtCallback = _CopyVarToExtCallback;
        s.m_ExtClientData = _ClientData;
        if( _Help!=NULL )
            s.m_Help = _Help;
    }
    return type;
}


//  ---------------------------------------------------------------------------

bool TwGetKeyCode(int *_Code, int *_Modif, const char *_String)
{
    assert(_Code!=NULL && _Modif!=NULL);
    bool Ok = true;
    *_Modif = TW_KMOD_NONE;
    *_Code = 0;
    size_t Start = strlen(_String)-1;
    if( Start<0 )
        return false;
    while( Start>0 && _String[Start-1]!='+' )
        --Start;
    while( _String[Start]==' ' || _String[Start]=='\t' )
        ++Start;
    char *CodeStr = _strdup(_String+Start);
    for( size_t i=strlen(CodeStr)-1; i>=0; ++i )
        if( CodeStr[i]==' ' || CodeStr[i]=='\t' )
            CodeStr[i] = '\0';
        else
            break;

    /*
    if( strstr(_String, "SHIFT")!=NULL || strstr(_String, "shift")!=NULL )
        *_Modif |= TW_KMOD_SHIFT;
    if( strstr(_String, "CTRL")!=NULL || strstr(_String, "ctrl")!=NULL )
        *_Modif |= TW_KMOD_CTRL;
    if( strstr(_String, "META")!=NULL || strstr(_String, "meta")!=NULL )
        *_Modif |= TW_KMOD_META;

    if( strstr(_String, "ALTGR")!=NULL || strstr(_String, "altgr")!=NULL )
        ((void)(0));    // *_Modif |= TW_KMOD_ALTGR;
    else // ALT and ALTGR are exclusive
        if( strstr(_String, "ALT")!=NULL || strstr(_String, "alt")!=NULL )  
            *_Modif |= TW_KMOD_ALT;
    */
    char *up = _strdup(_String);
    // _strupr(up);
    for( char *upch=up; *upch!='\0'; ++upch )
        *upch = (char)toupper(*upch);
    if( strstr(up, "SHIFT")!=NULL )
        *_Modif |= TW_KMOD_SHIFT;
    if( strstr(up, "CTRL")!=NULL )
        *_Modif |= TW_KMOD_CTRL;
    if( strstr(up, "META")!=NULL )
        *_Modif |= TW_KMOD_META;

    if( strstr(up, "ALTGR")!=NULL )
        ((void)(0));    // *_Modif |= TW_KMOD_ALTGR;
    else // ALT and ALTGR are exclusive
        if( strstr(up, "ALT")!=NULL )   
            *_Modif |= TW_KMOD_ALT;
    free(up);

    if( strlen(CodeStr)==1 )
        *_Code = (unsigned char)(CodeStr[0]);
    else if( _stricmp(CodeStr, "backspace")==0 || _stricmp(CodeStr, "bs")==0 )
        *_Code = TW_KEY_BACKSPACE;
    else if( _stricmp(CodeStr, "tab")==0 )
        *_Code = TW_KEY_TAB;
    else if( _stricmp(CodeStr, "clear")==0 || _stricmp(CodeStr, "clr")==0 )
        *_Code = TW_KEY_CLEAR;
    else if( _stricmp(CodeStr, "return")==0 || _stricmp(CodeStr, "ret")==0 )
        *_Code = TW_KEY_RETURN;
    else if( _stricmp(CodeStr, "pause")==0 )
        *_Code = TW_KEY_PAUSE;
    else if( _stricmp(CodeStr, "escape")==0 || _stricmp(CodeStr, "esc")==0 )
        *_Code = TW_KEY_ESCAPE;
    else if( _stricmp(CodeStr, "space")==0 )
        *_Code = TW_KEY_SPACE;
    else if( _stricmp(CodeStr, "delete")==0 || _stricmp(CodeStr, "del")==0 )
        *_Code = TW_KEY_DELETE;
    /*
    else if( strlen(CodeStr)==4 && CodeStr[3]>='0' && CodeStr[3]<='9' && (strstr(CodeStr, "pad")==CodeStr || strstr(CodeStr, "PAD")==CodeStr) )
        *_Code = TW_KEY_PAD_0 + CodeStr[3]-'0';
    else if( _stricmp(CodeStr, "pad.")==0 )
        *_Code = TW_KEY_PAD_PERIOD;
    else if( _stricmp(CodeStr, "pad/")==0 )
        *_Code = TW_KEY_PAD_DIVIDE;
    else if( _stricmp(CodeStr, "pad*")==0 )
        *_Code = TW_KEY_PAD_MULTIPLY;
    else if( _stricmp(CodeStr, "pad+")==0 )
        *_Code = TW_KEY_PAD_PLUS;
    else if( _stricmp(CodeStr, "pad-")==0 )
        *_Code = TW_KEY_PAD_MINUS;
    else if( _stricmp(CodeStr, "padenter")==0 )
        *_Code = TW_KEY_PAD_ENTER;
    else if( _stricmp(CodeStr, "pad=")==0 )
        *_Code = TW_KEY_PAD_EQUALS;
    */
    else if( _stricmp(CodeStr, "up")==0 )
        *_Code = TW_KEY_UP;
    else if( _stricmp(CodeStr, "down")==0 )
        *_Code = TW_KEY_DOWN;
    else if( _stricmp(CodeStr, "right")==0 )
        *_Code = TW_KEY_RIGHT;
    else if( _stricmp(CodeStr, "left")==0 )
        *_Code = TW_KEY_LEFT;
    else if( _stricmp(CodeStr, "insert")==0 || _stricmp(CodeStr, "ins")==0 )
        *_Code = TW_KEY_INSERT;
    else if( _stricmp(CodeStr, "home")==0 )
        *_Code = TW_KEY_HOME;
    else if( _stricmp(CodeStr, "end")==0 )
        *_Code = TW_KEY_END;
    else if( _stricmp(CodeStr, "pgup")==0 )
        *_Code = TW_KEY_PAGE_UP;
    else if( _stricmp(CodeStr, "pgdown")==0 )
        *_Code = TW_KEY_PAGE_DOWN;
    else if( (strlen(CodeStr)==2 || strlen(CodeStr)==3) && (CodeStr[0]=='f' || CodeStr[0]=='F') )
    {
        int n = 0;
        if( sscanf(CodeStr+1, "%d", &n)==1 && n>0 && n<16 )
            *_Code = TW_KEY_F1 + n-1;
        else
            Ok = false;
    }

    free(CodeStr);
    return Ok;
}

bool TwGetKeyString(std::string *_String, int _Code, int _Modif)
{
    assert(_String!=NULL);
    bool Ok = true;
    if( _Modif & TW_KMOD_SHIFT )
        *_String += "SHIFT+";
    if( _Modif & TW_KMOD_CTRL )
        *_String += "CTRL+";
    if ( _Modif & TW_KMOD_ALT )
        *_String += "ALT+";
    if ( _Modif & TW_KMOD_META )
        *_String += "META+";
    // if ( _Modif & TW_KMOD_ALTGR )
    //  *_String += "ALTGR+";
    switch( _Code )
    {
    case TW_KEY_BACKSPACE:
        *_String += "BackSpace";
        break;
    case TW_KEY_TAB:
        *_String += "Tab";
        break;
    case TW_KEY_CLEAR:
        *_String += "Clear";
        break;
    case TW_KEY_RETURN:
        *_String += "Return";
        break;
    case TW_KEY_PAUSE:
        *_String += "Pause";
        break;
    case TW_KEY_ESCAPE:
        *_String += "Esc";
        break;
    case TW_KEY_SPACE:
        *_String += "Space";
        break;
    case TW_KEY_DELETE:
        *_String += "Delete";
        break;
    /*
    case TW_KEY_PAD_0:
        *_String += "PAD0";
        break;
    case TW_KEY_PAD_1:
        *_String += "PAD1";
        break;
    case TW_KEY_PAD_2:
        *_String += "PAD2";
        break;
    case TW_KEY_PAD_3:
        *_String += "PAD3";
        break;
    case TW_KEY_PAD_4:
        *_String += "PAD4";
        break;
    case TW_KEY_PAD_5:
        *_String += "PAD5";
        break;
    case TW_KEY_PAD_6:
        *_String += "PAD6";
        break;
    case TW_KEY_PAD_7:
        *_String += "PAD7";
        break;
    case TW_KEY_PAD_8:
        *_String += "PAD8";
        break;
    case TW_KEY_PAD_9:
        *_String += "PAD9";
        break;
    case TW_KEY_PAD_PERIOD:
        *_String += "PAD.";
        break;
    case TW_KEY_PAD_DIVIDE:
        *_String += "PAD/";
        break;
    case TW_KEY_PAD_MULTIPLY:
        *_String += "PAD*";
        break;
    case TW_KEY_PAD_MINUS:
        *_String += "PAD-";
        break;
    case TW_KEY_PAD_PLUS:
        *_String += "PAD+";
        break;
    case TW_KEY_PAD_ENTER:
        *_String += "PADEnter";
        break;
    case TW_KEY_PAD_EQUALS:
        *_String += "PAD=";
        break;
    */
    case TW_KEY_UP:
        *_String += "Up";
        break;
    case TW_KEY_DOWN:
        *_String += "Down";
        break;
    case TW_KEY_RIGHT:
        *_String += "Right";
        break;
    case TW_KEY_LEFT:
        *_String += "Left";
        break;
    case TW_KEY_INSERT:
        *_String += "Insert";
        break;
    case TW_KEY_HOME:
        *_String += "Home";
        break;
    case TW_KEY_END:
        *_String += "End";
        break;
    case TW_KEY_PAGE_UP:
        *_String += "PgUp";
        break;
    case TW_KEY_PAGE_DOWN:
        *_String += "PgDown";
        break;
    case TW_KEY_F1:
        *_String += "F1";
        break;
    case TW_KEY_F2:
        *_String += "F2";
        break;
    case TW_KEY_F3:
        *_String += "F3";
        break;
    case TW_KEY_F4:
        *_String += "F4";
        break;
    case TW_KEY_F5:
        *_String += "F5";
        break;
    case TW_KEY_F6:
        *_String += "F6";
        break;
    case TW_KEY_F7:
        *_String += "F7";
        break;
    case TW_KEY_F8:
        *_String += "F8";
        break;
    case TW_KEY_F9:
        *_String += "F9";
        break;
    case TW_KEY_F10:
        *_String += "F10";
        break;
    case TW_KEY_F11:
        *_String += "F11";
        break;
    case TW_KEY_F12:
        *_String += "F12";
        break;
    case TW_KEY_F13:
        *_String += "F13";
        break;
    case TW_KEY_F14:
        *_String += "F14";
        break;
    case TW_KEY_F15:
        *_String += "F15";
        break;
    default:
        if( _Code>0 && _Code<256 )
            *_String += char(_Code);
        else
        {
            *_String += "Unknown";
            Ok = false;
        }
    }
    return Ok;
}

//  ---------------------------------------------------------------------------
 
const int        TW_MOUSE_NOMOTION = -1;
ETwMouseAction   TW_MOUSE_MOTION = (ETwMouseAction)(-2);
ETwMouseAction   TW_MOUSE_WHEEL = (ETwMouseAction)(-3);
ETwMouseButtonID TW_MOUSE_NA = (ETwMouseButtonID)(-1);

static int TwMouseEvent(ETwMouseAction _EventType, TwMouseButtonID _Button, int _MouseX, int _MouseY, int _WheelPos)
{
    CTwFPU fpu; // force fpu precision

    if( g_TwMgr==NULL || g_TwMgr->m_Graph==NULL )
    {
        // TwGlobalError(g_ErrNotInit); -> not an error here
        return 0; // not initialized
    }
    if( g_TwMgr->m_WndHeight<=0 || g_TwMgr->m_WndWidth<=0 )
    {
        //g_TwMgr->SetLastError(g_ErrBadWndSize);   // not an error, windows not yet ready.
        return 0;
    }

    // For multi-thread safety
    if( !TwFreeAsyncDrawing() )
        return 0;

    if( _MouseX==TW_MOUSE_NOMOTION )
        _MouseX = g_TwMgr->m_LastMouseX;
    else
        g_TwMgr->m_LastMouseX = _MouseX;
    if( _MouseY==TW_MOUSE_NOMOTION )
        _MouseY = g_TwMgr->m_LastMouseY;
    else
        g_TwMgr->m_LastMouseY = _MouseY;

    // for autorepeat
    if( (!g_TwMgr->m_IsRepeatingMousePressed || !g_TwMgr->m_CanRepeatMousePressed) && _EventType==TW_MOUSE_PRESSED )
    {
        g_TwMgr->m_LastMousePressedTime = g_TwMgr->m_Timer.GetTime();
        g_TwMgr->m_LastMousePressedButtonID = _Button;
        g_TwMgr->m_LastMousePressedPosition[0] = _MouseX;
        g_TwMgr->m_LastMousePressedPosition[1] = _MouseY;
        g_TwMgr->m_CanRepeatMousePressed = true;
        g_TwMgr->m_IsRepeatingMousePressed = false;
    }
    else if( _EventType==TW_MOUSE_RELEASED || _EventType==TW_MOUSE_WHEEL )
    {
        g_TwMgr->m_CanRepeatMousePressed = false;
        g_TwMgr->m_IsRepeatingMousePressed = false;
    }

    bool Handled = false;
    bool wasPopup = (g_TwMgr->m_PopupBar!=NULL);
    CTwBar *Bar = NULL;
    int i;

    // search for a bar with mousedrag enabled
    CTwBar *BarDragging = NULL;
    for( i=((int)g_TwMgr->m_Bars.size())-1; i>=0; --i )
    {
        Bar = g_TwMgr->m_Bars[g_TwMgr->m_Order[i]];
        if( Bar!=NULL && Bar->m_Visible && Bar->IsDragging() )
        {
            BarDragging = Bar;
            break;
        }
    }

    for( i=(int)g_TwMgr->m_Bars.size(); i>=0; --i )
    {
        if( i==(int)g_TwMgr->m_Bars.size() )    // first try the bar with mousedrag enabled (this bar has the focus)
            Bar = BarDragging;
        else
        {
            Bar = g_TwMgr->m_Bars[g_TwMgr->m_Order[i]];
            if( Bar==BarDragging )
                continue;
        }
        if( Bar!=NULL && Bar->m_Visible )
        {
            if( _EventType==TW_MOUSE_MOTION )
                Handled = Bar->MouseMotion(_MouseX, _MouseY);
            else if( _EventType==TW_MOUSE_PRESSED || _EventType==TW_MOUSE_RELEASED )
                Handled = Bar->MouseButton(_Button, (_EventType==TW_MOUSE_PRESSED), _MouseX, _MouseY);
            else if( _EventType==TW_MOUSE_WHEEL )
            {
                if( abs(_WheelPos-g_TwMgr->m_LastMouseWheelPos)<4 ) // avoid crazy wheel positions
                    Handled = Bar->MouseWheel(_WheelPos, g_TwMgr->m_LastMouseWheelPos, _MouseX, _MouseY);
            }
            if( Handled )
                break;
        }
    }

    if( g_TwMgr==NULL ) // Mgr might have been destroyed by the client inside a callback call
        return 1;

    /*
    if( i>=0 && Bar!=NULL && Handled && (_EventType==TW_MOUSE_PRESSED || Bar->IsMinimized()) && i!=((int)g_TwMgr->m_Bars.size())-1 )
    {
        int iOrder = g_TwMgr->m_Order[i];
        for( int j=i; j<(int)g_TwMgr->m_Bars.size()-1; ++j )
            g_TwMgr->m_Order[j] = g_TwMgr->m_Order[j+1];
        g_TwMgr->m_Order[(int)g_TwMgr->m_Bars.size()-1] = iOrder;
    }
    */
    if( _EventType==TW_MOUSE_PRESSED || (Bar!=NULL && Bar->IsMinimized() && Handled) )
    {
        if( wasPopup && Bar!=g_TwMgr->m_PopupBar && g_TwMgr->m_PopupBar!=NULL ) // delete popup
        {
            TwDeleteBar(g_TwMgr->m_PopupBar);
            g_TwMgr->m_PopupBar = NULL;
        }

        if( i>=0 && Bar!=NULL && Handled && !wasPopup )
            TwSetTopBar(Bar);
    }

    if( _EventType==TW_MOUSE_WHEEL )
        g_TwMgr->m_LastMouseWheelPos = _WheelPos;

    return Handled ? 1 : 0;
}

int ANT_CALL TwMouseButton(ETwMouseAction _EventType, TwMouseButtonID _Button)
{
    return TwMouseEvent(_EventType, _Button, TW_MOUSE_NOMOTION, TW_MOUSE_NOMOTION, 0);
}

int ANT_CALL TwMouseMotion(int _MouseX, int _MouseY)
{
    return TwMouseEvent(TW_MOUSE_MOTION, TW_MOUSE_NA, _MouseX, _MouseY, 0);
}

int ANT_CALL TwMouseWheel(int _Pos)
{
    return TwMouseEvent(TW_MOUSE_WHEEL, TW_MOUSE_NA, TW_MOUSE_NOMOTION, TW_MOUSE_NOMOTION, _Pos);
}

//  ---------------------------------------------------------------------------

static int TranslateKey(int _Key, int _Modifiers)
{
    // CTRL special cases
    //if( (_Modifiers&TW_KMOD_CTRL) && !(_Modifiers&TW_KMOD_ALT || _Modifiers&TW_KMOD_META) && _Key>0 && _Key<32 )
    //  _Key += 'a'-1;
    if( (_Modifiers&TW_KMOD_CTRL) )
    {
        if( _Key>='a' && _Key<='z' && ( ((_Modifiers&0x2000) && !(_Modifiers&TW_KMOD_SHIFT)) || (!(_Modifiers&0x2000) && (_Modifiers&TW_KMOD_SHIFT)) )) // 0x2000 is SDL's KMOD_CAPS
            _Key += 'A'-'a';
        else if ( _Key>='A' && _Key<='Z' && ( ((_Modifiers&0x2000) && (_Modifiers&TW_KMOD_SHIFT)) || (!(_Modifiers&0x2000) && !(_Modifiers&TW_KMOD_SHIFT)) )) // 0x2000 is SDL's KMOD_CAPS
            _Key += 'a'-'A';
    }

    // PAD translation (for SDL keysym)
    if( _Key>=256 && _Key<=272 ) // 256=SDLK_KP0 ... 272=SDLK_KP_EQUALS
    {
        //bool Num = ((_Modifiers&TW_KMOD_SHIFT) && !(_Modifiers&0x1000)) || (!(_Modifiers&TW_KMOD_SHIFT) && (_Modifiers&0x1000)); // 0x1000 is SDL's KMOD_NUM
        //_Modifiers &= ~TW_KMOD_SHIFT; // remove shift modifier
        bool Num = (!(_Modifiers&TW_KMOD_SHIFT) && (_Modifiers&0x1000)); // 0x1000 is SDL's KMOD_NUM
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
    return _Key;
}

//  ---------------------------------------------------------------------------

static int KeyPressed(int _Key, int _Modifiers, bool _TestOnly)
{
    CTwFPU fpu; // force fpu precision

    if( g_TwMgr==NULL || g_TwMgr->m_Graph==NULL )
    {
        // TwGlobalError(g_ErrNotInit); -> not an error here
        return 0; // not initialized
    }
    if( g_TwMgr->m_WndHeight<=0 || g_TwMgr->m_WndWidth<=0 )
    {
        //g_TwMgr->SetLastError(g_ErrBadWndSize);   // not an error, windows not yet ready.
        return 0;
    }

    // For multi-thread savety
    if( !TwFreeAsyncDrawing() )
        return 0;

    /*
    // Test for TwDeleteBar
    if( _Key>='0' && _Key<='9' )
    {
        int n = _Key-'0';
        if( (int)g_TwMgr->m_Bars.size()>n && g_TwMgr->m_Bars[n]!=NULL )
        {
            printf("Delete %s\n", g_TwMgr->m_Bars[n]->m_Name.c_str());
            TwDeleteBar(g_TwMgr->m_Bars[n]);
        }
        else
            printf("can't delete %d\n", n);
        return 1;
    }
    */

    //char s[256];
    //sprintf(s, "twkeypressed k=%d m=%x\n", _Key, _Modifiers);
    //OutputDebugString(s);

    _Key = TranslateKey(_Key, _Modifiers);
    if( _Key>' ' && _Key<256 ) // don't test SHIFT if _Key is a common key
        _Modifiers &= ~TW_KMOD_SHIFT;
    // complete partial modifiers comming from SDL
    if( _Modifiers & TW_KMOD_SHIFT )
        _Modifiers |= TW_KMOD_SHIFT;
    if( _Modifiers & TW_KMOD_CTRL )
        _Modifiers |= TW_KMOD_CTRL;
    if( _Modifiers & TW_KMOD_ALT )
        _Modifiers |= TW_KMOD_ALT;
    if( _Modifiers & TW_KMOD_META )
        _Modifiers |= TW_KMOD_META;

    bool Handled = false;
    CTwBar *Bar = NULL;
    CTwBar *PopupBar = g_TwMgr->m_PopupBar;
    //int Order = 0;
    int i;
    if( _Key>0 && _Key<TW_KEY_LAST )
    {
        // First send it to bar which includes the mouse pointer
        int MouseX = g_TwMgr->m_LastMouseX;
        int MouseY = g_TwMgr->m_LastMouseY;
        for( i=((int)g_TwMgr->m_Bars.size())-1; i>=0 && !Handled; --i )
        {
            Bar = g_TwMgr->m_Bars[g_TwMgr->m_Order[i]];
            if( Bar!=NULL && Bar->m_Visible && !Bar->IsMinimized() 
                && ( (MouseX>=Bar->m_PosX && MouseX<Bar->m_PosX+Bar->m_Width && MouseY>=Bar->m_PosY && MouseY<Bar->m_PosY+Bar->m_Height)
                     || Bar==PopupBar) )
            {
                if (_TestOnly)
                    Handled = Bar->KeyTest(_Key, _Modifiers);
                else
                    Handled = Bar->KeyPressed(_Key, _Modifiers);
            }
        }

        // If not handled, send it to non-iconified bars in the right order
        for( i=((int)g_TwMgr->m_Bars.size())-1; i>=0 && !Handled; --i )
        {
            Bar = g_TwMgr->m_Bars[g_TwMgr->m_Order[i]];
            /*
            for( size_t j=0; j<g_TwMgr->m_Bars.size(); ++j )
                if( g_TwMgr->m_Order[j]==i )
                {
                    Bar = g_TwMgr->m_Bars[j];
                    break;
                }
            Order = i;
            */

            if( Bar!=NULL && Bar->m_Visible && !Bar->IsMinimized() )
            {
                if( _TestOnly )
                    Handled = Bar->KeyTest(_Key, _Modifiers);
                else
                    Handled = Bar->KeyPressed(_Key, _Modifiers);
                if( g_TwMgr==NULL ) // Mgr might have been destroyed by the client inside a callback call
                    return 1;
            }
        }

        // If not handled, send it to iconified bars in the right order
        for( i=((int)g_TwMgr->m_Bars.size())-1; i>=0 && !Handled; --i )
        {
            Bar = g_TwMgr->m_Bars[g_TwMgr->m_Order[i]];
            if( Bar!=NULL && Bar->m_Visible && Bar->IsMinimized() )
            {
                if( _TestOnly )
                    Handled = Bar->KeyTest(_Key, _Modifiers);
                else
                    Handled = Bar->KeyPressed(_Key, _Modifiers);
            }
        }
        
        if( g_TwMgr->m_HelpBar!=NULL && g_TwMgr->m_Graph && !_TestOnly )
        {
            string Str;
            TwGetKeyString(&Str, _Key, _Modifiers);
            char Msg[256];
            sprintf(Msg, "Key pressed: %s", Str.c_str());
            g_TwMgr->m_KeyPressedStr = Msg;
            g_TwMgr->m_KeyPressedBuildText = true;
            // OutputDebugString(Msg);
        }
    }

    if( Handled && Bar!=g_TwMgr->m_PopupBar && g_TwMgr->m_PopupBar!=NULL && g_TwMgr->m_PopupBar==PopupBar )  // delete popup
    {
        TwDeleteBar(g_TwMgr->m_PopupBar);
        g_TwMgr->m_PopupBar = NULL;
    }

    if( Handled && Bar!=NULL && Bar!=g_TwMgr->m_PopupBar && Bar!=PopupBar ) // popup bar may have been destroyed
        TwSetTopBar(Bar);

    return Handled ? 1 : 0;
}

int ANT_CALL TwKeyPressed(int _Key, int _Modifiers)
{
    return KeyPressed(_Key, _Modifiers, false);
}

int ANT_CALL TwKeyTest(int _Key, int _Modifiers)
{
    return KeyPressed(_Key, _Modifiers, true);
}

//  ---------------------------------------------------------------------------

struct StructCompare : public binary_function<TwType, TwType, bool>
{
    bool operator()(const TwType& _Left, const TwType& _Right) const
    {
        assert( g_TwMgr!=NULL );
        int i0 = _Left-TW_TYPE_STRUCT_BASE;
        int i1 = _Right-TW_TYPE_STRUCT_BASE;
        if( i0>=0 && i0<(int)g_TwMgr->m_Structs.size() && i1>=0 && i1<(int)g_TwMgr->m_Structs.size() )
            return g_TwMgr->m_Structs[i0].m_Name < g_TwMgr->m_Structs[i1].m_Name;
        else
            return false;
    }
};

typedef set<TwType, StructCompare> StructSet;

static void InsertUsedStructs(StructSet& _Set, const CTwVarGroup *_Grp)
{
    assert( g_TwMgr!=NULL && _Grp!=NULL );

    for( size_t i=0; i<_Grp->m_Vars.size(); ++i )
        if( _Grp->m_Vars[i]!=NULL && _Grp->m_Vars[i]->m_Visible && _Grp->m_Vars[i]->IsGroup() )// && _Grp->m_Vars[i]->m_Help.length()>0 )
        {
            const CTwVarGroup *SubGrp = static_cast<const CTwVarGroup *>(_Grp->m_Vars[i]);
            if( SubGrp->m_StructValuePtr!=NULL && SubGrp->m_StructType>=TW_TYPE_STRUCT_BASE && SubGrp->m_StructType<TW_TYPE_STRUCT_BASE+(int)g_TwMgr->m_Structs.size() && g_TwMgr->m_Structs[SubGrp->m_StructType-TW_TYPE_STRUCT_BASE].m_Name.length()>0 )
            {
                if( SubGrp->m_Help.length()>0 )
                    _Set.insert(SubGrp->m_StructType);
                else
                {
                    int idx = SubGrp->m_StructType - TW_TYPE_STRUCT_BASE;
                    if( idx>=0 && idx<(int)g_TwMgr->m_Structs.size() && g_TwMgr->m_Structs[idx].m_Name.length()>0 )
                    {
                        for( size_t j=0; j<g_TwMgr->m_Structs[idx].m_Members.size(); ++j )
                            if( g_TwMgr->m_Structs[idx].m_Members[j].m_Help.length()>0 )
                            {
                                _Set.insert(SubGrp->m_StructType);
                                break;
                            }
                    }
                }
            }
            InsertUsedStructs(_Set, SubGrp);
        }
}

static void SplitString(vector<string>& _OutSplits, const char *_String, int _Width, const CTexFont *_Font)
{
    assert( _Font!=NULL && _String!=NULL );
    _OutSplits.resize(0);
    int l = (int)strlen(_String);
    if( l==0 )
    {
        _String = " ";
        l = 1;
    }

    if( _String!=NULL && l>0 && _Width>0 )
    {
        int w = 0;
        int i = 0;
        int First = 0;
        int Last = 0;
        bool PrevNotBlank = true;
        unsigned char c;
        bool Tab = false, CR = false;
        string Split;
        const string TabString(g_TabLength, ' ');

        while( i<l )
        {
            c = _String[i];
            if( c=='\t' )
            {
                w += g_TabLength * _Font->m_CharWidth[(int)' '];
                Tab = true;
            }
            else if( c=='\n' )
            {
                w += _Width+1; // force split
                Last = i;
                CR = true;
            }
            else
                w += _Font->m_CharWidth[(int)c];
            if( w>_Width || i==l-1 )
            {
                if( Last<=First || i==l-1 )
                    Last = i;
                if( Tab )
                {
                    Split.resize(0);
                    for(int k=0; k<Last-First+(CR?0:1); ++k)
                        if( _String[First+k]=='\t' )
                            Split += TabString;
                        else
                            Split += _String[First+k];
                    Tab = false;
                }
                else
                    Split.assign(_String+First, Last-First+(CR?0:1));
                _OutSplits.push_back(Split);
                First = Last+1;
                if( !CR )
                    while( First<l && (_String[First]==' ' || _String[First]=='\t') )   // skip blanks
                        ++First;
                Last = First;
                w = 0;
                PrevNotBlank = true;
                i = First;
                CR = false;
            }
            else if( c==' ' || c=='\t' )
            {
                if( PrevNotBlank )
                    Last = i-1;
                PrevNotBlank = false;
                ++i;
            }
            else
            {
                PrevNotBlank = true;
                ++i;
            }
        }
    }
}

static int AppendHelpString(CTwVarGroup *_Grp, const char *_String, int _Level, int _Width, ETwType _Type)
{
    assert( _Grp!=NULL && g_TwMgr!=NULL && g_TwMgr->m_HelpBar!=NULL);
    assert( _String!=NULL );
    int n = 0;
    const CTexFont *Font = g_TwMgr->m_HelpBar->m_Font;
    assert(Font!=NULL);
    string Decal;
    for( int s=0; s<_Level; ++s )
        Decal += ' ';
    int DecalWidth = (_Level+2)*Font->m_CharWidth[(int)' '];

    if( _Width>DecalWidth )
    {
        vector<string> Split;
        SplitString(Split, _String, _Width-DecalWidth, Font);
        for( int i=0; i<(int)Split.size(); ++i )
        {
            CTwVarAtom *Var = new CTwVarAtom;
            Var->m_Name = Decal + Split[i];
            Var->m_Ptr = NULL;
            if( _Type==TW_TYPE_HELP_HEADER )
                Var->m_ReadOnly = false;
            else
                Var->m_ReadOnly = true;
            Var->m_NoSlider = true;
            Var->m_DontClip = true;
            Var->m_Type = _Type;
            Var->m_LeftMargin = (signed short)((_Level+1)*Font->m_CharWidth[(int)' ']);
            Var->m_TopMargin  = (signed short)(-g_TwMgr->m_HelpBar->m_Sep);
            //Var->m_TopMargin  = 1;
            Var->m_ColorPtr = &(g_TwMgr->m_HelpBar->m_ColHelpText);
            Var->SetDefaults();
            _Grp->m_Vars.push_back(Var);
            ++n;
        }
    }
    return n;
}

static int AppendHelp(CTwVarGroup *_Grp, const CTwVarGroup *_ToAppend, int _Level, int _Width)
{
    assert( _Grp!=NULL );
    assert( _ToAppend!=NULL );
    int n = 0;
    string Decal;
    for( int s=0; s<_Level; ++s )
        Decal += ' ';

    if( _ToAppend->m_Help.size()>0 )
        n += AppendHelpString(_Grp, _ToAppend->m_Help.c_str(), _Level, _Width, TW_TYPE_HELP_GRP);

    for( size_t i=0; i<_ToAppend->m_Vars.size(); ++i )
        if( _ToAppend->m_Vars[i]!=NULL && _ToAppend->m_Vars[i]->m_Visible )
        {
            bool append = true;
            if( !_ToAppend->m_Vars[i]->IsGroup() )
            {
                const CTwVarAtom *a = static_cast<const CTwVarAtom *>(_ToAppend->m_Vars[i]);
                if( a->m_Type==TW_TYPE_BUTTON && a->m_Val.m_Button.m_Callback==NULL )
                    append = false;
                else if( a->m_KeyIncr[0]==0 && a->m_KeyIncr[1]==0 && a->m_KeyDecr[0]==0 && a->m_KeyDecr[1]==0 && a->m_Help.length()<=0 )
                    append = false;
            }
            else if( _ToAppend->m_Vars[i]->IsGroup() && static_cast<const CTwVarGroup *>(_ToAppend->m_Vars[i])->m_StructValuePtr!=NULL // that's a struct var
                     && _ToAppend->m_Vars[i]->m_Help.length()<=0 )
                 append = false;

            if( append )
            {
                CTwVarAtom *Var = new CTwVarAtom;
                Var->m_Name = Decal;
                if( _ToAppend->m_Vars[i]->m_Label.size()>0 )
                    Var->m_Name += _ToAppend->m_Vars[i]->m_Label;
                else
                    Var->m_Name += _ToAppend->m_Vars[i]->m_Name;
                Var->m_Ptr = NULL;
                if( _ToAppend->m_Vars[i]->IsGroup() && static_cast<const CTwVarGroup *>(_ToAppend->m_Vars[i])->m_StructValuePtr!=NULL )
                {   // That's a struct var
                    Var->m_Type = TW_TYPE_HELP_STRUCT;
                    Var->m_Val.m_HelpStruct.m_StructType = static_cast<const CTwVarGroup *>(_ToAppend->m_Vars[i])->m_StructType;
                    Var->m_ReadOnly = true;
                    Var->m_NoSlider = true;
                }
                else if( !_ToAppend->m_Vars[i]->IsGroup() )
                {
                    Var->m_Type = TW_TYPE_SHORTCUT;
                    Var->m_Val.m_Shortcut.m_Incr[0] = static_cast<const CTwVarAtom *>(_ToAppend->m_Vars[i])->m_KeyIncr[0];
                    Var->m_Val.m_Shortcut.m_Incr[1] = static_cast<const CTwVarAtom *>(_ToAppend->m_Vars[i])->m_KeyIncr[1];
                    Var->m_Val.m_Shortcut.m_Decr[0] = static_cast<const CTwVarAtom *>(_ToAppend->m_Vars[i])->m_KeyDecr[0];
                    Var->m_Val.m_Shortcut.m_Decr[1] = static_cast<const CTwVarAtom *>(_ToAppend->m_Vars[i])->m_KeyDecr[1];
                    Var->m_ReadOnly = static_cast<const CTwVarAtom *>(_ToAppend->m_Vars[i])->m_ReadOnly;
                    Var->m_NoSlider = true;
                }
                else
                {
                    Var->m_Type = TW_TYPE_HELP_GRP;
                    Var->m_DontClip = true;
                    Var->m_LeftMargin = (signed short)((_Level+2)*g_TwMgr->m_HelpBar->m_Font->m_CharWidth[(int)' ']);
                    //Var->m_TopMargin  = (signed short)(g_TwMgr->m_HelpBar->m_Font->m_CharHeight/2-2+2*(_Level-1));
                    Var->m_TopMargin  = 2;
                    if( Var->m_TopMargin>g_TwMgr->m_HelpBar->m_Font->m_CharHeight-3 )
                        Var->m_TopMargin = (signed short)(g_TwMgr->m_HelpBar->m_Font->m_CharHeight-3);
                    Var->m_ReadOnly = true;
                }
                Var->SetDefaults();
                _Grp->m_Vars.push_back(Var);
                size_t VarIndex = _Grp->m_Vars.size()-1;
                ++n;
                if( _ToAppend->m_Vars[i]->IsGroup() && static_cast<const CTwVarGroup *>(_ToAppend->m_Vars[i])->m_StructValuePtr==NULL )
                {
                    int nAppended = AppendHelp(_Grp, static_cast<const CTwVarGroup *>(_ToAppend->m_Vars[i]), _Level+1, _Width);
                    if( _Grp->m_Vars.size()==VarIndex+1 )
                    {
                        delete _Grp->m_Vars[VarIndex];
                        _Grp->m_Vars.resize(VarIndex);
                    }
                    else
                        n += nAppended;
                }
                else if( _ToAppend->m_Vars[i]->m_Help.length()>0 )
                    n += AppendHelpString(_Grp, _ToAppend->m_Vars[i]->m_Help.c_str(), _Level+1, _Width, TW_TYPE_HELP_ATOM);
            }
        }
    return n;
}


static void CopyHierarchy(CTwVarGroup *dst, const CTwVarGroup *src)
{
    if( dst==NULL || src==NULL )
        return;

    dst->m_Name = src->m_Name;
    dst->m_Open = src->m_Open;
    dst->m_Visible = src->m_Visible;
    dst->m_ColorPtr = src->m_ColorPtr;
    dst->m_DontClip = src->m_DontClip;
    dst->m_IsRoot = src->m_IsRoot;
    dst->m_LeftMargin = src->m_LeftMargin;
    dst->m_TopMargin = src->m_TopMargin;

    dst->m_Vars.resize(src->m_Vars.size());
    for(size_t i=0; i<src->m_Vars.size(); ++i)
        if( src->m_Vars[i]!=NULL && src->m_Vars[i]->IsGroup() )
        {
            CTwVarGroup *grp = new CTwVarGroup;
            CopyHierarchy(grp, static_cast<const CTwVarGroup *>(src->m_Vars[i]));
            dst->m_Vars[i] = grp;
        }
        else
            dst->m_Vars[i] = NULL;
}

// copy the 'open' flag from original hierarchy to current hierarchy
static void SynchroHierarchy(CTwVarGroup *cur, const CTwVarGroup *orig)
{
    if( cur==NULL || orig==NULL )
        return;

    if( strcmp(cur->m_Name.c_str(), orig->m_Name.c_str())==0 )
        cur->m_Open = orig->m_Open;

    size_t j = 0;
    while( j<orig->m_Vars.size() && (orig->m_Vars[j]==NULL || !orig->m_Vars[j]->IsGroup()) )
        ++j;

    for(size_t i=0; i<cur->m_Vars.size(); ++i)
        if( cur->m_Vars[i]!=NULL && cur->m_Vars[i]->IsGroup() && j<orig->m_Vars.size() && orig->m_Vars[j]!=NULL && orig->m_Vars[j]->IsGroup() )
        {
            CTwVarGroup *curGrp = static_cast<CTwVarGroup *>(cur->m_Vars[i]);
            const CTwVarGroup *origGrp = static_cast<const CTwVarGroup *>(orig->m_Vars[j]);
            if( strcmp(curGrp->m_Name.c_str(), origGrp->m_Name.c_str())==0 )
            {
                curGrp->m_Open = origGrp->m_Open;

                SynchroHierarchy(curGrp, origGrp);

                ++j;
                while( j<orig->m_Vars.size() && (orig->m_Vars[j]==NULL || !orig->m_Vars[j]->IsGroup()) )
                    ++j;
            }
        }
}


void CTwMgr::UpdateHelpBar()
{
    if( m_HelpBar==NULL || m_HelpBar->IsMinimized() )
        return;
    if( !m_HelpBarUpdateNow && (float)m_Timer.GetTime()<m_LastHelpUpdateTime+2 )    // update at most every 2 seconds
        return;
    m_HelpBarUpdateNow = false;
    m_LastHelpUpdateTime = (float)m_Timer.GetTime();
    #ifdef _DEBUG
        //printf("UPDATE HELPBAR\n");
    #endif // _DEBUG

    CTwVarGroup prevHierarchy;
    CopyHierarchy(&prevHierarchy, &m_HelpBar->m_VarRoot);

    TwRemoveAllVars(m_HelpBar);

    if( m_HelpBar->m_UpToDate )
        m_HelpBar->Update();

    if( m_Help.size()>0 )
        AppendHelpString(&(m_HelpBar->m_VarRoot), m_Help.c_str(), 0, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0, TW_TYPE_HELP_ATOM);
    if( m_HelpBar->m_Help.size()>0 )
        AppendHelpString(&(m_HelpBar->m_VarRoot), m_HelpBar->m_Help.c_str(), 0, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0, TW_TYPE_HELP_ATOM);
    AppendHelpString(&(m_HelpBar->m_VarRoot), "", 0, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0, TW_TYPE_HELP_HEADER);

    for( size_t ib=0; ib<m_Bars.size(); ++ib )
        if( m_Bars[ib]!=NULL && !(m_Bars[ib]->m_IsHelpBar) && m_Bars[ib]!=m_PopupBar && m_Bars[ib]->m_Visible )
        {
            // Create a group
            CTwVarGroup *Grp = new CTwVarGroup;
            Grp->m_SummaryCallback = NULL;
            Grp->m_SummaryClientData = NULL;
            Grp->m_StructValuePtr = NULL;
            if( m_Bars[ib]->m_Label.size()<=0 )
                Grp->m_Name = m_Bars[ib]->m_Name;
            else
                Grp->m_Name = m_Bars[ib]->m_Label;
            Grp->m_Open = true;
            Grp->m_ColorPtr = &(m_HelpBar->m_ColGrpText);
            m_HelpBar->m_VarRoot.m_Vars.push_back(Grp);
            if( m_Bars[ib]->m_Help.size()>0 )
                AppendHelpString(Grp, m_Bars[ib]->m_Help.c_str(), 0, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0, TW_TYPE_HELP_GRP);

            // Append variables (recursive)
            AppendHelp(Grp, &(m_Bars[ib]->m_VarRoot), 1, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0);

            // Append structures
            StructSet UsedStructs;
            InsertUsedStructs(UsedStructs, &(m_Bars[ib]->m_VarRoot));
            CTwVarGroup *StructGrp = NULL;
            int MemberCount = 0;
            for( StructSet::iterator it=UsedStructs.begin(); it!=UsedStructs.end(); ++it )
            {
                int idx = (*it) - TW_TYPE_STRUCT_BASE;
                if( idx>=0 && idx<(int)g_TwMgr->m_Structs.size() && g_TwMgr->m_Structs[idx].m_Name.length()>0 )
                {
                    if( StructGrp==NULL )
                    {
                        StructGrp = new CTwVarGroup;
                        StructGrp->m_StructType = TW_TYPE_HELP_STRUCT;  // a special line background color will be used
                        StructGrp->m_Name = "Structures";
                        StructGrp->m_Open = false;
                        StructGrp->m_ColorPtr = &(m_HelpBar->m_ColStructText);
                        //Grp->m_Vars.push_back(StructGrp);
                        MemberCount = 0;
                    }
                    CTwVarAtom *Var = new CTwVarAtom;
                    Var->m_Ptr = NULL;
                    Var->m_Type = TW_TYPE_HELP_GRP;
                    Var->m_DontClip = true;
                    Var->m_LeftMargin = (signed short)(3*g_TwMgr->m_HelpBar->m_Font->m_CharWidth[(int)' ']);
                    Var->m_TopMargin  = 2;
                    Var->m_ReadOnly = true;
                    Var->m_NoSlider = true;
                    Var->m_Name = '{'+g_TwMgr->m_Structs[idx].m_Name+'}';
                    StructGrp->m_Vars.push_back(Var);
                    size_t structIndex = StructGrp->m_Vars.size()-1;
                    if( g_TwMgr->m_Structs[idx].m_Help.size()>0 )
                        AppendHelpString(StructGrp, g_TwMgr->m_Structs[idx].m_Help.c_str(), 2, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0-2*Var->m_LeftMargin, TW_TYPE_HELP_ATOM);

                    // Append struct members
                    for( size_t im=0; im<g_TwMgr->m_Structs[idx].m_Members.size(); ++im )
                    {
                        if( g_TwMgr->m_Structs[idx].m_Members[im].m_Help.size()>0 )
                        {
                            CTwVarAtom *Var = new CTwVarAtom;
                            Var->m_Ptr = NULL;
                            Var->m_Type = TW_TYPE_SHORTCUT;
                            Var->m_Val.m_Shortcut.m_Incr[0] = 0;
                            Var->m_Val.m_Shortcut.m_Incr[1] = 0;
                            Var->m_Val.m_Shortcut.m_Decr[0] = 0;
                            Var->m_Val.m_Shortcut.m_Decr[1] = 0;
                            Var->m_ReadOnly = false;
                            Var->m_NoSlider = true;
                            if( g_TwMgr->m_Structs[idx].m_Members[im].m_Label.length()>0 )
                                Var->m_Name = "  "+g_TwMgr->m_Structs[idx].m_Members[im].m_Label;
                            else
                                Var->m_Name = "  "+g_TwMgr->m_Structs[idx].m_Members[im].m_Name;
                            StructGrp->m_Vars.push_back(Var);
                            //if( g_TwMgr->m_Structs[idx].m_Members[im].m_Help.size()>0 )
                            AppendHelpString(StructGrp, g_TwMgr->m_Structs[idx].m_Members[im].m_Help.c_str(), 3, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0-4*Var->m_LeftMargin, TW_TYPE_HELP_ATOM);
                        }
                    }

                    if( StructGrp->m_Vars.size()==structIndex+1 ) // remove struct from help
                    {
                        delete StructGrp->m_Vars[structIndex];
                        StructGrp->m_Vars.resize(structIndex);
                    }
                    else
                        ++MemberCount;
                }
            }
            if( StructGrp!=NULL )
            {
                if( MemberCount==1 )
                    StructGrp->m_Name = "Structure";
                if( StructGrp->m_Vars.size()>0 )
                    Grp->m_Vars.push_back(StructGrp);
                else
                {
                    delete StructGrp;
                    StructGrp = NULL;
                }
            }
        }

    // Append RotoSlider
    CTwVarGroup *RotoGrp = new CTwVarGroup;
    RotoGrp->m_SummaryCallback = NULL;
    RotoGrp->m_SummaryClientData = NULL;
    RotoGrp->m_StructValuePtr = NULL;
    RotoGrp->m_Name = "RotoSlider";
    RotoGrp->m_Open = false;
    RotoGrp->m_ColorPtr = &(m_HelpBar->m_ColGrpText);
    m_HelpBar->m_VarRoot.m_Vars.push_back(RotoGrp);
    AppendHelpString(RotoGrp, "The RotoSlider allows rapid editing of numerical values.", 0, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0, TW_TYPE_HELP_ATOM);
    AppendHelpString(RotoGrp, "To modify a numerical value, click on its label or on its roto [.] button, then move the mouse outside of the grey circle while keeping the mouse button pressed, and turn around the circle to increase or decrease the numerical value.", 0, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0, TW_TYPE_HELP_ATOM);
    AppendHelpString(RotoGrp, "The two grey lines depict the min and max bounds.", 0, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0, TW_TYPE_HELP_ATOM);
    AppendHelpString(RotoGrp, "Moving the mouse far form the circle allows precise increase or decrease, while moving near the circle allows fast increase or decrease.", 0, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0, TW_TYPE_HELP_ATOM);

    SynchroHierarchy(&m_HelpBar->m_VarRoot, &prevHierarchy);

    m_HelpBarNotUpToDate = false;
}

//  ---------------------------------------------------------------------------

#if defined(ANT_WINDOWS)

#include "res/TwXCursors.h"

void CTwMgr::CreateCursors()
{
    if( m_CursorsCreated )
        return;
    m_CursorArrow = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_ARROW));
    m_CursorMove = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_SIZEALL));
    m_CursorWE = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_SIZEWE));
    m_CursorNS = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_SIZENS));
    m_CursorTopRight = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_SIZENESW));
    m_CursorTopLeft = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_SIZENWSE));
    m_CursorBottomLeft = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_SIZENESW));
    m_CursorBottomRight = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_SIZENWSE));
    m_CursorHelp = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_HELP));
    m_CursorCross = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_CROSS));
    m_CursorUpArrow = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_UPARROW));
    m_CursorNo = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_NO));
    m_CursorIBeam = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_IBEAM));
    #ifdef IDC_HAND
        m_CursorHand = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_HAND));
    #else
        m_CursorHand = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_UPARROW));
    #endif
    int cur;
    HMODULE hdll = GetModuleHandle(ANT_TWEAK_BAR_DLL);
    if( hdll==NULL )
        g_UseCurRsc = false;    // force the use of built-in cursors (not using resources)
    if( g_UseCurRsc )
        m_CursorCenter = ::LoadCursor(hdll, MAKEINTRESOURCE(IDC_CURSOR1+0));
    else
        m_CursorCenter  = PixmapCursor(0);
    if( m_CursorCenter==NULL )
        m_CursorCenter = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_CROSS));
    if( g_UseCurRsc )
        m_CursorPoint = ::LoadCursor(hdll, MAKEINTRESOURCE(IDC_CURSOR1+1));
    else
        m_CursorPoint   = PixmapCursor(1);
    if( m_CursorPoint==NULL )
        m_CursorPoint = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_CROSS));

    for( cur=0; cur<NB_ROTO_CURSORS; ++cur )
    {
        if( g_UseCurRsc )
            m_RotoCursors[cur] = ::LoadCursor(hdll, MAKEINTRESOURCE(IDC_CURSOR1+2+cur));
        else
            m_RotoCursors[cur] = PixmapCursor(cur+2);
        if( m_RotoCursors[cur]==NULL )
            m_RotoCursors[cur] = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_CROSS));
    }
    
    m_CursorsCreated = true;
}


CTwMgr::CCursor CTwMgr::PixmapCursor(int _CurIdx)
{
    int x, y;
    unsigned char mask[32*4];
    unsigned char pict[32*4];
    for( y=0; y<32; ++y )
    {
        mask[y*4+0] = pict[y*4+0] = 0;
        mask[y*4+1] = pict[y*4+1] = 0;
        mask[y*4+2] = pict[y*4+2] = 0;
        mask[y*4+3] = pict[y*4+3] = 0;
        for( x=0; x<32; ++x )
        {
            mask[y*4+x/8] |= (((unsigned int)(g_CurMask[_CurIdx][x+y*32]))<<(7-(x%8)));
            pict[y*4+x/8] |= (((unsigned int)(g_CurPict[_CurIdx][x+y*32]))<<(7-(x%8)));
        }
    }

    unsigned char ands[32*4];
    unsigned char xors[32*4];
    for( y=0; y<32*4; ++y ) 
    {
        ands[y] = ~mask[y];
        xors[y] = pict[y];
    }

    HMODULE hdll = GetModuleHandle(ANT_TWEAK_BAR_DLL);
    CCursor cursor = ::CreateCursor(hdll, g_CurHot[_CurIdx][0], g_CurHot[_CurIdx][1], 32, 32, ands, xors);
 
    return cursor;
}

void CTwMgr::FreeCursors()
{
    if( !g_UseCurRsc )
    {
        if( m_CursorCenter!=NULL )
        {
            ::DestroyCursor(m_CursorCenter);
            m_CursorCenter = NULL;
        }
        if( m_CursorPoint!=NULL )
        {
            ::DestroyCursor(m_CursorPoint);
            m_CursorPoint = NULL;
        }
        for( int cur=0; cur<NB_ROTO_CURSORS; ++cur )
            if( m_RotoCursors[cur]!=NULL )
            {
                ::DestroyCursor(m_RotoCursors[cur]);
                m_RotoCursors[cur] = NULL;
            }
    }
    m_CursorsCreated = false;
}

void CTwMgr::SetCursor(CTwMgr::CCursor _Cursor)
{
    if( m_CursorsCreated )
    {
        CURSORINFO ci;
        memset(&ci, 0, sizeof(ci));
        ci.cbSize = sizeof(ci);
        BOOL ok = ::GetCursorInfo(&ci);
        if( ok && (ci.flags & CURSOR_SHOWING) )
            ::SetCursor(_Cursor);
    }
}


#elif defined(ANT_OSX)

#include "res/TwXCursors.h"

CTwMgr::CCursor CTwMgr::PixmapCursor(int _CurIdx)
{
    unsigned char *data;
    int x,y;
    
    NSBitmapImageRep *imgr = [[NSBitmapImageRep alloc] 
                              initWithBitmapDataPlanes: NULL
                              pixelsWide: 32
                              pixelsHigh: 32
                              bitsPerSample: 1
                              samplesPerPixel: 2
                              hasAlpha: YES
                              isPlanar: NO
                              colorSpaceName: NSCalibratedWhiteColorSpace
                              bitmapFormat: NSAlphaNonpremultipliedBitmapFormat
                              bytesPerRow: 8
                              bitsPerPixel: 2
                              ];
    data = [imgr bitmapData];
    memset(data,0x0,32*8);
    for (y=0;y<32;y++) {
        for (x=0;x<32;x++) {
            //printf("%d",g_CurMask[_CurIdx][x+y*32]);
            data[(x>>2) + y*8] |= (unsigned char)(g_CurPict[_CurIdx][x+y*32] << 2*(3-(x&3))+1); //turn whiteon
            data[(x>>2) + y*8] |= (unsigned char)(g_CurMask[_CurIdx][x+y*32] << 2*(3-(x&3))); //turn the alpha all the way up
        }
        //printf("\n");
    }
    NSImage *img = [[NSImage alloc] initWithSize: [imgr size]];
    [img addRepresentation: imgr];
    NSCursor *cur = [[NSCursor alloc] initWithImage: img hotSpot: NSMakePoint(g_CurHot[_CurIdx][0],g_CurHot[_CurIdx][1])];

    [imgr autorelease];
    [img autorelease];
    if (cur)
        return cur;
    else
        return [NSCursor arrowCursor];
}

void CTwMgr::CreateCursors()
{
    if (m_CursorsCreated)
        return;
    
    m_CursorArrow        = [[NSCursor arrowCursor] retain];
    m_CursorMove         = [[NSCursor crosshairCursor] retain];
    m_CursorWE           = [[NSCursor resizeLeftRightCursor] retain];
    m_CursorNS           = [[NSCursor resizeUpDownCursor] retain];
    m_CursorTopRight     = [[NSCursor arrowCursor] retain]; //osx not have one
    m_CursorTopLeft      = [[NSCursor arrowCursor] retain]; //osx not have one
    m_CursorBottomRight  = [[NSCursor arrowCursor] retain]; //osx not have one
    m_CursorBottomLeft   = [[NSCursor arrowCursor] retain]; //osx not have one
    m_CursorHelp         = [[NSCursor arrowCursor] retain]; //osx not have one
    m_CursorHand         = [[NSCursor pointingHandCursor] retain];
    m_CursorCross        = [[NSCursor arrowCursor] retain];
    m_CursorUpArrow      = [[NSCursor arrowCursor] retain];
    m_CursorNo           = [[NSCursor arrowCursor] retain];
    m_CursorIBeam        = [[NSCursor IBeamCursor] retain];
    for (int i=0;i<NB_ROTO_CURSORS; i++)
    {
        m_RotoCursors[i] = [PixmapCursor(i+2) retain];
    }
    m_CursorCenter  = [PixmapCursor(0) retain];
    m_CursorPoint   = [PixmapCursor(1) retain];
    m_CursorsCreated = true;
}

void CTwMgr::FreeCursors()
{
    [m_CursorArrow release];
    [m_CursorMove release];
    [m_CursorWE release];
    [m_CursorNS release];
    [m_CursorTopRight release];
    [m_CursorTopLeft release];
    [m_CursorBottomRight release];
    [m_CursorBottomLeft release];
    [m_CursorHelp release];
    [m_CursorHand release];
    [m_CursorCross release];
    [m_CursorUpArrow release];
    [m_CursorNo release];
    [m_CursorIBeam release];
    for( int i=0; i<NB_ROTO_CURSORS; ++i )
        [m_RotoCursors[i] release]; 
    [m_CursorCenter release];
    [m_CursorPoint release];
    m_CursorsCreated = false;
}

void CTwMgr::SetCursor(CTwMgr::CCursor _Cursor)
{
    if (m_CursorsCreated && _Cursor) {
        [_Cursor set];
    }
}


#elif defined(ANT_UNIX)

#include "res/TwXCursors.h"

static XErrorHandler s_PrevErrorHandler = NULL;

static int InactiveErrorHandler(Display *display, XErrorEvent *err)
{
    fprintf(stderr, "Ignoring Xlib error: error code %d request code %d\n", err->error_code, err->request_code);
    // No exit!
    return 0 ;
}

static void IgnoreXErrors()
{
    if( g_TwMgr!=NULL && g_TwMgr->m_CurrentXDisplay==glXGetCurrentDisplay() )
    {
        XFlush(g_TwMgr->m_CurrentXDisplay);
        XSync(g_TwMgr->m_CurrentXDisplay, False);
    }
    s_PrevErrorHandler = XSetErrorHandler(InactiveErrorHandler);
}

static void RestoreXErrors()
{
    if( g_TwMgr!=NULL && g_TwMgr->m_CurrentXDisplay==glXGetCurrentDisplay() )
    {
        XFlush(g_TwMgr->m_CurrentXDisplay);
        XSync(g_TwMgr->m_CurrentXDisplay, False);
    }
    XSetErrorHandler(s_PrevErrorHandler);
}

CTwMgr::CCursor CTwMgr::PixmapCursor(int _CurIdx)
{ 
    if( !m_CurrentXDisplay || !m_CurrentXWindow )
        return XC_left_ptr;
        
    IgnoreXErrors();    

    XColor black, white, exact;
    Colormap colmap = DefaultColormap(m_CurrentXDisplay, DefaultScreen(m_CurrentXDisplay));
    Status s1 = XAllocNamedColor(m_CurrentXDisplay, colmap, "black", &black, &exact);
    Status s2 = XAllocNamedColor(m_CurrentXDisplay, colmap, "white", &white, &exact);
    if( s1==0 || s2==0 )
        return XC_left_ptr; // cannot allocate colors!
    int x, y;
    unsigned int mask[32];
    unsigned int pict[32];
    for( y=0; y<32; ++y )
    {
        mask[y] = pict[y] = 0;
        for( x=0; x<32; ++x )
        {
            mask[y] |= (((unsigned int)(g_CurMask[_CurIdx][x+y*32]))<<x);
            pict[y] |= (((unsigned int)(g_CurPict[_CurIdx][x+y*32]))<<x);
        }
    }       
    Pixmap maskPix = XCreateBitmapFromData(m_CurrentXDisplay, m_CurrentXWindow, (char*)mask, 32, 32);
    Pixmap pictPix = XCreateBitmapFromData(m_CurrentXDisplay, m_CurrentXWindow, (char*)pict, 32, 32);
    Cursor cursor = XCreatePixmapCursor(m_CurrentXDisplay, pictPix, maskPix, &white, &black, g_CurHot[_CurIdx][0], g_CurHot[_CurIdx][1]);
    XFreePixmap(m_CurrentXDisplay, maskPix);
    XFreePixmap(m_CurrentXDisplay, pictPix);
    
    RestoreXErrors();
    
    if( cursor!=0 )
        return cursor;
    else
        return XC_left_ptr;
}

void CTwMgr::CreateCursors()
{
    if( m_CursorsCreated || !m_CurrentXDisplay || !m_CurrentXWindow )
        return;

    IgnoreXErrors();
    m_CursorArrow   = XCreateFontCursor(m_CurrentXDisplay, XC_left_ptr);
    m_CursorMove    = XCreateFontCursor(m_CurrentXDisplay, XC_plus);
    m_CursorWE      = XCreateFontCursor(m_CurrentXDisplay, XC_left_side);
    m_CursorNS      = XCreateFontCursor(m_CurrentXDisplay, XC_top_side);
    m_CursorTopRight= XCreateFontCursor(m_CurrentXDisplay, XC_top_right_corner);
    m_CursorTopLeft = XCreateFontCursor(m_CurrentXDisplay, XC_top_left_corner);
    m_CursorBottomRight = XCreateFontCursor(m_CurrentXDisplay, XC_bottom_right_corner);
    m_CursorBottomLeft  = XCreateFontCursor(m_CurrentXDisplay, XC_bottom_left_corner);
    m_CursorHelp    = XCreateFontCursor(m_CurrentXDisplay, XC_question_arrow);
    m_CursorHand    = XCreateFontCursor(m_CurrentXDisplay, XC_hand1);
    m_CursorCross   = XCreateFontCursor(m_CurrentXDisplay, XC_X_cursor);
    m_CursorUpArrow = XCreateFontCursor(m_CurrentXDisplay, XC_center_ptr);
    m_CursorNo      = XCreateFontCursor(m_CurrentXDisplay, XC_left_ptr);
    m_CursorIBeam   = XCreateFontCursor(m_CurrentXDisplay, XC_xterm);
    for( int i=0; i<NB_ROTO_CURSORS; ++i )
    {
        m_RotoCursors[i] = PixmapCursor(i+2);
    }
    m_CursorCenter  = PixmapCursor(0);
    m_CursorPoint   = PixmapCursor(1);
    m_CursorsCreated = true;
    
    RestoreXErrors();
}

void CTwMgr::FreeCursors()
{
    IgnoreXErrors();
    
    XFreeCursor(m_CurrentXDisplay, m_CursorArrow);
    XFreeCursor(m_CurrentXDisplay, m_CursorMove);
    XFreeCursor(m_CurrentXDisplay, m_CursorWE);
    XFreeCursor(m_CurrentXDisplay, m_CursorNS);
    XFreeCursor(m_CurrentXDisplay, m_CursorTopRight);
    XFreeCursor(m_CurrentXDisplay, m_CursorTopLeft);
    XFreeCursor(m_CurrentXDisplay, m_CursorBottomRight);
    XFreeCursor(m_CurrentXDisplay, m_CursorBottomLeft); 
    XFreeCursor(m_CurrentXDisplay, m_CursorHelp);
    XFreeCursor(m_CurrentXDisplay, m_CursorHand);
    XFreeCursor(m_CurrentXDisplay, m_CursorCross);
    XFreeCursor(m_CurrentXDisplay, m_CursorUpArrow);
    XFreeCursor(m_CurrentXDisplay, m_CursorNo); 
    for( int i=0; i<NB_ROTO_CURSORS; ++i )
        XFreeCursor(m_CurrentXDisplay, m_RotoCursors[i]);
    XFreeCursor(m_CurrentXDisplay, m_CursorCenter);
    XFreeCursor(m_CurrentXDisplay, m_CursorPoint);          

    m_CursorsCreated = false;
    
    RestoreXErrors();   
}

void CTwMgr::SetCursor(CTwMgr::CCursor _Cursor)
{
    if( m_CursorsCreated && m_CurrentXDisplay && m_CurrentXWindow )
    {
        Display *dpy = glXGetCurrentDisplay();
        if( dpy==g_TwMgr->m_CurrentXDisplay )
        {
            Window wnd = glXGetCurrentDrawable();
            if( wnd!=g_TwMgr->m_CurrentXWindow )
            {
                FreeCursors();
                g_TwMgr->m_CurrentXWindow = wnd;
                CreateCursors();
                // now _Cursor is not a valid cursor ID.
            }
            else
            {
                IgnoreXErrors();
                XDefineCursor(m_CurrentXDisplay, m_CurrentXWindow, _Cursor);
                RestoreXErrors();
            }
        }
    }
}

#endif //defined(ANT_UNIX)

//  ---------------------------------------------------------------------------

void ANT_CALL TwCopyCDStringToClientFunc(TwCopyCDStringToClient copyCDStringToClientFunc)
{
    g_InitCopyCDStringToClient = copyCDStringToClientFunc;
    if( g_TwMgr!=NULL )
        g_TwMgr->m_CopyCDStringToClient = copyCDStringToClientFunc;
}

void ANT_CALL TwCopyCDStringToLibrary(char **destinationLibraryStringPtr, const char *sourceClientString)
{
    if( g_TwMgr==NULL )
    {
        if( destinationLibraryStringPtr!=NULL )
            *destinationLibraryStringPtr = const_cast<char *>(sourceClientString);
        return;
    }

    // static buffer to store sourceClientString copy associated to sourceClientString pointer
    std::vector<char>& Buf = g_TwMgr->m_CDStdStringCopyBuffers[(void *)sourceClientString];

    size_t len = (sourceClientString!=NULL) ? strlen(sourceClientString) : 0;
    if( Buf.size()<len+1 )
        Buf.resize(len+128); // len + some margin
    char *SrcStrCopy = &(Buf[0]);
    SrcStrCopy[0] = '\0';
    if( sourceClientString!=NULL )
        memcpy(SrcStrCopy, sourceClientString, len+1);
    SrcStrCopy[len] = '\0';
    if( destinationLibraryStringPtr!=NULL )
        *destinationLibraryStringPtr = SrcStrCopy;
}

void ANT_CALL TwCopyStdStringToClientFunc(TwCopyStdStringToClient copyStdStringToClientFunc)
{
    g_InitCopyStdStringToClient = copyStdStringToClientFunc;
    if( g_TwMgr!=NULL )
        g_TwMgr->m_CopyStdStringToClient = copyStdStringToClientFunc;
}

void ANT_CALL TwCopyStdStringToLibrary(std::string& destLibraryString, const std::string& srcClientString)
{
    /*
    // check if destLibraryString should be initialized
    char *Mem = (char *)&destLibraryString;
    bool Init = true;
    for( int i=0; i<sizeof(std::string) && Init; ++i )
        if( Mem[i]!=0 )
            Init = false; // has already been initialized
    assert( !Init );
    //  ::new(&destLibraryString) std::string;
    
    // copy string
    destLibraryString = srcClientString;
    */

    if( g_TwMgr==NULL )
        return;

    CTwMgr::CLibStdString srcLibString; // Convert VC++ Debug/Release std::string
    srcLibString.FromClient(srcClientString);
    const char *SrcStr = srcLibString.ToLib().c_str();
    const char **DstStrPtr = (const char **)&destLibraryString;

    // SrcStr can be defined locally by the caller, so we need to copy it
    // ( *DstStrPtr = copy of SrcStr )

    // static buffer to store srcClientString copy associated to srcClientString pointer
    std::vector<char>& Buf = g_TwMgr->m_CDStdStringCopyBuffers[(void *)&srcClientString];

    size_t len = strlen(SrcStr);
    if( Buf.size()<len+1 )
        Buf.resize(len+128); // len + some margin
    char *SrcStrCopy = &(Buf[0]);

    memcpy(SrcStrCopy, SrcStr, len+1);
    SrcStrCopy[len] = '\0';
    *DstStrPtr = SrcStrCopy;
    //*(const char **)&destLibraryString = srcClientString.c_str();
}

//  ---------------------------------------------------------------------------

bool CRect::Subtract(const CRect& _Rect, vector<CRect>& _OutRects) const
{
    if( Empty() )
        return false;
    if( _Rect.Empty() || _Rect.Y>=Y+H || _Rect.Y+_Rect.H<=Y || _Rect.X>=X+W || _Rect.X+_Rect.W<=X )
    {
        _OutRects.push_back(*this);
        return true;
    }

    bool Ret = false;
    int Y0 = Y;
    int Y1 = Y+H-1;
    if( _Rect.Y>Y )
    {
        Y0 = _Rect.Y;
        _OutRects.push_back(CRect(X, Y, W, Y0-Y+1));
        Ret = true;
    }
    if( _Rect.Y+_Rect.H<Y+H )
    {
        Y1 = _Rect.Y+_Rect.H;
        _OutRects.push_back(CRect(X, Y1, W, Y+H-Y1));
        Ret = true;
    }
    int X0 = X;
    int X1 = X+W-1;
    if( _Rect.X>X )
    {
        X0 = _Rect.X; //-2;
        _OutRects.push_back(CRect(X, Y0, X0-X+1, Y1-Y0+1));
        Ret = true;
    }
    if( _Rect.X+_Rect.W<X+W )
    {
        X1 = _Rect.X+_Rect.W; //-1;
        _OutRects.push_back(CRect(X1, Y0, X+W-X1, Y1-Y0+1));
        Ret = true;
    }
    return Ret;
}

bool CRect::Subtract(const vector<CRect>& _Rects, vector<CRect>& _OutRects) const
{
    _OutRects.clear();
    size_t i, j, NbRects = _Rects.size();
    if( NbRects==0 )
    {
        _OutRects.push_back(*this);
        return true;
    }
    else
    {
        vector<CRect> TmpRects;
        Subtract(_Rects[0], _OutRects);
        
        for( i=1; i<NbRects; i++)
        {
            for( j=0; j<_OutRects.size(); j++ )
                _OutRects[j].Subtract(_Rects[i], TmpRects);
            _OutRects.swap(TmpRects);
            TmpRects.clear();
        }
        return _OutRects.empty();
    }
}

//  ---------------------------------------------------------------------------

