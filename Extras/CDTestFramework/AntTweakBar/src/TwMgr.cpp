//	---------------------------------------------------------------------------
//
//	@file		TwMgr.cpp
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
#include "TwFonts.h"
#include "TwOpenGL.h"
#ifdef ANT_WINDOWS
#	include "TwDirect3D9.h"
#	include "resource.h"
#	ifdef _DEBUG
#		include <crtdbg.h>
#	endif // _DEBUG
#endif // ANT_WINDOWS

using namespace std;


CTwMgr *g_TwMgr = NULL;
bool g_BreakOnError = false;
TwErrorHandler g_ErrorHandler = NULL;
int g_TabLength = 4;
CTwBar * const TW_GLOBAL_BAR = (CTwBar *)(-1);
int g_InitWndWidth = -1;
int g_InitWndHeight = -1;

extern const char *g_ErrUnknownAttrib;
extern const char *g_ErrNoValue;
extern const char *g_ErrBadValue;
const char *g_ErrInit		= "Already initialized";
const char *g_ErrShut		= "Already shutdown";
const char *g_ErrNotInit	= "Not initialized";
const char *g_ErrUnknownAPI	= "Unsupported graph API";
const char *g_ErrBadDevice	= "Invalid graph device";
const char *g_ErrBadParam	= "Invalid parameter";
const char *g_ErrExist		= "Exists already";
const char *g_ErrNotFound	= "Not found";
const char *g_ErrNthToDo	= "Nothing to do";
const char *g_ErrBadWndSize = "Bad window size";
const char *g_ErrOffset		= "Offset larger than StructSize";
const char *g_ErrDelStruct  = "Cannot delete a struct member";
const char *g_ErrNoBackQuote= "Name cannot include back-quote";
char g_ErrParse[512];

void ANT_CALL TwGlobalError(const char *_ErrorMessage);

#if defined(_UNIX)
#define _stricmp strcasecmp
#define _strdup strdup
#endif


//	---------------------------------------------------------------------------

//	a static global object to verify that Tweakbar module has been properly terminated (in debug mode only)
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

//	---------------------------------------------------------------------------

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
		if( g_TwMgr && g_TwMgr->m_GraphAPI!=TW_OPENGL )
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
	_SummaryString[0] = ' ';	// required to force background color for this value
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
	g_TwMgr->m_TypeColor32 = TwDefineStructExt("COLOR32", ColorExtMembers, 8, sizeof(unsigned int), sizeof(CColorExt), CColorExt::InitColor32CB, CColorExt::CopyVarFromExtCB, CColorExt::CopyVarToExtCB, CColorExt::SummaryCB, CTwMgr::CStruct::s_PassProxyAsClientData);
	g_TwMgr->m_TypeColor3F = TwDefineStructExt("COLOR3F", ColorExtMembers, 8, 3*sizeof(float), sizeof(CColorExt), CColorExt::InitColor3FCB, CColorExt::CopyVarFromExtCB, CColorExt::CopyVarToExtCB, CColorExt::SummaryCB, CTwMgr::CStruct::s_PassProxyAsClientData);
	g_TwMgr->m_TypeColor4F = TwDefineStructExt("COLOR4F", ColorExtMembers, 8, 4*sizeof(float), sizeof(CColorExt), CColorExt::InitColor4FCB, CColorExt::CopyVarFromExtCB, CColorExt::CopyVarToExtCB, CColorExt::SummaryCB, CTwMgr::CStruct::s_PassProxyAsClientData);
}


//	---------------------------------------------------------------------------

static int TwCreateGraph(ETwGraphAPI _GraphAPI)
{
	assert( g_TwMgr!=NULL && g_TwMgr->m_Graph==NULL );

	switch( _GraphAPI )
	{
	case TW_OPENGL:
		g_TwMgr->m_Graph = new CTwGraphOpenGL;
		break;
	case TW_DIRECT3D9:
		#ifdef ANT_WINDOWS
			if( g_TwMgr->m_Device!=NULL )
				g_TwMgr->m_Graph = new CTwGraphDirect3D9;
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

//	---------------------------------------------------------------------------

int ANT_CALL TwInit(ETwGraphAPI _GraphAPI, void *_Device)
{
#if defined(_DEBUG) && defined(ANT_WINDOWS)
	_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF|_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF));
#endif

	if( g_TwMgr!=NULL )
	{
		g_TwMgr->SetLastError(g_ErrInit);
		return 0;
	}

	g_TwMgr = new CTwMgr(_GraphAPI, _Device);

	TwGenerateDefaultFonts();
	g_TwMgr->m_CurrentFont = g_DefaultNormalFont;

	int Res = TwCreateGraph(_GraphAPI);

	if( Res )
	{
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
			g_TwMgr->m_HelpBar->m_Color = 0xd7ffffff;
			g_TwMgr->m_HelpBar->m_IsHelpBar = true;
			g_TwMgr->Minimize(g_TwMgr->m_HelpBar);
		}
		else
		{
			TwTerminate();
			Res = 0;
		}
	}

	if( Res )
		CColorExt::CreateTypes();

	return Res;
}

//	---------------------------------------------------------------------------

int ANT_CALL TwTerminate()
{
	if( g_TwMgr==NULL )
	{
		//TwGlobalError(g_ErrShut); -> not an error
		return 0;  // already shutdown
	}

	TwDeleteAllBars();
	if( g_TwMgr->m_CursorsCreated )
		g_TwMgr->FreeCursors();

	int Res = 1;
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
		Res = g_TwMgr->m_Graph->Shut();
		delete g_TwMgr->m_Graph;
		g_TwMgr->m_Graph = NULL;
	}

	TwDeleteDefaultFonts();

	delete g_TwMgr;
	g_TwMgr = NULL;

	return Res;
}

//	---------------------------------------------------------------------------

int ANT_CALL TwDraw()
{
	PERF( PerfTimer Timer; double DT; )
	//CTwFPU fpu;	// fpu precision only forced in update (do not modif dx draw calls)

	if( g_TwMgr==NULL || g_TwMgr->m_Graph==NULL )
	{
		TwGlobalError(g_ErrNotInit);
		return 0; // not initialized
	}

	assert(g_TwMgr->m_Bars.size()==g_TwMgr->m_Order.size());

	// Create cursors
	#if defined(ANT_WINDOWS)
		if( !g_TwMgr->m_CursorsCreated )
			g_TwMgr->CreateCursors();
	#elif defined(ANT_UNIX)
		if( !g_TwMgr->m_CurrentXDisplay )
			g_TwMgr->m_CurrentXDisplay = glXGetCurrentDisplay();
		if( !g_TwMgr->m_CurrentXWindow )
			g_TwMgr->m_CurrentXWindow = glXGetCurrentDrawable();
		if( g_TwMgr->m_CurrentXDisplay && !g_TwMgr->m_CursorsCreated )
			g_TwMgr->CreateCursors();
	#endif // defined(ANT_UNIX)

	// Autorepeat TW_MOUSE_PRESSED
	double RepeatDT = g_TwMgr->m_Timer.GetTime() - g_TwMgr->m_LastMousePressedTime;
	if(    fabs(RepeatDT)>2.0*g_TwMgr->m_RepeatMousePressedDelay 
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


	if( g_TwMgr->m_WndWidth<0 || g_TwMgr->m_WndHeight<0 )
	{
		g_TwMgr->SetLastError(g_ErrBadWndSize);
		return 0;
	}
	else if( g_TwMgr->m_WndWidth==0 || g_TwMgr->m_WndHeight==0 )	// probably iconified
		return 1;	// nothing to do

	// count number of bars to draw
	size_t i, idx;
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
		for( i=0; i<g_TwMgr->m_Bars.size(); ++i )
		{
			idx = g_TwMgr->m_Order[i];
			if( g_TwMgr->m_Bars[idx]->m_Visible )
			{
				g_TwMgr->m_Bars[idx]->Draw();
			}
		}
		PERF( DT = Timer.GetTime(); printf("Draw=%.4fms ", 1000.0*DT); )

		PERF( Timer.Reset(); )
		g_TwMgr->m_Graph->EndDraw();
		PERF( DT = Timer.GetTime(); printf("End=%.4fms\n", 1000.0*DT); )
	}

	return 1;
}

//	---------------------------------------------------------------------------

int ANT_CALL TwWindowSize(int _Width, int _Height)
{
	g_InitWndWidth = _Width;
	g_InitWndHeight = _Height;

	if( g_TwMgr==NULL || g_TwMgr->m_Graph==NULL )
	{
		//TwGlobalError(g_ErrNotInit);	-> not an error here
		return 0;  // not initialized
	}

	if( _Width<0 || _Height<0 )
	{
		g_TwMgr->SetLastError(g_ErrBadWndSize);
		return 0;
	}

	g_TwMgr->m_WndWidth = _Width;
	g_TwMgr->m_WndHeight = _Height;
	g_TwMgr->m_Graph->Restore();
	for( std::vector<TwBar*>::iterator it=g_TwMgr->m_Bars.begin(); it!=g_TwMgr->m_Bars.end(); ++it )
		(*it)->NotUpToDate();
	return 1;
}

//	---------------------------------------------------------------------------

CTwMgr::CTwMgr(ETwGraphAPI _GraphAPI, void *_Device)
{
	m_GraphAPI = _GraphAPI;
	m_Device = _Device;
	m_LastError = NULL;
	m_CurrentDbgFile = "";
	m_CurrentDbgLine = 0;
	m_Graph = NULL;
	m_WndWidth = g_InitWndWidth;
	m_WndHeight = g_InitWndHeight;
	m_CurrentFont = NULL;	// set after by TwIntialize
	m_NbMinimizedBars = 0;
	m_HelpBar = NULL;
	m_HelpBarNotUpToDate = true;
	m_HelpBarUpdateNow = false;
	m_LastHelpUpdateTime = 0;
	m_LastMouseX = -1;
	m_LastMouseY = -1;
	m_LastMouseWheelPos = 0;
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
	
	m_CursorsCreated = false;	
	#if defined(ANT_UNIX)
		m_CurrentXDisplay = NULL;
		m_CurrentXWindow = 0;
	#endif	// defined(ANT_UNIX)
}

//	---------------------------------------------------------------------------

CTwMgr::~CTwMgr()
{
}

//	---------------------------------------------------------------------------

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


//	---------------------------------------------------------------------------

enum EMgrAttribs
{
	MGR_HELP = 1,
};

int CTwMgr::HasAttrib(const char *_Attrib, bool *_HasValue) const
{
	*_HasValue = true;
	if( _stricmp(_Attrib, "help")==0 )
		return MGR_HELP;

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
			g_TwMgr->SetLastError(g_ErrNoValue);
			return 0;
		}
	default:
		g_TwMgr->SetLastError(g_ErrUnknownAttrib);
		return 0;
	}
}

//	---------------------------------------------------------------------------

void CTwMgr::Minimize(TwBar *_Bar)
{
	assert(m_Graph!=NULL && _Bar!=NULL);
	assert(m_Bars.size()==m_MinOccupied.size());
	if( _Bar->m_IsMinimized )
		return;
	size_t i = m_NbMinimizedBars;
	m_NbMinimizedBars++;
	for( i=0; i<m_MinOccupied.size(); ++i )
		if( !m_MinOccupied[i] )
			break;
	if( i<m_MinOccupied.size() )
		m_MinOccupied[i] = true;
	_Bar->m_MinNumber = (int)i;
	_Bar->m_IsMinimized = true;
	_Bar->NotUpToDate();
}

//	---------------------------------------------------------------------------

void CTwMgr::Maximize(TwBar *_Bar)
{
	assert(m_Graph!=NULL && _Bar!=NULL);
	assert(m_Bars.size()==m_MinOccupied.size());
	if( !_Bar->m_IsMinimized )
		return;
	--m_NbMinimizedBars;
	if( m_NbMinimizedBars<0 )
		m_NbMinimizedBars = 0;
	if( _Bar->m_MinNumber>=0 && _Bar->m_MinNumber<(int)m_MinOccupied.size() )
		m_MinOccupied[_Bar->m_MinNumber] = false;
	_Bar->m_IsMinimized = false;
	_Bar->NotUpToDate();
	if( _Bar->m_IsHelpBar )
		m_HelpBarNotUpToDate = true;
}

//	---------------------------------------------------------------------------

void CTwMgr::Hide(TwBar *_Bar)
{
	assert(m_Graph!=NULL && _Bar!=NULL);
	if( !_Bar->m_Visible )
		return;
	_Bar->m_Visible = false;
	if( !_Bar->m_IsHelpBar )
		m_HelpBarNotUpToDate = true;
}

//	---------------------------------------------------------------------------

void CTwMgr::Unhide(TwBar *_Bar)
{
	assert(m_Graph!=NULL && _Bar!=NULL);
	if( _Bar->m_Visible )
		return;
	_Bar->m_Visible = true;
	_Bar->NotUpToDate();
	if( !_Bar->m_IsHelpBar )
		m_HelpBarNotUpToDate = true;
}

//	---------------------------------------------------------------------------

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
				m_Bars[i]->m_PosX += (3*(fh-_Font->m_CharHeight))/2;
				m_Bars[i]->m_PosY += (fh-_Font->m_CharHeight)/2;
				m_Bars[i]->m_Width = (m_Bars[i]->m_Width*_Font->m_CharHeight)/fh;
				m_Bars[i]->m_Height = (m_Bars[i]->m_Height*_Font->m_CharHeight)/fh;
				m_Bars[i]->m_ValuesWidth = (m_Bars[i]->m_ValuesWidth*_Font->m_CharHeight)/fh;
			}
			m_Bars[i]->NotUpToDate();
		}

	if( g_TwMgr->m_HelpBar!=NULL )
		g_TwMgr->m_HelpBar->Update();
	g_TwMgr->m_InfoBuildText = true;
	g_TwMgr->m_KeyPressedBuildText = true;
	m_HelpBarNotUpToDate = true;
}

//	---------------------------------------------------------------------------

void ANT_CALL TwGlobalError(const char *_ErrorMessage)	// to be called when g_TwMgr is not created
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

//	---------------------------------------------------------------------------

void CTwMgr::SetLastError(const char *_ErrorMessage)	// _ErrorMessage must be a static string
{
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

//	---------------------------------------------------------------------------

const char *CTwMgr::GetLastError()
{
	const char *Err = m_LastError;
	m_LastError = NULL;
	return Err;
}

//	---------------------------------------------------------------------------

const char *CTwMgr::CheckLastError() const
{
	return m_LastError;
}

//	---------------------------------------------------------------------------

void CTwMgr::SetCurrentDbgParams(const char *dbgFile, int dbgLine)
{
	m_CurrentDbgFile = dbgFile;
	m_CurrentDbgLine = dbgLine;
}

//	---------------------------------------------------------------------------

int ANT_CALL __TwDbg(const char *dbgFile, int dbgLine)
{
	if( g_TwMgr!=NULL )
		g_TwMgr->SetCurrentDbgParams(dbgFile, dbgLine);
	return 0;	// always returns zero
}

//	---------------------------------------------------------------------------

void ANT_CALL TwHandleErrors(TwErrorHandler _ErrorHandler, int _BreakOnError)
{
	g_ErrorHandler = _ErrorHandler;
	g_BreakOnError = (_BreakOnError) ? true : false;
}

//	---------------------------------------------------------------------------

const char *ANT_CALL TwGetLastError()
{
	if( g_TwMgr==NULL )
	{
		TwGlobalError(g_ErrNotInit);
		return g_ErrNotInit;
	}
	else
		return g_TwMgr->GetLastError();
}

//	---------------------------------------------------------------------------

TwBar *ANT_CALL TwNewBar(const char *_Name)
{
	if( g_TwMgr==NULL || g_TwMgr->m_Graph==NULL )
	{
		TwGlobalError(g_ErrNotInit);
		return NULL; // not initialized
	}

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

	if( g_TwMgr->m_PopupBar!=NULL )	// delete popup bar if it exists
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

//	---------------------------------------------------------------------------

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

	if( g_TwMgr->m_PopupBar!=NULL && _Bar!=g_TwMgr->m_PopupBar )	// delete popup bar first if it exists
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

//	---------------------------------------------------------------------------

int ANT_CALL TwDeleteAllBars()
{
	if( g_TwMgr==NULL )
	{
		TwGlobalError(g_ErrNotInit);
		return 0; // not initialized
	}

	int n = 0;
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

	if( n==0 )
	{
		g_TwMgr->SetLastError(g_ErrNthToDo);
		return 0;
	}
	else
		return 1;
}

//	---------------------------------------------------------------------------

int	ANT_CALL TwSetTopBar(const TwBar *_Bar)
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

	int i = -1, iOrder;
	for( iOrder=0; iOrder<(int)g_TwMgr->m_Bars.size(); ++iOrder )
	{
		i = g_TwMgr->m_Order[iOrder];
		assert( i>=0 && i<(int)g_TwMgr->m_Bars.size() );
		if( g_TwMgr->m_Bars[i]==_Bar )
			break;
	}
	if( i<0 || iOrder>=(int)g_TwMgr->m_Bars.size() )	// bar not found
	{
		g_TwMgr->SetLastError(g_ErrNotFound);
		return 0;
	}

	for( int j=iOrder; j<(int)g_TwMgr->m_Bars.size()-1; ++j )
		g_TwMgr->m_Order[j] = g_TwMgr->m_Order[j+1];
	g_TwMgr->m_Order[(int)g_TwMgr->m_Bars.size()-1] = i;

	if( g_TwMgr->m_PopupBar!=NULL && _Bar!=g_TwMgr->m_PopupBar )
		TwSetTopBar(g_TwMgr->m_PopupBar);

	return 1;
}

//	---------------------------------------------------------------------------

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

//	---------------------------------------------------------------------------

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

	switch( _State )
	{
	case TW_STATE_SHOWN:
		g_TwMgr->Unhide(_Bar);
		return 1;
	case TW_STATE_ICONIFIED:
		g_TwMgr->Unhide(_Bar);
		g_TwMgr->Minimize(_Bar);
		return 1;
	case TW_STATE_HIDDEN:
		g_TwMgr->Maximize(_Bar);
		g_TwMgr->Hide(_Bar);
		return 1;
	default:
		g_TwMgr->SetLastError(g_ErrBadParam);
		return 0;
	}
}

//	---------------------------------------------------------------------------

TwState	ANT_CALL TwGetBarState(const TwBar *_Bar)
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

//	---------------------------------------------------------------------------

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

	return _Bar->m_Name.c_str();
}

//	---------------------------------------------------------------------------

static int s_PassProxy = 0;
void *CTwMgr::CStruct::s_PassProxyAsClientData = &s_PassProxy;	// special tag

CTwMgr::CStructProxy::CStructProxy()
{ 
	memset(this, 0, sizeof(*this)); 
}

CTwMgr::CStructProxy::~CStructProxy() 
{ 
	if( m_StructData!=NULL && m_DeleteStructData ) 
		delete[] (char*)m_StructData;
	if( m_StructExtData!=NULL )
		delete[] (char*)m_StructExtData;
	memset(this, 0, sizeof(*this));
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
							sProxy->m_StructSetCallback(sProxy->m_StructData, sProxy->m_StructClientData);
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

//	---------------------------------------------------------------------------

static int AddVar(TwBar *_Bar, const char *_Name, ETwType _Type, void *_VarPtr, bool _ReadOnly, TwSetVarCallback _SetCallback, TwGetVarCallback _GetCallback, TwButtonCallback _ButtonCallback, void *_ClientData, const char *_Def)
{
	CTwFPU fpu;	// force fpu precision

	if( g_TwMgr==NULL )
	{
		TwGlobalError(g_ErrNotInit);
		return 0; // not initialized
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
		_ReadOnly = true;	// force readonly in this case

	// Convert color types
	if( _Type==TW_TYPE_COLOR32 )
		_Type = g_TwMgr->m_TypeColor32;
	else if( _Type==TW_TYPE_COLOR3F )
		_Type = g_TwMgr->m_TypeColor3F;
	else if( _Type==TW_TYPE_COLOR4F )
		_Type = g_TwMgr->m_TypeColor4F;

	if( _Type<TW_TYPE_STRUCT_BASE || (_Type>=TW_TYPE_ENUM_BASE && _Type<TW_TYPE_ENUM_BASE+(int)g_TwMgr->m_Enums.size()) )
	{
		CTwVarAtom *Var = new CTwVarAtom;
		Var->m_Name = _Name;
		Var->m_Ptr = _VarPtr;
		Var->m_Type = _Type;
		if( _VarPtr!=NULL )
		{
			assert( _GetCallback==NULL && _SetCallback==NULL && _ButtonCallback==NULL );

			Var->m_ReadOnly = _ReadOnly;
			Var->m_GetCallback = NULL;
			Var->m_SetCallback = NULL;
			Var->m_ButtonCallback = NULL;
			Var->m_ClientData = NULL;
		}
		else
		{
			assert( _GetCallback!=NULL || _Type==TW_TYPE_BUTTON );

			Var->m_GetCallback = _GetCallback;
			Var->m_SetCallback = _SetCallback;
			Var->m_ButtonCallback = _ButtonCallback;
			Var->m_ClientData = _ClientData;
			if( _Type!=TW_TYPE_BUTTON )
				Var->m_ReadOnly = (_SetCallback==NULL || _ReadOnly);
			else
				Var->m_ReadOnly = (_ButtonCallback==NULL);
		}
		Var->m_Color = _Bar->m_ColLabelText;
		Var->SetDefaults();

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
			}
		}
		else // s.m_IsExt
		{
			assert( s.m_Size>0 && s.m_ClientStructSize>0 );
			vPtr = new char[s.m_Size];	// will be m_StructExtData
			memset(vPtr, 0, s.m_Size);
			// create a new StructProxy
			g_TwMgr->m_StructProxies.push_back(CTwMgr::CStructProxy());
			sProxy = &(g_TwMgr->m_StructProxies.back());
			sProxy->m_Type = _Type;
			sProxy->m_StructExtData = vPtr;
			sProxy->m_StructSetCallback = _SetCallback;
			sProxy->m_StructGetCallback = _GetCallback;
			sProxy->m_StructClientData = _ClientData;
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
			}
			_VarPtr = NULL;	// force use of TwAddVarCB for members

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
				if( TwAddVarCB(_Bar, name.c_str(), m.m_Type, CTwMgr::CMemberProxy::SetCB, CTwMgr::CMemberProxy::GetCB, &mProxy, def.c_str())==0 )
					return 0;
				mProxy.m_Var = _Bar->Find(name.c_str(), &mProxy.m_VarParent, NULL);
				mProxy.m_Bar = _Bar;
			}
		}
		char structInfo[64];
		sprintf(structInfo, "typeid=%d valptr=%p close ", _Type, vPtr);
		string grpDef = '`' + _Bar->m_Name + "`/`" + _Name + "` " + structInfo;
		if( _Def!=NULL && strlen(_Def)>0 )
			grpDef += _Def;
		if( TwDefine(grpDef.c_str()) )
		{
			for( int i=0; i<(int)s.m_Members.size(); ++i )
			{
				CTwMgr::CStructMember& m = s.m_Members[i];
				if( m.m_DefString.length()>0 )
				{
					string memberDef = '`' + _Bar->m_Name + "`/`" + _Name + '.' + m.m_Name + "` " + m.m_DefString;
					if( !TwDefine(memberDef.c_str()) )
						return 0;
				}
			}
			return 1;
		}
		else
			return 0;
	}
	else
	{
		g_TwMgr->SetLastError(g_ErrNotFound);
		return 0;
	}
}

//	---------------------------------------------------------------------------

int ANT_CALL TwAddVarRW(TwBar *_Bar, const char *_Name, ETwType _Type, void *_Var, const char *_Def)
{
	return AddVar(_Bar, _Name, _Type, _Var, false, NULL, NULL, NULL, NULL, _Def);
}

//	---------------------------------------------------------------------------

int ANT_CALL TwAddVarRO(TwBar *_Bar, const char *_Name, ETwType _Type, const void *_Var, const char *_Def)
{
	return AddVar(_Bar, _Name, _Type, const_cast<void *>(_Var), true, NULL, NULL, NULL, NULL, _Def);
}

//	---------------------------------------------------------------------------

int ANT_CALL TwAddVarCB(TwBar *_Bar, const char *_Name, ETwType _Type, TwSetVarCallback _SetCallback, TwGetVarCallback _GetCallback, void *_ClientData, const char *_Def)
{
	return AddVar(_Bar, _Name, _Type, NULL, false, _SetCallback, _GetCallback, NULL, _ClientData, _Def);
}

//	---------------------------------------------------------------------------

int ANT_CALL TwAddButton(TwBar *_Bar, const char *_Name, TwButtonCallback _Callback, void *_ClientData, const char *_Def)
{
	return AddVar(_Bar, _Name, TW_TYPE_BUTTON, NULL, false, NULL, NULL, _Callback, _ClientData, _Def);
}

//	---------------------------------------------------------------------------

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

	if( g_TwMgr->m_PopupBar!=NULL && _Bar!=g_TwMgr->m_PopupBar )	// delete popup bar first if it exists
	{
		TwDeleteBar(g_TwMgr->m_PopupBar);
		g_TwMgr->m_PopupBar = NULL;
	}

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

//	---------------------------------------------------------------------------

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

	if( g_TwMgr->m_PopupBar!=NULL && _Bar!=g_TwMgr->m_PopupBar )	// delete popup bar first if it exists
	{
		TwDeleteBar(g_TwMgr->m_PopupBar);
		g_TwMgr->m_PopupBar = NULL;
	}

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

//	---------------------------------------------------------------------------

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
	while(	  (Quote==0 && (*Cur!='\0' && *Cur!=' ' && *Cur!='\t' && *Cur!='\r' && *Cur!='\n' && *Cur!=_Sep1 && *Cur!=_Sep2))
		   || (Quote!=0 && (*Cur!='\0' /* && *Cur!='\r' && *Cur!='\n' */)) ) // allow multi-line strings
	{
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
			Column = 1;
		}
		else
			++Column;
	}

	if( Quote!=0 )
	{
		Line = QuoteLine;
		Column = QuoteColumn;
		return -(int)(Cur-_Def);	// unclosed quote
	}
	else
	{
		if( *Cur=='\n' )
		{
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

//	---------------------------------------------------------------------------

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
		return 0;	// parse error
	int BarIdx = g_TwMgr->FindBar(Names[0].c_str());
	if( BarIdx<0 )
	{
		if( Names.size()==1 && _stricmp(Names[0].c_str(), "global")==0 )
		{
			*_Bar = TW_GLOBAL_BAR;
			return +3;	// 'GLOBAL' found
		}
		else
			return -1;	// bar not found
	}
	*_Bar = g_TwMgr->m_Bars[BarIdx];
	if( Names.size()==1 )
		return 1;	// bar found, no var name parsed
	*_Var = (*_Bar)->Find(Names[1].c_str(), _VarParent, _VarIndex);
	if( *_Var==NULL )
		return -2;	// var not found
	return 2;		// bar and var found
}

//	---------------------------------------------------------------------------

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

//	---------------------------------------------------------------------------

int BarVarSetAttrib(CTwBar *_Bar, CTwVar *_Var, CTwVarGroup *_VarParent, int _VarIndex, int _AttribID, const char *_Value)
{
	assert(_Bar!=NULL && _AttribID>0);

	/* don't delete popupbar here: if any attrib is changed every frame by the app, popup will not work anymore.
	if( g_TwMgr->m_PopupBar!=NULL && _Bar!=g_TwMgr->m_PopupBar && g_TwMgr->m_PopupBar->m_BarLinkedToPopupList==_Bar )	// delete popup bar first if it exists
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
}

//	---------------------------------------------------------------------------

int ANT_CALL TwDefine(const char *_Def)
{
	CTwFPU fpu;	// force fpu precision

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

	int Line = 1;
	int Column = 1;
	const char *Cur = _Def;
	enum EState { PARSE_NAME, PARSE_ATTRIB };
	EState State = PARSE_NAME;
	string Token;
	string Value;
	CTwBar *Bar = NULL;
	CTwVar *Var = NULL;
	CTwVarGroup *VarParent = NULL;
	int VarIndex = -1;
	int p; 
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
			sprintf(g_ErrParse, "Parsing error in def string, line %d column %d [%-16s...]", Line, Column, (p<0)?(Cur-p):PrevCur);
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
					sprintf(g_ErrParse, "Parsing error in def string: Bar not found line %d column %d [%-16s...]", Line, Column, Token.c_str());
				else if( Err==-2 )
					sprintf(g_ErrParse, "Parsing error in def string: Variable not found line %d column %d [%-16s...]", Line, Column, Token.c_str());
				else
					sprintf(g_ErrParse, "Parsing error in def string, line %d column %d [%-16s...]", Line, Column, Token.c_str());
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
				sprintf(g_ErrParse, "Parsing error in def string: Unknown attribute line %d column %d [%-16s...]", Line, Column, Token.c_str());
				g_TwMgr->SetLastError(g_ErrParse);
				return 0;
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
						sprintf(g_ErrParse, "Parsing error in def string: '=' not found while reading attribute value line %d column %d [%-16s...]", Line, Column, Token.c_str());
						g_TwMgr->SetLastError(g_ErrParse);
						return 0;
					}
					Cur += p + 1;
				}
				p = ParseToken(Value, Cur, Line, Column, false, true);
				if( p<=0 )
				{
					sprintf(g_ErrParse, "Parsing error in def string: can't read attribute value line %d column %d [%-16s...]", Line, Column, Token.c_str());
					g_TwMgr->SetLastError(g_ErrParse);
					return 0;
				}
				CurSep = Cur[p];
				Cur += p + ((CurSep!='\0')?1:0);
			}
			if( BarVarSetAttrib(Bar, Var, VarParent, VarIndex, AttribID, HasValue?Value.c_str():NULL)==0 )
			{
				if( g_TwMgr->CheckLastError()==NULL || strlen(g_TwMgr->CheckLastError())<=0 )
					sprintf(g_ErrParse, "Parsing error in def string: wrong attribute value line %d column %d [%-16s...]", Line, Column, Token.c_str());
				else
					sprintf(g_ErrParse, "%s line %d column %d [%-16s...]", g_TwMgr->CheckLastError(), Line, Column, Token.c_str());
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
			if( *Cur=='\n' )	// new line detected
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

//	---------------------------------------------------------------------------

TwType ANT_CALL TwDefineEnum(const char *_Name, const TwEnumVal *_EnumValues, unsigned int _NbValues)
{
	CTwFPU fpu;	// force fpu precision

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

	if( g_TwMgr->m_PopupBar!=NULL )	// delete popup bar first if it exists
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

//	---------------------------------------------------------------------------

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

//	---------------------------------------------------------------------------

TwType ANT_CALL TwDefineStruct(const char *_StructName, const TwStructMember *_StructMembers, unsigned int _NbMembers, size_t _StructSize, TwSummaryCallback _SummaryCallback, void *_SummaryClientData)
{
	CTwFPU fpu;	// force fpu precision

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
		m.m_Size = 0;	// to avoid endless recursivity in GetDataSize
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

//	---------------------------------------------------------------------------

TwType ANT_CALL TwDefineStructExt(const char *_StructName, const TwStructMember *_StructExtMembers, unsigned int _NbExtMembers, size_t _StructSize, size_t _StructExtSize, TwStructExtInitCallback _StructExtInitCallback, TwCopyVarFromExtCallback _CopyVarFromExtCallback, TwCopyVarToExtCallback _CopyVarToExtCallback, TwSummaryCallback _SummaryCallback, void *_ClientData)
{
	CTwFPU fpu;	// force fpu precision

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
	}
	return type;
}


//	---------------------------------------------------------------------------

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
		((void)(0));	// *_Modif |= TW_KMOD_ALTGR;
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
		((void)(0));	// *_Modif |= TW_KMOD_ALTGR;
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
	//	*_String += "ALTGR+";
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

//	---------------------------------------------------------------------------
 
const int		 TW_MOUSE_NOMOTION = -1;
ETwMouseAction   TW_MOUSE_MOTION = (ETwMouseAction)(-2);
ETwMouseAction   TW_MOUSE_WHEEL = (ETwMouseAction)(-3);
ETwMouseButtonID TW_MOUSE_NA = (ETwMouseButtonID)(-1);

static int TwMouseEvent(ETwMouseAction _EventType, TwMouseButtonID _Button, int _MouseX, int _MouseY, int _WheelPos)
{
	CTwFPU fpu;	// force fpu precision

	if( g_TwMgr==NULL || g_TwMgr->m_Graph==NULL )
	{
		// TwGlobalError(g_ErrNotInit); -> not an error here
		return 0; // not initialized
	}
	if( g_TwMgr->m_WndHeight<=0 || g_TwMgr->m_WndWidth<=0 )
	{
		//g_TwMgr->SetLastError(g_ErrBadWndSize);	// not an error, windows not yet ready.
		return 0;
	}

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
	for( i=((int)g_TwMgr->m_Bars.size())-1; i>=0; --i )
	{
		Bar = g_TwMgr->m_Bars[g_TwMgr->m_Order[i]];
		if( Bar!=NULL && Bar->m_Visible )
		{
			if( _EventType==TW_MOUSE_MOTION )
				Handled = Bar->MouseMotion(_MouseX, _MouseY);
			else if( _EventType==TW_MOUSE_PRESSED || _EventType==TW_MOUSE_RELEASED )
				Handled = Bar->MouseButton(_Button, (_EventType==TW_MOUSE_PRESSED), _MouseX, _MouseY);
			else if( _EventType==TW_MOUSE_WHEEL )
			{
				if( abs(_WheelPos-g_TwMgr->m_LastMouseWheelPos)<4 )	// avoid crazy wheel positions
					Handled = Bar->MouseWheel(_WheelPos, g_TwMgr->m_LastMouseWheelPos, _MouseX, _MouseY);
			}
			if( Handled )
				break;
		}
	}

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
		if( wasPopup && Bar!=g_TwMgr->m_PopupBar && g_TwMgr->m_PopupBar!=NULL )	// delete popup
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

//	---------------------------------------------------------------------------

static int TranslateKey(int _Key, int _Modifiers)
{
	// CTRL special cases
//	if( (_Modifiers&TW_KMOD_CTRL) && !(_Modifiers&TW_KMOD_ALT || _Modifiers&TW_KMOD_META) && _Key>0 && _Key<32 )
//		_Key += 'a'-1;
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
		//_Modifiers &= ~TW_KMOD_SHIFT;	// remove shift modifier
		bool Num = (!(_Modifiers&TW_KMOD_SHIFT) && (_Modifiers&0x1000)); // 0x1000 is SDL's KMOD_NUM
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
	return _Key;
}

//	---------------------------------------------------------------------------

int ANT_CALL TwKeyPressed(int _Key, int _Modifiers)
{
	CTwFPU fpu;	// force fpu precision

	if( g_TwMgr==NULL || g_TwMgr->m_Graph==NULL )
	{
		// TwGlobalError(g_ErrNotInit); -> not an error here
		return 0; // not initialized
	}
	if( g_TwMgr->m_WndHeight<=0 || g_TwMgr->m_WndWidth<=0 )
	{
		//g_TwMgr->SetLastError(g_ErrBadWndSize); 	// not an error, windows not yet ready.
		return 0;
	}

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
			if( Bar!=NULL && Bar->m_Visible && !Bar->IsMinimized() && MouseX>=Bar->m_PosX && MouseX<Bar->m_PosX+Bar->m_Width && MouseY>=Bar->m_PosY && MouseY<Bar->m_PosY+Bar->m_Height )
				Handled = Bar->KeyPressed(_Key, _Modifiers);
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
				Handled = Bar->KeyPressed(_Key, _Modifiers);
		}

		// If not handled, send it to iconified bars in the right order
		for( i=((int)g_TwMgr->m_Bars.size())-1; i>=0 && !Handled; --i )
		{
			Bar = g_TwMgr->m_Bars[g_TwMgr->m_Order[i]];
			if( Bar!=NULL && Bar->m_Visible && Bar->IsMinimized() )
				Handled = Bar->KeyPressed(_Key, _Modifiers);
		}
		
		if( g_TwMgr->m_HelpBar!=NULL && g_TwMgr->m_Graph )
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

	if( Handled && Bar!=g_TwMgr->m_PopupBar && g_TwMgr->m_PopupBar!=NULL )	// delete popup
	{
		TwDeleteBar(g_TwMgr->m_PopupBar);
		g_TwMgr->m_PopupBar = NULL;
	}

	if( Handled && Bar!=NULL && Bar!=g_TwMgr->m_PopupBar )
		TwSetTopBar(Bar);

	return Handled ? 1 : 0;
}

//	---------------------------------------------------------------------------

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
		if( _Grp->m_Vars[i]!=NULL && _Grp->m_Vars[i]->m_Visible && _Grp->m_Vars[i]->IsGroup() )
		{
			const CTwVarGroup *SubGrp = static_cast<const CTwVarGroup *>(_Grp->m_Vars[i]);
			if( SubGrp->m_StructValuePtr!=NULL && SubGrp->m_StructType>=TW_TYPE_STRUCT_BASE && SubGrp->m_StructType<TW_TYPE_STRUCT_BASE+(int)g_TwMgr->m_Structs.size() && g_TwMgr->m_Structs[SubGrp->m_StructType-TW_TYPE_STRUCT_BASE].m_Name.length()>0 )
				_Set.insert(SubGrp->m_StructType);
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
					while( First<l && (_String[First]==' ' || _String[First]=='\t') )	// skip blanks
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
			Var->m_LeftMargin = (signed short)(_Level*Font->m_CharWidth[(int)' ']);
			Var->m_TopMargin  = (signed short)(-g_TwMgr->m_HelpBar->m_Sep);
			//Var->m_TopMargin  = 1;
			Var->m_Color = g_TwMgr->m_HelpBar->m_ColHelpText;
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
			CTwVarAtom *Var = new CTwVarAtom;
			Var->m_Name = Decal;
			if( _ToAppend->m_Vars[i]->m_Label.size()>0 )
				Var->m_Name += _ToAppend->m_Vars[i]->m_Label;
			else
				Var->m_Name += _ToAppend->m_Vars[i]->m_Name;
			Var->m_Ptr = NULL;
			if( _ToAppend->m_Vars[i]->IsGroup() && static_cast<const CTwVarGroup *>(_ToAppend->m_Vars[i])->m_StructValuePtr!=NULL )
			{	// That's a struct var
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
				Var->m_LeftMargin = (signed short)((_Level+1)*g_TwMgr->m_HelpBar->m_Font->m_CharWidth[(int)' ']);
				//Var->m_TopMargin  = (signed short)(g_TwMgr->m_HelpBar->m_Font->m_CharHeight/2-2+2*(_Level-1));
				Var->m_TopMargin  = 2;
				if( Var->m_TopMargin>g_TwMgr->m_HelpBar->m_Font->m_CharHeight-3 )
					Var->m_TopMargin = (signed short)(g_TwMgr->m_HelpBar->m_Font->m_CharHeight-3);
				Var->m_ReadOnly = true;
			}
			Var->SetDefaults();
			_Grp->m_Vars.push_back(Var);
			++n;
			if( _ToAppend->m_Vars[i]->IsGroup() && static_cast<const CTwVarGroup *>(_ToAppend->m_Vars[i])->m_StructValuePtr==NULL )
				n += AppendHelp(_Grp, static_cast<const CTwVarGroup *>(_ToAppend->m_Vars[i]), _Level+1, _Width);
			else if( _ToAppend->m_Vars[i]->m_Help.length()>0 )
				n += AppendHelpString(_Grp, _ToAppend->m_Vars[i]->m_Help.c_str(), _Level+1, _Width, TW_TYPE_HELP_ATOM);
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
	dst->m_Color = src->m_Color;
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
	if( !m_HelpBarUpdateNow && (float)m_Timer.GetTime()<m_LastHelpUpdateTime+2 )	// update at most every 2 seconds
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
			Grp->m_Color = m_HelpBar->m_ColGrpText;
			m_HelpBar->m_VarRoot.m_Vars.push_back(Grp);
			if( m_Bars[ib]->m_Help.size()>0 )
				AppendHelpString(Grp, m_Bars[ib]->m_Help.c_str(), 0, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0, TW_TYPE_HELP_GRP);

			// Append variables (recursive)
			AppendHelp(Grp, &(m_Bars[ib]->m_VarRoot), 1, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0);

			// Append structures
			StructSet UsedStructs;
			InsertUsedStructs(UsedStructs, &(m_Bars[ib]->m_VarRoot));
			CTwVarGroup *StructGrp = NULL;
			for( StructSet::iterator it=UsedStructs.begin(); it!=UsedStructs.end(); ++it )
			{
				int idx = (*it) - TW_TYPE_STRUCT_BASE;
				if( idx>=0 && idx<(int)g_TwMgr->m_Structs.size() && g_TwMgr->m_Structs[idx].m_Name.length()>0 )
				{
					if( StructGrp==NULL )
					{
						StructGrp = new CTwVarGroup;
						StructGrp->m_StructType = TW_TYPE_HELP_STRUCT;	// a special line background color will be used
						StructGrp->m_Name = "Structures";
						StructGrp->m_Open = false;
						StructGrp->m_Color = m_HelpBar->m_ColStructText;
						Grp->m_Vars.push_back(StructGrp);
					}
					CTwVarAtom *Var = new CTwVarAtom;
					Var->m_Ptr = NULL;
					Var->m_Type = TW_TYPE_HELP_GRP;
					Var->m_DontClip = true;
					Var->m_LeftMargin = (signed short)(2*g_TwMgr->m_HelpBar->m_Font->m_CharWidth[(int)' ']);
					Var->m_TopMargin  = 2;
					Var->m_ReadOnly = true;
					Var->m_NoSlider = true;
					Var->m_Name = '{'+g_TwMgr->m_Structs[idx].m_Name+'}';
					StructGrp->m_Vars.push_back(Var);
					if( g_TwMgr->m_Structs[idx].m_Help.size()>0 )
						AppendHelpString(StructGrp, g_TwMgr->m_Structs[idx].m_Help.c_str(), 2, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0-2*Var->m_LeftMargin, TW_TYPE_HELP_GRP);

					// Append struct members
					for( size_t im=0; im<g_TwMgr->m_Structs[idx].m_Members.size(); ++im )
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
						if( g_TwMgr->m_Structs[idx].m_Members[im].m_Help.size()>0 )
							AppendHelpString(StructGrp, g_TwMgr->m_Structs[idx].m_Members[im].m_Help.c_str(), 3, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0-4*Var->m_LeftMargin, TW_TYPE_HELP_ATOM);
					}
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
	RotoGrp->m_Color = m_HelpBar->m_ColGrpText;
	m_HelpBar->m_VarRoot.m_Vars.push_back(RotoGrp);
	AppendHelpString(RotoGrp, "The RotoSlider allows rapid editing of numerical values.", 0, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0, TW_TYPE_HELP_ATOM);
	AppendHelpString(RotoGrp, "To modify a numerical value, click on it, then move the mouse outside of the grey circle while keeping the mouse button pressed, and turn around the circle to increase or decrease the numerical value.", 0, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0, TW_TYPE_HELP_ATOM);
	AppendHelpString(RotoGrp, "The two grey lines depict the min and max bounds.", 0, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0, TW_TYPE_HELP_ATOM);
	AppendHelpString(RotoGrp, "Moving the mouse far form the circle allows precise increase or decrease, while moving near the circle allows fast increase or decrease.", 0, m_HelpBar->m_VarX2-m_HelpBar->m_VarX0, TW_TYPE_HELP_ATOM);

	SynchroHierarchy(&m_HelpBar->m_VarRoot, &prevHierarchy);

	m_HelpBarNotUpToDate = false;
}

//	---------------------------------------------------------------------------

#if defined(ANT_WINDOWS)

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
	#ifdef IDC_HAND
		m_CursorHand = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_HAND));
	#else
		m_CursorHand = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_UPARROW));
	#endif
	int cur;
	HMODULE hdll = GetModuleHandle(ANT_TWEAK_BAR_DLL);
	m_CursorCenter = ::LoadCursor(hdll, MAKEINTRESOURCE(IDC_CURSOR1+0));
	if( m_CursorCenter==NULL )
		m_CursorCenter = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_CROSS));
	m_CursorPoint = ::LoadCursor(hdll, MAKEINTRESOURCE(IDC_CURSOR1+1));
	if( m_CursorPoint==NULL )
		m_CursorPoint = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_CROSS));

	for( cur=0; cur<NB_ROTO_CURSORS; ++cur )
	{
		m_RotoCursors[cur] = ::LoadCursor(hdll, MAKEINTRESOURCE(IDC_CURSOR1+2+cur));
		if( m_RotoCursors[cur]==NULL )
			m_RotoCursors[cur] = ::LoadCursor(NULL ,MAKEINTRESOURCE(IDC_CROSS));
	}
	
	m_CursorsCreated = true;
}

void CTwMgr::FreeCursors()
{
	m_CursorsCreated = false;
}

void CTwMgr::SetCursor(CTwMgr::CCursor _Cursor)
{
	if( m_CursorsCreated )
		::SetCursor(_Cursor);
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
	m_CursorArrow 	= XCreateFontCursor(m_CurrentXDisplay, XC_left_ptr);
	m_CursorMove 	= XCreateFontCursor(m_CurrentXDisplay, XC_plus);
	m_CursorWE 		= XCreateFontCursor(m_CurrentXDisplay, XC_left_side);
	m_CursorNS 		= XCreateFontCursor(m_CurrentXDisplay, XC_top_side);
	m_CursorTopRight= XCreateFontCursor(m_CurrentXDisplay, XC_top_right_corner);
	m_CursorTopLeft	= XCreateFontCursor(m_CurrentXDisplay, XC_top_left_corner);
	m_CursorBottomRight	= XCreateFontCursor(m_CurrentXDisplay, XC_bottom_right_corner);
	m_CursorBottomLeft	= XCreateFontCursor(m_CurrentXDisplay, XC_bottom_left_corner);
	m_CursorHelp 	= XCreateFontCursor(m_CurrentXDisplay, XC_question_arrow);
	m_CursorHand 	= XCreateFontCursor(m_CurrentXDisplay, XC_hand1);
	m_CursorCross 	= XCreateFontCursor(m_CurrentXDisplay, XC_X_cursor);
	m_CursorUpArrow	= XCreateFontCursor(m_CurrentXDisplay, XC_center_ptr);
	m_CursorNo 		= XCreateFontCursor(m_CurrentXDisplay, XC_left_ptr);
	for( int i=0; i<NB_ROTO_CURSORS; ++i )
	{
		m_RotoCursors[i] = PixmapCursor(i+2);
	}
	m_CursorCenter 	= PixmapCursor(0);
	m_CursorPoint 	= PixmapCursor(1);
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

//	---------------------------------------------------------------------------
