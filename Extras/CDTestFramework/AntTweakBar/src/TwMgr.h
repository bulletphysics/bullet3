//	---------------------------------------------------------------------------
//
//	@file		TwMgr.h
//	@brief		Tweak bar manager.
//	@author		Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//				Copyright © 2005, 2006 Philippe Decaudin.
//              For conditions of distribution and use, see License.txt
//
//	notes:		Private header
//				TAB=4
//
//	---------------------------------------------------------------------------


#if !defined ANT_TW_MGR_INCLUDED
#define ANT_TW_MGR_INCLUDED

#include <AntTweakBar.h>
#define ANT_CALL TW_CALL

#include "TwColors.h"
#include "TwFonts.h"
#include "TwGraph.h"
#include "AntPerfTimer.h"


//#define BENCH	// uncomment to activate bench

#ifdef BENCH
#	define PERF(cmd)	cmd
#else	// BENCH
#	define PERF(cmd)
#endif	// BENCH

const int NB_ROTO_CURSORS = 12;


//	---------------------------------------------------------------------------
//	API unexposed by AntTweakBar.h
//	---------------------------------------------------------------------------

// bar states -> use TwDefine instead
typedef enum ETwState
{
	TW_STATE_SHOWN		= 1,
	TW_STATE_ICONIFIED	= 2,
	TW_STATE_HIDDEN		= 3,
	TW_STATE_ERROR		= 0
} TwState;
/*ANT_TWEAK_BAR_API*/ int		ANT_CALL TwSetBarState(TwBar *bar, TwState state);
/*ANT_TWEAK_BAR_API*/ TwState	ANT_CALL TwGetBarState(const TwBar *bar);
// var states -> use TwDefine instead: show/hide/iconify implemented only as string commands
//ANT_TWEAK_BAR_API int 	ANT_CALL TwSetVarState(TwBar *bar, const char *name, TwState state);
//ANT_TWEAK_BAR_API TwState	ANT_CALL TwGetVarState(const TwBar *bar, const char *name);

typedef void (ANT_CALL *TwStructExtInitCallback)(void *structExtValue, void *clientData);
typedef void (ANT_CALL *TwCopyVarFromExtCallback)(void *structValue, const void *structExtValue, unsigned int structExtMemberIndex, void *clientData);
typedef void (ANT_CALL *TwCopyVarToExtCallback)(const void *structValue, void *structExtValue, unsigned int structExtMemberIndex, void *clientData);
/*ANT_TWEAK_BAR_API*/ TwType	ANT_CALL TwDefineStructExt(const char *name, const TwStructMember *structExtMembers, unsigned int nbExtMembers, size_t structSize, size_t structExtSize, TwStructExtInitCallback structExtInitCallback, TwCopyVarFromExtCallback copyVarFromExtCallback, TwCopyVarToExtCallback copyVarToExtCallback, TwSummaryCallback summaryCallback, void *clientData);


//	---------------------------------------------------------------------------
//	AntTweakBar Manager
//	---------------------------------------------------------------------------

struct CTwMgr
{
	ETwGraphAPI			m_GraphAPI;
	void *				m_Device;
	class ITwGraph *	m_Graph;
	int					m_WndWidth;
	int					m_WndHeight;
	const CTexFont *	m_CurrentFont;

	std::vector<TwBar*>	m_Bars;
	std::vector<int>	m_Order;

	std::vector<bool>	m_MinOccupied;
	void				Minimize(TwBar *_Bar);
	void				Maximize(TwBar *_Bar);
	void				Hide(TwBar *_Bar);
	void				Unhide(TwBar *_Bar);
	void				SetFont(const CTexFont *_Font, bool _ResizeBars);
	int					m_LastMouseX;
	int					m_LastMouseY;
	int					m_LastMouseWheelPos;

	std::string			m_Help;
	TwBar *				m_HelpBar;
	float				m_LastHelpUpdateTime;
	void				UpdateHelpBar();
	bool				m_HelpBarNotUpToDate;
	bool				m_HelpBarUpdateNow;
	void *				m_KeyPressedTextObj;
	bool				m_KeyPressedBuildText;
	std::string			m_KeyPressedStr;
	float				m_KeyPressedTime;
	void *				m_InfoTextObj;
	bool				m_InfoBuildText;
	int					m_BarInitColorHue;
	int					FindBar(const char *_Name) const;
	int					HasAttrib(const char *_Attrib, bool *_HasValue) const;
	int					SetAttrib(int _AttribID, const char *_Value);
	void				SetLastError(const char *_StaticErrorMesssage);	// _StaticErrorMesssage must be a static string
	const char *		GetLastError();									// returns a static string describing the error, and set LastError to NULL
	const char *		CheckLastError() const;							// returns the LastError, but does not set it to NULL
	void				SetCurrentDbgParams(const char *file, int line);
	TwBar *				m_PopupBar;
						CTwMgr(ETwGraphAPI _GraphAPI, void *_Device);
						~CTwMgr();

	struct CStructMember
	{
		std::string		m_Name;
		std::string		m_Label;
		TwType			m_Type;
		size_t			m_Offset;
		std::string		m_DefString;
		size_t			m_Size;
		std::string		m_Help;
	};
	struct CStruct
	{
		std::string					m_Name;
		std::vector<CStructMember>	m_Members;
		size_t						m_Size;
		TwSummaryCallback			m_SummaryCallback;
		void *						m_SummaryClientData;
		std::string					m_Help;
		bool						m_IsExt;
		size_t						m_ClientStructSize;
		TwStructExtInitCallback		m_StructExtInitCallback;
		TwCopyVarFromExtCallback	m_CopyVarFromExtCallback;
		TwCopyVarToExtCallback		m_CopyVarToExtCallback;
		void *						m_ExtClientData;
		CStruct() : m_IsExt(false), m_StructExtInitCallback(NULL), m_CopyVarFromExtCallback(NULL), m_CopyVarToExtCallback(NULL), m_ExtClientData(NULL) {}
		static void ANT_CALL		DefaultSummary(char *_SummaryString, size_t _SummaryMaxLength, const void *_Value, void *_ClientData);
		static void *				s_PassProxyAsClientData;
	};
	std::vector<CStruct> m_Structs;

	// followings are used for TwAddVarCB( ... StructType ... )
	struct CStructProxy
	{
		TwType			 m_Type;
		void *			 m_StructData;
		bool			 m_DeleteStructData;
		void *			 m_StructExtData;
		TwSetVarCallback m_StructSetCallback;
		TwGetVarCallback m_StructGetCallback;
		void *			 m_StructClientData;
		CStructProxy();
		~CStructProxy();
	};
	struct CMemberProxy
	{
		CStructProxy *	m_StructProxy;
		int				m_MemberIndex;
		struct CTwVar *	m_Var;
		struct CTwVarGroup * m_VarParent;
		CTwBar *		m_Bar;
		CMemberProxy();
		~CMemberProxy();
		static void ANT_CALL SetCB(const void *_Value, void *_ClientData);
		static void ANT_CALL GetCB(void *_Value, void *_ClientData);
	};
	std::list<CStructProxy>	m_StructProxies;	// elements should not move
	std::list<CMemberProxy> m_MemberProxies;	// elements should not move

	struct CEnum
	{
		std::string		m_Name;
		typedef std::map<unsigned int, std::string> CEntries;
		CEntries		m_Entries;
	};
	std::vector<CEnum>	m_Enums;

	TwType				m_TypeColor32;
	TwType				m_TypeColor3F;
	TwType				m_TypeColor4F;

	PerfTimer			m_Timer;
	double				m_LastMousePressedTime;
	TwMouseButtonID		m_LastMousePressedButtonID;
	int					m_LastMousePressedPosition[2];
	double				m_RepeatMousePressedDelay;
	double				m_RepeatMousePressedPeriod;
	bool				m_CanRepeatMousePressed;
	bool				m_IsRepeatingMousePressed;

	#if defined(ANT_WINDOWS)
		typedef HCURSOR	CCursor;
	#elif defined(ANT_UNIX)
		typedef Cursor	CCursor;
		CCursor			PixmapCursor(int _CurIdx);
		Display *		m_CurrentXDisplay;
		Window			m_CurrentXWindow;
	#endif	// defined(ANT_UNIX)
	bool				m_CursorsCreated;
	void				CreateCursors();
	void				FreeCursors();
	void				SetCursor(CCursor _Cursor);
	CCursor				m_CursorArrow;
	CCursor				m_CursorMove;
	CCursor				m_CursorWE;
	CCursor				m_CursorNS;
	CCursor				m_CursorTopLeft;
	CCursor				m_CursorTopRight;
	CCursor				m_CursorBottomLeft;
	CCursor				m_CursorBottomRight;	
	CCursor				m_CursorHelp;
	CCursor				m_CursorHand;
	CCursor				m_CursorCross;
	CCursor				m_CursorUpArrow;
	CCursor				m_CursorNo;
	CCursor				m_RotoCursors[NB_ROTO_CURSORS];
	CCursor				m_CursorCenter;
	CCursor				m_CursorPoint;

protected:
	int					m_NbMinimizedBars;
	const char *		m_LastError;
	const char *		m_CurrentDbgFile;
	int					m_CurrentDbgLine;
};

extern CTwMgr *g_TwMgr;


//	---------------------------------------------------------------------------
//	Extra functions and twtypes
//	---------------------------------------------------------------------------


bool TwGetKeyCode(int *_Code, int *_Modif, const char *_String);
bool TwGetKeyString(std::string *_String, int _Code, int _Modif); 

const ETwType TW_TYPE_SHORTCUT	  = ETwType(0xfff1);
const ETwType TW_TYPE_HELP_GRP	  = ETwType(0xfff2);
const ETwType TW_TYPE_HELP_ATOM	  = ETwType(0xfff3);
const ETwType TW_TYPE_HELP_HEADER = ETwType(0xfff4);
const ETwType TW_TYPE_HELP_STRUCT = ETwType(0xfff5);
const ETwType TW_TYPE_BUTTON	  = ETwType(0xfff6);
const ETwType TW_TYPE_STRUCT_BASE = ETwType(0x10000000);
const ETwType TW_TYPE_ENUM_BASE	  = ETwType(0x20000000);


//	---------------------------------------------------------------------------
//	Color struct ext
//	---------------------------------------------------------------------------


struct CColorExt
{
	int					 R, G, B;
	int					 H, L, S;
	int					 A;
	bool				 m_HLS, m_HasAlpha, m_OGL;
	bool				 m_CanHaveAlpha;
	bool				 m_IsColorF;
	unsigned int		 m_PrevConvertedColor;
	CTwMgr::CStructProxy*m_StructProxy;
	void				 RGB2HLS();
	void				 HLS2RGB();
	static void ANT_CALL InitColor32CB(void *_ExtValue, void *_ClientData);
	static void ANT_CALL InitColor3FCB(void *_ExtValue, void *_ClientData);
	static void ANT_CALL InitColor4FCB(void *_ExtValue, void *_ClientData);
	static void ANT_CALL CopyVarFromExtCB(void *_VarValue, const void *_ExtValue, unsigned int _ExtMemberIndex, void *_ClientData);
	static void ANT_CALL CopyVarToExtCB(const void *_VarValue, void *_ExtValue, unsigned int _ExtMemberIndex, void *_ClientData);
	static void ANT_CALL SummaryCB(char *_SummaryString, size_t _SummaryMaxLength, const void *_ExtValue, void *_ClientData);
	static void			 CreateTypes();
};


//	---------------------------------------------------------------------------
//	CTwFPU objects set and restore the fpu precision if needed.
//	(could be usefull because DirectX changes it and AntTweakBar requires default double precision)
//	---------------------------------------------------------------------------

struct CTwFPU
{
	CTwFPU()	
	{ 
	#ifdef ANT_WINDOWS
		state0 = _controlfp(0, 0); 
		if( (state0&MCW_PC)==_PC_24 )	// we need at least _PC_53
			_controlfp(_PC_53, MCW_PC);
	#else
		state0 = 0;
	#endif
	}
	~CTwFPU()
	{
	#ifdef ANT_WINDOWS		
		if( (state0&MCW_PC)==_PC_24 )
			_controlfp(_PC_24, MCW_PC);
	#else
		state0 = 0;
	#endif
	}
private:
	unsigned int state0;
};

//	---------------------------------------------------------------------------


#endif // !defined ANT_TW_MGR_INCLUDED
