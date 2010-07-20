//--------------------------------------------------------------------------------------
// File: ImeUi.h
//
// Copyright (c) Microsoft Corporation. All rights reserved.
//--------------------------------------------------------------------------------------
#ifndef _IMEUI_H_
#define _IMEUI_H_
#if _WIN32_WINNT < 0x0400
#error IMEUI requires _WIN32_WINNT to be 0x0400 or higher. Please add "_WIN32_WINNT=0x0400" to your project's preprocessor setting.
#endif
#include <windows.h>

class CImeUiFont_Base
{
public:
    virtual void    SetHeight( UINT uHeight )
    {
        uHeight;
    }; // for backward compatibility
    virtual void    SetColor( DWORD color ) = 0;
    virtual void    SetPosition( int x, int y ) = 0;
    virtual void    GetTextExtent( LPCTSTR szText, DWORD* puWidth, DWORD* puHeight ) = 0;
    virtual void    DrawText( LPCTSTR pszText ) = 0;
};

typedef struct
{
    // symbol (Henkan-kyu)
    DWORD symbolColor;
    DWORD symbolColorOff;
    DWORD symbolColorText;
    BYTE symbolHeight;
    BYTE symbolTranslucence;
    BYTE symbolPlacement;
    CImeUiFont_Base* symbolFont;

    // candidate list
    DWORD candColorBase;
    DWORD candColorBorder;
    DWORD candColorText;

    // composition string
    DWORD compColorInput;
    DWORD compColorTargetConv;
    DWORD compColorConverted;
    DWORD compColorTargetNotConv;
    DWORD compColorInputErr;
    BYTE compTranslucence;
    DWORD compColorText;

    // caret
    BYTE caretWidth;
    BYTE caretYMargin;
}               IMEUI_APPEARANCE;

typedef struct	// D3DTLVERTEX compatible
{
    float sx;
    float sy;
    float sz;
    float rhw;
    DWORD color;
    DWORD specular;
    float tu;
    float tv;
}               IMEUI_VERTEX;

// IME States
#define IMEUI_STATE_OFF		0
#define IMEUI_STATE_ON		1
#define IMEUI_STATE_ENGLISH	2

// IME const
#define MAX_CANDLIST 10

// IME Flags
#define IMEUI_FLAG_SUPPORT_CARET	0x00000001

bool ImeUi_Initialize( HWND hwnd, bool bDisable = false );
void ImeUi_Uninitialize();
void ImeUi_SetAppearance( const IMEUI_APPEARANCE* pia );
void ImeUi_GetAppearance( IMEUI_APPEARANCE* pia );
bool ImeUi_IgnoreHotKey( const MSG* pmsg );
LPARAM ImeUi_ProcessMessage( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM& lParam, bool* trapped );
void ImeUi_SetScreenDimension( UINT width, UINT height );
void ImeUi_RenderUI( bool bDrawCompAttr = true, bool bDrawOtherUi = true );
void ImeUi_SetCaretPosition( UINT x, UINT y );
void ImeUi_SetCompStringAppearance( CImeUiFont_Base* pFont, DWORD color, const RECT* prc );
bool ImeUi_GetCaretStatus();
void ImeUi_SetInsertMode( bool bInsert );
void ImeUi_SetState( DWORD dwState );
DWORD ImeUi_GetState();
void ImeUi_EnableIme( bool bEnable );
bool ImeUi_IsEnabled( void );
void ImeUi_FinalizeString( bool bSend = false );
void ImeUi_ToggleLanguageBar( BOOL bRestore );
bool ImeUi_IsSendingKeyMessage();
void ImeUi_SetWindow( HWND hwnd );
UINT ImeUi_GetInputCodePage();
DWORD ImeUi_GetFlags();
void ImeUi_SetFlags( DWORD dwFlags, bool bSet );

WORD ImeUi_GetPrimaryLanguage();
DWORD ImeUi_GetImeId( UINT uIndex );
WORD ImeUi_GetLanguage();
LPTSTR ImeUi_GetIndicatior();
bool ImeUi_IsShowReadingWindow();
bool ImeUi_IsShowCandListWindow();
bool ImeUi_IsVerticalCand();
bool ImeUi_IsHorizontalReading();
TCHAR*          ImeUi_GetCandidate( UINT idx );
TCHAR*          ImeUi_GetCompositionString();
DWORD ImeUi_GetCandidateSelection();
DWORD ImeUi_GetCandidateCount();
BYTE*           ImeUi_GetCompStringAttr();
DWORD ImeUi_GetImeCursorChars();

extern void ( CALLBACK*ImeUiCallback_DrawRect )( int x1, int y1, int x2, int y2, DWORD color );
extern void*    ( __cdecl*ImeUiCallback_Malloc )( size_t bytes );
extern void ( __cdecl*ImeUiCallback_Free )( void* ptr );
extern void ( CALLBACK*ImeUiCallback_DrawFans )( const IMEUI_VERTEX* paVertex, UINT uNum );
extern void ( CALLBACK*ImeUiCallback_OnChar )( WCHAR wc );

#endif //_IMEUI_H_
