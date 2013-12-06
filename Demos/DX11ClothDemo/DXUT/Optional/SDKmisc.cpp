//--------------------------------------------------------------------------------------
// File: SDKmisc.cpp
//
// Various helper functionality that is shared between SDK samples
//
// Copyright (c) Microsoft Corporation. All rights reserved
//--------------------------------------------------------------------------------------
#include "dxut.h"
#include "SDKmisc.h"
#include "DXUTres.h"
#undef min // use __min instead
#undef max // use __max instead

#include "DXUTGui.h"

//--------------------------------------------------------------------------------------
// Global/Static Members
//--------------------------------------------------------------------------------------
CDXUTResourceCache& WINAPI DXUTGetGlobalResourceCache()
{
    // Using an accessor function gives control of the construction order
    static CDXUTResourceCache cache;
    return cache;
}


//--------------------------------------------------------------------------------------
// Internal functions forward declarations
//--------------------------------------------------------------------------------------
bool DXUTFindMediaSearchTypicalDirs( __in_ecount(cchSearch) WCHAR* strSearchPath, 
                                     int cchSearch, 
                                     __in LPCWSTR strLeaf, 
                                     __in WCHAR* strExePath,
                                     __in WCHAR* strExeName );
bool DXUTFindMediaSearchParentDirs( __in_ecount(cchSearch) WCHAR* strSearchPath, 
                                    int cchSearch, 
                                    __in WCHAR* strStartAt, 
                                    __in WCHAR* strLeafName );
INT_PTR CALLBACK DisplaySwitchToREFWarningProc( HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam );


//--------------------------------------------------------------------------------------
// Shared code for samples to ask user if they want to use a REF device or quit
//--------------------------------------------------------------------------------------
void WINAPI DXUTDisplaySwitchingToREFWarning( DXUTDeviceVersion ver )
{
    if( DXUTGetShowMsgBoxOnError() )
    {
        DWORD dwSkipWarning = 0, dwRead = 0, dwWritten = 0;
        HANDLE hFile = NULL;

        // Read previous user settings
        WCHAR strPath[MAX_PATH];
        SHGetFolderPath( DXUTGetHWND(), CSIDL_LOCAL_APPDATA, NULL, SHGFP_TYPE_CURRENT, strPath );
        wcscat_s( strPath, MAX_PATH, L"\\DXUT\\SkipRefWarning.dat" );
        if( ( hFile = CreateFile( strPath, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, 0,
                                  NULL ) ) != INVALID_HANDLE_VALUE )
        {
            ReadFile( hFile, &dwSkipWarning, sizeof( DWORD ), &dwRead, NULL );
            CloseHandle( hFile );
        }

        if( dwSkipWarning == 0 )
        {
            // Compact code to create a custom dialog box without using a template in a resource file.
            // If this dialog were in a .rc file, this would be a lot simpler but every sample calling this function would
            // need a copy of the dialog in its own .rc file. Also MessageBox API could be used here instead, but 
            // the MessageBox API is simpler to call but it can't provide a "Don't show again" checkbox
            typedef struct
            {
                DLGITEMTEMPLATE a;
                WORD b;
                WORD c;
                WORD d;
                WORD e;
                WORD f;
            } DXUT_DLG_ITEM;
            typedef struct
            {
                DLGTEMPLATE a;
                WORD b;
                WORD c;
                WCHAR   d[2];
                WORD e;
                WCHAR   f[16];
                DXUT_DLG_ITEM i1;
                DXUT_DLG_ITEM i2;
                DXUT_DLG_ITEM i3;
                DXUT_DLG_ITEM i4;
                DXUT_DLG_ITEM i5;
            } DXUT_DLG_DATA;

            DXUT_DLG_DATA dtp =
            {
                {WS_CAPTION | WS_POPUP | WS_VISIBLE | WS_SYSMENU | DS_ABSALIGN | DS_3DLOOK | DS_SETFONT |
                    DS_MODALFRAME | DS_CENTER,0,5,0,0,269,82},0,0,L" ",8,L"MS Shell Dlg 2",
                { {WS_CHILD | WS_VISIBLE | SS_ICON | SS_CENTERIMAGE,0,7,7,24,24,0x100},0xFFFF,0x0082,0,0,0}, // icon
                { {WS_CHILD | WS_VISIBLE,0,40,7,230,25,0x101},0xFFFF,0x0082,0,0,0}, // static text
                { {WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_DEFPUSHBUTTON,0,80,39,50,14,IDYES},0xFFFF,0x0080,0,0,0}, // Yes button
                { {WS_CHILD | WS_VISIBLE | WS_TABSTOP,0,133,39,50,14,IDNO},0xFFFF,0x0080,0,0,0}, // No button
                { {WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_CHECKBOX,0,7,59,70,16,IDIGNORE},0xFFFF,0x0080,0,0,0}, // checkbox
            };

            LPARAM lParam;
            if( ver == DXUT_D3D9_DEVICE )
                lParam = 9;
            else
                lParam = 11;
            int nResult = ( int )DialogBoxIndirectParam( DXUTGetHINSTANCE(), ( DLGTEMPLATE* )&dtp, DXUTGetHWND(),
                                                         DisplaySwitchToREFWarningProc, lParam );

            if( ( nResult & 0x80 ) == 0x80 ) // "Don't show again" checkbox was checked
            {
                // Save user settings
                dwSkipWarning = 1;
                SHGetFolderPath( DXUTGetHWND(), CSIDL_LOCAL_APPDATA, NULL, SHGFP_TYPE_CURRENT, strPath );
                wcscat_s( strPath, MAX_PATH, L"\\DXUT" );
                CreateDirectory( strPath, NULL );
                wcscat_s( strPath, MAX_PATH, L"\\SkipRefWarning.dat" );
                if( ( hFile = CreateFile( strPath, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, 0,
                                          NULL ) ) != INVALID_HANDLE_VALUE )
                {
                    WriteFile( hFile, &dwSkipWarning, sizeof( DWORD ), &dwWritten, NULL );
                    CloseHandle( hFile );
                }
            }

            // User choose not to continue
            if( ( nResult & 0x0F ) == IDNO )
                DXUTShutdown( 1 );
        }
    }
}


//--------------------------------------------------------------------------------------
// MsgProc for DXUTDisplaySwitchingToREFWarning() dialog box
//--------------------------------------------------------------------------------------
INT_PTR CALLBACK DisplaySwitchToREFWarningProc( HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam )
{
    switch( message )
    {
        case WM_INITDIALOG:
            // Easier to set text here than in the DLGITEMTEMPLATE
            SetWindowText( hDlg, DXUTGetWindowTitle() );
            SendMessage( GetDlgItem( hDlg, 0x100 ), STM_SETIMAGE, IMAGE_ICON, ( LPARAM )LoadIcon( 0, IDI_QUESTION ) );
            WCHAR sz[512];
            swprintf_s( sz, 512,
                             L"This program needs to use the Direct3D %d reference device.  This device implements the entire Direct3D %d feature set, but runs very slowly.  Do you wish to continue?", lParam, lParam );
            SetDlgItemText( hDlg, 0x101, sz );
            SetDlgItemText( hDlg, IDYES, L"&Yes" );
            SetDlgItemText( hDlg, IDNO, L"&No" );
            SetDlgItemText( hDlg, IDIGNORE, L"&Don't show again" );
            break;

        case WM_COMMAND:
            switch( LOWORD( wParam ) )
            {
                case IDIGNORE:
                    CheckDlgButton( hDlg, IDIGNORE, ( IsDlgButtonChecked( hDlg,
                                                                          IDIGNORE ) == BST_CHECKED ) ? BST_UNCHECKED :
                                    BST_CHECKED );
                    EnableWindow( GetDlgItem( hDlg, IDNO ), ( IsDlgButtonChecked( hDlg, IDIGNORE ) != BST_CHECKED ) );
                    break;
                case IDNO:
                    EndDialog( hDlg, ( IsDlgButtonChecked( hDlg, IDIGNORE ) == BST_CHECKED ) ? IDNO | 0x80 : IDNO |
                               0x00 ); return TRUE;
                case IDCANCEL:
                case IDYES:
                    EndDialog( hDlg, ( IsDlgButtonChecked( hDlg, IDIGNORE ) == BST_CHECKED ) ? IDYES | 0x80 : IDYES |
                               0x00 ); return TRUE;
            }
            break;
    }
    return FALSE;
}


//--------------------------------------------------------------------------------------
// Returns pointer to static media search buffer
//--------------------------------------------------------------------------------------
WCHAR* DXUTMediaSearchPath()
{
    static WCHAR s_strMediaSearchPath[MAX_PATH] =
    {
        0
    };
    return s_strMediaSearchPath;

}

//--------------------------------------------------------------------------------------
LPCWSTR WINAPI DXUTGetMediaSearchPath()
{
    return DXUTMediaSearchPath();
}


//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTSetMediaSearchPath( LPCWSTR strPath )
{
    HRESULT hr;

    WCHAR* s_strSearchPath = DXUTMediaSearchPath();

    hr = wcscpy_s( s_strSearchPath, MAX_PATH, strPath );
    if( SUCCEEDED( hr ) )
    {
        // append slash if needed
        size_t ch = 0;
        ch = wcsnlen( s_strSearchPath, MAX_PATH);
        if( SUCCEEDED( hr ) && s_strSearchPath[ch - 1] != L'\\' )
        {
            hr = wcscat_s( s_strSearchPath, MAX_PATH, L"\\" );
        }
    }

    return hr;
}


//--------------------------------------------------------------------------------------
// Tries to find the location of a SDK media file
//       cchDest is the size in WCHARs of strDestPath.  Be careful not to 
//       pass in sizeof(strDest) on UNICODE builds.
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTFindDXSDKMediaFileCch(__in_ecount(cchDest) WCHAR* strDestPath,
                                         int cchDest, 
                                         __in LPCWSTR strFilename )
{
    bool bFound;
    WCHAR strSearchFor[MAX_PATH];

    if( NULL == strFilename || strFilename[0] == 0 || NULL == strDestPath || cchDest < 10 )
        return E_INVALIDARG;

    // Get the exe name, and exe path
    WCHAR strExePath[MAX_PATH] =
    {
        0
    };
    WCHAR strExeName[MAX_PATH] =
    {
        0
    };
    WCHAR* strLastSlash = NULL;
    GetModuleFileName( NULL, strExePath, MAX_PATH );
    strExePath[MAX_PATH - 1] = 0;
    strLastSlash = wcsrchr( strExePath, TEXT( '\\' ) );
    if( strLastSlash )
    {
        wcscpy_s( strExeName, MAX_PATH, &strLastSlash[1] );

        // Chop the exe name from the exe path
        *strLastSlash = 0;

        // Chop the .exe from the exe name
        strLastSlash = wcsrchr( strExeName, TEXT( '.' ) );
        if( strLastSlash )
            *strLastSlash = 0;
    }

    // Typical directories:
    //      .\
    //      ..\
    //      ..\..\
    //      %EXE_DIR%\
    //      %EXE_DIR%\..\
    //      %EXE_DIR%\..\..\
    //      %EXE_DIR%\..\%EXE_NAME%
    //      %EXE_DIR%\..\..\%EXE_NAME%

    // Typical directory search
    bFound = DXUTFindMediaSearchTypicalDirs( strDestPath, cchDest, strFilename, strExePath, strExeName );
    if( bFound )
        return S_OK;

    // Typical directory search again, but also look in a subdir called "\media\" 
    swprintf_s( strSearchFor, MAX_PATH, L"media\\%s", strFilename );
    bFound = DXUTFindMediaSearchTypicalDirs( strDestPath, cchDest, strSearchFor, strExePath, strExeName );
    if( bFound )
        return S_OK;

    WCHAR strLeafName[MAX_PATH] =
    {
        0
    };

    // Search all parent directories starting at .\ and using strFilename as the leaf name
    wcscpy_s( strLeafName, MAX_PATH, strFilename );
    bFound = DXUTFindMediaSearchParentDirs( strDestPath, cchDest, L".", strLeafName );
    if( bFound )
        return S_OK;

    // Search all parent directories starting at the exe's dir and using strFilename as the leaf name
    bFound = DXUTFindMediaSearchParentDirs( strDestPath, cchDest, strExePath, strLeafName );
    if( bFound )
        return S_OK;

    // Search all parent directories starting at .\ and using "media\strFilename" as the leaf name
    swprintf_s( strLeafName, MAX_PATH, L"media\\%s", strFilename );
    bFound = DXUTFindMediaSearchParentDirs( strDestPath, cchDest, L".", strLeafName );
    if( bFound )
        return S_OK;

    // Search all parent directories starting at the exe's dir and using "media\strFilename" as the leaf name
    bFound = DXUTFindMediaSearchParentDirs( strDestPath, cchDest, strExePath, strLeafName );
    if( bFound )
        return S_OK;

    // On failure, return the file as the path but also return an error code
    wcscpy_s( strDestPath, cchDest, strFilename );

    return DXUTERR_MEDIANOTFOUND;
}


//--------------------------------------------------------------------------------------
// Search a set of typical directories
//--------------------------------------------------------------------------------------
bool DXUTFindMediaSearchTypicalDirs( __in_ecount(cchSearch) WCHAR* strSearchPath, 
                                    int cchSearch, 
                                    __in LPCWSTR strLeaf, 
                                    __in WCHAR* strExePath,
                                    __in WCHAR* strExeName )
{
    // Typical directories:
    //      .\
    //      ..\
    //      ..\..\
    //      %EXE_DIR%\
    //      %EXE_DIR%\..\
    //      %EXE_DIR%\..\..\
    //      %EXE_DIR%\..\%EXE_NAME%
    //      %EXE_DIR%\..\..\%EXE_NAME%
    //      DXSDK media path

    // Search in .\  
    wcscpy_s( strSearchPath, cchSearch, strLeaf );
    if( GetFileAttributes( strSearchPath ) != 0xFFFFFFFF )
        return true;

    // Search in ..\  
    swprintf_s( strSearchPath, cchSearch, L"..\\%s", strLeaf );
    if( GetFileAttributes( strSearchPath ) != 0xFFFFFFFF )
        return true;

    // Search in ..\..\ 
    swprintf_s( strSearchPath, cchSearch, L"..\\..\\%s", strLeaf );
    if( GetFileAttributes( strSearchPath ) != 0xFFFFFFFF )
        return true;

    // Search in ..\..\ 
    swprintf_s( strSearchPath, cchSearch, L"..\\..\\%s", strLeaf );
    if( GetFileAttributes( strSearchPath ) != 0xFFFFFFFF )
        return true;

    // Search in the %EXE_DIR%\ 
    swprintf_s( strSearchPath, cchSearch, L"%s\\%s", strExePath, strLeaf );
    if( GetFileAttributes( strSearchPath ) != 0xFFFFFFFF )
        return true;

    // Search in the %EXE_DIR%\..\ 
    swprintf_s( strSearchPath, cchSearch, L"%s\\..\\%s", strExePath, strLeaf );
    if( GetFileAttributes( strSearchPath ) != 0xFFFFFFFF )
        return true;

    // Search in the %EXE_DIR%\..\..\ 
    swprintf_s( strSearchPath, cchSearch, L"%s\\..\\..\\%s", strExePath, strLeaf );
    if( GetFileAttributes( strSearchPath ) != 0xFFFFFFFF )
        return true;

    // Search in "%EXE_DIR%\..\%EXE_NAME%\".  This matches the DirectX SDK layout
    swprintf_s( strSearchPath, cchSearch, L"%s\\..\\%s\\%s", strExePath, strExeName, strLeaf );
    if( GetFileAttributes( strSearchPath ) != 0xFFFFFFFF )
        return true;

    // Search in "%EXE_DIR%\..\..\%EXE_NAME%\".  This matches the DirectX SDK layout
    swprintf_s( strSearchPath, cchSearch, L"%s\\..\\..\\%s\\%s", strExePath, strExeName, strLeaf );
    if( GetFileAttributes( strSearchPath ) != 0xFFFFFFFF )
        return true;

	// Search in "%EXE_DIR%\Demos\DX11ClothDemo\Media\".  This matches the Bullet SDK layout
    swprintf_s( strSearchPath, cchSearch, L"%s\\Demos\\DX11ClothDemo\\Media\\%s", strExePath, strLeaf );
    if( GetFileAttributes( strSearchPath ) != 0xFFFFFFFF )
        return true;

	// Search in "%EXE_DIR%\Demos\DX11ClothDemo\".  This matches the Bullet SDK layout
    swprintf_s( strSearchPath, cchSearch, L"%s\\Demos\\DX11ClothDemo\\%s", strExePath, strLeaf );
    if( GetFileAttributes( strSearchPath ) != 0xFFFFFFFF )
        return true;


    // Search in media search dir 
    WCHAR* s_strSearchPath = DXUTMediaSearchPath();
    if( s_strSearchPath[0] != 0 )
    {
        swprintf_s( strSearchPath, cchSearch, L"%s%s", s_strSearchPath, strLeaf );
        if( GetFileAttributes( strSearchPath ) != 0xFFFFFFFF )
            return true;
    }

    return false;
}



//--------------------------------------------------------------------------------------
// Search parent directories starting at strStartAt, and appending strLeafName
// at each parent directory.  It stops at the root directory.
//--------------------------------------------------------------------------------------
bool DXUTFindMediaSearchParentDirs( __in_ecount(cchSearch) WCHAR* strSearchPath, 
                                    int cchSearch, 
                                    __in WCHAR* strStartAt, 
                                    __in WCHAR* strLeafName )
{
    WCHAR strFullPath[MAX_PATH] =
    {
        0
    };
    WCHAR strFullFileName[MAX_PATH] =
    {
        0
    };
    WCHAR strSearch[MAX_PATH] =
    {
        0
    };
    WCHAR* strFilePart = NULL;

    GetFullPathName( strStartAt, MAX_PATH, strFullPath, &strFilePart );
    if( strFilePart == NULL )
        return false;

    while( strFilePart != NULL && *strFilePart != '\0' )
    {
        swprintf_s( strFullFileName, MAX_PATH, L"%s\\%s", strFullPath, strLeafName );
        if( GetFileAttributes( strFullFileName ) != 0xFFFFFFFF )
        {
            wcscpy_s( strSearchPath, cchSearch, strFullFileName );
            return true;
        }

        swprintf_s( strSearch, MAX_PATH, L"%s\\..", strFullPath );
        GetFullPathName( strSearch, MAX_PATH, strFullPath, &strFilePart );
    }

    return false;
}


//--------------------------------------------------------------------------------------
// CDXUTResourceCache
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
CDXUTResourceCache::~CDXUTResourceCache()
{
    OnDestroyDevice();

    m_TextureCache.RemoveAll();
    m_EffectCache.RemoveAll();
    m_FontCache.RemoveAll();
}

//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateTextureFromFile( LPDIRECT3DDEVICE9 pDevice, LPCTSTR pSrcFile,
                                                   LPDIRECT3DTEXTURE9* ppTexture )
{
    return CreateTextureFromFileEx( pDevice, pSrcFile, D3DX_DEFAULT, D3DX_DEFAULT, D3DX_DEFAULT,
                                    0, D3DFMT_UNKNOWN, D3DPOOL_MANAGED, D3DX_DEFAULT, D3DX_DEFAULT,
                                    0, NULL, NULL, ppTexture );
}

//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateTextureFromFile( LPDIRECT3DDEVICE9 pDevice, LPCSTR pSrcFile,
                                                   LPDIRECT3DTEXTURE9* ppTexture )
{
    WCHAR szSrcFile[MAX_PATH];
    MultiByteToWideChar( CP_ACP, 0, pSrcFile, -1, szSrcFile, MAX_PATH );
    szSrcFile[MAX_PATH - 1] = 0;

    return CreateTextureFromFile( pDevice, szSrcFile, ppTexture );
}

//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateTextureFromFile( ID3D11Device* pDevice, ID3D11DeviceContext *pContext, LPCTSTR pSrcFile,
                                                   ID3D11ShaderResourceView** ppOutputRV, bool bSRGB )
{
    return CreateTextureFromFileEx( pDevice, pContext, pSrcFile, NULL, NULL, ppOutputRV, bSRGB );
}

//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateTextureFromFile( ID3D11Device* pDevice, ID3D11DeviceContext *pContext, LPCSTR pSrcFile,
                                                   ID3D11ShaderResourceView** ppOutputRV, bool bSRGB )
{
    WCHAR szSrcFile[MAX_PATH];
    MultiByteToWideChar( CP_ACP, 0, pSrcFile, -1, szSrcFile, MAX_PATH );
    szSrcFile[MAX_PATH - 1] = 0;

    return CreateTextureFromFile( pDevice, pContext, szSrcFile, ppOutputRV, bSRGB );
}

//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateTextureFromFileEx( LPDIRECT3DDEVICE9 pDevice, LPCTSTR pSrcFile, UINT Width,
                                                     UINT Height, UINT MipLevels, DWORD Usage, D3DFORMAT Format,
                                                     D3DPOOL Pool, DWORD Filter, DWORD MipFilter, D3DCOLOR ColorKey,
                                                     D3DXIMAGE_INFO* pSrcInfo, PALETTEENTRY* pPalette,
                                                     LPDIRECT3DTEXTURE9* ppTexture )
{
    // Search the cache for a matching entry.
    for( int i = 0; i < m_TextureCache.GetSize(); ++i )
    {
        DXUTCache_Texture& Entry = m_TextureCache[i];
        if( Entry.Location == DXUTCACHE_LOCATION_FILE &&
            !lstrcmpW( Entry.wszSource, pSrcFile ) &&
            Entry.Width == Width &&
            Entry.Height == Height &&
            Entry.MipLevels == MipLevels &&
            Entry.Usage9 == Usage &&
            Entry.Format9 == Format &&
            Entry.Pool9 == Pool &&
            Entry.Type9 == D3DRTYPE_TEXTURE )
        {
            // A match is found. Obtain the IDirect3DTexture9 interface and return that.
            return Entry.pTexture9->QueryInterface( IID_IDirect3DTexture9, ( LPVOID* )ppTexture );
        }
    }

    HRESULT hr;

    // No matching entry.  Load the resource and create a new entry.
    hr = D3DXCreateTextureFromFileEx( pDevice, pSrcFile, Width, Height, MipLevels, Usage, Format,
                                      Pool, Filter, MipFilter, ColorKey, pSrcInfo, pPalette, ppTexture );
    if( FAILED( hr ) )
        return hr;

    DXUTCache_Texture NewEntry;
    NewEntry.Location = DXUTCACHE_LOCATION_FILE;
    wcscpy_s( NewEntry.wszSource, MAX_PATH, pSrcFile );
    NewEntry.Width = Width;
    NewEntry.Height = Height;
    NewEntry.MipLevels = MipLevels;
    NewEntry.Usage9 = Usage;
    NewEntry.Format9 = Format;
    NewEntry.Pool9 = Pool;
    NewEntry.Type9 = D3DRTYPE_TEXTURE;
    ( *ppTexture )->QueryInterface( IID_IDirect3DBaseTexture9, ( LPVOID* )&NewEntry.pTexture9 );

    m_TextureCache.Add( NewEntry );
    return S_OK;
}



//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateTextureFromFileEx( ID3D11Device* pDevice, ID3D11DeviceContext* pContext, LPCTSTR pSrcFile,
                                                     D3DX11_IMAGE_LOAD_INFO* pLoadInfo, ID3DX11ThreadPump* pPump,
                                                     ID3D11ShaderResourceView** ppOutputRV, bool bSRGB )
{

    bool is10L9 = DXUTGetDeviceSettings().d3d11.DeviceFeatureLevel < D3D_FEATURE_LEVEL_10_0; 
    HRESULT hr = S_OK;
    D3DX11_IMAGE_LOAD_INFO ZeroInfo;	//D3DX11_IMAGE_LOAD_INFO has a default constructor
    D3DX11_IMAGE_INFO SrcInfo;

    if( !pLoadInfo )
    {
        pLoadInfo = &ZeroInfo;
    }

    if( !pLoadInfo->pSrcInfo )
    {
        D3DX11GetImageInfoFromFile( pSrcFile, NULL, &SrcInfo, NULL );
        pLoadInfo->pSrcInfo = &SrcInfo;

        pLoadInfo->Format = pLoadInfo->pSrcInfo->Format;
    }

    // Search the cache for a matching entry.
    for( int i = 0; i < m_TextureCache.GetSize(); ++i )
    {
        DXUTCache_Texture& Entry = m_TextureCache[i];
        if( Entry.Location == DXUTCACHE_LOCATION_FILE &&
            !lstrcmpW( Entry.wszSource, pSrcFile ) &&
            Entry.Width == pLoadInfo->Width &&
            Entry.Height == pLoadInfo->Height &&
            Entry.MipLevels == pLoadInfo->MipLevels &&
            Entry.Usage11 == pLoadInfo->Usage &&
            Entry.Format == pLoadInfo->Format &&
            Entry.CpuAccessFlags == pLoadInfo->CpuAccessFlags &&
            Entry.BindFlags == pLoadInfo->BindFlags &&
            Entry.MiscFlags == pLoadInfo->MiscFlags )
        {
            // A match is found. Obtain the IDirect3DTexture9 interface and return that.
            return Entry.pSRV11->QueryInterface( __uuidof( ID3D11ShaderResourceView ), ( LPVOID* )ppOutputRV );
        }
    }

    //Ready a new entry to the texture cache
    //Do this before creating the texture since pLoadInfo may be volatile
    DXUTCache_Texture NewEntry;
    NewEntry.Location = DXUTCACHE_LOCATION_FILE;
    wcscpy_s( NewEntry.wszSource, MAX_PATH, pSrcFile );
    NewEntry.Width = pLoadInfo->Width;
    NewEntry.Height = pLoadInfo->Height;
    NewEntry.MipLevels = pLoadInfo->MipLevels;
    NewEntry.Usage11 = pLoadInfo->Usage;
    // 10L9 can't handle typesless, so we cant make a typesless format 
    if (is10L9 && bSRGB) {
        NewEntry.Format = MAKE_SRGB(pLoadInfo->Format);
    }else {
        NewEntry.Format = pLoadInfo->Format;
    }
    NewEntry.CpuAccessFlags = pLoadInfo->CpuAccessFlags;
    NewEntry.BindFlags = pLoadInfo->BindFlags;
    NewEntry.MiscFlags = pLoadInfo->MiscFlags;

    //Create the rexture
    ID3D11Texture2D* pRes = NULL;
    hr = D3DX11CreateTextureFromFile( pDevice, pSrcFile, pLoadInfo, pPump, ( ID3D11Resource** )&pRes, NULL );

    if( FAILED( hr ) )
        return hr;
    D3D11_TEXTURE2D_DESC tex_dsc;
    pRes->GetDesc(&tex_dsc);



    if (bSRGB ) {
        // This is a workaround so that we can load linearly, but sample in SRGB.  Right now, we can't load
        // as linear since D3DX will try to do conversion on load.  Loading as TYPELESS doesn't work either, and
        // loading as typed _UNORM doesn't allow us to create an SRGB view.

        // on d3d11 featuer levels this is just a copy, but on 10L9 we must use a cpu side copy with 2 staging resources.
        ID3D11Texture2D* unormStaging = NULL;
        ID3D11Texture2D* srgbStaging = NULL;

        D3D11_TEXTURE2D_DESC CopyDesc;
        pRes->GetDesc( &CopyDesc );

        pLoadInfo->BindFlags = 0;
        pLoadInfo->CpuAccessFlags = D3D11_CPU_ACCESS_WRITE | D3D11_CPU_ACCESS_READ;
        pLoadInfo->Depth = 0;
        pLoadInfo->Filter = D3DX11_FILTER_POINT;
        pLoadInfo->FirstMipLevel = 0;
        pLoadInfo->Format = CopyDesc.Format;
        pLoadInfo->Height = CopyDesc.Height;
        pLoadInfo->MipFilter = D3DX11_FILTER_POINT;
        pLoadInfo->MiscFlags = CopyDesc.MiscFlags;
        pLoadInfo->Usage = D3D11_USAGE_STAGING;
        pLoadInfo->Width = CopyDesc.Width;

        CopyDesc.BindFlags = 0;
        CopyDesc.Usage = D3D11_USAGE_STAGING;
        CopyDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE | D3D11_CPU_ACCESS_READ;
        CopyDesc.Format = MAKE_SRGB(CopyDesc.Format);

        hr = D3DX11CreateTextureFromFile( pDevice, pSrcFile, pLoadInfo, pPump, ( ID3D11Resource** )&unormStaging, NULL );

        hr = pDevice->CreateTexture2D(&CopyDesc, NULL, &srgbStaging);
        pContext->CopyResource( srgbStaging, unormStaging );
        ID3D11Texture2D* srgbGPU;

        pRes->GetDesc( &CopyDesc );
        CopyDesc.Format = MAKE_SRGB(CopyDesc.Format);
        hr = pDevice->CreateTexture2D(&CopyDesc, NULL, &srgbGPU);
        pContext->CopyResource( srgbGPU, srgbStaging );

        SAFE_RELEASE(pRes);
        SAFE_RELEASE(srgbStaging);
        SAFE_RELEASE(unormStaging);
        pRes = srgbGPU;

    }

    D3D11_SHADER_RESOURCE_VIEW_DESC SRVDesc;
    if( bSRGB )
        SRVDesc.Format = MAKE_SRGB( ZeroInfo.Format );
    else
        SRVDesc.Format = ZeroInfo.Format;
    if( pLoadInfo->pSrcInfo->ResourceDimension == D3D11_RESOURCE_DIMENSION_TEXTURE1D )
    {
        SRVDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE1D;
        SRVDesc.Texture1D.MostDetailedMip = 0;
        SRVDesc.Texture1D.MipLevels = pLoadInfo->pSrcInfo->MipLevels;
    }
    else if( pLoadInfo->pSrcInfo->ResourceDimension == D3D11_RESOURCE_DIMENSION_TEXTURE2D )
    {
        SRVDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
        SRVDesc.Texture2D.MostDetailedMip = 0;
        SRVDesc.Texture2D.MipLevels = pLoadInfo->pSrcInfo->MipLevels;

        if( pLoadInfo->pSrcInfo->MiscFlags & D3D11_RESOURCE_MISC_TEXTURECUBE )
        {
            SRVDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURECUBE;
            SRVDesc.TextureCube.MostDetailedMip = 0;
            SRVDesc.TextureCube.MipLevels = pLoadInfo->pSrcInfo->MipLevels;
        }
    }
    else if( pLoadInfo->pSrcInfo->ResourceDimension == D3D11_RESOURCE_DIMENSION_TEXTURE3D )
    {
        SRVDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE3D;
        SRVDesc.Texture3D.MostDetailedMip = 0;
        SRVDesc.Texture3D.MipLevels = pLoadInfo->pSrcInfo->MipLevels;
    }
    if (bSRGB) {
        SRVDesc.Format = MAKE_SRGB(tex_dsc.Format);
    }else {
        SRVDesc.Format = tex_dsc.Format;
    }
    SRVDesc.Texture2D.MipLevels = tex_dsc.MipLevels;
    SRVDesc.Texture2D.MostDetailedMip = 0;
    hr = pDevice->CreateShaderResourceView( pRes, &SRVDesc, ppOutputRV );
    pRes->Release();
    if( FAILED( hr ) )
        return hr;

    ( *ppOutputRV )->QueryInterface( __uuidof( ID3D11ShaderResourceView ), ( LPVOID* )&NewEntry.pSRV11 );

    m_TextureCache.Add( NewEntry );

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateTextureFromResource( LPDIRECT3DDEVICE9 pDevice, HMODULE hSrcModule,
                                                       LPCTSTR pSrcResource, LPDIRECT3DTEXTURE9* ppTexture )
{
    return CreateTextureFromResourceEx( pDevice, hSrcModule, pSrcResource, D3DX_DEFAULT, D3DX_DEFAULT,
                                        D3DX_DEFAULT, 0, D3DFMT_UNKNOWN, D3DPOOL_MANAGED, D3DX_DEFAULT,
                                        D3DX_DEFAULT, 0, NULL, NULL, ppTexture );
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateTextureFromResourceEx( LPDIRECT3DDEVICE9 pDevice, HMODULE hSrcModule,
                                                         LPCTSTR pSrcResource, UINT Width, UINT Height, UINT MipLevels,
                                                         DWORD Usage, D3DFORMAT Format, D3DPOOL Pool, DWORD Filter,
                                                         DWORD MipFilter, D3DCOLOR ColorKey, D3DXIMAGE_INFO* pSrcInfo,
                                                         PALETTEENTRY* pPalette, LPDIRECT3DTEXTURE9* ppTexture )
{
    // Search the cache for a matching entry.
    for( int i = 0; i < m_TextureCache.GetSize(); ++i )
    {
        DXUTCache_Texture& Entry = m_TextureCache[i];
        if( Entry.Location == DXUTCACHE_LOCATION_RESOURCE &&
            Entry.hSrcModule == hSrcModule &&
            !lstrcmpW( Entry.wszSource, pSrcResource ) &&
            Entry.Width == Width &&
            Entry.Height == Height &&
            Entry.MipLevels == MipLevels &&
            Entry.Usage9 == Usage &&
            Entry.Format9 == Format &&
            Entry.Pool9 == Pool &&
            Entry.Type9 == D3DRTYPE_TEXTURE )
        {
            // A match is found. Obtain the IDirect3DTexture9 interface and return that.
            return Entry.pTexture9->QueryInterface( IID_IDirect3DTexture9, ( LPVOID* )ppTexture );
        }
    }

    HRESULT hr;

    // No matching entry.  Load the resource and create a new entry.
    hr = D3DXCreateTextureFromResourceEx( pDevice, hSrcModule, pSrcResource, Width, Height, MipLevels, Usage,
                                          Format, Pool, Filter, MipFilter, ColorKey, pSrcInfo, pPalette, ppTexture );
    if( FAILED( hr ) )
        return hr;

    DXUTCache_Texture NewEntry;
    NewEntry.Location = DXUTCACHE_LOCATION_RESOURCE;
    NewEntry.hSrcModule = hSrcModule;
    wcscpy_s( NewEntry.wszSource, MAX_PATH, pSrcResource );
    NewEntry.Width = Width;
    NewEntry.Height = Height;
    NewEntry.MipLevels = MipLevels;
    NewEntry.Usage9 = Usage;
    NewEntry.Format9 = Format;
    NewEntry.Pool9 = Pool;
    NewEntry.Type9 = D3DRTYPE_TEXTURE;
    ( *ppTexture )->QueryInterface( IID_IDirect3DBaseTexture9, ( LPVOID* )&NewEntry.pTexture9 );

    m_TextureCache.Add( NewEntry );
    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateCubeTextureFromFile( LPDIRECT3DDEVICE9 pDevice, LPCTSTR pSrcFile,
                                                       LPDIRECT3DCUBETEXTURE9* ppCubeTexture )
{
    return CreateCubeTextureFromFileEx( pDevice, pSrcFile, D3DX_DEFAULT, D3DX_DEFAULT, 0,
                                        D3DFMT_UNKNOWN, D3DPOOL_MANAGED, D3DX_DEFAULT, D3DX_DEFAULT,
                                        0, NULL, NULL, ppCubeTexture );
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateCubeTextureFromFileEx( LPDIRECT3DDEVICE9 pDevice, LPCTSTR pSrcFile, UINT Size,
                                                         UINT MipLevels, DWORD Usage, D3DFORMAT Format, D3DPOOL Pool,
                                                         DWORD Filter, DWORD MipFilter, D3DCOLOR ColorKey,
                                                         D3DXIMAGE_INFO* pSrcInfo, PALETTEENTRY* pPalette,
                                                         LPDIRECT3DCUBETEXTURE9* ppCubeTexture )
{
    // Search the cache for a matching entry.
    for( int i = 0; i < m_TextureCache.GetSize(); ++i )
    {
        DXUTCache_Texture& Entry = m_TextureCache[i];
        if( Entry.Location == DXUTCACHE_LOCATION_FILE &&
            !lstrcmpW( Entry.wszSource, pSrcFile ) &&
            Entry.Width == Size &&
            Entry.MipLevels == MipLevels &&
            Entry.Usage9 == Usage &&
            Entry.Format9 == Format &&
            Entry.Pool9 == Pool &&
            Entry.Type9 == D3DRTYPE_CUBETEXTURE )
        {
            // A match is found. Obtain the IDirect3DCubeTexture9 interface and return that.
            return Entry.pTexture9->QueryInterface( IID_IDirect3DCubeTexture9, ( LPVOID* )ppCubeTexture );
        }
    }

    HRESULT hr;

    // No matching entry.  Load the resource and create a new entry.
    hr = D3DXCreateCubeTextureFromFileEx( pDevice, pSrcFile, Size, MipLevels, Usage, Format, Pool, Filter,
                                          MipFilter, ColorKey, pSrcInfo, pPalette, ppCubeTexture );
    if( FAILED( hr ) )
        return hr;

    DXUTCache_Texture NewEntry;
    NewEntry.Location = DXUTCACHE_LOCATION_FILE;
    wcscpy_s( NewEntry.wszSource, MAX_PATH, pSrcFile );
    NewEntry.Width = Size;
    NewEntry.MipLevels = MipLevels;
    NewEntry.Usage9 = Usage;
    NewEntry.Format9 = Format;
    NewEntry.Pool9 = Pool;
    NewEntry.Type9 = D3DRTYPE_CUBETEXTURE;
    ( *ppCubeTexture )->QueryInterface( IID_IDirect3DBaseTexture9, ( LPVOID* )&NewEntry.pTexture9 );

    m_TextureCache.Add( NewEntry );
    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateCubeTextureFromResource( LPDIRECT3DDEVICE9 pDevice, HMODULE hSrcModule,
                                                           LPCTSTR pSrcResource,
                                                           LPDIRECT3DCUBETEXTURE9* ppCubeTexture )
{
    return CreateCubeTextureFromResourceEx( pDevice, hSrcModule, pSrcResource, D3DX_DEFAULT, D3DX_DEFAULT,
                                            0, D3DFMT_UNKNOWN, D3DPOOL_MANAGED, D3DX_DEFAULT, D3DX_DEFAULT,
                                            0, NULL, NULL, ppCubeTexture );
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateCubeTextureFromResourceEx( LPDIRECT3DDEVICE9 pDevice, HMODULE hSrcModule,
                                                             LPCTSTR pSrcResource, UINT Size, UINT MipLevels,
                                                             DWORD Usage, D3DFORMAT Format, D3DPOOL Pool, DWORD Filter,
                                                             DWORD MipFilter, D3DCOLOR ColorKey,
                                                             D3DXIMAGE_INFO* pSrcInfo, PALETTEENTRY* pPalette,
                                                             LPDIRECT3DCUBETEXTURE9* ppCubeTexture )
{
    // Search the cache for a matching entry.
    for( int i = 0; i < m_TextureCache.GetSize(); ++i )
    {
        DXUTCache_Texture& Entry = m_TextureCache[i];
        if( Entry.Location == DXUTCACHE_LOCATION_RESOURCE &&
            Entry.hSrcModule == hSrcModule &&
            !lstrcmpW( Entry.wszSource, pSrcResource ) &&
            Entry.Width == Size &&
            Entry.MipLevels == MipLevels &&
            Entry.Usage9 == Usage &&
            Entry.Format9 == Format &&
            Entry.Pool9 == Pool &&
            Entry.Type9 == D3DRTYPE_CUBETEXTURE )
        {
            // A match is found. Obtain the IDirect3DCubeTexture9 interface and return that.
            return Entry.pTexture9->QueryInterface( IID_IDirect3DCubeTexture9, ( LPVOID* )ppCubeTexture );
        }
    }

    HRESULT hr;

    // No matching entry.  Load the resource and create a new entry.
    hr = D3DXCreateCubeTextureFromResourceEx( pDevice, hSrcModule, pSrcResource, Size, MipLevels, Usage, Format,
                                              Pool, Filter, MipFilter, ColorKey, pSrcInfo, pPalette, ppCubeTexture );
    if( FAILED( hr ) )
        return hr;

    DXUTCache_Texture NewEntry;
    NewEntry.Location = DXUTCACHE_LOCATION_RESOURCE;
    NewEntry.hSrcModule = hSrcModule;
    wcscpy_s( NewEntry.wszSource, MAX_PATH, pSrcResource );
    NewEntry.Width = Size;
    NewEntry.MipLevels = MipLevels;
    NewEntry.Usage9 = Usage;
    NewEntry.Format9 = Format;
    NewEntry.Pool9 = Pool;
    NewEntry.Type9 = D3DRTYPE_CUBETEXTURE;
    ( *ppCubeTexture )->QueryInterface( IID_IDirect3DBaseTexture9, ( LPVOID* )&NewEntry.pTexture9 );

    m_TextureCache.Add( NewEntry );
    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateVolumeTextureFromFile( LPDIRECT3DDEVICE9 pDevice, LPCTSTR pSrcFile,
                                                         LPDIRECT3DVOLUMETEXTURE9* ppVolumeTexture )
{
    return CreateVolumeTextureFromFileEx( pDevice, pSrcFile, D3DX_DEFAULT, D3DX_DEFAULT, D3DX_DEFAULT, D3DX_DEFAULT,
                                          0, D3DFMT_UNKNOWN, D3DPOOL_MANAGED, D3DX_DEFAULT, D3DX_DEFAULT,
                                          0, NULL, NULL, ppVolumeTexture );
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateVolumeTextureFromFileEx( LPDIRECT3DDEVICE9 pDevice, LPCTSTR pSrcFile, UINT Width,
                                                           UINT Height, UINT Depth, UINT MipLevels, DWORD Usage,
                                                           D3DFORMAT Format, D3DPOOL Pool, DWORD Filter,
                                                           DWORD MipFilter, D3DCOLOR ColorKey,
                                                           D3DXIMAGE_INFO* pSrcInfo, PALETTEENTRY* pPalette,
                                                           LPDIRECT3DVOLUMETEXTURE9* ppTexture )
{
    // Search the cache for a matching entry.
    for( int i = 0; i < m_TextureCache.GetSize(); ++i )
    {
        DXUTCache_Texture& Entry = m_TextureCache[i];
        if( Entry.Location == DXUTCACHE_LOCATION_FILE &&
            !lstrcmpW( Entry.wszSource, pSrcFile ) &&
            Entry.Width == Width &&
            Entry.Height == Height &&
            Entry.Depth == Depth &&
            Entry.MipLevels == MipLevels &&
            Entry.Usage9 == Usage &&
            Entry.Format9 == Format &&
            Entry.Pool9 == Pool &&
            Entry.Type9 == D3DRTYPE_VOLUMETEXTURE )
        {
            // A match is found. Obtain the IDirect3DVolumeTexture9 interface and return that.
            return Entry.pTexture9->QueryInterface( IID_IDirect3DVolumeTexture9, ( LPVOID* )ppTexture );
        }
    }

    HRESULT hr;

    // No matching entry.  Load the resource and create a new entry.
    hr = D3DXCreateVolumeTextureFromFileEx( pDevice, pSrcFile, Width, Height, Depth, MipLevels, Usage, Format,
                                            Pool, Filter, MipFilter, ColorKey, pSrcInfo, pPalette, ppTexture );
    if( FAILED( hr ) )
        return hr;

    DXUTCache_Texture NewEntry;
    NewEntry.Location = DXUTCACHE_LOCATION_FILE;
    wcscpy_s( NewEntry.wszSource, MAX_PATH, pSrcFile );
    NewEntry.Width = Width;
    NewEntry.Height = Height;
    NewEntry.Depth = Depth;
    NewEntry.MipLevels = MipLevels;
    NewEntry.Usage9 = Usage;
    NewEntry.Format9 = Format;
    NewEntry.Pool9 = Pool;
    NewEntry.Type9 = D3DRTYPE_VOLUMETEXTURE;
    ( *ppTexture )->QueryInterface( IID_IDirect3DBaseTexture9, ( LPVOID* )&NewEntry.pTexture9 );

    m_TextureCache.Add( NewEntry );
    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateVolumeTextureFromResource( LPDIRECT3DDEVICE9 pDevice, HMODULE hSrcModule,
                                                             LPCTSTR pSrcResource,
                                                             LPDIRECT3DVOLUMETEXTURE9* ppVolumeTexture )
{
    return CreateVolumeTextureFromResourceEx( pDevice, hSrcModule, pSrcResource, D3DX_DEFAULT, D3DX_DEFAULT,
                                              D3DX_DEFAULT, D3DX_DEFAULT, 0, D3DFMT_UNKNOWN, D3DPOOL_MANAGED,
                                              D3DX_DEFAULT, D3DX_DEFAULT, 0, NULL, NULL, ppVolumeTexture );
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateVolumeTextureFromResourceEx( LPDIRECT3DDEVICE9 pDevice, HMODULE hSrcModule,
                                                               LPCTSTR pSrcResource, UINT Width, UINT Height,
                                                               UINT Depth, UINT MipLevels, DWORD Usage,
                                                               D3DFORMAT Format, D3DPOOL Pool, DWORD Filter,
                                                               DWORD MipFilter, D3DCOLOR ColorKey,
                                                               D3DXIMAGE_INFO* pSrcInfo, PALETTEENTRY* pPalette,
                                                               LPDIRECT3DVOLUMETEXTURE9* ppVolumeTexture )
{
    // Search the cache for a matching entry.
    for( int i = 0; i < m_TextureCache.GetSize(); ++i )
    {
        DXUTCache_Texture& Entry = m_TextureCache[i];
        if( Entry.Location == DXUTCACHE_LOCATION_RESOURCE &&
            Entry.hSrcModule == hSrcModule &&
            !lstrcmpW( Entry.wszSource, pSrcResource ) &&
            Entry.Width == Width &&
            Entry.Height == Height &&
            Entry.Depth == Depth &&
            Entry.MipLevels == MipLevels &&
            Entry.Usage9 == Usage &&
            Entry.Format9 == Format &&
            Entry.Pool9 == Pool &&
            Entry.Type9 == D3DRTYPE_VOLUMETEXTURE )
        {
            // A match is found. Obtain the IDirect3DVolumeTexture9 interface and return that.
            return Entry.pTexture9->QueryInterface( IID_IDirect3DVolumeTexture9, ( LPVOID* )ppVolumeTexture );
        }
    }

    HRESULT hr;

    // No matching entry.  Load the resource and create a new entry.
    hr = D3DXCreateVolumeTextureFromResourceEx( pDevice, hSrcModule, pSrcResource, Width, Height, Depth, MipLevels,
                                                Usage,
                                                Format, Pool, Filter, MipFilter, ColorKey, pSrcInfo, pPalette,
                                                ppVolumeTexture );
    if( FAILED( hr ) )
        return hr;

    DXUTCache_Texture NewEntry;
    NewEntry.Location = DXUTCACHE_LOCATION_RESOURCE;
    NewEntry.hSrcModule = hSrcModule;
    wcscpy_s( NewEntry.wszSource, MAX_PATH, pSrcResource );
    NewEntry.Width = Width;
    NewEntry.Height = Height;
    NewEntry.Depth = Depth;
    NewEntry.MipLevels = MipLevels;
    NewEntry.Usage9 = Usage;
    NewEntry.Format9 = Format;
    NewEntry.Pool9 = Pool;
    NewEntry.Type9 = D3DRTYPE_VOLUMETEXTURE;
    ( *ppVolumeTexture )->QueryInterface( IID_IDirect3DBaseTexture9, ( LPVOID* )&NewEntry.pTexture9 );

    m_TextureCache.Add( NewEntry );
    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateFont( LPDIRECT3DDEVICE9 pDevice, UINT Height, UINT Width, UINT Weight,
                                        UINT MipLevels, BOOL Italic, DWORD CharSet, DWORD OutputPrecision,
                                        DWORD Quality, DWORD PitchAndFamily, LPCTSTR pFacename, LPD3DXFONT* ppFont )
{
    D3DXFONT_DESCW Desc;

    Desc.Height = Height;
    Desc.Width = Width;
    Desc.Weight = Weight;
    Desc.MipLevels = MipLevels;
    Desc.Italic = Italic;
    Desc.CharSet = ( BYTE )CharSet;
    Desc.OutputPrecision = ( BYTE )OutputPrecision;
    Desc.Quality = ( BYTE )Quality;
    Desc.PitchAndFamily = ( BYTE )PitchAndFamily;
    wcscpy_s( Desc.FaceName, LF_FACESIZE, pFacename );

    return CreateFontIndirect( pDevice, &Desc, ppFont );
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateFontIndirect( LPDIRECT3DDEVICE9 pDevice, CONST D3DXFONT_DESC *pDesc, LPD3DXFONT *ppFont )
 {
    // Search the cache for a matching entry.
    for( int i = 0; i < m_FontCache.GetSize(); ++i )
 {
        DXUTCache_Font &Entry = m_FontCache[i];

        if( Entry.Width == pDesc->Width &&
            Entry.Height == pDesc->Height &&
            Entry.Weight == pDesc->Weight &&
            Entry.MipLevels == pDesc->MipLevels &&
            Entry.Italic == pDesc->Italic &&
            Entry.CharSet == pDesc->CharSet &&
            Entry.OutputPrecision == pDesc->OutputPrecision &&
            Entry.Quality == pDesc->Quality &&
            Entry.PitchAndFamily == pDesc->PitchAndFamily &&
            CompareString( LOCALE_USER_DEFAULT, NORM_IGNORECASE,
                           Entry.FaceName, -1,
                           pDesc->FaceName, -1 ) == CSTR_EQUAL )
 {
    // A match is found.  Increment the reference and return the ID3DXFont object.
            Entry.pFont->AddRef();
            *ppFont = Entry.pFont;
            return S_OK;
        }
    }

    HRESULT hr;

    // No matching entry.  Load the resource and create a new entry.
    hr = D3DXCreateFontIndirect( pDevice, pDesc, ppFont );
    if( FAILED( hr ) )
        return hr;

    DXUTCache_Font NewEntry;
    ( D3DXFONT_DESC & )NewEntry = *pDesc;
    NewEntry.pFont = *ppFont;
    NewEntry.pFont->AddRef();

    m_FontCache.Add( NewEntry );
    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateEffectFromFile( LPDIRECT3DDEVICE9 pDevice, LPCTSTR pSrcFile,
                                                  const D3DXMACRO* pDefines, LPD3DXINCLUDE pInclude, DWORD Flags,
                                                  LPD3DXEFFECTPOOL pPool, LPD3DXEFFECT* ppEffect,
                                                  LPD3DXBUFFER* ppCompilationErrors )
{
    // Search the cache for a matching entry.
    for( int i = 0; i < m_EffectCache.GetSize(); ++i )
    {
        DXUTCache_Effect& Entry = m_EffectCache[i];

        if( Entry.Location == DXUTCACHE_LOCATION_FILE &&
            !lstrcmpW( Entry.wszSource, pSrcFile ) &&
            Entry.dwFlags == Flags )
        {
            // A match is found.  Increment the ref coutn and return the ID3DXEffect object.
            *ppEffect = Entry.pEffect;
            ( *ppEffect )->AddRef();
            return S_OK;
        }
    }

    HRESULT hr;

    // No matching entry.  Load the resource and create a new entry.
    hr = D3DXCreateEffectFromFile( pDevice, pSrcFile, pDefines, pInclude, Flags, pPool, ppEffect,
                                   ppCompilationErrors );
    if( FAILED( hr ) )
        return hr;

    DXUTCache_Effect NewEntry;
    NewEntry.Location = DXUTCACHE_LOCATION_FILE;
    wcscpy_s( NewEntry.wszSource, MAX_PATH, pSrcFile );
    NewEntry.dwFlags = Flags;
    NewEntry.pEffect = *ppEffect;
    NewEntry.pEffect->AddRef();

    m_EffectCache.Add( NewEntry );
    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::CreateEffectFromResource( LPDIRECT3DDEVICE9 pDevice, HMODULE hSrcModule,
                                                      LPCTSTR pSrcResource, const D3DXMACRO* pDefines,
                                                      LPD3DXINCLUDE pInclude, DWORD Flags, LPD3DXEFFECTPOOL pPool,
                                                      LPD3DXEFFECT* ppEffect, LPD3DXBUFFER* ppCompilationErrors )
{
    // Search the cache for a matching entry.
    for( int i = 0; i < m_EffectCache.GetSize(); ++i )
    {
        DXUTCache_Effect& Entry = m_EffectCache[i];

        if( Entry.Location == DXUTCACHE_LOCATION_RESOURCE &&
            Entry.hSrcModule == hSrcModule &&
            !lstrcmpW( Entry.wszSource, pSrcResource ) &&
            Entry.dwFlags == Flags )
        {
            // A match is found.  Increment the ref coutn and return the ID3DXEffect object.
            *ppEffect = Entry.pEffect;
            ( *ppEffect )->AddRef();
            return S_OK;
        }
    }

    HRESULT hr;

    // No matching entry.  Load the resource and create a new entry.
    hr = D3DXCreateEffectFromResource( pDevice, hSrcModule, pSrcResource, pDefines, pInclude, Flags,
                                       pPool, ppEffect, ppCompilationErrors );
    if( FAILED( hr ) )
        return hr;

    DXUTCache_Effect NewEntry;
    NewEntry.Location = DXUTCACHE_LOCATION_RESOURCE;
    NewEntry.hSrcModule = hSrcModule;
    wcscpy_s( NewEntry.wszSource, MAX_PATH, pSrcResource );
    NewEntry.dwFlags = Flags;
    NewEntry.pEffect = *ppEffect;
    NewEntry.pEffect->AddRef();

    m_EffectCache.Add( NewEntry );
    return S_OK;
}


//--------------------------------------------------------------------------------------
// Device event callbacks
//--------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::OnCreateDevice( IDirect3DDevice9* pd3dDevice )
{
    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::OnResetDevice( IDirect3DDevice9* pd3dDevice )
{
    // Call OnResetDevice on all effect and font objects
    for( int i = 0; i < m_EffectCache.GetSize(); ++i )
        m_EffectCache[i].pEffect->OnResetDevice();
    for( int i = 0; i < m_FontCache.GetSize(); ++i )
        m_FontCache[i].pFont->OnResetDevice();


    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::OnLostDevice()
{
    // Call OnLostDevice on all effect and font objects
    for( int i = 0; i < m_EffectCache.GetSize(); ++i )
        m_EffectCache[i].pEffect->OnLostDevice();
    for( int i = 0; i < m_FontCache.GetSize(); ++i )
        m_FontCache[i].pFont->OnLostDevice();

    // Release all the default pool textures
    for( int i = m_TextureCache.GetSize() - 1; i >= 0; --i )
        if( m_TextureCache[i].Pool9 == D3DPOOL_DEFAULT )
        {
            SAFE_RELEASE( m_TextureCache[i].pTexture9 );
            m_TextureCache.Remove( i );  // Remove the entry
        }

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTResourceCache::OnDestroyDevice()
{
    // Release all resources
    for( int i = m_EffectCache.GetSize() - 1; i >= 0; --i )
    {
        SAFE_RELEASE( m_EffectCache[i].pEffect );
        m_EffectCache.Remove( i );
    }
    for( int i = m_FontCache.GetSize() - 1; i >= 0; --i )
    {
        SAFE_RELEASE( m_FontCache[i].pFont );
        m_FontCache.Remove( i );
    }
    for( int i = m_TextureCache.GetSize() - 1; i >= 0; --i )
    {
        SAFE_RELEASE( m_TextureCache[i].pTexture9 );
        SAFE_RELEASE( m_TextureCache[i].pSRV11 );
        m_TextureCache.Remove( i );
    }

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Desc: Returns a view matrix for rendering to a face of a cubemap.
//--------------------------------------------------------------------------------------
D3DXMATRIX WINAPI DXUTGetCubeMapViewMatrix( DWORD dwFace )
{
    D3DXVECTOR3 vEyePt = D3DXVECTOR3( 0.0f, 0.0f, 0.0f );
    D3DXVECTOR3 vLookDir;
    D3DXVECTOR3 vUpDir;

    switch( dwFace )
    {
        case D3DCUBEMAP_FACE_POSITIVE_X:
            vLookDir = D3DXVECTOR3( 1.0f, 0.0f, 0.0f );
            vUpDir = D3DXVECTOR3( 0.0f, 1.0f, 0.0f );
            break;
        case D3DCUBEMAP_FACE_NEGATIVE_X:
            vLookDir = D3DXVECTOR3( -1.0f, 0.0f, 0.0f );
            vUpDir = D3DXVECTOR3( 0.0f, 1.0f, 0.0f );
            break;
        case D3DCUBEMAP_FACE_POSITIVE_Y:
            vLookDir = D3DXVECTOR3( 0.0f, 1.0f, 0.0f );
            vUpDir = D3DXVECTOR3( 0.0f, 0.0f, -1.0f );
            break;
        case D3DCUBEMAP_FACE_NEGATIVE_Y:
            vLookDir = D3DXVECTOR3( 0.0f, -1.0f, 0.0f );
            vUpDir = D3DXVECTOR3( 0.0f, 0.0f, 1.0f );
            break;
        case D3DCUBEMAP_FACE_POSITIVE_Z:
            vLookDir = D3DXVECTOR3( 0.0f, 0.0f, 1.0f );
            vUpDir = D3DXVECTOR3( 0.0f, 1.0f, 0.0f );
            break;
        case D3DCUBEMAP_FACE_NEGATIVE_Z:
            vLookDir = D3DXVECTOR3( 0.0f, 0.0f, -1.0f );
            vUpDir = D3DXVECTOR3( 0.0f, 1.0f, 0.0f );
            break;
    }

    // Set the view transform for this cubemap surface
    D3DXMATRIXA16 mView;
    D3DXMatrixLookAtLH( &mView, &vEyePt, &vLookDir, &vUpDir );
    return mView;
}


//--------------------------------------------------------------------------------------
CDXUTLineManager::CDXUTLineManager()
{
    m_pd3dDevice = NULL;
    m_pD3DXLine = NULL;
}


//--------------------------------------------------------------------------------------
CDXUTLineManager::~CDXUTLineManager()
{
    OnDeletedDevice();
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTLineManager::OnCreatedDevice( IDirect3DDevice9* pd3dDevice )
{
    m_pd3dDevice = pd3dDevice;

    HRESULT hr;
    hr = D3DXCreateLine( m_pd3dDevice, &m_pD3DXLine );
    if( FAILED( hr ) )
        return hr;

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTLineManager::OnResetDevice()
{
    if( m_pD3DXLine )
        m_pD3DXLine->OnResetDevice();

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTLineManager::OnRender()
{
    HRESULT hr;
    if( NULL == m_pD3DXLine )
        return E_INVALIDARG;

    bool bDrawingHasBegun = false;
    float fLastWidth = 0.0f;
    bool bLastAntiAlias = false;

    for( int i = 0; i < m_LinesList.GetSize(); i++ )
    {
        LINE_NODE* pLineNode = m_LinesList.GetAt( i );
        if( pLineNode )
        {
            if( !bDrawingHasBegun ||
                fLastWidth != pLineNode->fWidth ||
                bLastAntiAlias != pLineNode->bAntiAlias )
            {
                if( bDrawingHasBegun )
                {
                    hr = m_pD3DXLine->End();
                    if( FAILED( hr ) )
                        return hr;
                }

                m_pD3DXLine->SetWidth( pLineNode->fWidth );
                m_pD3DXLine->SetAntialias( pLineNode->bAntiAlias );

                fLastWidth = pLineNode->fWidth;
                bLastAntiAlias = pLineNode->bAntiAlias;

                hr = m_pD3DXLine->Begin();
                if( FAILED( hr ) )
                    return hr;
                bDrawingHasBegun = true;
            }

            hr = m_pD3DXLine->Draw( pLineNode->pVertexList, pLineNode->dwVertexListCount, pLineNode->Color );
            if( FAILED( hr ) )
                return hr;
        }
    }

    if( bDrawingHasBegun )
    {
        hr = m_pD3DXLine->End();
        if( FAILED( hr ) )
            return hr;
    }

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTLineManager::OnLostDevice()
{
    if( m_pD3DXLine )
        m_pD3DXLine->OnLostDevice();

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTLineManager::OnDeletedDevice()
{
    RemoveAllLines();
    SAFE_RELEASE( m_pD3DXLine );

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTLineManager::AddLine( int* pnLineID, D3DXVECTOR2* pVertexList, DWORD dwVertexListCount, D3DCOLOR Color,
                                   float fWidth, float fScaleRatio, bool bAntiAlias )
{
    if( pVertexList == NULL || dwVertexListCount == 0 )
        return E_INVALIDARG;

    LINE_NODE* pLineNode = new LINE_NODE;
    if( pLineNode == NULL )
        return E_OUTOFMEMORY;
    ZeroMemory( pLineNode, sizeof( LINE_NODE ) );

    pLineNode->nLineID = m_LinesList.GetSize();
    pLineNode->Color = Color;
    pLineNode->fWidth = fWidth;
    pLineNode->bAntiAlias = bAntiAlias;
    pLineNode->dwVertexListCount = dwVertexListCount;

    if( pnLineID )
        *pnLineID = pLineNode->nLineID;

    pLineNode->pVertexList = new D3DXVECTOR2[dwVertexListCount];
    if( pLineNode->pVertexList == NULL )
    {
        delete pLineNode;
        return E_OUTOFMEMORY;
    }
    for( DWORD i = 0; i < dwVertexListCount; i++ )
    {
        pLineNode->pVertexList[i] = pVertexList[i] * fScaleRatio;
    }

    m_LinesList.Add( pLineNode );

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTLineManager::AddRect( int* pnLineID, RECT rc, D3DCOLOR Color, float fWidth, float fScaleRatio,
                                   bool bAntiAlias )
{
    if( fWidth > 2.0f )
    {
        D3DXVECTOR2 vertexList[8];

        vertexList[0].x = ( float )rc.left;
        vertexList[0].y = ( float )rc.top - ( fWidth / 2.0f );

        vertexList[1].x = ( float )rc.left;
        vertexList[1].y = ( float )rc.bottom + ( fWidth / 2.0f );

        vertexList[2].x = ( float )rc.left;
        vertexList[2].y = ( float )rc.bottom - 0.5f;

        vertexList[3].x = ( float )rc.right;
        vertexList[3].y = ( float )rc.bottom - 0.5f;

        vertexList[4].x = ( float )rc.right;
        vertexList[4].y = ( float )rc.bottom + ( fWidth / 2.0f );

        vertexList[5].x = ( float )rc.right;
        vertexList[5].y = ( float )rc.top - ( fWidth / 2.0f );

        vertexList[6].x = ( float )rc.right;
        vertexList[6].y = ( float )rc.top;

        vertexList[7].x = ( float )rc.left;
        vertexList[7].y = ( float )rc.top;

        return AddLine( pnLineID, vertexList, 8, Color, fWidth, fScaleRatio, bAntiAlias );
    }
    else
    {
        D3DXVECTOR2 vertexList[5];
        vertexList[0].x = ( float )rc.left;
        vertexList[0].y = ( float )rc.top;

        vertexList[1].x = ( float )rc.left;
        vertexList[1].y = ( float )rc.bottom;

        vertexList[2].x = ( float )rc.right;
        vertexList[2].y = ( float )rc.bottom;

        vertexList[3].x = ( float )rc.right;
        vertexList[3].y = ( float )rc.top;

        vertexList[4].x = ( float )rc.left;
        vertexList[4].y = ( float )rc.top;

        return AddLine( pnLineID, vertexList, 5, Color, fWidth, fScaleRatio, bAntiAlias );
    }
}



//--------------------------------------------------------------------------------------
HRESULT CDXUTLineManager::RemoveLine( int nLineID )
{
    for( int i = 0; i < m_LinesList.GetSize(); i++ )
    {
        LINE_NODE* pLineNode = m_LinesList.GetAt( i );
        if( pLineNode && pLineNode->nLineID == nLineID )
        {
            SAFE_DELETE_ARRAY( pLineNode->pVertexList );
            delete pLineNode;
            m_LinesList.SetAt( i, NULL );
        }
    }

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTLineManager::RemoveAllLines()
{
    for( int i = 0; i < m_LinesList.GetSize(); i++ )
    {
        LINE_NODE* pLineNode = m_LinesList.GetAt( i );
        if( pLineNode )
        {
            SAFE_DELETE_ARRAY( pLineNode->pVertexList );
            delete pLineNode;
        }
    }
    m_LinesList.RemoveAll();

    return S_OK;
}


//--------------------------------------------------------------------------------------
CDXUTTextHelper::CDXUTTextHelper( ID3DXFont* pFont9, ID3DXSprite* pSprite9,  int nLineHeight )
{
    Init( pFont9, pSprite9, nLineHeight );
}

CDXUTTextHelper::CDXUTTextHelper( ID3D11Device* pd3d11Device, ID3D11DeviceContext* pd3d11DeviceContext, CDXUTDialogResourceManager* pManager, int nLineHeight )
{
    Init( NULL, NULL, nLineHeight );
    m_pd3d11Device = pd3d11Device;
    m_pd3d11DeviceContext = pd3d11DeviceContext;
    m_pManager = pManager;
}
CDXUTTextHelper::~CDXUTTextHelper()
{

}

//--------------------------------------------------------------------------------------
void CDXUTTextHelper::Init( ID3DXFont* pFont9, ID3DXSprite* pSprite9, 
                            int nLineHeight )
{
    m_pFont9 = pFont9;
    m_pSprite9 = pSprite9;
    m_clr = D3DXCOLOR( 1, 1, 1, 1 );
    m_pt.x = 0;
    m_pt.y = 0;
    m_nLineHeight = nLineHeight;
    m_pd3d11Device = NULL;
    m_pd3d11DeviceContext = NULL;
    m_pManager = NULL; 

    // Create a blend state if a sprite is passed in
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTTextHelper::DrawFormattedTextLine( const WCHAR* strMsg, ... )
{
    WCHAR strBuffer[512];

    va_list args;
    va_start( args, strMsg );
    vswprintf_s( strBuffer, 512, strMsg, args );
    strBuffer[511] = L'\0';
    va_end( args );

    return DrawTextLine( strBuffer );
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTTextHelper::DrawTextLine( const WCHAR* strMsg )
{
    if( NULL == m_pFont9 && NULL == m_pd3d11DeviceContext )
        return DXUT_ERR_MSGBOX( L"DrawTextLine", E_INVALIDARG );

    HRESULT hr = S_OK;
    RECT rc;
    SetRect( &rc, m_pt.x, m_pt.y, 0, 0 );
    if( m_pFont9 )
        hr = m_pFont9->DrawText( m_pSprite9, strMsg, -1, &rc, DT_NOCLIP, m_clr );
    else if( m_pd3d11DeviceContext )
        DrawText11DXUT( m_pd3d11Device, m_pd3d11DeviceContext, strMsg, rc, m_clr,
                        (float)m_pManager->m_nBackBufferWidth, (float)m_pManager->m_nBackBufferHeight, false );

    if( FAILED( hr ) )
        return DXTRACE_ERR_MSGBOX( L"DrawText", hr );

    m_pt.y += m_nLineHeight;

    return S_OK;
}


HRESULT CDXUTTextHelper::DrawFormattedTextLine( RECT& rc, DWORD dwFlags, const WCHAR* strMsg, ... )
{
    WCHAR strBuffer[512];

    va_list args;
    va_start( args, strMsg );
    vswprintf_s( strBuffer, 512, strMsg, args );
    strBuffer[511] = L'\0';
    va_end( args );

    return DrawTextLine( rc, dwFlags, strBuffer );
}


HRESULT CDXUTTextHelper::DrawTextLine( RECT& rc, DWORD dwFlags, const WCHAR* strMsg )
{
    if( NULL == m_pFont9 && NULL == m_pd3d11DeviceContext )
        return DXUT_ERR_MSGBOX( L"DrawTextLine", E_INVALIDARG );

    HRESULT hr = S_OK;
    if( m_pFont9 )
        hr = m_pFont9->DrawText( m_pSprite9, strMsg, -1, &rc, dwFlags, m_clr );
    else if( m_pd3d11DeviceContext )
        DrawText11DXUT( m_pd3d11Device, m_pd3d11DeviceContext, strMsg, rc, m_clr,
                        (float)m_pManager->m_nBackBufferWidth, (float)m_pManager->m_nBackBufferHeight, false );

    if( FAILED( hr ) )
        return DXTRACE_ERR_MSGBOX( L"DrawText", hr );

    m_pt.y += m_nLineHeight;

    return S_OK;
}


//--------------------------------------------------------------------------------------
void CDXUTTextHelper::Begin()
{
    if( m_pSprite9 )
        m_pSprite9->Begin( D3DXSPRITE_ALPHABLEND | D3DXSPRITE_SORT_TEXTURE );

    if( m_pd3d11DeviceContext )
    {
        m_pManager->StoreD3D11State( m_pd3d11DeviceContext );
        m_pManager->ApplyRenderUI11( m_pd3d11DeviceContext );
    }


}
void CDXUTTextHelper::End()
{
    if( m_pSprite9 )
        m_pSprite9->End();

    if( m_pd3d11DeviceContext )
    {
        m_pManager->RestoreD3D11State( m_pd3d11DeviceContext );
    }
}
