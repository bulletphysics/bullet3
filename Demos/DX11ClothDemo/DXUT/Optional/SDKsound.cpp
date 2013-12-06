//-----------------------------------------------------------------------------
// File: DXUTsound.cpp
//
// Desc: DirectSound framework classes for playing wav files in DirectSound
//       buffers. Feel free to use these classes as a starting point for adding
//       extra functionality.
//
// Copyright (c) Microsoft Corp. All rights reserved.
//-----------------------------------------------------------------------------
#define STRICT
#include "DXUT.h"
#include <mmsystem.h>
#include <dsound.h>
#include "SDKsound.h"
#include "SDKwavefile.h"
#undef min // use __min instead
#undef max // use __max instead


//-----------------------------------------------------------------------------
// Name: CSoundManager::CSoundManager()
// Desc: Constructs the class
//-----------------------------------------------------------------------------
CSoundManager::CSoundManager()
{
    m_pDS = NULL;
}


//-----------------------------------------------------------------------------
// Name: CSoundManager::~CSoundManager()
// Desc: Destroys the class
//-----------------------------------------------------------------------------
CSoundManager::~CSoundManager()
{
    SAFE_RELEASE( m_pDS );
}


//-----------------------------------------------------------------------------
// Name: CSoundManager::Initialize()
// Desc: Initializes the IDirectSound object and also sets the primary buffer
//       format.  This function must be called before any others.
//-----------------------------------------------------------------------------
HRESULT CSoundManager::Initialize( HWND hWnd,
                                   DWORD dwCoopLevel )
{
    HRESULT hr;

    SAFE_RELEASE( m_pDS );

    // Create IDirectSound using the primary sound device
    if( FAILED( hr = DirectSoundCreate8( NULL, &m_pDS, NULL ) ) )
        return DXUT_ERR( L"DirectSoundCreate8", hr );

    // Set DirectSound coop level
    if( FAILED( hr = m_pDS->SetCooperativeLevel( hWnd, dwCoopLevel ) ) )
        return DXUT_ERR( L"SetCooperativeLevel", hr );

    return S_OK;
}


//-----------------------------------------------------------------------------
// Name: CSoundManager::SetPrimaryBufferFormat()
// Desc: Set primary buffer to a specified format
//       !WARNING! - Setting the primary buffer format and then using this
//                   same DirectSound object for DirectMusic messes up
//                   DirectMusic!
//       For example, to set the primary buffer format to 22kHz stereo, 16-bit
//       then:   dwPrimaryChannels = 2
//               dwPrimaryFreq     = 22050,
//               dwPrimaryBitRate  = 16
//-----------------------------------------------------------------------------
HRESULT CSoundManager::SetPrimaryBufferFormat( DWORD dwPrimaryChannels,
                                               DWORD dwPrimaryFreq,
                                               DWORD dwPrimaryBitRate )
{
    HRESULT hr;
    LPDIRECTSOUNDBUFFER pDSBPrimary = NULL;

    if( m_pDS == NULL )
        return CO_E_NOTINITIALIZED;

    // Get the primary buffer
    DSBUFFERDESC dsbd;
    ZeroMemory( &dsbd, sizeof( DSBUFFERDESC ) );
    dsbd.dwSize = sizeof( DSBUFFERDESC );
    dsbd.dwFlags = DSBCAPS_PRIMARYBUFFER;
    dsbd.dwBufferBytes = 0;
    dsbd.lpwfxFormat = NULL;

    if( FAILED( hr = m_pDS->CreateSoundBuffer( &dsbd, &pDSBPrimary, NULL ) ) )
        return DXUT_ERR( L"CreateSoundBuffer", hr );

    WAVEFORMATEX wfx;
    ZeroMemory( &wfx, sizeof( WAVEFORMATEX ) );
    wfx.wFormatTag = ( WORD )WAVE_FORMAT_PCM;
    wfx.nChannels = ( WORD )dwPrimaryChannels;
    wfx.nSamplesPerSec = ( DWORD )dwPrimaryFreq;
    wfx.wBitsPerSample = ( WORD )dwPrimaryBitRate;
    wfx.nBlockAlign = ( WORD )( wfx.wBitsPerSample / 8 * wfx.nChannels );
    wfx.nAvgBytesPerSec = ( DWORD )( wfx.nSamplesPerSec * wfx.nBlockAlign );

    if( FAILED( hr = pDSBPrimary->SetFormat( &wfx ) ) )
        return DXUT_ERR( L"SetFormat", hr );

    SAFE_RELEASE( pDSBPrimary );

    return S_OK;
}


//-----------------------------------------------------------------------------
// Name: CSoundManager::Get3DListenerInterface()
// Desc: Returns the 3D listener interface associated with primary buffer.
//-----------------------------------------------------------------------------
HRESULT CSoundManager::Get3DListenerInterface( LPDIRECTSOUND3DLISTENER* ppDSListener )
{
    HRESULT hr;
    DSBUFFERDESC dsbdesc;
    LPDIRECTSOUNDBUFFER pDSBPrimary = NULL;

    if( ppDSListener == NULL )
        return E_INVALIDARG;
    if( m_pDS == NULL )
        return CO_E_NOTINITIALIZED;

    *ppDSListener = NULL;

    // Obtain primary buffer, asking it for 3D control
    ZeroMemory( &dsbdesc, sizeof( DSBUFFERDESC ) );
    dsbdesc.dwSize = sizeof( DSBUFFERDESC );
    dsbdesc.dwFlags = DSBCAPS_CTRL3D | DSBCAPS_PRIMARYBUFFER;
    if( FAILED( hr = m_pDS->CreateSoundBuffer( &dsbdesc, &pDSBPrimary, NULL ) ) )
        return DXUT_ERR( L"CreateSoundBuffer", hr );

    if( FAILED( hr = pDSBPrimary->QueryInterface( IID_IDirectSound3DListener,
                                                  ( VOID** )ppDSListener ) ) )
    {
        SAFE_RELEASE( pDSBPrimary );
        return DXUT_ERR( L"QueryInterface", hr );
    }

    // Release the primary buffer, since it is not need anymore
    SAFE_RELEASE( pDSBPrimary );

    return S_OK;
}


//-----------------------------------------------------------------------------
// Name: CSoundManager::Create()
// Desc:
//-----------------------------------------------------------------------------
HRESULT CSoundManager::Create( CSound** ppSound,
                               LPWSTR strWaveFileName,
                               DWORD dwCreationFlags,
                               GUID guid3DAlgorithm,
                               DWORD dwNumBuffers )
{
    HRESULT hr;
    HRESULT hrRet = S_OK;
    DWORD i;
    LPDIRECTSOUNDBUFFER* apDSBuffer = NULL;
    DWORD dwDSBufferSize = NULL;
    CWaveFile* pWaveFile = NULL;

    if( m_pDS == NULL )
        return CO_E_NOTINITIALIZED;
    if( strWaveFileName == NULL || ppSound == NULL || dwNumBuffers < 1 )
        return E_INVALIDARG;

    apDSBuffer = new LPDIRECTSOUNDBUFFER[dwNumBuffers];
    if( apDSBuffer == NULL )
    {
        hr = E_OUTOFMEMORY;
        goto LFail;
    }

    pWaveFile = new CWaveFile();
    if( pWaveFile == NULL )
    {
        hr = E_OUTOFMEMORY;
        goto LFail;
    }

    pWaveFile->Open( strWaveFileName, NULL, WAVEFILE_READ );

    if( pWaveFile->GetSize() == 0 )
    {
        // Wave is blank, so don't create it.
        hr = E_FAIL;
        goto LFail;
    }

    // Make the DirectSound buffer the same size as the wav file
    dwDSBufferSize = pWaveFile->GetSize();

    // Create the direct sound buffer, and only request the flags needed
    // since each requires some overhead and limits if the buffer can
    // be hardware accelerated
    DSBUFFERDESC dsbd;
    ZeroMemory( &dsbd, sizeof( DSBUFFERDESC ) );
    dsbd.dwSize = sizeof( DSBUFFERDESC );
    dsbd.dwFlags = dwCreationFlags;
    dsbd.dwBufferBytes = dwDSBufferSize;
    dsbd.guid3DAlgorithm = guid3DAlgorithm;
    dsbd.lpwfxFormat = pWaveFile->m_pwfx;

    // DirectSound is only guarenteed to play PCM data.  Other
    // formats may or may not work depending the sound card driver.
    hr = m_pDS->CreateSoundBuffer( &dsbd, &apDSBuffer[0], NULL );

    // Be sure to return this error code if it occurs so the
    // callers knows this happened.
    if( hr == DS_NO_VIRTUALIZATION )
        hrRet = DS_NO_VIRTUALIZATION;

    if( FAILED( hr ) )
    {
        // DSERR_BUFFERTOOSMALL will be returned if the buffer is
        // less than DSBSIZE_FX_MIN and the buffer is created
        // with DSBCAPS_CTRLFX.

        // It might also fail if hardware buffer mixing was requested
        // on a device that doesn't support it.
        DXUT_ERR( L"CreateSoundBuffer", hr );

        goto LFail;
    }

    // Default to use DuplicateSoundBuffer() when created extra buffers since always
    // create a buffer that uses the same memory however DuplicateSoundBuffer() will fail if
    // DSBCAPS_CTRLFX is used, so use CreateSoundBuffer() instead in this case.
    if( ( dwCreationFlags & DSBCAPS_CTRLFX ) == 0 )
    {
        for( i = 1; i < dwNumBuffers; i++ )
        {
            if( FAILED( hr = m_pDS->DuplicateSoundBuffer( apDSBuffer[0], &apDSBuffer[i] ) ) )
            {
                DXUT_ERR( L"DuplicateSoundBuffer", hr );
                goto LFail;
            }
        }
    }
    else
    {
        for( i = 1; i < dwNumBuffers; i++ )
        {
            hr = m_pDS->CreateSoundBuffer( &dsbd, &apDSBuffer[i], NULL );
            if( FAILED( hr ) )
            {
                DXUT_ERR( L"CreateSoundBuffer", hr );
                goto LFail;
            }
        }
    }

    // Create the sound
    *ppSound = new CSound( apDSBuffer, dwDSBufferSize, dwNumBuffers, pWaveFile, dwCreationFlags );

    SAFE_DELETE_ARRAY( apDSBuffer );
    return hrRet;

LFail:
    // Cleanup
    SAFE_DELETE( pWaveFile );
    SAFE_DELETE_ARRAY( apDSBuffer );
    return hr;
}


//-----------------------------------------------------------------------------
// Name: CSoundManager::CreateFromMemory()
// Desc:
//-----------------------------------------------------------------------------
HRESULT CSoundManager::CreateFromMemory( CSound** ppSound,
                                         BYTE* pbData,
                                         ULONG ulDataSize,
                                         LPWAVEFORMATEX pwfx,
                                         DWORD dwCreationFlags,
                                         GUID guid3DAlgorithm,
                                         DWORD dwNumBuffers )
{
    HRESULT hr;
    DWORD i;
    LPDIRECTSOUNDBUFFER* apDSBuffer = NULL;
    DWORD dwDSBufferSize = NULL;
    CWaveFile* pWaveFile = NULL;

    if( m_pDS == NULL )
        return CO_E_NOTINITIALIZED;
    if( pbData == NULL || ppSound == NULL || dwNumBuffers < 1 )
        return E_INVALIDARG;

    apDSBuffer = new LPDIRECTSOUNDBUFFER[dwNumBuffers];
    if( apDSBuffer == NULL )
    {
        hr = E_OUTOFMEMORY;
        goto LFail;
    }

    pWaveFile = new CWaveFile();
    if( pWaveFile == NULL )
    {
        hr = E_OUTOFMEMORY;
        goto LFail;
    }

    pWaveFile->OpenFromMemory( pbData, ulDataSize, pwfx, WAVEFILE_READ );


    // Make the DirectSound buffer the same size as the wav file
    dwDSBufferSize = ulDataSize;

    // Create the direct sound buffer, and only request the flags needed
    // since each requires some overhead and limits if the buffer can
    // be hardware accelerated
    DSBUFFERDESC dsbd;
    ZeroMemory( &dsbd, sizeof( DSBUFFERDESC ) );
    dsbd.dwSize = sizeof( DSBUFFERDESC );
    dsbd.dwFlags = dwCreationFlags;
    dsbd.dwBufferBytes = dwDSBufferSize;
    dsbd.guid3DAlgorithm = guid3DAlgorithm;
    dsbd.lpwfxFormat = pwfx;

    if( FAILED( hr = m_pDS->CreateSoundBuffer( &dsbd, &apDSBuffer[0], NULL ) ) )
    {
        DXUT_ERR( L"CreateSoundBuffer", hr );
        goto LFail;
    }

    // Default to use DuplicateSoundBuffer() when created extra buffers since always
    // create a buffer that uses the same memory however DuplicateSoundBuffer() will fail if
    // DSBCAPS_CTRLFX is used, so use CreateSoundBuffer() instead in this case.
    if( ( dwCreationFlags & DSBCAPS_CTRLFX ) == 0 )
    {
        for( i = 1; i < dwNumBuffers; i++ )
        {
            if( FAILED( hr = m_pDS->DuplicateSoundBuffer( apDSBuffer[0], &apDSBuffer[i] ) ) )
            {
                DXUT_ERR( L"DuplicateSoundBuffer", hr );
                goto LFail;
            }
        }
    }
    else
    {
        for( i = 1; i < dwNumBuffers; i++ )
        {
            hr = m_pDS->CreateSoundBuffer( &dsbd, &apDSBuffer[i], NULL );
            if( FAILED( hr ) )
            {
                DXUT_ERR( L"CreateSoundBuffer", hr );
                goto LFail;
            }
        }
    }

    // Create the sound
    *ppSound = new CSound( apDSBuffer, dwDSBufferSize, dwNumBuffers, pWaveFile, dwCreationFlags );

    SAFE_DELETE_ARRAY( apDSBuffer );
    return S_OK;

LFail:
    // Cleanup

    SAFE_DELETE_ARRAY( apDSBuffer );
    return hr;
}


//-----------------------------------------------------------------------------
// Name: CSoundManager::CreateStreaming()
// Desc:
//-----------------------------------------------------------------------------
HRESULT CSoundManager::CreateStreaming( CStreamingSound** ppStreamingSound,
                                        LPWSTR strWaveFileName,
                                        DWORD dwCreationFlags,
                                        GUID guid3DAlgorithm,
                                        DWORD dwNotifyCount,
                                        DWORD dwNotifySize,
                                        HANDLE hNotifyEvent )
{
    HRESULT hr;

    if( m_pDS == NULL )
        return CO_E_NOTINITIALIZED;
    if( strWaveFileName == NULL || ppStreamingSound == NULL || hNotifyEvent == NULL )
        return E_INVALIDARG;

    LPDIRECTSOUNDBUFFER pDSBuffer = NULL;
    DWORD dwDSBufferSize = NULL;
    CWaveFile* pWaveFile = NULL;
    DSBPOSITIONNOTIFY* aPosNotify = NULL;
    LPDIRECTSOUNDNOTIFY pDSNotify = NULL;

    pWaveFile = new CWaveFile();
    if( pWaveFile == NULL )
        return E_OUTOFMEMORY;
    pWaveFile->Open( strWaveFileName, NULL, WAVEFILE_READ );

    // Figure out how big the DirectSound buffer should be
    dwDSBufferSize = dwNotifySize * dwNotifyCount;

    // Set up the direct sound buffer.  Request the NOTIFY flag, so
    // that we are notified as the sound buffer plays.  Note, that using this flag
    // may limit the amount of hardware acceleration that can occur.
    DSBUFFERDESC dsbd;
    ZeroMemory( &dsbd, sizeof( DSBUFFERDESC ) );
    dsbd.dwSize = sizeof( DSBUFFERDESC );
    dsbd.dwFlags = dwCreationFlags |
        DSBCAPS_CTRLPOSITIONNOTIFY |
        DSBCAPS_GETCURRENTPOSITION2;
    dsbd.dwBufferBytes = dwDSBufferSize;
    dsbd.guid3DAlgorithm = guid3DAlgorithm;
    dsbd.lpwfxFormat = pWaveFile->m_pwfx;

    if( FAILED( hr = m_pDS->CreateSoundBuffer( &dsbd, &pDSBuffer, NULL ) ) )
    {
        // If wave format isn't then it will return
        // either DSERR_BADFORMAT or E_INVALIDARG
        if( hr == DSERR_BADFORMAT || hr == E_INVALIDARG )
            return DXUT_ERR( L"CreateSoundBuffer", hr );

        return DXUT_ERR( L"CreateSoundBuffer", hr );
    }

    // Create the notification events, so that we know when to fill
    // the buffer as the sound plays.
    if( FAILED( hr = pDSBuffer->QueryInterface( IID_IDirectSoundNotify,
                                                ( VOID** )&pDSNotify ) ) )
    {
        SAFE_DELETE_ARRAY( aPosNotify );
        return DXUT_ERR( L"QueryInterface", hr );
    }

    aPosNotify = new DSBPOSITIONNOTIFY[ dwNotifyCount ];
    if( aPosNotify == NULL )
        return E_OUTOFMEMORY;

    for( DWORD i = 0; i < dwNotifyCount; i++ )
    {
        aPosNotify[i].dwOffset = ( dwNotifySize * i ) + dwNotifySize - 1;
        aPosNotify[i].hEventNotify = hNotifyEvent;
    }

    // Tell DirectSound when to notify us. The notification will come in the from
    // of signaled events that are handled in WinMain()
    if( FAILED( hr = pDSNotify->SetNotificationPositions( dwNotifyCount,
                                                          aPosNotify ) ) )
    {
        SAFE_RELEASE( pDSNotify );
        SAFE_DELETE_ARRAY( aPosNotify );
        return DXUT_ERR( L"SetNotificationPositions", hr );
    }

    SAFE_RELEASE( pDSNotify );
    SAFE_DELETE_ARRAY( aPosNotify );

    // Create the sound
    *ppStreamingSound = new CStreamingSound( pDSBuffer, dwDSBufferSize, pWaveFile, dwNotifySize );

    return S_OK;
}


//-----------------------------------------------------------------------------
// Name: CSound::CSound()
// Desc: Constructs the class
//-----------------------------------------------------------------------------
CSound::CSound( LPDIRECTSOUNDBUFFER* apDSBuffer, DWORD dwDSBufferSize,
                DWORD dwNumBuffers, CWaveFile* pWaveFile, DWORD dwCreationFlags )
{
    DWORD i;

    if( dwNumBuffers <= 0 )
        return;

    m_apDSBuffer = new LPDIRECTSOUNDBUFFER[dwNumBuffers];
    if( NULL != m_apDSBuffer )
    {
        for( i = 0; i < dwNumBuffers; i++ )
            m_apDSBuffer[i] = apDSBuffer[i];

        m_dwDSBufferSize = dwDSBufferSize;
        m_dwNumBuffers = dwNumBuffers;
        m_pWaveFile = pWaveFile;
        m_dwCreationFlags = dwCreationFlags;

        FillBufferWithSound( m_apDSBuffer[0], FALSE );
    }
}


//-----------------------------------------------------------------------------
// Name: CSound::~CSound()
// Desc: Destroys the class
//-----------------------------------------------------------------------------
CSound::~CSound()
{
    for( DWORD i = 0; i < m_dwNumBuffers; i++ )
    {
        SAFE_RELEASE( m_apDSBuffer[i] );
    }

    SAFE_DELETE_ARRAY( m_apDSBuffer );
    SAFE_DELETE( m_pWaveFile );
}


//-----------------------------------------------------------------------------
// Name: CSound::FillBufferWithSound()
// Desc: Fills a DirectSound buffer with a sound file
//-----------------------------------------------------------------------------
HRESULT CSound::FillBufferWithSound( LPDIRECTSOUNDBUFFER pDSB, BOOL bRepeatWavIfBufferLarger )
{
    HRESULT hr;
    VOID* pDSLockedBuffer = NULL; // Pointer to locked buffer memory
    DWORD dwDSLockedBufferSize = 0;    // Size of the locked DirectSound buffer
    DWORD dwWavDataRead = 0;    // Amount of data read from the wav file

    if( pDSB == NULL )
        return CO_E_NOTINITIALIZED;

    // Make sure we have focus, and we didn't just switch in from
    // an app which had a DirectSound device
    if( FAILED( hr = RestoreBuffer( pDSB, NULL ) ) )
        return DXUT_ERR( L"RestoreBuffer", hr );

    // Lock the buffer down
    if( FAILED( hr = pDSB->Lock( 0, m_dwDSBufferSize,
                                 &pDSLockedBuffer, &dwDSLockedBufferSize,
                                 NULL, NULL, 0L ) ) )
        return DXUT_ERR( L"Lock", hr );

    // Reset the wave file to the beginning
    m_pWaveFile->ResetFile();

    if( FAILED( hr = m_pWaveFile->Read( ( BYTE* )pDSLockedBuffer,
                                        dwDSLockedBufferSize,
                                        &dwWavDataRead ) ) )
        return DXUT_ERR( L"Read", hr );

    if( dwWavDataRead == 0 )
    {
        // Wav is blank, so just fill with silence
        FillMemory( ( BYTE* )pDSLockedBuffer,
                    dwDSLockedBufferSize,
                    ( BYTE )( m_pWaveFile->m_pwfx->wBitsPerSample == 8 ? 128 : 0 ) );
    }
    else if( dwWavDataRead < dwDSLockedBufferSize )
    {
        // If the wav file was smaller than the DirectSound buffer,
        // we need to fill the remainder of the buffer with data
        if( bRepeatWavIfBufferLarger )
        {
            // Reset the file and fill the buffer with wav data
            DWORD dwReadSoFar = dwWavDataRead;    // From previous call above.
            while( dwReadSoFar < dwDSLockedBufferSize )
            {
                // This will keep reading in until the buffer is full
                // for very short files
                if( FAILED( hr = m_pWaveFile->ResetFile() ) )
                    return DXUT_ERR( L"ResetFile", hr );

                hr = m_pWaveFile->Read( ( BYTE* )pDSLockedBuffer + dwReadSoFar,
                                        dwDSLockedBufferSize - dwReadSoFar,
                                        &dwWavDataRead );
                if( FAILED( hr ) )
                    return DXUT_ERR( L"Read", hr );

                dwReadSoFar += dwWavDataRead;
            }
        }
        else
        {
            // Don't repeat the wav file, just fill in silence
            FillMemory( ( BYTE* )pDSLockedBuffer + dwWavDataRead,
                        dwDSLockedBufferSize - dwWavDataRead,
                        ( BYTE )( m_pWaveFile->m_pwfx->wBitsPerSample == 8 ? 128 : 0 ) );
        }
    }

    // Unlock the buffer, we don't need it anymore.
    pDSB->Unlock( pDSLockedBuffer, dwDSLockedBufferSize, NULL, 0 );

    return S_OK;
}


//-----------------------------------------------------------------------------
// Name: CSound::RestoreBuffer()
// Desc: Restores the lost buffer. *pbWasRestored returns TRUE if the buffer was
//       restored.  It can also NULL if the information is not needed.
//-----------------------------------------------------------------------------
HRESULT CSound::RestoreBuffer( LPDIRECTSOUNDBUFFER pDSB, BOOL* pbWasRestored )
{
    HRESULT hr;

    if( pDSB == NULL )
        return CO_E_NOTINITIALIZED;
    if( pbWasRestored )
        *pbWasRestored = FALSE;

    DWORD dwStatus;
    if( FAILED( hr = pDSB->GetStatus( &dwStatus ) ) )
        return DXUT_ERR( L"GetStatus", hr );

    if( dwStatus & DSBSTATUS_BUFFERLOST )
    {
        // Since the app could have just been activated, then
        // DirectSound may not be giving us control yet, so
        // the restoring the buffer may fail.
        // If it does, sleep until DirectSound gives us control.
        do
        {
            hr = pDSB->Restore();
            if( hr == DSERR_BUFFERLOST )
                Sleep( 10 );
        }        while( ( hr = pDSB->Restore() ) == DSERR_BUFFERLOST );

        if( pbWasRestored != NULL )
            *pbWasRestored = TRUE;

        return S_OK;
    }
    else
    {
        return S_FALSE;
    }
}


//-----------------------------------------------------------------------------
// Name: CSound::GetFreeBuffer()
// Desc: Finding the first buffer that is not playing and return a pointer to
//       it, or if all are playing return a pointer to a randomly selected buffer.
//-----------------------------------------------------------------------------
LPDIRECTSOUNDBUFFER CSound::GetFreeBuffer()
{
    if( m_apDSBuffer == NULL )
        return FALSE;

    DWORD i;
    for( i = 0; i < m_dwNumBuffers; i++ )
    {
        if( m_apDSBuffer[i] )
        {
            DWORD dwStatus = 0;
            m_apDSBuffer[i]->GetStatus( &dwStatus );
            if( ( dwStatus & DSBSTATUS_PLAYING ) == 0 )
                break;
        }
    }

    if( i != m_dwNumBuffers )
        return m_apDSBuffer[ i ];
    else
        return m_apDSBuffer[ rand() % m_dwNumBuffers ];
}


//-----------------------------------------------------------------------------
// Name: CSound::GetBuffer()
// Desc:
//-----------------------------------------------------------------------------
LPDIRECTSOUNDBUFFER CSound::GetBuffer( DWORD dwIndex )
{
    if( m_apDSBuffer == NULL )
        return NULL;
    if( dwIndex >= m_dwNumBuffers )
        return NULL;

    return m_apDSBuffer[dwIndex];
}


//-----------------------------------------------------------------------------
// Name: CSound::Get3DBufferInterface()
// Desc:
//-----------------------------------------------------------------------------
HRESULT CSound::Get3DBufferInterface( DWORD dwIndex, LPDIRECTSOUND3DBUFFER* ppDS3DBuffer )
{
    if( m_apDSBuffer == NULL )
        return CO_E_NOTINITIALIZED;
    if( dwIndex >= m_dwNumBuffers )
        return E_INVALIDARG;

    *ppDS3DBuffer = NULL;

    return m_apDSBuffer[dwIndex]->QueryInterface( IID_IDirectSound3DBuffer,
                                                  ( VOID** )ppDS3DBuffer );
}


//-----------------------------------------------------------------------------
// Name: CSound::Play()
// Desc: Plays the sound using voice management flags.  Pass in DSBPLAY_LOOPING
//       in the dwFlags to loop the sound
//-----------------------------------------------------------------------------
HRESULT CSound::Play( DWORD dwPriority, DWORD dwFlags, LONG lVolume, LONG lFrequency, LONG lPan )
{
    HRESULT hr;
    BOOL bRestored;

    if( m_apDSBuffer == NULL )
        return CO_E_NOTINITIALIZED;

    LPDIRECTSOUNDBUFFER pDSB = GetFreeBuffer();

    if( pDSB == NULL )
        return DXUT_ERR( L"GetFreeBuffer", E_FAIL );

    // Restore the buffer if it was lost
    if( FAILED( hr = RestoreBuffer( pDSB, &bRestored ) ) )
        return DXUT_ERR( L"RestoreBuffer", hr );

    if( bRestored )
    {
        // The buffer was restored, so we need to fill it with new data
        if( FAILED( hr = FillBufferWithSound( pDSB, FALSE ) ) )
            return DXUT_ERR( L"FillBufferWithSound", hr );
    }

    if( m_dwCreationFlags & DSBCAPS_CTRLVOLUME )
    {
        pDSB->SetVolume( lVolume );
    }

    if( lFrequency != -1 &&
        ( m_dwCreationFlags & DSBCAPS_CTRLFREQUENCY ) )
    {
        pDSB->SetFrequency( lFrequency );
    }

    if( m_dwCreationFlags & DSBCAPS_CTRLPAN )
    {
        pDSB->SetPan( lPan );
    }

    return pDSB->Play( 0, dwPriority, dwFlags );
}


//-----------------------------------------------------------------------------
// Name: CSound::Play3D()
// Desc: Plays the sound using voice management flags.  Pass in DSBPLAY_LOOPING
//       in the dwFlags to loop the sound
//-----------------------------------------------------------------------------
HRESULT CSound::Play3D( LPDS3DBUFFER p3DBuffer, DWORD dwPriority, DWORD dwFlags, LONG lFrequency )
{
    HRESULT hr;
    BOOL bRestored;
    DWORD dwBaseFrequency;

    if( m_apDSBuffer == NULL )
        return CO_E_NOTINITIALIZED;

    LPDIRECTSOUNDBUFFER pDSB = GetFreeBuffer();
    if( pDSB == NULL )
        return DXUT_ERR( L"GetFreeBuffer", E_FAIL );

    // Restore the buffer if it was lost
    if( FAILED( hr = RestoreBuffer( pDSB, &bRestored ) ) )
        return DXUT_ERR( L"RestoreBuffer", hr );

    if( bRestored )
    {
        // The buffer was restored, so we need to fill it with new data
        if( FAILED( hr = FillBufferWithSound( pDSB, FALSE ) ) )
            return DXUT_ERR( L"FillBufferWithSound", hr );
    }

    if( m_dwCreationFlags & DSBCAPS_CTRLFREQUENCY )
    {
        pDSB->GetFrequency( &dwBaseFrequency );
        pDSB->SetFrequency( dwBaseFrequency + lFrequency );
    }

    // QI for the 3D buffer
    LPDIRECTSOUND3DBUFFER pDS3DBuffer;
    hr = pDSB->QueryInterface( IID_IDirectSound3DBuffer, ( VOID** )&pDS3DBuffer );
    if( SUCCEEDED( hr ) )
    {
        hr = pDS3DBuffer->SetAllParameters( p3DBuffer, DS3D_IMMEDIATE );
        if( SUCCEEDED( hr ) )
        {
            hr = pDSB->Play( 0, dwPriority, dwFlags );
        }

        pDS3DBuffer->Release();
    }

    return hr;
}


//-----------------------------------------------------------------------------
// Name: CSound::Stop()
// Desc: Stops the sound from playing
//-----------------------------------------------------------------------------
HRESULT CSound::Stop()
{
    if( m_apDSBuffer == NULL )
        return CO_E_NOTINITIALIZED;

    HRESULT hr = 0;

    for( DWORD i = 0; i < m_dwNumBuffers; i++ )
        hr |= m_apDSBuffer[i]->Stop();

    return hr;
}


//-----------------------------------------------------------------------------
// Name: CSound::Reset()
// Desc: Reset all of the sound buffers
//-----------------------------------------------------------------------------
HRESULT CSound::Reset()
{
    if( m_apDSBuffer == NULL )
        return CO_E_NOTINITIALIZED;

    HRESULT hr = 0;

    for( DWORD i = 0; i < m_dwNumBuffers; i++ )
        hr |= m_apDSBuffer[i]->SetCurrentPosition( 0 );

    return hr;
}


//-----------------------------------------------------------------------------
// Name: CSound::IsSoundPlaying()
// Desc: Checks to see if a buffer is playing and returns TRUE if it is.
//-----------------------------------------------------------------------------
BOOL CSound::IsSoundPlaying()
{
    BOOL bIsPlaying = FALSE;

    if( m_apDSBuffer == NULL )
        return FALSE;

    for( DWORD i = 0; i < m_dwNumBuffers; i++ )
    {
        if( m_apDSBuffer[i] )
        {
            DWORD dwStatus = 0;
            m_apDSBuffer[i]->GetStatus( &dwStatus );
            bIsPlaying |= ( ( dwStatus & DSBSTATUS_PLAYING ) != 0 );
        }
    }

    return bIsPlaying;
}


//-----------------------------------------------------------------------------
// Name: CStreamingSound::CStreamingSound()
// Desc: Setups up a buffer so data can be streamed from the wave file into
//       a buffer.  This is very useful for large wav files that would take a
//       while to load.  The buffer is initially filled with data, then
//       as sound is played the notification events are signaled and more data
//       is written into the buffer by calling HandleWaveStreamNotification()
//-----------------------------------------------------------------------------
CStreamingSound::CStreamingSound( LPDIRECTSOUNDBUFFER pDSBuffer, DWORD dwDSBufferSize,
                                  CWaveFile* pWaveFile, DWORD dwNotifySize ) : CSound( &pDSBuffer, dwDSBufferSize, 1,
                                                                                       pWaveFile, 0 )
{
    m_dwLastPlayPos = 0;
    m_dwPlayProgress = 0;
    m_dwNotifySize = dwNotifySize;
    m_dwNextWriteOffset = 0;
    m_bFillNextNotificationWithSilence = FALSE;
}


//-----------------------------------------------------------------------------
// Name: CStreamingSound::~CStreamingSound()
// Desc: Destroys the class
//-----------------------------------------------------------------------------
CStreamingSound::~CStreamingSound()
{
}


//-----------------------------------------------------------------------------
// Name: CStreamingSound::HandleWaveStreamNotification()
// Desc: Handle the notification that tells us to put more wav data in the
//       circular buffer
//-----------------------------------------------------------------------------
HRESULT CStreamingSound::HandleWaveStreamNotification( BOOL bLoopedPlay )
{
    HRESULT hr;
    DWORD dwCurrentPlayPos;
    DWORD dwPlayDelta;
    DWORD dwBytesWrittenToBuffer;
    VOID* pDSLockedBuffer = NULL;
    VOID* pDSLockedBuffer2 = NULL;
    DWORD dwDSLockedBufferSize;
    DWORD dwDSLockedBufferSize2;

    if( m_apDSBuffer == NULL || m_pWaveFile == NULL )
        return CO_E_NOTINITIALIZED;

    // Restore the buffer if it was lost
    BOOL bRestored;
    if( FAILED( hr = RestoreBuffer( m_apDSBuffer[0], &bRestored ) ) )
        return DXUT_ERR( L"RestoreBuffer", hr );

    if( bRestored )
    {
        // The buffer was restored, so we need to fill it with new data
        if( FAILED( hr = FillBufferWithSound( m_apDSBuffer[0], FALSE ) ) )
            return DXUT_ERR( L"FillBufferWithSound", hr );
        return S_OK;
    }

    // Lock the DirectSound buffer
    if( FAILED( hr = m_apDSBuffer[0]->Lock( m_dwNextWriteOffset, m_dwNotifySize,
                                            &pDSLockedBuffer, &dwDSLockedBufferSize,
                                            &pDSLockedBuffer2, &dwDSLockedBufferSize2, 0L ) ) )
        return DXUT_ERR( L"Lock", hr );

    // m_dwDSBufferSize and m_dwNextWriteOffset are both multiples of m_dwNotifySize,
    // it should the second buffer, so it should never be valid
    if( pDSLockedBuffer2 != NULL )
        return E_UNEXPECTED;

    if( !m_bFillNextNotificationWithSilence )
    {
        // Fill the DirectSound buffer with wav data
        if( FAILED( hr = m_pWaveFile->Read( ( BYTE* )pDSLockedBuffer,
                                            dwDSLockedBufferSize,
                                            &dwBytesWrittenToBuffer ) ) )
            return DXUT_ERR( L"Read", hr );
    }
    else
    {
        // Fill the DirectSound buffer with silence
        FillMemory( pDSLockedBuffer, dwDSLockedBufferSize,
                    ( BYTE )( m_pWaveFile->m_pwfx->wBitsPerSample == 8 ? 128 : 0 ) );
        dwBytesWrittenToBuffer = dwDSLockedBufferSize;
    }

    // If the number of bytes written is less than the
    // amount we requested, we have a short file.
    if( dwBytesWrittenToBuffer < dwDSLockedBufferSize )
    {
        if( !bLoopedPlay )
        {
            // Fill in silence for the rest of the buffer.
            FillMemory( ( BYTE* )pDSLockedBuffer + dwBytesWrittenToBuffer,
                        dwDSLockedBufferSize - dwBytesWrittenToBuffer,
                        ( BYTE )( m_pWaveFile->m_pwfx->wBitsPerSample == 8 ? 128 : 0 ) );

            // Any future notifications should just fill the buffer with silence
            m_bFillNextNotificationWithSilence = TRUE;
        }
        else
        {
            // We are looping, so reset the file and fill the buffer with wav data
            DWORD dwReadSoFar = dwBytesWrittenToBuffer;    // From previous call above.
            while( dwReadSoFar < dwDSLockedBufferSize )
            {
                // This will keep reading in until the buffer is full (for very short files).
                if( FAILED( hr = m_pWaveFile->ResetFile() ) )
                    return DXUT_ERR( L"ResetFile", hr );

                if( FAILED( hr = m_pWaveFile->Read( ( BYTE* )pDSLockedBuffer + dwReadSoFar,
                                                    dwDSLockedBufferSize - dwReadSoFar,
                                                    &dwBytesWrittenToBuffer ) ) )
                    return DXUT_ERR( L"Read", hr );

                dwReadSoFar += dwBytesWrittenToBuffer;
            }
        }
    }

    // Unlock the DirectSound buffer
    m_apDSBuffer[0]->Unlock( pDSLockedBuffer, dwDSLockedBufferSize, NULL, 0 );

    // Figure out how much data has been played so far.  When we have played
    // past the end of the file, we will either need to start filling the
    // buffer with silence or starting reading from the beginning of the file,
    // depending if the user wants to loop the sound
    if( FAILED( hr = m_apDSBuffer[0]->GetCurrentPosition( &dwCurrentPlayPos, NULL ) ) )
        return DXUT_ERR( L"GetCurrentPosition", hr );

    // Check to see if the position counter looped
    if( dwCurrentPlayPos < m_dwLastPlayPos )
        dwPlayDelta = ( m_dwDSBufferSize - m_dwLastPlayPos ) + dwCurrentPlayPos;
    else
        dwPlayDelta = dwCurrentPlayPos - m_dwLastPlayPos;

    m_dwPlayProgress += dwPlayDelta;
    m_dwLastPlayPos = dwCurrentPlayPos;

    // If we are now filling the buffer with silence, then we have found the end so
    // check to see if the entire sound has played, if it has then stop the buffer.
    if( m_bFillNextNotificationWithSilence )
    {
        // We don't want to cut off the sound before it's done playing.
        if( m_dwPlayProgress >= m_pWaveFile->GetSize() )
        {
            m_apDSBuffer[0]->Stop();
        }
    }

    // Update where the buffer will lock (for next time)
    m_dwNextWriteOffset += dwDSLockedBufferSize;
    m_dwNextWriteOffset %= m_dwDSBufferSize; // Circular buffer

    return S_OK;
}


//-----------------------------------------------------------------------------
// Name: CStreamingSound::Reset()
// Desc: Resets the sound so it will begin playing at the beginning
//-----------------------------------------------------------------------------
HRESULT CStreamingSound::Reset()
{
    HRESULT hr;

    if( m_apDSBuffer[0] == NULL || m_pWaveFile == NULL )
        return CO_E_NOTINITIALIZED;

    m_dwLastPlayPos = 0;
    m_dwPlayProgress = 0;
    m_dwNextWriteOffset = 0;
    m_bFillNextNotificationWithSilence = FALSE;

    // Restore the buffer if it was lost
    BOOL bRestored;
    if( FAILED( hr = RestoreBuffer( m_apDSBuffer[0], &bRestored ) ) )
        return DXUT_ERR( L"RestoreBuffer", hr );

    if( bRestored )
    {
        // The buffer was restored, so we need to fill it with new data
        if( FAILED( hr = FillBufferWithSound( m_apDSBuffer[0], FALSE ) ) )
            return DXUT_ERR( L"FillBufferWithSound", hr );
    }

    m_pWaveFile->ResetFile();

    return m_apDSBuffer[0]->SetCurrentPosition( 0L );
}
