//-----------------------------------------------------------------------------
// File: DXUTsound.h
//
// Copyright (c) Microsoft Corp. All rights reserved.
//-----------------------------------------------------------------------------
#ifndef DXUTSOUND_H
#define DXUTSOUND_H

//-----------------------------------------------------------------------------
// Header Includes
//-----------------------------------------------------------------------------
#include <dsound.h>
#include <ks.h>

//-----------------------------------------------------------------------------
// Classes used by this header
//-----------------------------------------------------------------------------
class CSoundManager;
class CSound;
class CStreamingSound;
class CWaveFile;


//-----------------------------------------------------------------------------
// Typing macros 
//-----------------------------------------------------------------------------
#define DXUT_StopSound(s)         { if(s) s->Stop(); }
#define DXUT_PlaySound(s)         { if(s) s->Play( 0, 0 ); }
#define DXUT_PlaySoundLooping(s)  { if(s) s->Play( 0, DSBPLAY_LOOPING ); }


//-----------------------------------------------------------------------------
// Name: class CSoundManager
// Desc: 
//-----------------------------------------------------------------------------
class CSoundManager
{
protected:
    IDirectSound8* m_pDS;

public:
                            CSoundManager();
                            ~CSoundManager();

    HRESULT                 Initialize( HWND hWnd, DWORD dwCoopLevel );
    inline  LPDIRECTSOUND8  GetDirectSound()
    {
        return m_pDS;
    }
    HRESULT                 SetPrimaryBufferFormat( DWORD dwPrimaryChannels, DWORD dwPrimaryFreq,
                                                    DWORD dwPrimaryBitRate );
    HRESULT                 Get3DListenerInterface( LPDIRECTSOUND3DLISTENER* ppDSListener );

    HRESULT                 Create( CSound** ppSound, LPWSTR strWaveFileName, DWORD dwCreationFlags = 0,
                                    GUID guid3DAlgorithm = GUID_NULL, DWORD dwNumBuffers = 1 );
    HRESULT                 CreateFromMemory( CSound** ppSound, BYTE* pbData, ULONG ulDataSize, LPWAVEFORMATEX pwfx,
                                              DWORD dwCreationFlags = 0, GUID guid3DAlgorithm = GUID_NULL,
                                              DWORD dwNumBuffers = 1 );
    HRESULT                 CreateStreaming( CStreamingSound** ppStreamingSound, LPWSTR strWaveFileName,
                                             DWORD dwCreationFlags, GUID guid3DAlgorithm, DWORD dwNotifyCount,
                                             DWORD dwNotifySize, HANDLE hNotifyEvent );
};


//-----------------------------------------------------------------------------
// Name: class CSound
// Desc: Encapsulates functionality of a DirectSound buffer.
//-----------------------------------------------------------------------------
class CSound
{
protected:
    LPDIRECTSOUNDBUFFER* m_apDSBuffer;
    DWORD m_dwDSBufferSize;
    CWaveFile* m_pWaveFile;
    DWORD m_dwNumBuffers;
    DWORD m_dwCreationFlags;

    HRESULT             RestoreBuffer( LPDIRECTSOUNDBUFFER pDSB, BOOL* pbWasRestored );

public:
                        CSound( LPDIRECTSOUNDBUFFER* apDSBuffer, DWORD dwDSBufferSize, DWORD dwNumBuffers,
                                CWaveFile* pWaveFile, DWORD dwCreationFlags );
    virtual             ~CSound();

    HRESULT             Get3DBufferInterface( DWORD dwIndex, LPDIRECTSOUND3DBUFFER* ppDS3DBuffer );
    HRESULT             FillBufferWithSound( LPDIRECTSOUNDBUFFER pDSB, BOOL bRepeatWavIfBufferLarger );
    LPDIRECTSOUNDBUFFER GetFreeBuffer();
    LPDIRECTSOUNDBUFFER GetBuffer( DWORD dwIndex );

    HRESULT             Play( DWORD dwPriority = 0, DWORD dwFlags = 0, LONG lVolume = 0, LONG lFrequency = -1,
                              LONG lPan = 0 );
    HRESULT             Play3D( LPDS3DBUFFER p3DBuffer, DWORD dwPriority = 0, DWORD dwFlags = 0, LONG lFrequency = 0 );
    HRESULT             Stop();
    HRESULT             Reset();
    BOOL                IsSoundPlaying();
};


//-----------------------------------------------------------------------------
// Name: class CStreamingSound
// Desc: Encapsulates functionality to play a wave file with DirectSound.  
//       The Create() method loads a chunk of wave file into the buffer, 
//       and as sound plays more is written to the buffer by calling 
//       HandleWaveStreamNotification() whenever hNotifyEvent is signaled.
//-----------------------------------------------------------------------------
class CStreamingSound : public CSound
{
protected:
    DWORD m_dwLastPlayPos;
    DWORD m_dwPlayProgress;
    DWORD m_dwNotifySize;
    DWORD m_dwNextWriteOffset;
    BOOL m_bFillNextNotificationWithSilence;

public:
            CStreamingSound( LPDIRECTSOUNDBUFFER pDSBuffer, DWORD dwDSBufferSize, CWaveFile* pWaveFile,
                             DWORD dwNotifySize );
            ~CStreamingSound();

    HRESULT HandleWaveStreamNotification( BOOL bLoopedPlay );
    HRESULT Reset();
};

#endif // DXUTSOUND_H
