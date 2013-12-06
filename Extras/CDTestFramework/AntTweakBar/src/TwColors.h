//  ---------------------------------------------------------------------------
//
//  @file       TwColors.h
//  @brief      Color conversions
//  @author     Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//              For conditions of distribution and use, see License.txt
//
//  note:       Private header
//
//  ---------------------------------------------------------------------------


#if !defined ANT_TW_COLORS_INCLUDED
#define ANT_TW_COLORS_INCLUDED


//  ---------------------------------------------------------------------------


typedef unsigned int color32;


const color32 COLOR32_BLACK     = 0xff000000;   // Black 
const color32 COLOR32_WHITE     = 0xffffffff;   // White 
const color32 COLOR32_ZERO      = 0x00000000;   // Zero 
const color32 COLOR32_RED       = 0xffff0000;   // Red 
const color32 COLOR32_GREEN     = 0xff00ff00;   // Green 
const color32 COLOR32_BLUE      = 0xff0000ff;   // Blue 
   

template <typename _T> inline const _T& TClamp(const _T& _X, const _T& _Limit1, const _T& _Limit2)
{
    if( _Limit1<_Limit2 )
        return (_X<=_Limit1) ? _Limit1 : ( (_X>=_Limit2) ? _Limit2 : _X );
    else
        return (_X<=_Limit2) ? _Limit2 : ( (_X>=_Limit1) ? _Limit1 : _X );
}

inline color32 Color32FromARGBi(int _A, int _R, int _G, int _B)
{
    return (((color32)TClamp(_A, 0, 255))<<24) | (((color32)TClamp(_R, 0, 255))<<16) | (((color32)TClamp(_G, 0, 255))<<8) | ((color32)TClamp(_B, 0, 255));
}

inline color32 Color32FromARGBf(float _A, float _R, float _G, float _B)
{
    return (((color32)TClamp(_A*256.0f, 0.0f, 255.0f))<<24) | (((color32)TClamp(_R*256.0f, 0.0f, 255.0f))<<16) | (((color32)TClamp(_G*256.0f, 0.0f, 255.0f))<<8) | ((color32)TClamp(_B*256.0f, 0.0f, 255.0f));
}

inline void Color32ToARGBi(color32 _Color, int *_A, int *_R, int *_G, int *_B)
{
    if(_A) *_A = (_Color>>24)&0xff;
    if(_R) *_R = (_Color>>16)&0xff;
    if(_G) *_G = (_Color>>8)&0xff;
    if(_B) *_B = _Color&0xff;
}

inline void Color32ToARGBf(color32 _Color, float *_A, float *_R, float *_G, float *_B)
{
    if(_A) *_A = (1.0f/255.0f)*float((_Color>>24)&0xff);
    if(_R) *_R = (1.0f/255.0f)*float((_Color>>16)&0xff);
    if(_G) *_G = (1.0f/255.0f)*float((_Color>>8)&0xff);
    if(_B) *_B = (1.0f/255.0f)*float(_Color&0xff);
}

void ColorRGBToHLSf(float _R, float _G, float _B, float *_Hue, float *_Light, float *_Saturation);

void ColorRGBToHLSi(int _R, int _G, int _B, int *_Hue, int *_Light, int *_Saturation);

void ColorHLSToRGBf(float _Hue, float _Light, float _Saturation, float *_R, float *_G, float *_B);

void ColorHLSToRGBi(int _Hue, int _Light, int _Saturation, int *_R, int *_G, int *_B);

color32 ColorBlend(color32 _Color1, color32 _Color2, float _S);


//  ---------------------------------------------------------------------------


#endif // !defined ANT_TW_COLORS_INCLUDED
