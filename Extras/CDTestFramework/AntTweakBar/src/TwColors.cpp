//  ---------------------------------------------------------------------------
//
//  @file       TwColors.cpp
//  @author     Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//              For conditions of distribution and use, see License.txt
//
//  ---------------------------------------------------------------------------


#include "TwPrecomp.h"
#include "TwColors.h"


void ColorRGBToHLSf(float _R, float _G, float _B, float *_Hue, float *_Light, float *_Saturation)
{
    // Compute HLS from RGB. The r,g,b triplet is between [0,1], 
    // hue is between [0,360], light and saturation are [0,1].

    float rnorm, gnorm, bnorm, minval, maxval, msum, mdiff, r, g, b;
    r = g = b = 0;
    if(_R>0) r = _R; if(r>1) r = 1;
    if(_G>0) g = _G; if(g>1) g = 1;
    if(_B>0) b = _B; if(b>1) b = 1;

    minval = r;
    if(g<minval) minval = g;
    if(b<minval) minval = b;
    maxval = r;
    if(g>maxval) maxval = g;
    if(b>maxval) maxval = b;

    rnorm = gnorm = bnorm = 0;
    mdiff = maxval - minval;
    msum  = maxval + minval;
    float l = 0.5f * msum;
    if(_Light) 
        *_Light = l;
    if(maxval!=minval) 
    {
        rnorm = (maxval - r)/mdiff;
        gnorm = (maxval - g)/mdiff;
        bnorm = (maxval - b)/mdiff;
    } 
    else 
    {
        if(_Saturation)
            *_Saturation = 0;
        if(_Hue)
            *_Hue = 0;
        return;
    }

    if(_Saturation)
    {
        if(l<0.5f)
            *_Saturation = mdiff/msum;
        else
            *_Saturation = mdiff/(2.0f - msum);
    }

    if(_Hue)
    {
        if(r==maxval)
            *_Hue = 60.0f * (6.0f + bnorm - gnorm);
        else if(g==maxval)
            *_Hue = 60.0f * (2.0f + rnorm - bnorm);
        else
            *_Hue = 60.0f * (4.0f + gnorm - rnorm);

        if(*_Hue>360.0f)
            *_Hue -= 360.0f;
    }
}


void ColorRGBToHLSi(int _R, int _G, int _B, int *_Hue, int *_Light, int *_Saturation)
{
    float h, l, s;
    ColorRGBToHLSf((1.0f/255.0f)*float(_R), (1.0f/255.0f)*float(_G), (1.0f/255.0f)*float(_B), &h, &l, &s);
    if(_Hue)        *_Hue       = (int)TClamp(h*(256.0f/360.0f), 0.0f, 255.0f);
    if(_Light)      *_Light     = (int)TClamp(l*256.0f, 0.0f, 255.0f);
    if(_Saturation) *_Saturation= (int)TClamp(s*256.0f, 0.0f, 255.0f);
}


void ColorHLSToRGBf(float _Hue, float _Light, float _Saturation, float *_R, float *_G, float *_B)
{
    // Compute RGB from HLS. The light and saturation are between [0,1]
    // and hue is between [0,360]. The returned r,g,b triplet is between [0,1].

    // a local auxiliary function
    struct CLocal
    {
        static float HLSToRGB(float _Rn1, float _Rn2, float _Huei)
        {
            float hue = _Huei;
            if(hue>360) hue = hue - 360;
            if(hue<0)   hue = hue + 360;
            if(hue<60 ) return _Rn1 + (_Rn2-_Rn1)*hue/60;
            if(hue<180) return _Rn2;
            if(hue<240) return _Rn1 + (_Rn2-_Rn1)*(240-hue)/60;
            return _Rn1;
        }
    };

    float rh, rl, rs, rm1, rm2;
    rh = rl = rs = 0;
    if(_Hue>0)        rh = _Hue;        if(rh>360) rh = 360;
    if(_Light>0)      rl = _Light;      if(rl>1)   rl = 1;
    if(_Saturation>0) rs = _Saturation; if(rs>1)   rs = 1;

    if(rl<=0.5f)
        rm2 = rl*(1.0f + rs);
    else
        rm2 = rl + rs - rl*rs;
    rm1 = 2.0f*rl - rm2;

    if(!rs) 
    { 
        if(_R) *_R = rl; 
        if(_G) *_G = rl; 
        if(_B) *_B = rl; 
    }
    else
    {
        if(_R) *_R = CLocal::HLSToRGB(rm1, rm2, rh+120);
        if(_G) *_G = CLocal::HLSToRGB(rm1, rm2, rh);
        if(_B) *_B = CLocal::HLSToRGB(rm1, rm2, rh-120);
    }
}


void ColorHLSToRGBi(int _Hue, int _Light, int _Saturation, int *_R, int *_G, int *_B)
{
    float r, g, b;
    ColorHLSToRGBf((360.0f/255.0f)*float(_Hue), (1.0f/255.0f)*float(_Light), (1.0f/255.0f)*float(_Saturation), &r, &g, &b);
    if(_R) *_R = (int)TClamp(r*256.0f, 0.0f, 255.0f);
    if(_G) *_G = (int)TClamp(g*256.0f, 0.0f, 255.0f);
    if(_B) *_B = (int)TClamp(b*256.0f, 0.0f, 255.0f);
}


color32 ColorBlend(color32 _Color1, color32 _Color2, float _S)
{
    float a1, r1, g1, b1, a2, r2, g2, b2;
    Color32ToARGBf(_Color1, &a1, &r1, &g1, &b1);
    Color32ToARGBf(_Color2, &a2, &r2, &g2, &b2);
    float t = 1.0f-_S;
    return Color32FromARGBf(t*a1+_S*a2, t*r1+_S*r2, t*g1+_S*g2, t*b1+_S*b2);
}


