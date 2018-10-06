#version 330
precision highp float;
in vec4 scale_obuid;
out vec4 color;



void main(void)
{
	highp int obuid = int(scale_obuid.w);
	float r = 1*1.f/255.f;//((obuid>>0 )&0xff)*(1./255.f);
	float g = ((obuid>>8 )&0xff)*(1./255.f);
	float b = ((obuid>>16)&0xff)*(1./255.f);
	float a = ((obuid>>24)&0xff)*(1./255.f);
	color  = vec4(r,g,b,a);
}
