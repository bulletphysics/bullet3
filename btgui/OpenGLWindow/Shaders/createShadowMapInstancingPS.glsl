#version 330
precision highp float;


layout(location = 0) out float fragmentdepth;

void main(void)
{
	fragmentdepth = gl_FragCoord.z;
}
