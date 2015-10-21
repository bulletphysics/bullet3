#version 330
precision highp float;

in Fragment
{
     vec4 color;
} fragment;

in Vert
{
	vec2 texcoord;
} vert;

uniform sampler2D Diffuse;
in vec3 lightDir,normal,ambient;
out vec4 color;

void main_textured(void)
{
   color  = vec4(0.1,0.2,0.3,0.3);
}

void main(void)
{
    vec4 texel = fragment.color*texture(Diffuse,vert.texcoord);//fragment.color;
	vec3 ct,cf;
	float intensity,at,af;
	intensity = max(dot(lightDir,normalize(normal)),0);
	cf = intensity*(vec3(1.0,1.0,1.0)-ambient)+ambient;
	af = 1.0;
		
	ct = texel.rgb;
	at = texel.a;
		
	color  = vec4(ct * cf, at * af);	
}
