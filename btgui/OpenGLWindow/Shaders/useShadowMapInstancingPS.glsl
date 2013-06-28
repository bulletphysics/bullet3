#version 330 core
//precision highp float;

in Fragment
{
     vec4 color;
} fragment;

in Vert
{
	vec2 texcoord;
} vert;

uniform sampler2D Diffuse;
uniform sampler2DShadow shadowMap;

in vec3 lightDir,normal,ambient;
in vec4 ShadowCoord;

out vec4 color;



void main(void)
{
    vec4 texel = fragment.color*texture(Diffuse,vert.texcoord);//fragment.color;
	vec3 ct,cf;
	float intensity,at,af;
	intensity = max(dot(lightDir,normalize(normal)),0);
	cf = intensity*vec3(1.0,1.0,1.0)+ambient;
	af = 1.0;
		
	ct = texel.rgb;
	at = texel.a;
		
	float bias = 0.005f;
	float visibility = texture(shadowMap, vec3(ShadowCoord.xy,(ShadowCoord.z-bias)/ShadowCoord.w));
	
	color  = vec4(ct * cf * visibility, at * af);	
}
