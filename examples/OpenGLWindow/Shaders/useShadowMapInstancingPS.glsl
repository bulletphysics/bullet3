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
  vec4 texel = fragment.color*texture(Diffuse,vert.texcoord);
	vec3 ct,cf;
	float intensity,at,af;
	
	intensity = 0.5+0.5*clamp( dot( normalize(normal),lightDir ), -1,1 );
	
	af = 1.0;
		
	ct = texel.rgb;
	at = texel.a;
		
	//float bias = 0.005f;
	
	

	float visibility = texture(shadowMap, vec3(ShadowCoord.xy,(ShadowCoord.z)/ShadowCoord.w));
	
	intensity = 0.7*intensity  + 0.3*intensity*visibility;
	
	cf = intensity*(vec3(1.0,1.0,1.0)-ambient)+ambient;
		
	color  = vec4(ct * cf, fragment.color.w);
}
