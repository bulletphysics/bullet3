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
	
	intensity = clamp( dot( normalize(normal),lightDir ), 0,1 );
	
	
	af = 1.0;
		
	ct = texel.rgb;
	at = texel.a;
		
	//float bias = 0.005f;
	
	float bias = 0.0001*tan(acos(intensity));
	bias = clamp(bias, 0,0.01);


	float visibility = texture(shadowMap, vec3(ShadowCoord.xy,(ShadowCoord.z-bias)/ShadowCoord.w));
	
	intensity = 0.7*intensity  + 0.3*intensity*visibility;
	
	cf = intensity*(vec3(1.0,1.0,1.0)-ambient)+ambient;
		
	color  = vec4(ct * cf, fragment.color.w);
}
