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
	
	intensity = clamp( dot( normalize(normal),lightDir ), 0,1 );
	
	cf = ambient;
	af = 1.0;
		
	ct = texel.rgb;
	at = texel.a;
		
	//float bias = 0.005f;
	
	float bias = 0.0001*tan(acos(intensity));
	bias = clamp(bias, 0,0.01);


	float visibility = texture(shadowMap, vec3(ShadowCoord.xy,(ShadowCoord.z-bias)/ShadowCoord.w));
	
	intensity*=2;
	if (intensity>1)
		intensity=1.f;
	
	visibility *= intensity;
	
	if (visibility<0.6)
		visibility=0.6f;
		
	color  = vec4(ct * visibility, fragment.color.w);
}
