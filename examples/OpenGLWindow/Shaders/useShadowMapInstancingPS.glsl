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
uniform mat4 ViewMatrixInverse;

in vec3 lightPos,cameraPosition, normal,ambient;
in vec4 ShadowCoord;
in vec4 vertexPos;
in float materialShininess;
in float shadowmapIntensity;
in vec3 lightSpecularIntensity;
in vec3 materialSpecularColor;

out vec4 color;



void main(void)
{
	vec4 texel = fragment.color*texture(Diffuse,vert.texcoord);
	vec3 ct,cf;
	float intensity,at,af;
	if (fragment.color.w==0)
		discard;
	vec3 lightDir = normalize(lightPos);
	
	vec3 normalDir = normalize(normal);
 
	intensity = 0.5+0.5*clamp( dot( normalDir,lightDir ), -1,1 );
	
	af = 1.0;
		
	ct = texel.rgb;
	at = texel.a;
		
	//float bias = 0.005f;
	
	vec3 specularReflection;
	
	if (dot(normalDir, lightDir) < 0.0) 
	{
		specularReflection = vec3(0.0, 0.0, 0.0);
	}
  else // light source on the right side
	{
		vec3 surfaceToLight = normalize(lightPos - vertexPos.xyz);
    vec3 surfaceToCamera = normalize(cameraPosition - vertexPos.xyz);
    
    
    float specularCoefficient = 0.0;
		specularCoefficient = pow(max(0.0, dot(surfaceToCamera, reflect(-surfaceToLight, normalDir))), materialShininess);
    specularReflection = specularCoefficient * materialSpecularColor * lightSpecularIntensity;
  
	}
    

	float visibility = texture(shadowMap, vec3(ShadowCoord.xy,(ShadowCoord.z)/ShadowCoord.w));
	if (intensity<0.5)
		visibility = 0;

	intensity = (1.0-shadowmapIntensity)*intensity  + shadowmapIntensity*intensity*visibility;
	
	cf = intensity*(vec3(1.0,1.0,1.0)-ambient)+ambient+specularReflection*visibility;
	color  = vec4(ct * cf, fragment.color.w);
}
