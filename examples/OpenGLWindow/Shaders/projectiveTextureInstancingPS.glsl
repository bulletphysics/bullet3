#version 330 core
//precision highp float;

in Fragment
{
     vec4 color;
} fragment;

uniform sampler2D Diffuse;
uniform mat4 ViewMatrixInverse;
uniform mat4 TextureMVP;

in vec3 lightPos,cameraPosition, normal,ambient;
in vec4 vertexPos;
in float materialShininess;
in vec3 lightSpecularIntensity;
in vec3 materialSpecularColor;

out vec4 color;


void main(void)
{
    vec4 projcoords = TextureMVP * vertexPos;
    vec2 texturecoords = projcoords.xy/projcoords.w;
	vec4 texel = fragment.color*texture(Diffuse,texturecoords);
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
    

	float visibility = 1.0;

	intensity = 0.7*intensity  + 0.3*intensity*visibility;
	
	cf = intensity*(vec3(1.0,1.0,1.0)-ambient)+ambient+specularReflection*visibility;
	color  = vec4(ct * cf, fragment.color.w);
}
