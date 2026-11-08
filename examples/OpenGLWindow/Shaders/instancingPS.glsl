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
uniform vec3 lightColor;
uniform vec3 lightCoefficients;
uniform bool useLightParameters;
in vec3 lightDir,normal,ambient;
in vec3 worldPosition;
uniform vec3 lightPosition;
uniform vec3 cameraPosition;
uniform float materialShininess;
uniform vec3 materialSpecularColor;
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
	
	intensity = 0.5+0.5*clamp( dot( normalize(normal),lightDir ), -1,1 );
	cf = intensity*(vec3(1.0,1.0,1.0)-ambient)+ambient;
	if (useLightParameters)
	{
		vec3 normalDir = normalize(normal);
		float diffuse = max(dot(normalDir, normalize(lightDir)), 0.0);
		float specular = 0.0;
		if (diffuse > 0.0)
		{
			vec3 surfaceToLight = normalize(lightPosition - worldPosition);
			vec3 surfaceToCamera = normalize(cameraPosition - worldPosition);
			specular = pow(max(dot(surfaceToCamera, reflect(-surfaceToLight, normalDir)), 0.0), materialShininess);
		}
		cf = vec3(lightCoefficients.x) + lightColor *
			(lightCoefficients.y * diffuse + lightCoefficients.z * specular * materialSpecularColor);
	}
	af = 1.0;
		
	ct = texel.rgb;
	at = texel.a;
		
	color  = vec4(ct * cf, at * af);	
}
