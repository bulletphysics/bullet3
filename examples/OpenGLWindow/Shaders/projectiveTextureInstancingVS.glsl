#version 330 
precision highp float;


layout (location = 0) in vec4 position;
layout (location = 1) in vec4 instance_position;
layout (location = 2) in vec4 instance_quaternion;
layout (location = 3) in vec2 uvcoords;
layout (location = 4) in vec3 vertexnormal;
layout (location = 5) in vec4 instance_color;
layout (location = 6) in vec4 instance_scale_obUid;


uniform mat4 TextureMVP;
uniform mat4 MVP;
uniform vec3 lightPosIn;
uniform vec3 cameraPositionIn;
uniform mat4 ViewMatrixInverse;
uniform float materialShininessIn;
uniform vec3 lightSpecularIntensityIn;
uniform vec3 materialSpecularColorIn;

out vec4 ShadowCoord;

out Fragment
{
     vec4 color;
} fragment;

out Vert
{
	vec2 texcoord;
} vert;


vec4 quatMul ( in vec4 q1, in vec4 q2 )
{
    vec3  im = q1.w * q2.xyz + q1.xyz * q2.w + cross ( q1.xyz, q2.xyz );
    vec4  dt = q1 * q2;
    float re = dot ( dt, vec4 ( -1.0, -1.0, -1.0, 1.0 ) );
    return vec4 ( im, re );
}

vec4 quatFromAxisAngle(vec4 axis, in float angle)
{
    float cah = cos(angle*0.5);
    float sah = sin(angle*0.5);
	float d = inversesqrt(dot(axis,axis));
	vec4 q = vec4(axis.x*sah*d,axis.y*sah*d,axis.z*sah*d,cah);
	return q;
}
//
// vector rotation via quaternion
//
vec4 quatRotate3 ( in vec3 p, in vec4 q )
{
    vec4 temp = quatMul ( q, vec4 ( p, 0.0 ) );
    return quatMul ( temp, vec4 ( -q.x, -q.y, -q.z, q.w ) );
}
vec4 quatRotate ( in vec4 p, in vec4 q )
{
    vec4 temp = quatMul ( q, p );
    return quatMul ( temp, vec4 ( -q.x, -q.y, -q.z, q.w ) );
}

out vec3 lightPos,normal,ambient;
out vec4 vertexPos;
out vec3 cameraPosition;
out float materialShininess;
out vec3 lightSpecularIntensity;
out vec3 materialSpecularColor;


void main(void)
{
	vec4 q = instance_quaternion;
	ambient = vec3(0.5,.5,0.5);
			
	vec4 worldNormal = (quatRotate3( vertexnormal,q));
	
	normal = worldNormal.xyz;

	lightPos = lightPosIn;
	cameraPosition = cameraPositionIn;
	materialShininess = materialShininessIn;
	lightSpecularIntensity = lightSpecularIntensityIn;
	materialSpecularColor = materialSpecularColorIn;
	
	vec4 localcoord = quatRotate3( position.xyz*instance_scale_obUid.xyz,q);
	vertexPos = vec4((instance_position+localcoord).xyz,1);
	
	vec4 vertexLoc = MVP* vec4((instance_position+localcoord).xyz,1);
	gl_Position = vertexLoc;

	fragment.color = instance_color;
}

