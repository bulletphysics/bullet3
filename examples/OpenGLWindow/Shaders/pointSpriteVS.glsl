#version 330
precision highp float;



layout (location = 0) in vec4 position;
layout (location = 1) in vec4 instance_position;
layout (location = 3) in vec2 uvcoords;
layout (location = 4) in vec3 vertexnormal;
layout (location = 5) in vec4 instance_color;
layout (location = 6) in vec4 instance_scale_obUid;


uniform float screenWidth = 700.f;
uniform mat4 ModelViewMatrix;
uniform mat4 ProjectionMatrix;

out Fragment
{
     vec4 color;
} fragment;



//
// vector rotation via quaternion
//

out vec3 ambient;

void main(void)
{
	ambient = vec3(0.3,.3,0.3);
		
		
	vec4 axis = vec4(1,1,1,0);
	vec4 vertexPos = ProjectionMatrix * ModelViewMatrix *(instance_position);
	vec3 posEye = vec3(ModelViewMatrix * vec4(instance_position.xyz, 1.0));
   float dist = length(posEye);
	float pointRadius = 1.f;
    gl_PointSize = instance_scale_obUid.x * pointRadius * (screenWidth / dist);

	gl_Position = vertexPos;
	
	fragment.color = instance_color;
}
