#extension GL_ARB_shader_viewport_layer_array: enable
#extension GL_NV_viewport_array : enable
precision highp float;

layout(invocations = cameraArraySize) in;
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

uniform mat4 ModelViewMatrix[cameraArraySize];
uniform mat4 ProjectionMatrix[cameraArraySize];
uniform vec3 lightDirIn;


in Fragment
{
     vec4 color;
} fragment_[];
in Vert
{
	vec4 pos;
	vec2 texcoord;
} vert_[];
in vec3 normal_[],ambient_[];


out Fragment
{
     vec4 color;
} fragment;
out Vert
{
	vec2 texcoord;
} vert;
out vec3 lightDir,normal,ambient;



void main(void)
{
	mat4 mvp = ProjectionMatrix[gl_InvocationID] * ModelViewMatrix[gl_InvocationID];
	
    for (int i = 0; i < gl_in.length(); i++)
    {
        fragment.color = fragment_[i].color;
		vert.texcoord = vert_[i].texcoord;
		lightDir = lightDirIn;
		normal = normal_[i];
		ambient = ambient_[i];
		
		gl_Position =  mvp * vert_[i].pos;
        gl_ViewportIndex = gl_InvocationID;
        EmitVertex();
    }
    EndPrimitive();
};
