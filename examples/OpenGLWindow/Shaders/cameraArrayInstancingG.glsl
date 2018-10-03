#version 400
#extension GL_ARB_shader_viewport_layer_array: enable
#extension GL_NV_viewport_array : enable

layout(invocations = 16) in;
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;
uniform mat4 MVP;
in Fragment
{
     vec4 color;
} fragment_[];
in Vert
{
	vec2 texcoord;
} vert_[];
in vec3 lightPos_[],cameraPosition_[], normal_[],ambient_[];
in vec4 ShadowCoord_[];
in vec4 vertexPos_[];
in float materialShininess_[];
in vec3 lightSpecularIntensity_[];
in vec3 materialSpecularColor_[];


out vec4 ShadowCoord;
out Fragment
{
     vec4 color;
} fragment;
out Vert
{
	vec2 texcoord;
} vert;

out vec3 lightPos,normal,ambient;
out vec4 vertexPos;
out vec3 cameraPosition;
out float materialShininess;
out vec3 lightSpecularIntensity;
out vec3 materialSpecularColor;


void main(void)
{
    for (int i = 0; i < gl_in.length(); i++)
    {
        gl_Position = MVP[gl_InvocationID] * vertexPos_[i];
		 ShadowCoord = ShadowCoord_[i];"
		 fragment.color = fragment_[i].color;
		 vert.texcoord = vert_[i].texcoord;
		 lightPos = lightPos_[i];
		 normal = normal_[i];
		 ambient = ambient_[i];
		 vertexPos = vertexPos_[i];
		 cameraPosition = cameraPosition_[i];
		 materialShininess = materialShininess_[i];
		 lightSpecularIntensity = lightSpecularIntensity_[i];
		 materialSpecularColor = materialSpecularColor_[i];
        gl_ViewportIndex = gl_InvocationID;
        EmitVertex();
    }
    EndPrimitive();
};
