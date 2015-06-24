#version 150   

uniform mat4 ModelViewMatrix;
uniform mat4 ProjectionMatrix;
uniform vec4 colour;

in vec4 position;


out vec4 colourV;

void main (void)
{
    colourV = colour;
		gl_Position = ProjectionMatrix * ModelViewMatrix * position;
		
}
