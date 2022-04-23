#version 150   

uniform mat4 ModelViewMatrix;
uniform mat4 ProjectionMatrix;
uniform vec4 colour;
in vec4 position;
in vec4 colourIn;
out vec4 colourV;

void main (void)
{
		colourV = (colour[3] == -1) ? colourIn : colour;
		gl_Position = ProjectionMatrix * ModelViewMatrix * position;
		
}
