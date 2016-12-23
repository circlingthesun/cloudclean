#version 330
 
layout(location = 0) out vec4 fragColor;

uniform vec3 colour;
 
void main( void )
{
    fragColor = vec4(colour, 1);
}
