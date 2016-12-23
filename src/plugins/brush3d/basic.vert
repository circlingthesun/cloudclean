#version 330
 
layout(location = 0) in vec3 vertex;
uniform vec3 colour;
uniform mat4 proj;
uniform mat4 mv;

void main( void )
{
    gl_Position = proj * mv * vec4(vertex.xyz, 1);
}
