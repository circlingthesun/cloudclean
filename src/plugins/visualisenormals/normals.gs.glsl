#version 330

layout (points) in;
layout (line_strip, max_vertices=2) out;

in vec4 normal[];
out vec4 colour;

uniform vec3 line_colour;
uniform mat4 proj;
uniform mat4 mv;

void main(void) {
    vec4 n = normal[0];
    gl_Position = proj * mv * gl_in[0].gl_Position;
    colour = vec4(n.xyz, 1.0f);
    EmitVertex();
    
    vec4 short_normal = vec4(0.1f, 0.1f, 0.1f, 0.00f) * n;
    gl_Position = proj * mv * (gl_in[0].gl_Position + short_normal);
    EmitVertex();    
    EndPrimitive();
}
