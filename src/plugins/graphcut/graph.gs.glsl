#version 330

layout (lines) in;
layout (triangle_strip, max_vertices=10) out;

out vec4 colour;

uniform samplerBuffer sampler;
uniform float max_line_width;
uniform vec3 elColour;
uniform mat4 cameraToClipMatrix;
uniform mat4 modelToCameraMatrix;

void main(void)
{

    float normalised_width = texelFetch(sampler, gl_PrimitiveIDIn).x;

    float width = normalised_width * max_line_width;

    vec4 start = gl_in[0].gl_Position;
    vec4 end = gl_in[1].gl_Position;

    mat4 trans = cameraToClipMatrix * modelToCameraMatrix;

    // find orhogonal vector to set width along
    vec3 line_sight = end.xyz * 0.5f + start.xyz * 0.5f;
    vec3 line_dir = end.xyz - start.xyz;

    vec4 width_dir = vec4(normalize(cross(line_sight, line_dir)), 0.0f);
    vec4 height_dir = vec4(normalize(cross(width_dir.xyz, line_dir)), 0.0f);

    vec4 ofset_dirs[2];
    ofset_dirs[0] = width_dir;
    ofset_dirs[1] = height_dir;

    float signs [4];
    signs[0] = -1.0f;
    signs[1] = 1.0f;
    signs[2] = 1.0f;
    signs[3] = -1.0f;

    // emit 4 rectangles in trangle stripa
    for(int idx = 0; idx < 5; idx++){
        int i = idx%4;

        vec4 dir = ofset_dirs[i&1];
        float sign = signs[i];

        gl_Position = trans * (start + sign*(width/2.0f)*dir);
        colour = vec4(elColour, 1.0f);
        EmitVertex();

        gl_Position = trans * (end + sign*(width/2.0f)*dir);
        colour = vec4(elColour, 1.0f);
        EmitVertex();
    }


    EndPrimitive();
}
