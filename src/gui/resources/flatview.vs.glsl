#version 330
layout(location = 0) in float intensity;
layout(location = 1) in int color_index;
layout(location = 2) in int flags;
layout(location = 3) in vec2 position;

uniform samplerBuffer sampler;
uniform mat3 camera;
out vec4 colour;

const vec4 select_colours[8] = vec4[8](
        vec4(1.0f, 0.0f, 0.0f, 1.0f), // Red
        vec4(0.0f, 1.0f, 0.0f, 1.0f), // Green
        vec4(0.0f, 0.0f, 1.0f, 1.0f), // Blue
        vec4(1.0f, 1.0f, 0.0f, 1.0f), // Yellow
        vec4(0.0f, 1.0f, 1.0f, 1.0f), // Cyan
        vec4(1.0f, 0.0f, 1.0f, 1.0f), // Magenta
        vec4(1.0f, 0.5f, 0.0f, 1.0f), // Orange
        vec4(0.5f, 0.0f, 1.0f, 1.0f) // Purple
);

void main( void ) {
    // Fetch the colour from the buffer texture
    vec4 layer_colour = texelFetch(sampler, color_index);

    // Adjust the colour intensity
    colour = layer_colour * intensity;

    int select_count = 0;
    vec4 select_colour = vec4(0.0f, 0.0f, 0.0f, 0.0f);
    for(int i = 0; i < 8; i++){
        int mask = 1 << i;
        if( bool(mask & flags) ) {
            select_colour += select_colours[i];
            select_count ++;
        }
    }

    if(select_count != 0){
        colour = colour * 0.7f + select_colour/select_count * 0.3f;
    }

    // Make it white if invisible
    if(colour.a == 0) {
        colour = vec4(1.0f, 1.0f, 1.0f, 1.0f);
        return;
    }

    vec3 pos = vec3(position, 1.0f);

    gl_Position = vec4((camera*pos).xy, 0, 1);
}
