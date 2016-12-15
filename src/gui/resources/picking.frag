#version 330

layout(location = 0) out uint fragColor;

void main( void ){
    fragColor = uint(gl_PrimitiveID);
}
