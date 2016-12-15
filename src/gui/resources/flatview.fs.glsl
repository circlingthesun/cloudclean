#version 330

layout(location = 0) out vec4 fragColor;
in vec4 fcolour;

void main(void)
{
    fragColor = fcolour;
}
