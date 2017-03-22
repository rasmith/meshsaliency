#version 330

uniform sampler2D texture_sampler;
in vec2 coord;
out vec4 fragment_color;

void main(void) {
  fragment_color = texture(texture_sampler, coord);
}
