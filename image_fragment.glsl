#version 330

uniform sampler2D texture_sampler;
in vec2 coord;
out vec4 fragment_color;

void main(void) {
  fragment_color = texture(texture_sampler, coord);
//	fragment_color = vec4(coord[0], coord[1], 0.0, 1.0);
}
