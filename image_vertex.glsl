#version 330

in vec3 vertex_position;
in vec2 texture_coordinate;
out vec2 coord;

void main(void)
{
  gl_Position = vec4(vertex_position, 1.0);
  coord = texture_coordinate;
}
