#version 330

in vec4 vertex_position;
in vec2 texture_coordinate;
out vec2 coord;

void main(void)
{
  gl_Position = vertex_position;
  coord = texture_coordinate;
}
