#version 330

in vec3 vertex_position;
in vec3 vertex_normal;
uniform vec3 light_position;
uniform mat4 model;
uniform mat4 projection;
uniform mat4 view;
out vec4 normal;
out vec4 light_direction;
out vec4 eye_direction;
out vec4 reflect_direction;

void main(void)
{
    mat4 model_view  = view * model;
    mat4 normal_matrix = transpose(inverse(model_view));
    mat4 mvp = projection*model_view;
    gl_Position = mvp * vec4(vertex_position, 1.0);
    normal =  normalize(normal_matrix * vec4(vertex_normal, 0.0));
    light_direction = vec4(normalize(light_position - vertex_position), 0.0);
    eye_direction = -gl_Position;
    reflect_direction = reflect(light_direction, normal);
}
