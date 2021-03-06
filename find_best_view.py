#!/usr/bin/env python3
import glfw
import os
import sys
import traceback
import graphics_math as gm
import numpy as np
import pyigl as igl
from OpenGL.GL import *
from OpenGL.arrays import ArrayDatatype
from iglhelpers import *
from shader_program import *

vertex = """
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
"""

fragment = """
#version 330
in vec4 normal;
in vec4 light_direction;
in vec4 reflect_direction;
in vec4 eye_direction;
out vec4 fragment_color;

void main(void)
{
    vec4 color = vec4(1.0, 0.0, 0.0, 1.0);
    float dot_nl = dot(normalize(light_direction), normalize(normal));
    dot_nl = clamp(dot_nl, 0.0, 1.0);
    float dot_rv = dot(normalize(eye_direction), normalize(reflect_direction));
    dot_rv = clamp(dot_rv, 0.0, 1.0);
    float pow_rv = ceil(dot_nl) * clamp(pow(dot_rv, 5.0), 0.0, 1.0);
    fragment_color = clamp(0.5 * (dot_nl + pow_rv) * color, 0.0, 1.0);
}
"""

def check_matrix_uniform(name, program, location):
    check = (GLfloat * 16)()
    glGetUniformfv(program, location, check)
    ptr = ctypes.cast(check, ctypes.POINTER(ctypes.c_float))
    value_list = [ptr[i] for i in range(16)]
    print("%s =\n%s" % (name, str(
      np.ndarray(buffer=np.array(value_list), shape=(4, 4)).transpose())))

def check_gl_error(window):
    error = glGetError()
    if not (error == GL_NO_ERROR):
        print("OpenGL Error: code = %d" % error)
        glfw.terminate()
        sys.exit(-1)


def key_callback(window, key, scancode, action, mods):
    if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
        glfw.set_window_should_close(window, True)


def error_callback(error, description):
    print("GFLW Error: %d:%s " % (error, description))

# def get_model()
    # model = ml_net_model(img_cols=shape_c, img_rows=shape_r, 
        # downsampling_factor_product=10)
    # sgd = SGD(lr=1e-3, decay=0.0005, momentum=0.9, nesterov=True)
    # print("Compile ML-Net Model")
    # model.compile(sgd, loss)

if __name__ == "__main__":
    mesh_path = sys.argv[1]
    print("mesh_path = %s\n" % mesh_path)
    vertices = igl.eigen.MatrixXd()
    faces = igl.eigen.MatrixXi()

    try:
        if not igl.read_triangle_mesh(mesh_path, vertices, faces):
            print("failed to read mesh\n")
    except:
        traceback.print_exc(file=sys.stdout)
        sys.exit(-1)

    face_normals = igl.eigen.MatrixXd()
    igl.per_face_normals(vertices, faces, face_normals)
    vertex_normals = igl.eigen.MatrixXd()
    igl.per_vertex_normals(vertices, faces,
        igl.PER_VERTEX_NORMALS_WEIGHTING_TYPE_AREA, vertex_normals)

    vertex_data = e2p(vertices).flatten(
        'C').astype(dtype=np.float32, order='C')
    index_data = e2p(faces).flatten(
        'C').astype(dtype=np.uint32, order='C')
    face_normal_data = e2p(face_normals).flatten(
        'C').astype(dtype=np.float32, order='C')
    vertex_normal_data = e2p(vertex_normals).flatten(
        'C').astype(dtype=np.float32, order='C')

    num_faces = len(index_data) / 3
    num_vertices = len(vertex_data) / 3
    print("#vertices = %d and #faces = %d" %
          (num_vertices, num_faces))
    print("vertices =\n")
    for i in range(10):
        va = vertex_data[3 * i]
        vb = vertex_data[3 * i + 1]
        vc = vertex_data[3 * i + 2]
        print("%d : (%f, %f, %f)" % (i, va, vb, vc))
    print("indices =\n")
    for i in range(10):
        ia = index_data[3 * i]
        ib = index_data[3 * i + 1]
        ic = index_data[3 * i + 2]
        print("%d : (%f, %f, %f)" % (i, ia, ib, ic))
    center = np.mean(
        np.reshape(vertex_data, (int(num_vertices), 3)), axis=0) 
    max_vals = np.max(
        np.reshape(vertex_data, (int(num_vertices), 3)), axis=0) 
    min_vals = np.min(
        np.reshape(vertex_data, (int(num_vertices), 3)), axis=0) 
    extents = max_vals - min_vals

    print("center = %s" % str(center))
    print("max = %s" % str(max_vals))
    print("min = %s" % str(min_vals))

    vertex_data = np.reshape(vertex_data, (int(num_vertices), 3))
    vertex_data = (vertex_data - center) / extents
    vertex_data = np.reshape(vertex_data, 3 * int(num_vertices))

    if not glfw.init():
        print('GLFW initialization failed')
        sys.exit(-1)

    glfw.set_error_callback(error_callback)

    # Setup window hints: set the GLSL version and profile.
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, GL_TRUE)

    window_width = 800
    window_height = 600
    window = glfw.create_window(window_width, window_height, "Hello World",
                                None, None)
    if not window:
        print("Failed to open GFLW window.")
        glfw.terminate()
        sys.exit(-1)

    glfw.make_context_current(window)

    # Setup the key callback.
    glfw.set_key_callback(window, key_callback)
    glfw.swap_interval(1)

    glfw.set_window_title(window, "Modern OpenGL")

    # Print out some environment information.
    print('Vendor: %s' % (glGetString(GL_VENDOR)))
    print('Opengl version: %s' % (glGetString(GL_VERSION)))
    print('GLSL Version: %s' %
          (glGetString(GL_SHADING_LANGUAGE_VERSION)))
    print('Renderer: %s' % (glGetString(GL_RENDERER)))

    # Compile shaders and link program.
    program = ShaderProgram(fragment=fragment, vertex=vertex)

    fragment_color_location = glGetFragDataLocation(
        program.program_id, "fragment_color")
    print("fragment_color_location = %d" % fragment_color_location)

    # Generate VAOs.
    vao_id = glGenVertexArrays(1)
    glBindVertexArray(vao_id)

    # Generate VBOs.
    vbo_id = glGenBuffers(3)

    # Setup the vertex data in VBO.
    vertex_location = program.attribute_location('vertex_position')
    vertex_byte_count = ArrayDatatype.arrayByteCount(vertex_data)
    print("vertex_location = ", vertex_location)
    print("vertex_byte_count = ", vertex_byte_count)
    glBindBuffer(GL_ARRAY_BUFFER, vbo_id[0])
    glBufferData(GL_ARRAY_BUFFER, vertex_byte_count,
                 vertex_data, GL_STATIC_DRAW)
    glVertexAttribPointer(vertex_location, 3,
                          GL_FLOAT, GL_FALSE, 0, None)
    glEnableVertexAttribArray(vertex_location)

    # Setup the normal data in VBO.
    vertex_normal_location = program.attribute_location('vertex_normal')
    vertex_normal_byte_count = ArrayDatatype.arrayByteCount(vertex_normal_data)
    print("normal_location = ", vertex_normal_location)
    print("normal_byte_count = ", vertex_normal_byte_count)
    glBindBuffer(GL_ARRAY_BUFFER, vbo_id[1])
    glBufferData(GL_ARRAY_BUFFER, vertex_normal_byte_count,
                 vertex_normal_data, GL_STATIC_DRAW)
    glVertexAttribPointer(vertex_normal_location, 3,
                          GL_FLOAT, GL_FALSE, 0, None)
    glEnableVertexAttribArray(vertex_normal_location)

    # Setup the indices data VBO.
    index_byte_count = ArrayDatatype.arrayByteCount(index_data)
    print("index_byte_count = ", index_byte_count)
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_id[2])
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,  index_byte_count, index_data,
                 GL_STATIC_DRAW)

    model_location = program.uniform_location('model')
    view_location = program.uniform_location('view')
    projection_location = program.uniform_location('projection')
    light_position_location = program.uniform_location('light_position')
    print("matrix locations = (%d, %d, %d, %d)"
          % (model_location, view_location, projection_location,
            light_position_location))

    diagonal = max_vals - min_vals

    eye = np.transpose([[0.0, 0.0, 2.0, 1.0]])
    at = np.transpose([[0.0, 0.0, 0.0, 1.0]])
    up = np.transpose([[0.0, 1.0, 0.0, 1.0]])
    fov = 45.0
    near = 0.0001
    far =  100

    model_matrix = np.eye(4)

    once = True

    # Render until told to close the window.
    while not glfw.window_should_close(window):
        # Get current screen width/height.
        framebuffer_width, framebuffer_height = glfw.get_framebuffer_size(
            window)

        glViewport(0, 0, framebuffer_width, framebuffer_height)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glEnable(GL_DEPTH_TEST)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glDepthFunc(GL_LESS)

        aspect = float(framebuffer_width) / float(framebuffer_height)

        projection_matrix = gm.perspective(fov, aspect, near, far)
        view_matrix = gm.lookat(eye, at, up)
        light_position = np.array([0.0, 5.0, 1.0])


        # Specify program to be used
        glUseProgram(program.program_id)
        model_matrix_py = model_matrix.transpose().flatten().tolist()
        glUniformMatrix4fv(model_location, 1, GL_FALSE,
                           (GLfloat * 16)(*model_matrix_py))
        view_matrix_py = view_matrix.transpose().flatten().tolist()
        glUniformMatrix4fv(view_location, 1, GL_FALSE,
                           (GLfloat * 16)(*view_matrix_py))
        projection_matrix_py = projection_matrix.transpose().flatten().tolist()
        glUniformMatrix4fv(projection_location, 1, GL_FALSE,
                           (GLfloat * 16)(*projection_matrix_py))
        light_position_py = light_position.tolist()
        glUniform3fv(light_position_location, 1,
            (GLfloat * 3)(*light_position_py))

        if once:
            np.set_printoptions(precision=6, suppress=True)
            print("width = %d, height = %d\n" %
                  (framebuffer_width, framebuffer_height))
            print("fov = %f, aspect = %f, near = %f, far = %f\n" %
                  (fov, aspect, near, far))
            print("model =\n %s\n" % str(model_matrix))
            print("view =\n %s\n" % str(view_matrix))
            print("projection =\n %s\n" % str(projection_matrix))
            # check_matrix_uniform('model',program.program_id, model_location)
            # check_matrix_uniform('view',program.program_id, view_location)
            # check_matrix_uniform('projection',program.program_id,
                # projection_location)
            once = False

        # Bind to VAO.
        glBindVertexArray(vao_id)

        # Draw the triangles.
        glDrawElements(GL_TRIANGLES,  int(
            num_faces * 3), GL_UNSIGNED_INT, None)

        # Poll for window events.
        glfw.poll_events()

        # Swap buffers.
        glfw.swap_buffers(window)
