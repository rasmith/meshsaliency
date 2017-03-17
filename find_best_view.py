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
uniform mat4 model;
uniform mat4 projection;
uniform mat4 view;

void main(void)
{
    mat4 mvp = projection * view * model;
    gl_Position = mvp * vec4(vertex_position, 1.0);
}
"""

fragment = """
#version 330
out vec4 fout_color;

void main(void)
{
    fout_color = vec4(1.0, 0.0, 0.0, 1.0);
}
"""

def check_gl_error(window): 
  error = glGetError()
  if not (error == GL_NO_ERROR): 
    print("OpenGL Error: code = %d" % error)
    glfw.terminate();
    sys.exit(-1)

def key_callback(window, key, scancode, action, mods):
  if key == glfw.KEY_ESCAPE  and action == glfw.PRESS:
    glfw.set_window_should_close(window, True)

if __name__ == "__main__":
  if not glfw.init():
    print('GLFW initialization failed')
    sys.exit(-1)

  mesh_path = sys.argv[1]
  print("mesh_path = %s\n" % mesh_path)
  vertices = igl.eigen.MatrixXd()
  faces  = igl.eigen.MatrixXi()

  try:
    if not igl.read_triangle_mesh(mesh_path, vertices, faces):
      print("failed to read mesh\n")
  except:
    traceback.print_exc(file=sys.stdout)
    sys.exit(-1)

  vertex_data = e2p(vertices).flatten('C').astype(dtype=np.float32, order='C')
  print("vertex_data.dtype = ", vertex_data.dtype)
  index_data = e2p(faces).flatten('C').astype(dtype=np.int32, order = 'C')
  print("index_data.dtype = ", index_data.dtype)
  print("vertices =\n")
  for i in range(10):
    print("%d : (%f, %f, %f)" % 
        (i, vertex_data[3*i], vertex_data[3*i+1], vertex_data[3*i+2]))
  print("indices =\n")
  for i in range(10):
    print("%d : (%d, %d, %d)" % 
        (i, index_data[3*i], index_data[3*i+1], index_data[3*i+2]))

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
    print("OpenWindow failed")
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
  print('GLSL Version: %s' % (glGetString(GL_SHADING_LANGUAGE_VERSION)))
  print('Renderer: %s' % (glGetString(GL_RENDERER)))

  # Compile shaders and link program.
  program = ShaderProgram(fragment=fragment, vertex=vertex)

  # Generate VAOs.
  vao_id = glGenVertexArrays(1)
  glBindVertexArray(vao_id)

  # Generate VBOs.
  vbo_id = glGenBuffers(2)

  # Setup the vertex data in VBO.
  vertex_location = program.attribute_location('vertex_position')
  print("location = ", vertex_location)
  glBindBuffer(GL_ARRAY_BUFFER, vbo_id[0])
  glBufferData(GL_ARRAY_BUFFER, ArrayDatatype.arrayByteCount(
    vertex_data), vertex_data, GL_STATIC_DRAW)
  glVertexAttribPointer(vertex_location, 3, GL_FLOAT, GL_FALSE, 0, None)
  glEnableVertexAttribArray(vertex_location)

  # Setup the indices data VBO.
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_id[1])
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, ArrayDatatype.arrayByteCount(
    index_data), index_data, GL_STATIC_DRAW)

  model_location = program.uniform_location('model')
  view_location = program.uniform_location('view')
  projection_location = program.uniform_location('projection')

  eye = np.transpose([[0.0, 0.1, 0.4, 1.0]])
  at = np.transpose([[0.0, 0.1, 0.0, 1.0]])
  up= np.transpose([[0.0, 1.0, 0.0, 1.0]])
  fov = 45.0
  near = 0.0001
  far = 1000

  model_matrix = np.eye(4).astype(dtype=np.float32, order='F')

  once = True

  num_faces = len(index_data) / 3

  # Render until told to close the window.
  while not glfw.window_should_close(window):
    # Get current screen width/height.
    framebuffer_width, framebuffer_height = glfw.get_framebuffer_size(window)

    glViewport(0, 0, framebuffer_width, framebuffer_height)
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDepthFunc(GL_LESS);

    aspect = float(framebuffer_width) / float(framebuffer_height)

    projection_matrix = gm.perspective(fov, aspect, near, far).astype(
        dtype=np.float32, order='F')
    view_matrix = gm.lookat(eye, at, up).astype(dtype=np.float32, order='F')

    if once:
      np.set_printoptions(precision=6, suppress=True)
      print("width = %d, height = %d\n" %
          (framebuffer_width, framebuffer_height))
      print("fov = %f, aspect = %f, near = %f, far = %f\n" %
          (fov, aspect, near, far))
      print("model =\n %s\n" % str(model_matrix))
      print("view =\n %s\n" % str(view_matrix))
      print("projection =\n %s\n" % str(projection_matrix))
      print("#faces = %d" % num_faces)
      once = False

    # Specify program to be used
    glUseProgram(program.program_id)
    glUniformMatrix4fv(model_location, 1, GL_FALSE, model_matrix)
    glUniformMatrix4fv(view_location, 1, GL_FALSE, view_matrix)
    glUniformMatrix4fv(projection_location, 1, GL_FALSE, projection_matrix)

    # Bind to VAO.
    glBindVertexArray(vao_id)

    # Draw the triangles.
    glDrawElements(GL_TRIANGLES,  int(num_faces * 3), GL_UNSIGNED_INT, None)

    # Poll for window events.
    glfw.poll_events()

    # Swap buffers.
    glfw.swap_buffers(window)
