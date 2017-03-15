#!/usr/bin/env python
import glfw
import numpy as np
from OpenGL.arrays import ArrayDatatype
from OpenGL.GL import (GL_ARRAY_BUFFER, GL_COLOR_BUFFER_BIT,
        GL_COMPILE_STATUS, GL_FALSE, GL_FLOAT, GL_FRAGMENT_SHADER,
        GL_LINK_STATUS, GL_RENDERER, GL_SHADING_LANGUAGE_VERSION,
        GL_STATIC_DRAW, GL_TRIANGLES, GL_TRUE, GL_VENDOR, GL_VERSION,
        GL_VERTEX_SHADER, glAttachShader, glBindBuffer, glBindVertexArray,
        glBufferData, glClear, glClearColor, glCompileShader,
        glCreateProgram, glCreateShader, glDeleteProgram,
        glDeleteShader, glDrawArrays, glEnableVertexAttribArray,
        glGenBuffers, glGenVertexArrays, glGetAttribLocation,
        glGetProgramInfoLog, glGetProgramiv, glGetShaderInfoLog,
        glGetShaderiv, glGetString, glGetUniformLocation, glLinkProgram,
        glShaderSource, glUseProgram, glVertexAttribPointer)
import sys, os
import traceback

sys.path.insert(0, "/usr/local/opt/libigl/python/")

import pyigl as igl
from iglhelpers import *

vertex = """
#version 330
in vec3 vin_position;
in vec3 vin_color;
out vec3 vout_color;

void main(void)
{
    vout_color = vin_color;
    gl_Position = vec4(vin_position, 1.0);
}
"""


fragment = """
#version 330
in vec3 vout_color;
out vec4 fout_color;

void main(void)
{
    fout_color = vec4(vout_color, 1.0);
}
"""

vertex_data = np.array([0.75, 0.75, 0.0,
    0.75, -0.75, 0.0,
    -0.75, -0.75, 0.0], dtype=np.float32)

color_data = np.array([1, 0, 0,
    0, 1, 0,
    0, 0, 1], dtype=np.float32)


class ShaderProgram(object):
    """ Helper class for using GLSL shader programs
    """

    def __init__(self, vertex, fragment):
        """
        Parameters
        ----------
        vertex : str
            String containing shader source code for the vertex
            shader
        fragment : str
            String containing shader source code for the fragment
            shader

        """
        self.program_id = glCreateProgram()
        vs_id = self.add_shader(vertex, GL_VERTEX_SHADER)
        frag_id = self.add_shader(fragment, GL_FRAGMENT_SHADER)

        glAttachShader(self.program_id, vs_id)
        glAttachShader(self.program_id, frag_id)
        glLinkProgram(self.program_id)

        if glGetProgramiv(self.program_id, GL_LINK_STATUS) != GL_TRUE:
            info = glGetProgramInfoLog(self.program_id)
            glDeleteProgram(self.program_id)
            glDeleteShader(vs_id)
            glDeleteShader(frag_id)
            raise RuntimeError('Error linking program: %s' % (info))
        glDeleteShader(vs_id)
        glDeleteShader(frag_id)

    def add_shader(self, source, shader_type):
        """ Helper function for compiling a GLSL shader

        Parameters
        ----------
        source : str
            String containing shader source code

        shader_type : valid OpenGL shader type
            Type of shader to compile

        Returns
        -------
        value : int
            Identifier for shader if compilation is successful

        """
        try:
            shader_id = glCreateShader(shader_type)
            glShaderSource(shader_id, source)
            glCompileShader(shader_id)
            if glGetShaderiv(shader_id, GL_COMPILE_STATUS) != GL_TRUE:
                info = glGetShaderInfoLog(shader_id)
                raise RuntimeError('Shader compilation failed: %s' % (info))
            return shader_id
        except:
            glDeleteShader(shader_id)
            raise

    def uniform_location(self, name):
        """ Helper function to get location of an OpenGL uniform variable

        Parameters
        ----------
        name : str
            Name of the variable for which location is to be returned

        Returns
        -------
        value : int
            Integer describing location

        """
        return glGetUniformLocation(self.program_id, name)

    def attribute_location(self, name):
        """ Helper function to get location of an OpenGL attribute variable

        Parameters
        ----------
        name : str
            Name of the variable for which location is to be returned

        Returns
        -------
        value : int
            Integer describing location

        """
        return glGetAttribLocation(self.program_id, name)


def key_callback(window, key, scancode, action, mods):
    """ Sample keyboard callback function """

if __name__ == "__main__":
    if not glfw.init():
        print('GLFW initialization failed')
        sys.exit(-1)

    mesh_path = sys.argv[1]
    print("mesh_path = %s\n" % mesh_path)
    V = igl.eigen.MatrixXd()
    F = igl.eigen.MatrixXi()
    try:
        if not igl.read_triangle_mesh(mesh_path, V, F):
            print("failed to read mesh\n")
    except: # catch *all* exceptions
        traceback.print_exc(file=sys.stdout)
        sys.exit(-1)
    print("V.size=%d" % V.size())
    print("V.cols=%d" % V.cols())
    print("V.rows=%d" % V.rows())
    print("F.size=%d" % F.size())
    print("F.cols=%d" % F.cols())
    print("F.rows=%d" % F.rows())
    vertices_np = e2p(V)
    vertex_data_2 = vertices_np.flatten(np.float32)
    print("vertices_np.shape = (%d, %d)" % vertices_np.shape)
    indices_np = e2p(F)
    color_data_2 = np.full(vertex_data_2.shape, 0.5)
    print("indices_np.shape = (%d, %d" % indices_np.shape)


    # These Window hints are used to specify
    # which opengl version to use and other details
    # for the opengl context that will be created
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 2)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, GL_TRUE)

    window = glfw.create_window(640, 480,  "Hello World", None, None)
    if not window:
        print ("OpenWindow failed")
        glfw.terminate()
        sys.exit(-1)

    glfw.make_context_current(window)

    # Every time a key on the keyboard is clicked
    # call our callback function
    glfw.set_key_callback(window, key_callback)
    glfw.swap_interval(1)

    glfw.set_window_title(window,"Modern opengl example")

    # If everything went well the following calls
    # will display the version of opengl being used
    print('Vendor: %s' % (glGetString(GL_VENDOR)))
    print('Opengl version: %s' % (glGetString(GL_VERSION)))
    print('GLSL Version: %s' % (glGetString(GL_SHADING_LANGUAGE_VERSION)))
    print('Renderer: %s' % (glGetString(GL_RENDERER)))

    glClearColor(0.95, 1.0, 0.95, 0)

    # Lets compile our shaders since the use of shaders is now
    # mandatory. We need at least a vertex and fragment shader
    # begore we can draw anything
    program = ShaderProgram(fragment=fragment, vertex=vertex)

    # Lets create a VAO and bind it
    # Think of VAO's as object that encapsulate buffer state
    # Using a VAO enables you to cut down on calls in your draw
    # loop which generally makes things run faster
    vao_id = glGenVertexArrays(1)
    glBindVertexArray(vao_id)

    # Lets create our Vertex Buffer objects - these are the buffers
    # that will contain our per vertex data
    vbo_id = glGenBuffers(2)

    # Bind a buffer before we can use it
    glBindBuffer(GL_ARRAY_BUFFER, vbo_id[0])

    # Now go ahead and fill this bound buffer with some data
    glBufferData(GL_ARRAY_BUFFER, ArrayDatatype.arrayByteCount(
        vertex_data), vertex_data, GL_STATIC_DRAW)

    # Now specify how the shader program will be receiving this data
    # In this case the data from this buffer will be available in the shader
    # as the vin_position vertex attribute
    glVertexAttribPointer(program.attribute_location(
        'vin_position'), 3, GL_FLOAT, GL_FALSE, 0, None)

    # Turn on this vertex attribute in the shader
    glEnableVertexAttribArray(0)

    # Now do the same for the other vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER, vbo_id[1])
    glBufferData(GL_ARRAY_BUFFER, ArrayDatatype.arrayByteCount(
        color_data), color_data, GL_STATIC_DRAW)
    glVertexAttribPointer(program.attribute_location(
        'vin_color'), 3, GL_FLOAT, GL_FALSE, 0, None)
    glEnableVertexAttribArray(1)

    # Lets unbind our vbo and vao state
    # We will bind these again in the draw loop
    glBindBuffer(GL_ARRAY_BUFFER, 0)
    glBindVertexArray(0)

    while not glfw.window_should_close(window):
        glClear(GL_COLOR_BUFFER_BIT)

        # Specify shader to be used
        glUseProgram(program.program_id)

        # Bind VAO - this will automatically
        # bind all the vbo's saving us a bunch
        # of calls
        glBindVertexArray(vao_id)

        # Modern GL makes the draw call really simple
        # All the complexity has been pushed elsewhere
        glDrawArrays(GL_TRIANGLES, 0, 3)

        # Lets unbind the shader and vertex array state
        glUseProgram(0)
        glBindVertexArray(0)

        # Now lets show our master piece on the screen
        glfw.poll_events()
        glfw.swap_buffers(window)
