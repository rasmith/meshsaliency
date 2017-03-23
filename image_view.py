import glfw
import sys
import graphics_math as gm
import numpy as np
import pyigl as igl
import traceback
from OpenGL.GL import *
from OpenGL.arrays import ArrayDatatype
from iglhelpers import *
from shader_program import *
from glfw_controller import *
from skimage import io

class ImageModel(GlfwModel):
    def __init__(self, image_path):
        self.image_path = image_path

    def initialize(self):
        self.image = io.imread(self.image_path)
        self.image_byte_count = ArrayDatatype.arrayByteCount(self.image)

class ImageView(GlfwView):
    def __init__(self, fragment_shader_path, vertex_shader_path):
        self.fragment_shader_path = fragment_shader_path
        self.vertex_shader_path = vertex_shader_path
        self.vertex_data = np.array([
          -1.0, -1.0, 0.0, 1.0,
          1.0, -1.0, 0.0, 1.0,
          1.0, 1.0, 0.0, 1.0,
          -1.0, 1.0, 0.0, 1.0
          ]).astype(dtype=np.float32, order='C')
        self.vertex_byte_count = ArrayDatatype.arrayByteCount(self.vertex_data)
        self.index_data = np.array([
          0, 1, 2, 3
          ]).astype(dtype=np.uint32, order='C')
        self.index_byte_count = ArrayDatatype.arrayByteCount(self.index_data)
        self.texture_data = np.array([
          0.0, 0.0,
          1.0, 0.0, 
          1.0, 0.0,
          0.0, 0.0
          ]).astype(dtype=np.float32, order='C')
        self.texture_byte_count = ArrayDatatype.arrayByteCount(self.texture_data)
        self.image_loaded = False

    def update_vbos(self):
        glBindVertexArray(self.vao_id)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_id[0])
        glBufferData(GL_ARRAY_BUFFER, self.vertex_byte_count,
                     self.vertex_data, GL_STATIC_DRAW)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_id[1])
        glBufferData(GL_ARRAY_BUFFER, self.texture_byte_count,
                     self.texture_data, GL_STATIC_DRAW)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.vbo_id[2])
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,  self.index_byte_count,
                     self.index_data, GL_STATIC_DRAW)

    def set_hints(self):
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, GL_TRUE)

    def initialize(self):
        # Load shaders.
        self.fragment = open(self.fragment_shader_path, 'r').read()
        self.vertex = open(self.vertex_shader_path, 'r').read()

        # Compile shaders and link program.
        self.program = ShaderProgram(
            fragment=self.fragment, vertex=self.vertex)

        glUseProgram(self.program.program_id)

        fragment_color_location = glGetFragDataLocation(
            self.program.program_id, "fragment_color")

        # Generate VAOs.
        self.vao_id = glGenVertexArrays(1)
        glBindVertexArray(self.vao_id)

        # Generate VBOs.
        self.vbo_id = glGenBuffers(3)

        # Generate texture.
        self.texture_id = glGenTextures(1)

        # Generate sampler.
        self.sampler_id = glGenSamplers(1)
        glSamplerParameteri(self.sampler_id, GL_TEXTURE_WRAP_S, GL_REPEAT)
        glSamplerParameteri(self.sampler_id, GL_TEXTURE_WRAP_T, GL_REPEAT)
        glSamplerParameteri(self.sampler_id, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glSamplerParameteri(self.sampler_id, GL_TEXTURE_MIN_FILTER, GL_LINEAR)


        # Setup the vertex data in VBO.
        self.vertex_location = self.program.attribute_location(
            'vertex_position')
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_id[0])
        glVertexAttribPointer(self.vertex_location, 4,
                              GL_FLOAT, GL_FALSE, 0, None)
        glEnableVertexAttribArray(self.vertex_location)
        # Setup the texture data in VBO.
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_id[1])
        self.texture_location = self.program.attribute_location(
            'texture_coordinate')
        glVertexAttribPointer(self.texture_location, 2,
                              GL_FLOAT, GL_FALSE, 0, None)
        glEnableVertexAttribArray(self.texture_location)


        self.texture_sampler_location = self.program.uniform_location(
            'texture_sampler')

        self.update_vbos()


    def render(self, width, height):
        glViewport(0, 0, width, height)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glEnable(GL_DEPTH_TEST)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glDepthFunc(GL_LESS)

        aspect = float(width) / float(height)

        # Specify program to be used
        glUseProgram(self.program.program_id)

        # Activate texture unit.
        glActiveTexture(GL_TEXTURE0);

        # Bind to texture.
        glBindTexture(GL_TEXTURE_2D, self.texture_id)

        image_width, image_height, image_channels = self.model.image.shape
        if not self.image_loaded:
            glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGB8, image_width, image_height)
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image_width, image_height,
                            GL_RGB, GL_UNSIGNED_BYTE, self.model.image)
            self.image_loaded = True

        # Set sampler unit.
        glUniform1i(self.texture_sampler_location, 0);

        # Bind to sampler.
        glBindSampler(0, self.sampler_id)

        # Bind to VAO.
        glBindVertexArray(self.vao_id)

        # Draw the triangles.
        glDrawElements(GL_TRIANGLE_FAN, 4, GL_UNSIGNED_INT, None)
