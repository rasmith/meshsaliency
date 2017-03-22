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


class MeshModel(GlfwModel):
    def __init__(self, mesh_path):
        self.mesh_path = mesh_path

    def initialize(self):
        self.vertices = igl.eigen.MatrixXd()
        self.faces = igl.eigen.MatrixXi()

        try:
            if not igl.read_triangle_mesh(self.mesh_path, self.vertices,
                                          self.faces):
                print("failed to read mesh\n")
        except:
            traceback.print_exc(file=sys.stdout)
            sys.exit(-1)

        self.face_normals = igl.eigen.MatrixXd()
        igl.per_face_normals(self.vertices, self.faces, self.face_normals)
        self.vertex_normals = igl.eigen.MatrixXd()
        igl.per_vertex_normals(self.vertices, self.faces,
                               igl.PER_VERTEX_NORMALS_WEIGHTING_TYPE_AREA,
                               self.vertex_normals)

        self.vertex_data = e2p(self.vertices).flatten(
            'C').astype(dtype=np.float32, order='C')
        self.index_data = e2p(self.faces).flatten(
            'C').astype(dtype=np.uint32, order='C')
        self.face_normal_data = e2p(self.face_normals).flatten(
            'C').astype(dtype=np.float32, order='C')
        self.vertex_normal_data = e2p(self.vertex_normals).flatten(
            'C').astype(dtype=np.float32, order='C')

        self.num_faces = int(len(self.index_data) / 3)
        self.num_vertices = int(len(self.vertex_data) / 3)
        self.center = np.mean(
            np.reshape(self.vertex_data, (self.num_vertices, 3)), axis=0)
        self.max_vals = np.max(
            np.reshape(self.vertex_data, (self.num_vertices, 3)), axis=0)
        self.min_vals = np.min(
            np.reshape(self.vertex_data, (self.num_vertices, 3)), axis=0)
        self.extents = self.max_vals - self.min_vals

        self.vertex_data = np.reshape(self.vertex_data, (self.num_vertices, 3))
        self.vertex_data = (self.vertex_data - self.center) / self.extents
        self.vertex_data = np.reshape(self.vertex_data, 3 * self.num_vertices)

        self.vertex_byte_count = ArrayDatatype.arrayByteCount(self.vertex_data)

        self.vertex_normal_byte_count = ArrayDatatype.arrayByteCount(
            self.vertex_normal_data)
        self.index_byte_count = ArrayDatatype.arrayByteCount(self.index_data)
        self.diagonal = self.max_vals - self.min_vals


class MeshView(GlfwView):
    def __init__(self, fragment_shader_path, vertex_shader_path):
        self.fragment_shader_path = fragment_shader_path
        self.vertex_shader_path = vertex_shader_path

    def update_vbos(self):
        glBindVertexArray(self.vao_id)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_id[0])
        glBufferData(GL_ARRAY_BUFFER, self.model.vertex_byte_count,
                     self.model.vertex_data, GL_STATIC_DRAW)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_id[1])
        glBufferData(GL_ARRAY_BUFFER, self.model.vertex_normal_byte_count,
                     self.model.vertex_normal_data, GL_STATIC_DRAW)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.vbo_id[2])
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,  self.model.index_byte_count,
                     self.model.index_data, GL_STATIC_DRAW)

    def set_hints(self):
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, GL_TRUE)

    def initialize(self):
        self.set_hints()

        # Load shaders.
        self.fragment = open(self.fragment_shader_path, 'r').read()
        self.vertex = open(self.vertex_shader_path, 'r').read()

        # Compile shaders and link program.
        self.program = ShaderProgram(
            fragment=self.fragment, vertex=self.vertex)

        fragment_color_location = glGetFragDataLocation(
            self.program.program_id, "fragment_color")

        # Generate VAOs.
        self.vao_id = glGenVertexArrays(1)
        glBindVertexArray(self.vao_id)

        # Generate VBOs.
        self.vbo_id = glGenBuffers(3)

        # Setup the vertex data in VBO.
        self.vertex_location = self.program.attribute_location(
            'vertex_position')
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_id[0])
        glVertexAttribPointer(self.vertex_location, 3,
                              GL_FLOAT, GL_FALSE, 0, None)
        glEnableVertexAttribArray(self.vertex_location)

        # Setup the normal data in VBO.
        self.vertex_normal_location = self.program.attribute_location(
            'vertex_normal')
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_id[1])
        glVertexAttribPointer(self.vertex_normal_location, 3,
                              GL_FLOAT, GL_FALSE, 0, None)
        glEnableVertexAttribArray(self.vertex_normal_location)

        # Setup the indices data VBO.
        self.model_location = self.program.uniform_location('model')
        self.view_location = self.program.uniform_location('view')
        self.projection_location = self.program.uniform_location('projection')
        self.light_position_location = self.program.uniform_location(
            'light_position')

        self.update_vbos()

        self.eye = np.transpose([[0.0, 0.0, 2.0, 1.0]])
        self.at = np.transpose([[0.0, 0.0, 0.0, 1.0]])
        self.up = np.transpose([[0.0, 1.0, 0.0, 1.0]])
        self.fov = 45.0
        self.near = 0.0001
        self.far = 100

    def render(self, width, height):
        glViewport(0, 0, width, height)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glEnable(GL_DEPTH_TEST)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glDepthFunc(GL_LESS)

        aspect = float(width) / float(height)

        projection_matrix = gm.perspective(
            self.fov, aspect, self.near, self.far)
        model_matrix = np.eye(4)
        view_matrix = gm.lookat(self.eye, self.at, self.up)
        light_position = np.array([0.0, 5.0, 1.0])

        # Specify program to be used
        glUseProgram(self.program.program_id)
        model_matrix_py = model_matrix.transpose().flatten().tolist()
        glUniformMatrix4fv(self.model_location, 1, GL_FALSE,
                           (GLfloat * 16)(*model_matrix_py))
        view_matrix_py = view_matrix.transpose().flatten().tolist()
        glUniformMatrix4fv(self.view_location, 1, GL_FALSE,
                           (GLfloat * 16)(*view_matrix_py))
        projection_matrix_py = projection_matrix.transpose().flatten().tolist()
        glUniformMatrix4fv(self.projection_location, 1, GL_FALSE,
                           (GLfloat * 16)(*projection_matrix_py))
        light_position_py = light_position.tolist()
        glUniform3fv(self.light_position_location, 1,
                     (GLfloat * 3)(*light_position_py))

        # Bind to VAO.
        glBindVertexArray(self.vao_id)

        # Draw the triangles.
        glDrawElements(GL_TRIANGLES, self.model.num_faces *
                       3, GL_UNSIGNED_INT, None)
