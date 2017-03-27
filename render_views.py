#!/usr/bin/env python3
import numpy as np
import time
from glfw_controller import *
from mesh_view import *
from image_view import *
from OpenGL.GL import *
from PIL import *




class SphericalCameraGenerator(object):
    def __init__(self, radius_min, radius_max, radius_n_steps, theta_min,
                theta_max, theta_n_steps, phi_min, phi_max, phi_n_steps):
        self.radius_min = radius_min
        self.radius_max = radius_max
        self.radius_n_steps = radius_n_steps
        self.radius_step = (
            self.radius_max - self.radius_min) / self.radius_n_steps
        self.theta_min = theta_min
        self.theta_max = theta_max
        self.theta_n_steps = theta_n_steps
        self.theta_step = (self.theta_max - self.theta_min) / \
            self.theta_n_steps
        self.phi_min = phi_min
        self.phi_max = phi_max
        self.phi_n_steps = phi_n_steps
        self.phi_step = (self.phi_max - self.phi_min) / self.phi_n_steps
        self.ri = 0
        self.ti = 0
        self.pi = 0

    def __next__(self):
        if self.pi >= self.phi_n_steps:
            self.pi = 0
            self.ti = self.ti + 1
        if self.ti >= self.theta_n_steps:
            self.ti = 0
            self.ri = self.ri + 1
        if self.ri >= self.radius_n_steps:
            raise StopIteration
        radius = self.radius_min + self.radius_step * self.ri
        theta = self.theta_min + self.theta_step * self.ti
        phi = self.phi_min + self.phi_step * self.pi
        eye = [radius * np.cos(theta) * np.sin(phi),
               radius * np.sin(theta) * np.sin(phi),
               radius * np.cos(phi)]
        at = [0.0, 0.0, 0.0]
        up = [0.0, 1.0, 0.0]
        self.pi = self.pi + 1
        return eye, at, up


class RenderViewsController(GlfwController):
    def set_camera_generator(self, generator):
        self.generator = generator
        self.last  = time.clock()
        self.first = True
        self.i = 0

    def pre_render(self):
        try:
            current = time.clock()
            eye, at, up = next(self.generator)
            self.i = self.i + 1
            self.view.set_camera(eye, at, up, 45.0, 0.0001, 100)
            self.view.set_light_position(eye)
            self.last = current
            self.first = False
        except StopIteration:
            glfw.set_window_should_close(self.window, True)

    def post_render(self):
        pixels = glReadPixels(0, 0, self.current_width, self.current_height, 
                        GL_RGB, GL_UNSIGNED_BYTE)
        im = Image.new("RGB", (self.current_width, self.current_height))
        im.frombytes(pixels)
        im= im.transpose(Image.FLIP_TOP_BOTTOM)
        im.save("out"+str(self.i)+".png")
        glfw.post_empty_event()


app = GlfwApp()
app.init()

mesh_path = "living_room/model.obj"
mesh_fragment_shader = "mesh_fragment.glsl"
mesh_vertex_shader = "mesh_vertex.glsl"

model = MeshModel(mesh_path)
view = MeshView(mesh_fragment_shader, mesh_vertex_shader)

title = "Mesh View"
mesh_controller = RenderViewsController(400, 300, 100, 100, title, view, model)
mesh_controller.set_camera_generator(SphericalCameraGenerator(
    0.5, 1.0, 2, 0.0, 2.0 * np.pi, 5, 0.0, 2.0 * np.pi, 5))
mesh_controller.open()
mesh_controller.run()
