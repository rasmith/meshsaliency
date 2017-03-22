#!/usr/bin/env python3
from glfw_controller import *
from mesh_view import *
from image_view import *

app = GlfwApp()
app.init()

multi_controller = GlfwMultiController()

mesh_path = "obj/bunny.obj"
mesh_fragment_shader = "mesh_fragment.glsl"
mesh_vertex_shader = "mesh_vertex.glsl"

model = MeshModel(mesh_path)
view = MeshView(mesh_fragment_shader, mesh_vertex_shader)

mesh_controller = GlfwController(800, 600, "Mesh View", view, model)
multi_controller.add(mesh_controller)

image_fragment_shader = "image_fragment.glsl"
image_vertex_shader = "image_vertex.glsl"

model = ImageModel()
view = ImageView(image_fragment_shader, image_vertex_shader)

image_controller = GlfwController(800, 600, "Image View", view, model)
multi_controller.add(image_controller)

multi_controller.run()
