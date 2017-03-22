#!/usr/bin/env python3
from glfw_controller import *
from mesh_view import *

app = GlfwApp()
app.init()

mesh_path = "obj/bunny.obj"
mesh_fragment_shader = "mesh_fragment.glsl"
mesh_vertex_shader = "mesh_vertex.glsl"

model = MeshModel(mesh_path)
view = MeshView(mesh_fragment_shader, mesh_vertex_shader)

c = GlfwController(800, 600, "Hello", view, model)
c.open()
c.run()
