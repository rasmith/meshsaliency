import glfw
import sys
from OpenGL.GL import *


class GlfwApp(object):
    def __init__(self):
        self.initialized = False

    def init(self):
        if self.initialized:
            return
        if not glfw.init():
            print('GLFW initialization failed')
            sys.exit(-1)
        glfw.set_error_callback(lambda error, description:
                                self.error_callback(error, description))
        self.initialized = True

        def error_callback(self, error, description):
            print("GFLW Error: %d:%s " % (error, description))


class GlfwModel(object):
    def __init__(self):
        return


class GlfwView(object):
    def __init__(self):
        return

    def set_model(self, model):
        self.model = model

    def render(self, width, height):
        return


class GlfwController(object):
    def __init__(self, width, height, title, view, model):
        self.title = title
        self.height = height
        self.width = width
        self.view = view
        self.model = model

    def open(self):
        self.view.set_hints()

        self.window = glfw.create_window(self.width, self.height, self.title, None,
                                         None)
        if not self.window:
            print("Failed to open GFLW window.")
            glfw.terminate()
            sys.exit(-1)
        glfw.make_context_current(self.window)
        glfw.set_key_callback(self.window,
                              lambda window, key, scancode, action, mods:
                              self.key_callback(key, scancode, action, mods))
        glfw.swap_interval(1)
        glfw.set_window_title(self.window, self.title)

    def key_callback(self, key, scancode, action, mods):
        if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
            glfw.set_window_should_close(self.window, True)

    def run(self):
        glfw.make_context_current(self.window)

        if self.model is not None:
            self.model.initialize()

        if self.view is not None and self.model is not None:
            self.view.set_model(self.model)
            self.view.initialize()

           # Render until told to close the window.
        while not glfw.window_should_close(self.window):
            # Switch to the current context.
            glfw.make_context_current(self.window)

            # Get current screen width/height.
            framebuffer_width, framebuffer_height = glfw.get_framebuffer_size(
                self.window)
            glViewport(0, 0, framebuffer_width, framebuffer_height)

            if self.view is not None:
                self.view.render(framebuffer_width, framebuffer_height)

            # Poll for window events.
            glfw.poll_events()

            # Swap buffers.
            glfw.swap_buffers(self.window)
