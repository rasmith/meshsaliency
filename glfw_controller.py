import glfw
import sys
from OpenGL.GL import *

class GlfwApp(object):
    _instance = None
    def __new__(cls):
        if GlfwApp._instance is None:
            GlfwApp._instance = object.__new__(cls)
        return GlfwApp._instance

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
        pass


class GlfwView(object):
    def __init__(self):
        pass

    def set_model(self, model):
        self.model = model

    def render(self, width, height):
        pass

class GlfwController(object):
    def __init__(self, width, height, xpos, ypos, title, view, model):
        self.title = title
        self.height = height
        self.width = width
        self.view = view
        self.model = model
        self.initialized = False
        self.xpos = xpos
        self.ypos = ypos

    def open(self):
        self.view.set_hints()

        self.window = glfw.create_window(self.width, self.height, self.title, None,
                                         None)
        if not self.window:
            print("Failed to open GFLW window.")
            glfw.terminate()
            sys.exit(-1)
        glfw.make_context_current(self.window)
        glfw.set_window_pos(self.window, self.xpos, self.ypos)
        glfw.set_key_callback(self.window,
                              lambda window, key, scancode, action, mods:
                              self.key_callback(key, scancode, action, mods))
        glfw.swap_interval(1)
        glfw.set_window_title(self.window, self.title)

    def key_callback(self, key, scancode, action, mods):
        if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
            glfw.set_window_should_close(self.window, True)

    def initialize(self):
        if self.initialized:
            return
        if self.model is not None:
            self.model.initialize()

        if self.view is not None and self.model is not None:
            self.view.set_model(self.model)
            self.view.initialize()
        self.initialized = True

    def pre_render(self):
        pass

    def post_render(self):
        pass

    def close(self):
        glfw.destroy_window(self.window)

    def render_and_update(self):
        if glfw.window_should_close(self.window):
            return True

        # Switch to the current context.
        glfw.make_context_current(self.window)

        # Get current screen width/height.
        framebuffer_width, framebuffer_height = glfw.get_framebuffer_size(
            self.window)
        glViewport(0, 0, framebuffer_width, framebuffer_height)

        self.current_width = framebuffer_width
        self.current_height = framebuffer_height

        self.pre_render()

        if self.view is not None:
            self.view.render(framebuffer_width, framebuffer_height)

        self.post_render()

        # Poll for window events.
        glfw.poll_events()

        # Swap buffers.
        glfw.swap_buffers(self.window)

        return False

    def run(self):
        glfw.make_context_current(self.window)

        self.initialize()

        while not self.render_and_update():
            pass

        self.close()

class GlfwMultiController(object):
    def __init__(self):
        self.controllers = []

    def add(self, controller):
        self.controllers.append(
            {'controller': controller,
             'initialized': False,
             'opened': False,
             'closed': False})

    def run(self):
        running = True
        while running:
            running = False
            for info in self.controllers:
                controller = info['controller']
                if not info['opened']:
                    controller.open()
                    info['opened'] = True
                if not info['initialized']:
                    controller.initialize()
                    info['initialized'] = True
                if not info['closed']:
                    running = True
                    should_close = controller.render_and_update()
                    if should_close:
                        controller.close()
                        info['closed'] = True
