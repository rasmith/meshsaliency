from OpenGL.GL import *

class ShaderProgram(object):

  def __init__(self, vertex, fragment):
    self.program_id = glCreateProgram()
    vs_id = self.add_shader(vertex, GL_VERTEX_SHADER)
    frag_id = self.add_shader(fragment, GL_FRAGMENT_SHADER)

    glAttachShader(self.program_id, vs_id)
    glAttachShader(self.program_id, frag_id)
    glLinkProgram(self.program_id)

    if glGetProgramiv(self.program_id, GL_LINK_STATUS) != GL_TRUE:
      info = glGetProgramInfoLog(self.program_id)
      glDeleteShader(vs_id)
      glDeleteShader(frag_id)
      raise RuntimeError('Error linking program: %s' % (info))

  def add_shader(self, source, shader_type):
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
    return glGetUniformLocation(self.program_id, name)

  def attribute_location(self, name):
    return glGetAttribLocation(self.program_id, name)
