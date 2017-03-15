import numpy as np

def perspective(fovy, aspect, near, far):
  f = 1.0 / np.tan(fovy * np.pi / 360.0);
  d = near - far;
  matrix = np.zeros((4, 4))
  matrix[0,0]= f / aspect
  matrix[1,1] = f
  matrix[2,2] = (near + far) / d
  matrix[3,2] = 2.0 * near * far / d
  matrix[2,3] = -1.0
  return matrix

def lookat(eye, at, up):
  z_direction = eye[0:3, 0] - at[0:3, 0];
  z_direction = z_direction / np.linalg.norm(z_direction)
  x_direction = np.cross(up[0:3, 0] / np.linalg.norm(up[0:3, 0]), z_direction)
  x_direction = x_direction / np.linalg.norm(x_direction)
  y_direction = np.cross(z_direction, x_direction)
  y_direction = y_direction / np.linalg.norm(y_direction)
  matrix = np.eye(4)
  matrix[0, 0:3] = x_direction
  matrix[1, 0:3] = y_direction
  matrix[2, 0:3] = z_direction
  matrix[0:3, 3] = -matrix[0:3,0:3].dot(eye[0:3,0])
  return matrix
