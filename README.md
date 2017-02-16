#Mesh Saliency

## Introduction
Not much to say yet!

## Dependencies

### Libraries
* [libigl](https://github.com/libigl/)
    * [GLFW](http://www.glfw.org/)
    * [Eigen3](http://http://eigen.tuxfamily.org/)
    * [OpenGL](https://www.opengl.org/)

### Environment Variables

None required so far.

### Tools

* [cmake](https://cmake.org/)

## Build Instructions

Make sure that libigl is somewhere the `FindLIBIGL.cmake` can find it. For a
list of locations check `./cmake/Modules/FindLIBIGL.cmake` and add your
location to this list, if it is not already included.

### OS X
Make sure the [Xcode Developer Tools](https://developer.apple.com/xcode/) are
installed. Using [brew](http://brew.sh/), the following will install needed
dependencies for OS X:

```
brew install glfw
brew install eigen
```

Once GLFW and Eigen3 are installed, the following will build the main program:
```
mkdir build
cd build
cmake ..
```

### Linux

### Windows


### Running

The render_views program loads an 
[OFF](http://www.geomview.org/docs/html/OFF.html) file and runs the libigl
viewer.

This is done by typing in the build directory
```
// ./bin/render_views <model_path> <output_directory> <sample_type>
//                    <num_samples> <width> <height>
//  model_path - path to the model file to render [.OFF]
//  output_directory - path to the directory for rendered views
//  sample_type - type of sampling to use:
//              - 0 : sample the vertices of an icosahedron
//              - 1 : sample the vertices of a cylinder with <num_samples>
//                    samples.
//              - 2 : sample a sphere uniformly at random using <num_samples>
//                    samples.
// num_samples - number of samples to use, only useful for sample_type = 1,2.
// width - output image width
// height - output image height
```
and currently a window is launched.


