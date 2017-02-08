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

The main program loads an [OFF](http://www.geomview.org/docs/html/OFF.html)
file and runs the libigl viewer.

This is done by typing in the build directory
```
bin/main path/to/file/mesh.off
```
and currently a window is launched.


