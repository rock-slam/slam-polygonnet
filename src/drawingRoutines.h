#ifndef DRAWINGROUTINES_H
#define DRAWINGROUTINES_H

#include "cml/cml.h"
#include <QtOpenGL>
#include "GL/glut.h"

#define Vector3 cml::vector3d
#define Vector4 cml::vector4d
#define Matrix44 cml::matrix44d_c

void drawFrame(Matrix44 f, double length=2.0, double width=1.0);

#endif // DRAWINGROUTINES_H
