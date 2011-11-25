#ifndef TOOLS_H
#define TOOLS_H

#include "cml/cml.h"
#include "opencv/cxcore.h"

cml::vector3d intersectLineTriangle(cml::vector3d X, cml::vector3d dir,
                                    cml::vector3d V0, cml::vector3d V1,
                                    cml::vector3d V2);

void write_png_file(char* file_name, IplImage* img);
IplImage* read_png_file(char* file_name);


#endif // TOOLS_H
