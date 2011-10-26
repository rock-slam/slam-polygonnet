#ifndef DATATYPES_H
#define DATATYPES_H

namespace RTM{

    enum FITTING_STRATEGY
    {
	FS_SQUARED_Z_DISTANCE = 0,
	FS_SQUARED_ORTHOGONAL_DISTANCE = 1,
	FS_SIMPLE_FIT = 2,
	FS_ABSOLUTE_Z_DISTANCE = 3,
	FS_ABSOLUTE_ORTHOGONAL_DISTANCE = 4,
	FS_SQUARED_Z_DISTANCE_BRENT = 5
    };

    class RegularTriangularMesh;
}

namespace Fitting{
    class FitThread;
}

#endif // DATATYPES_H
