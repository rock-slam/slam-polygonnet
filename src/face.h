#ifndef FACE_H
#define FACE_H

#include "RegularTriangularMesh.h"
#include "anchor.h"
#include "point.h"
#include <deque>
#include "base/eigen.h"
#include "base/samples/pointcloud.h"

namespace RTM
{
    class Anchor;
    class RegularTriangularMesh;

    class Face
    {
    public:
	Face();

	int addPoint(Point* x,int max_point_size_in_face);

        base::Vector3d getNormal(bool updateNormal=true, bool modelCoordinates=false);
	base::Vector3d getMean(bool updateMean=true);
	void draw(float r=1.0, float g=1.0, float b=1.0, float a=1.0);
	void drawNormal( float len=1.0f, bool update=true );

	int getK();
	int getL();

	RTM::Anchor *A_n;	//References to anchors in such a way like OpenGL treats triangle strips:
	RTM::Anchor *A_nPlus1;	//One triangle is defined for each vertex presented after the first two vertices.
	RTM::Anchor *A_nPlus2;	//For odd n, vertices n, n+1, and n+2 define triangle n. For even n, vertices n+1,
				//n, n+2 define triangle n. N-2 triangles are drawn.
	RTM::RegularTriangularMesh *parent_mesh;

	bool isOdd;
	std::deque <Point*> X;
	int nX;

	int id;
	int k;
	int l;

	//Statistiken über die Punkte
	double sum_height;
	double mean_height;
        double mean_var; // Varianz = sum( (height - mean_height)^2 ) / Anzahl... Mittlere Varianz der Punkte der Kachel
        double mean_std_dev; //Standardabweichung = sqrt(var).... Mittlere Staandardabweichung der Punkte der Kachel
        double max_std_dev; //Maximale abweichung vom Mittelwert = sqrt( max((height_i-mean_height)^2) )... Maximale Standardabweichung eines Punktes der Kachel
        double max_var;
        double sum_std_dev;
        double sum_var;

	void calculateStatistics();
	double height;
        float relativeHeight;
        QMutex statMutex;

        double relativeOccupancy;
	double meanC;

	//Geometrischer Kram
	base::Vector3d n;  //Normal on the face
	base::Vector3d M;  //Mean point on the triangle
    };
}

#endif // FACE_H
