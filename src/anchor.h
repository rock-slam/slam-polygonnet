#ifndef ANCHOR_H
#define ANCHOR_H

#include "RegularTriangularMesh.h"
#include "face.h"
#include "base/eigen.h"

namespace RTM
{
    class Face;
    class RegularTriangularMesh;

    class Anchor
    {
    public:
	Anchor();

	base::Vector3d v0_w;	//Vertex in the World Plane (World Coordinates)
	base::Vector3d v_w;	//Vertex with elevation (World Coordinates)
	base::Vector3d v0_m;	//Vertex in the World Plane (World Coordinates)
	base::Vector3d v_m;	//Vertex with elevation (World Coordinates)
	base::Vector3d *dir_h;	//Direction for h
	base::Vector3d n; //Surface normal at anchor
        std::vector<Face*> F; //Faces connected to this anchor

	RTM::RegularTriangularMesh *parent_mesh;
        void setUpdated();
        void unsetUpdated();
        bool getUpdated();

        int idx;
	int u;
	int v;

        QMutex updatedMutex;

        void setHeight( double h );
        inline double getH(){return h;}

	void drawNormal(double len);
	int getU();
	int getV();

	//Statistics
	void calculateStatistics();
	int nX;
        double relativeOccupancy;
	double meanC;
        float relativeHeight;
        double zErrorSquared;
        double oErrorSquared;
        double zErrorAbs;
        double oErrorAbs;
        double meanZErrorSquared;
        double meanOErrorSquared;
        double meanZErrorAbs;
        double meanOErrorAbs;
        double smoothErrorAbs;
        double smoothErrorSquared;

    private:
        double h;	//Elevation: h = ||v-v0||
        bool updated;
    };
}
#endif // ANCHOR_H
