#ifndef ANCHOR_H
#define ANCHOR_H

#include "RegularTriangularMesh.h"
#include "face.h"

namespace RTM
{
    class Face;
    class RegularTriangularMesh;

    class Anchor
    {
    public:
	Anchor();

	Vector3 v0_w;	//Vertex in the World Plane (World Coordinates)
	Vector3 v_w;	//Vertex with elevation (World Coordinates)
	Vector3 v0_m;	//Vertex in the World Plane (World Coordinates)
	Vector3 v_m;	//Vertex with elevation (World Coordinates)
	Vector3 *dir_h;	//Direction for h
	Vector3 n; //Surface normal at anchor
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
