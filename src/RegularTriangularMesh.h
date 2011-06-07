/*
 * TriangularMesh.h
 *
 *  Created on: Nov 13, 2009
 *      Author: malte
 */

#ifndef TRIANGULARMESH_H_
#define TRIANGULARMESH_H_

#include "cml/cml.h"
#include "dataTypes.h"
#include <vector>
#include <QtOpenGL>
#include <QList>
#include <iostream>
#include "drawingRoutines.h"
#include "face.h"
#include "anchor.h"
#include "point.h"
#include "fitting.h"

#define Vector3 cml::vector3d
#define Vector4 cml::vector4d
#define Matrix44 cml::matrix44d_c

//#define MAX_NX 67108863 // = 4GB == 4294967296 Bytes / sizeof(Point)
#define MAX_NX 15000000
#define MAX_POINTS_BUFFER 1000000 //for add points thread

#define SERVER_ADDRESS "127.0.0.1"
#define SERVER_PORT 50000

namespace RTM{
    class RegularTriangularMesh;


    class AddPointsThread : public QThread
    {
    public:
	AddPointsThread(RTM::RegularTriangularMesh *mesh);
	~AddPointsThread();
	void run();
	bool stop();
	bool appendPoints(QList<Point1> points);
        bool appendPoints(QString point_file);
    private:
	QMutex *pointsToAddMutex;
	QList<Point1> pointsToAdd;
	bool loop;
	RTM::RegularTriangularMesh *mesh;
        QTime t1;
        QTime t2;
    };

    class RegularTriangularMesh : public QObject
    {
	Q_OBJECT
    public:
	RegularTriangularMesh();
	~RegularTriangularMesh();

	void setU( double x, double y, double z, double len=0 );
	void setV( double x, double y, double z, double len=0 );
	void setNU( int n_u );
	void setNV( int n_v );
	void setOrigin( double x, double y, double z );

	int getNU();
	int getNV();
	double getLenU();
	double getLenV();
	int getNK();
	int getNL();
	int getNX();
        inline Vector3 getU(){return u;}
        inline Vector3 getV(){return v;}
	Anchor* getA();
	inline int getNA(){return nA;}

	void setAnchorHeight( int i, int j, double h );
	void resetAnchorHeights();

	int addPointWorldCoordinates( double x, double y, double z, double c=1.0 );
	int addPointModelCoordinates( double x, double y, double z, double c=1.0 );
	void addPointCloud(QList<Point1> points);
        void addPointCloud(QString filename);

        double calculateDistanceErrorAtAnchor(RTM::Anchor* a, FITTING_STRATEGY FS, bool globalError=false);
        void calculateDistanceErrorAtMesh(FITTING_STRATEGY FS);
        double calculateSmoothnessErrorAtAnchor(RTM::Anchor* a, bool globalError=false, bool absoluteError=false);
        void calculateSmoothnessErrorAtMesh(bool absoluteError);
        void calculateTotalErrorAtMesh(FITTING_STRATEGY FS,double alpha);

	Face* getFace(int k, int l);
	Face* getFace(int id);
	Anchor* getAnchor(int u, int v);

	//adjustColor:: 0:co changes, 1:take X[i].c, 2:take X[i].Xm[2] (height)
	void draw(int adjustColor=0);
	void drawOrigin();
	void drawNormals();
	void drawPoints();
	void highlightFan(int u=-1, int v=-1);
	void highlightFace(int id=-1);

	Vector4 convertWorldToModel(Vector4 Xw);
	Vector3 convertWorldToModel(Vector3 Xw);
	Vector4 convertModelToWorld(Vector4 Xp);
	Vector3 convertModelToWorld(Vector3 Xp);

	void mapModelToFace(Vector4 Xp, int *k, int *l);
	void mapWorldToFace(Vector4 Xw, int *k, int *l);
	int mapModelToFaceIndex(Vector4 Xp );
	int mapWorldToFaceIndex(Vector4 Xw );

	Face *F;	//Contains all the Faces and the world points assigned to the face. Also links to the anchors supporting the face
	QMutex F_mutex;
	int nF;
	int n_k;	//number of faces along u: n_k = 2*(n_u-1)
	int n_l;	//number of faces along v: n_l = n_v-1

	//Statistics
        double max_std_dev;
        double mean_std_dev;
	double var;
	int n_occupied;
	float occupied_ratio;
	double averageFaceOccupancy;
	double averageAnchorOccupancy;
        void calculateStatistics(bool measureOnlyCurrentFittingDistances=true);
	float divAverageAnchorOccupancy;
	float divAverageFaceOccupancy;
	float averageAnchorHeight;

        double zErrorSquared[4];
        double oErrorSquared[4];
        double zErrorAbs[4];
        double oErrorAbs[4];
        double smoothSquared[4];
        double smoothAbs[4];
        double totalSquaredZError[2];
        double totalSquaredOError[2];
        double totalAbsZError[2];
        double totalAbsOError[2];

    public slots:
	void update();
	void fitToPoints(FITTING_STRATEGY FS, double alpha=0.0);

    private:
	void createA();
	void createF();
	void createW2MP();

	int addPoint(Vector3 Xw, Vector3 Xm, double c=1.0);

	Vector3 u; //Represents the u axis in full length
	Vector3 v; //Represents the v axis in full length
	Vector3 dir_u; //Direction of u: u / ||u||
	Vector3 dir_v; //Direction of v: v / ||v||
	Vector3 dir_h; //Direction of h: ||dir_u x dir_v||
	Matrix44 Mp; //The Frame representing the origin of the Model-Plane
	Vector3 O; //Represents the origin of the Model-Plane in World-Coordinates
	Matrix44 W2MP; //The transformation matrix to convert from World Frame to Model-Plane-Frame
	int n_u; //Number of vertices along u
	int n_v; //Number of vertices along v
	double len_u; //Length of u: ||u||
	double len_v; //Length of v: ||v||
	double delta_u; //Distance between vertices along u: len_u / (n_u-1)
	double delta_v; //Distance between vertices along v: len_v / (n_v-1)

	Anchor *A; 	//Contains all anchors of the mesh with their elevations and its corresponding faces
	QMutex A_mutex;
	int nA;

	QMutex X_mutex;
	Point X[MAX_NX];
	//Vector3 X_w[MAX_NX];
	//Vector3 X_m[MAX_NX];
	double conf_X[MAX_NX];//Confidence value for each point
	int n_x;

	Vector4 tmp;

    public:
	Fitting::FitThread *fitthread;
	AddPointsThread *addPointsThread;
	QMutex basicsMutex;

    private slots:

    };
} //namespace RTM


#endif /* TRIANGULARMESH_H_ */
