/*
 * TriangularMesh.h
 *
 *  Created on: Nov 13, 2009
 *      Author: malte
 */

#ifndef TRIANGULARMESH_H_
#define TRIANGULARMESH_H_

#include <polygonnet/DataTypes.h>
#include <vector>
#include <string>
#include <QtOpenGL>
#include <QList>
#include <iostream>
//#include "drawingRoutines.h"
#include <polygonnet/Face.h>
#include <polygonnet/Anchor.h>
#include <polygonnet/Point.h>
#include <polygonnet/Fitting.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <base/eigen.h>

//#define MAX_NX 67108863 // = 4GB == 4294967296 Bytes / sizeof(Point)
#define MAX_NX 15000000
#define MAX_POINTS_BUFFER 1000000 //for add points thread

#define SERVER_ADDRESS "127.0.0.1"
#define SERVER_PORT 50000

#define HEIGHTMAPSCALE 1 // real height in meter 

namespace RTM{
    typedef Eigen::Matrix<double, 4, 1, Eigen::DontAlign>     Vector4d;
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
        inline base::Vector3d getU(){return u;}
        inline base::Vector3d getV(){return v;}
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

	RTM::Vector4d convertWorldToModel(RTM::Vector4d Xw);
	base::Vector3d convertWorldToModel(base::Vector3d Xw);
	RTM::Vector4d convertModelToWorld(RTM::Vector4d Xp);
	base::Vector3d convertModelToWorld(base::Vector3d Xp);
        
	void mapModelToFace(base::Vector3d Xp, int *k, int *l);
	void mapWorldToFace(base::Vector3d Xw, int *k, int *l);
	int mapModelToFaceIndex(base::Vector3d Xp );
	int mapWorldToFaceIndex(base::Vector3d Xw );
	void exportHeightmap(int nu, int nv, std::string filename);
	void importHeightmap(std::string filename);

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

	int addPoint(base::Vector3d Xw, base::Vector3d Xm, double c=1.0);

	base::Vector3d u; //Represents the u axis in full length
	base::Vector3d v; //Represents the v axis in full length
	base::Vector3d dir_u; //Direction of u: u / ||u||
	base::Vector3d dir_v; //Direction of v: v / ||v||
	base::Vector3d dir_h; //Direction of h: ||dir_u x dir_v||
	base::Matrix4d Mp; //The Frame representing the origin of the Model-Plane
	base::Vector3d O; //Represents the origin of the Model-Plane in World-Coordinates
	base::Matrix4d W2MP; //The transformation matrix to convert from World Frame to Model-Plane-Frame
	int n_u; //Number of vertices along u
	int n_v; //Number of vertices along v
	double len_u; //Length of u: ||u||
	double len_v; //Length of v: ||v||
	double delta_u; //Distance between vertices along u: len_u / (n_u-1)
	double delta_v; //Distance between vertices along v: len_v / (n_v-1)

        int max_point_size_in_face; // Maximum points one facede can contain    

	Anchor *A; 	//Contains all anchors of the mesh with their elevations and its corresponding faces
	QMutex A_mutex;
	int nA;

	QMutex X_mutex;
	Point X[MAX_NX];
	//Vector3 X_w[MAX_NX];
	//Vector3 X_m[MAX_NX];
	double conf_X[MAX_NX];//Confidence value for each point
	int n_x;

    public:
	Fitting::FitThread *fitthread;
	AddPointsThread *addPointsThread;
	QMutex basicsMutex;

    private slots:

    };
} //namespace RTM


#endif /* TRIANGULARMESH_H_ */
