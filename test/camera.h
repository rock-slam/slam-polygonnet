#ifndef CAMERA_H
#define CAMERA_H

/*#ifdef _WIN32
	#define DllExport   __declspec( dllexport )
#else
	#define DllExport
#endif*/

#include <GL/glut.h>		// Need to include it here because the GL* types are required
#include "cml/cml.h"

#define PI 3.1415926535897932384626433832795
#define PIdiv180 (PI/180.0)
#define deg2rad(d) (double)d*(PI/180.0)
#define rad2deg(r) (double)r*(180.0/PI)

typedef cml::vector3d vector3;
typedef cml::matrix44d_c matrix;


class /*DllExport*/ OGLCamera
{
private:
	
	vector3 view;
	vector3 right;
	vector3 up;

	vector3 target;

	int width,height;
	double fov,aspect;
	double nearPlane;
	double farPlane;
	double lambda;
	
	void updateFrames();
	
public:
	OGLCamera(int width, int height, double alpha_u, double alpha_v,
		double uc=0, double vc=0, double nearPlane=10.0, double farPlane=200.0,
		vector3 offset_L=vector3(0,0,0), vector3 offset_R=vector3(0,0,0));
	
	void render(int camera=0);
	void initViewPort(int w, int h);

	void move ( vector3 dir );
	void rotateLocalX ( double angle );
	void rotateLocalY ( double angle );
	void rotateLocalZ ( double angle );
	void rotateGlobalX ( double angle );
	void rotateGlobalY (double angle );
	void rotateGlobalZ ( double angle );
	void resetRotation();
	void setPanTilt( double pan, double tilt );

	void moveForward ( double distance );
	void moveUpward ( double distance );
	void strafeRight ( double distance );
	void resetPosition();
	void setCameraPose(cml::matrix44d_c P); //UNTESTED!!!

	vector3 offset_L;
	vector3 offset_R;
	matrix head;
	matrix headL;
	matrix headR;
	vector3 position;

	double alpha_u;
	double alpha_v;
	double uc,vc;

	double u0;
	double v0;

};

#endif /*CAMERA_H*/


