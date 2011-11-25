#include "camera.h"

#define SQR(x) (x*x)

/***************************************************************************************/

OGLCamera::OGLCamera(int width, int height, double alpha_u, double alpha_v,
		     double uc, double vc, double nearPlane, double farPlane,
		     vector3 offset_L, vector3 offset_R)
{
    this->width = width;
    this->height = height;
    this->nearPlane = nearPlane;
    this->farPlane = farPlane;
    if(uc==0) uc = width/2.0;
    if(vc==0) vc = height/2.0;
    this->uc = uc; this->vc = vc;
    this->alpha_u =alpha_u;
    this->alpha_v =alpha_v;

    this->offset_L = offset_L;
    this->offset_R = offset_R;

    //Init camera's pose with standart values
    resetPosition();
    resetRotation();

    //Init OGL viewport
    //initViewPort(width, height);
}

void OGLCamera::initViewPort(int w, int h)
{
    double scale_width = w / (double)this->width;
    double scale_height = h / (double)this->height;
    double uc_scaled = this->uc * scale_width;
    double vc_scaled = this->vc * scale_height;
    double alpha_u_scaled = this->alpha_u * scale_width;
    double alpha_v_scaled = this->alpha_v * scale_height;

    //Calculate Perspective parameters

    fov = 2 * atan(h/(2*alpha_v_scaled)) * 180/M_PI; //FOV in y direction in degrees See http://en.wikipedia.org/wiki/Angle_of_view
    aspect = (w/(float)h)*(alpha_v_scaled/alpha_u_scaled); //Aspect ratio.. alpha_u/alpha_v is usually near 1.0
    u0=uc_scaled-w/2.0;
    v0=vc_scaled-h/2.0;

    //Set Viewport
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport( u0, -v0, w, h );
    gluPerspective( fov, aspect, this->nearPlane, this->farPlane );
    glMatrixMode(GL_MODELVIEW);

}

void OGLCamera::updateFrames()
{
    matrix T;

    cml::matrix_set_x_basis_vector(head,right);
    cml::matrix_set_y_basis_vector(head,up);
    cml::matrix_set_z_basis_vector(head,-view);
    cml::matrix_set_translation(head,position);

    cml::matrix_set_x_basis_vector(headL,right);
    cml::matrix_set_y_basis_vector(headL,up);
    cml::matrix_set_z_basis_vector(headL,-view);

    cml::matrix_set_translation(headL,position);
    cml::matrix_translation(T,offset_L);
    headL*=T;

    cml::matrix_set_x_basis_vector(headR,right);
    cml::matrix_set_y_basis_vector(headR,up);
    cml::matrix_set_z_basis_vector(headR,-view);

    cml::matrix_set_translation(headR,position);
    cml::matrix_translation(T,offset_R);
    headR*=T;
}

void OGLCamera::move (vector3 dir)
{
    position = position + dir;

    updateFrames();
}

void OGLCamera::rotateLocalX (double angle)
{
    if(angle==0)
	return;

    view = cml::rotate_vector(view,right,angle);
    view = cml::normalize(view);
    up = cml::cross(right,view);

    updateFrames();
}

void OGLCamera::rotateLocalY (double angle)
{
    if(angle==0)
	return;

    view = cml::rotate_vector(view,up,angle);
    view = cml::normalize(view);
    right = cml::cross(view,up);

    updateFrames();
}

void OGLCamera::rotateLocalZ (double angle)
{
    if(angle==0)
	return;

    up = cml::rotate_vector(up,view,angle);
    up = cml::normalize(up);
    right = cml::cross(view,up);

    updateFrames();
}

void OGLCamera::rotateGlobalX (double angle)
{
    if(angle==0)
	return;

    view = cml::rotate_vector(view,cml::vector3d(1,0,0),angle);
    view = cml::normalize(view);
    right = cml::rotate_vector(right,cml::vector3d(1,0,0),angle);
    right = cml::normalize(right);
    up = cml::cross(right,view);

    updateFrames();
}

void OGLCamera::rotateGlobalY (double angle)
{
    if(angle==0)
	return;

    view = cml::rotate_vector(view,cml::vector3d(0,1,0),angle);
    view = cml::normalize(view);
    right = cml::rotate_vector(right,cml::vector3d(0,1,0),angle);
    right = cml::normalize(right);
    up = cml::cross(right,view);

    updateFrames();
}

void OGLCamera::rotateGlobalZ (double angle)
{
    if(angle==0)
	return;

    view = cml::rotate_vector(view,cml::vector3d(0,0,1),angle);
    view = cml::normalize(view);
    right = cml::rotate_vector(right,cml::vector3d(0,0,1),angle);
    right = cml::normalize(right);
    up = cml::cross(right,view);

    updateFrames();
}


void OGLCamera::resetRotation()
{
    view = vector3( 0.0, 0.0, -1.0);
    right = vector3 (1.0, 0.0, 0.0);
    up = vector3 (0.0, 1.0, 0.0);

    cml::matrix_set_x_basis_vector(head,right);
    cml::matrix_set_y_basis_vector(head,up);
    cml::matrix_set_z_basis_vector(head,view);
    cml::matrix_set_translation(head,position);

    updateFrames();
}

void OGLCamera::resetPosition()
{
    position = vector3 (0.0, 0.0, 0.0);
    updateFrames();
}


void OGLCamera::render( int camera )
{
    vector3 cam_position; //Position of actual camera (mounting position+offset for right or left)
    vector3 u;
    vector3 r;
    vector3 v;
    matrix lookAt;

    updateFrames();

    if(camera == 0)	//Left camera (or default if mono)
    {
	cam_position = cml::matrix_get_translation(headL);
	cml::matrix_get_basis_vectors(headL,r,u,v);
	cml::matrix_look_at_RH(lookAt,cam_position,cam_position-v,u);
	glLoadMatrixd(lookAt.data());
    }
    if(camera == 1)
    {
	cam_position = cml::matrix_get_translation(headR);
	cml::matrix_get_basis_vectors(headR,r,u,v);
	cml::matrix_look_at_RH(lookAt,cam_position,cam_position-v,u);
	glLoadMatrixd(lookAt.data());
    }
}

void OGLCamera::moveForward( double distance )
{
    position = position + (view*distance);
    updateFrames();
}

void OGLCamera::strafeRight ( double distance )
{
    position = position + (right*distance);

    updateFrames();
}

void OGLCamera::moveUpward( double distance )
{
    position = position + (up*distance);

    updateFrames();
}

void OGLCamera::setPanTilt (double pan, double tilt )
{
    view = vector3( 0.0, 0.0, -1.0);
    right = vector3 (1.0, 0.0, 0.0);
    up = vector3 (0.0, 1.0, 0.0);

    this->rotateLocalY(pan);
    this->rotateLocalX(tilt);
}

void OGLCamera::setCameraPose(cml::matrix44d_c P)
{
    //UNTESTED!!!!

    cml::matrix_get_basis_vectors(P,right,up,view);
    view = -view;
    this->position = cml::matrix_get_translation(P);
    updateFrames();
}
