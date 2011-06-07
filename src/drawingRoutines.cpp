#include "drawingRoutines.h"

void print_bitmap_string(void* font, char* s)
{
    if (s && strlen(s)) {
	while (*s) {
	    glutBitmapCharacter(font, *s);
	    s++;
	}
    }
    return;
}

void drawFrame(Matrix44 f, double length, double width)
{
    Vector3 x,y,z,tmp;
    Vector3 p = cml::matrix_get_translation(f);
    cml::matrix_get_basis_vectors(f,x,y,z);

    glPushAttrib(GL_LINE_BIT);

    glLineWidth(width);
    glBegin(GL_LINES);

    glColor3f(1,0,0);
    glVertex3dv(p.data());
    tmp=x*length;
    tmp=p+tmp;
    glVertex3dv(tmp.data());

    glColor3f(0,1,0);
    glVertex3dv(p.data());
    tmp=y*length;
    tmp=p+tmp;
    glVertex3dv(tmp.data());

    glColor3f(0,1,1);
    glVertex3dv(p.data());
    tmp=z*length;
    tmp=p+tmp;
    glVertex3dv(tmp.data());

    glEnd();

    glPopAttrib();
}


