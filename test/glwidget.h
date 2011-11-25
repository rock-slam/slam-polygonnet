#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include "GL/glut.h"
#include "camera.h"
#include "RegularTriangularMesh.h"

class GLWidget : public QGLWidget
{
Q_OBJECT

public:
    GLWidget(QWidget *parent=0);
    ~GLWidget();
    QSize sizeHint() const;
    QSize minimumSizeHint() const;

    RTM::RegularTriangularMesh *mesh;

    void highlightFan(int u, int v);
    void highlightFace(int id);

    bool drawMesh,drawNormals,drawPointCloud,drawFaces,light, drawOccupancy, drawStdDev, drawUpdated;
    int drawError; //0: Do not draw, 1: Sq z-Dist, 2: Sq o-Dist, 3: Abs. z, 4: Abs. o
    int drawSmoothError; //0: Do not draw, 1: Sq, 2: Abs
    cml::matrix44d_c *frame;

    inline int getT(){return t;}

    OGLCamera *cam;

protected:
    void drawOverlay();
    void paintGL();
    void initializeGL();
    void resizeGL(int w, int h);
    int rx,ry,rz;
    void keyPressEvent(QKeyEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void mousePressEvent(QMouseEvent *e);
    void leaveEvent ( QEvent * event );
    void focusInEvent( QFocusEvent * event );
    void focusOutEvent( QFocusEvent * event );
    QPoint lastPoint;

    int highlightFan_u;
    int highlightFan_v;
    int highlightFace_id;
    bool mouseLook;
    int dif_x;
    int dif_y;

private:
    QTimer *repaintTimer;
    QTime fpsTimer;
    int t;
    float step;

public slots:
    inline void resetCameraPosition() { cam->resetPosition();updateGL(); }
    inline void resetCameraRotation() { cam->resetRotation(); updateGL(); }
    inline void setStepDistance(double step){this->step = step;}
    void setCameraPose(double tx,double ty,double tz, double rx,double ry,double rz, bool isRadian=false);
    inline void setDrawMesh(bool b){this->drawMesh=b;}
    inline void setDrawFaces(bool b){this->drawFaces=b;}
    inline void setDrawNormals(bool b){this->drawNormals=b;}
    inline void setDrawOccupancy(bool b){this->drawOccupancy=b;}
    inline void setDrawPointCloud(bool b){this->drawPointCloud=b;}
    inline void setDrawStdDev(bool b){this->drawStdDev=b;}
    inline void setDrawLight(bool b){this->light=b;}

signals:

};

#endif // GLWIDGET_H
