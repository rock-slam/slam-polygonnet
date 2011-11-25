#include "glwidget.h"
#include "math.h"
#include <QtGui>
#include <QtOpenGL>

GLWidget::GLWidget(QWidget *parent)
{
	this->cam = new OGLCamera(640,480,300,300,0,0,0.2,1000);
	this->mesh = 0;// new RTM::RegularTriangularMesh();
	grabMouse();
	rx=ry=rz=0;
	this->highlightFan_u=-1;
	this->highlightFan_v=-1;
	this->highlightFace_id=-1;
	this->mouseLook=false;

	this->drawMesh=true;
	this->drawNormals=false;
	this->drawPointCloud=true;
	this->drawFaces=false;
	this->light=false;
	this->drawOccupancy = false;
	this->drawStdDev = false;
	this->frame = 0;
        this->drawError=false;
        this->drawSmoothError=false;

	this->step=0.5;

	repaintTimer = new QTimer(this);
	connect(repaintTimer, SIGNAL(timeout()), this, SLOT(updateGL()));
        repaintTimer->start(200);
}

void GLWidget::focusInEvent( QFocusEvent * event )
{
    grabKeyboard();
}

void GLWidget::focusOutEvent( QFocusEvent * event )
{
    releaseKeyboard();
}

GLWidget::~GLWidget()
{
    repaintTimer->stop();
    delete repaintTimer;
}

void GLWidget::paintGL()
{
    this->mesh->basicsMutex.lock();

    //Start timer
    fpsTimer.start();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1,1,1,1);
    glLoadIdentity();

    cam->render();

    if(light)
    {
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	//glEnable(GL_LIGHT1);
	//glEnable(GL_LIGHT2);

	// Create light components
	GLfloat ambientLight[] = { 0.f, 0.f, 0.f, 1.0f };
	GLfloat diffuseLight[] = { 0.65f, .65f, .65f, 1.f };
	GLfloat specularLight[] = { .85f, .85f, .85f, 1.0f };
	GLfloat position[] = { -0.0f, 100.f, 500.0f, 100.0f };
	GLfloat lightDirection[] = {0.0f,-0.2,-0.8};

	// Assign created components to GL_LIGHT0
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
	glLightfv(GL_LIGHT0, GL_POSITION, position);
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, lightDirection);
	glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 50.0);
	glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.005);

	// LIGHT1
	//
	// Create light components
	//position[0]=-10.0f; position[0]=5.0f; position[0]=-0.0f; position[0]=1.0f;

	// Assign created components to GL_LIGHT0
	/*glLightfv(GL_LIGHT1, GL_AMBIENT, ambientLight);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuseLight);
	glLightfv(GL_LIGHT1, GL_SPECULAR, specularLight);
	glLightfv(GL_LIGHT1, GL_POSITION, position);

	// LIGHT2
	//
	// Create light components
	position[0]=10.0f; position[0]=5.0f; position[0]=-0.0f; position[0]=1.0f;

	// Assign created components to GL_LIGHT0
	glLightfv(GL_LIGHT1, GL_AMBIENT, ambientLight);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuseLight);
	glLightfv(GL_LIGHT1, GL_SPECULAR, specularLight);
	glLightfv(GL_LIGHT1, GL_POSITION, position);*/
	glColorMaterial ( GL_FRONT_AND_BACK, GL_EMISSION ) ;
	glEnable ( GL_COLOR_MATERIAL ) ;
    }
    else
	glDisable ( GL_LIGHTING ) ;

    if(drawFaces)
    {
	glColor3f(.25f,0.5f,.1f);
	//glMaterialf(GL_FRONT_AND_BACK,GL_EMISSION,
	glPolygonMode(GL_FRONT, GL_FILL);

	mesh->draw(1);
    }
    
    if(drawOccupancy)
    {
        glPolygonMode(GL_FRONT, GL_FILL);
	for(int i=0; i<mesh->nF; i++)
	{
            float c = (mesh->F[i].relativeOccupancy);

            if(c==0)
                mesh->F[i].draw(1,0,0,1);
            else
                mesh->F[i].draw(c-1,c,c-1,1);
	}
    }

    if(drawStdDev)
    {
        glPolygonMode(GL_FRONT, GL_FILL);
	for(int i=0; i<mesh->nF; i++)
	{
            float c = (mesh->F[i].mean_std_dev);

            mesh->F[i].draw(c-1,c-1,c,1);
	}
    }

    glDisable ( GL_LIGHTING ) ;
    if(drawMesh)
    {
        glColor3f(0.f,0.f,0.f);
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
        mesh->draw(2);
	//mesh->highlightFan(this->highlightFan_u,this->highlightFan_v);
        mesh->highlightFace(this->highlightFace_id);
        //mesh->drawOrigin();
    }
    if(drawUpdated)
    {
        glPointSize(2);
        glBegin(GL_POINTS);
        for(int i=0; i<this->mesh->getNA(); i++)
        {
            glColor3f(this->mesh->getA()[i].getUpdated(),!this->mesh->getA()[i].getUpdated(),0);
            glVertex3d(this->mesh->getA()[i].v_w[0],this->mesh->getA()[i].v_w[1],this->mesh->getA()[i].v_w[2]);
        }
        glEnd();
        glPointSize(1);
    }

    if(drawNormals)
	mesh->drawNormals();

    if(drawPointCloud)
        mesh->drawPoints();

    if(drawError || drawSmoothError)
    {
        glPointSize(3);
        glBegin(GL_POINTS);
        for(int i=0; i<this->mesh->getNA(); i++)
        {
            float c=0;
            if(drawError==1)
                c+=this->mesh->getA()[i].meanZErrorSquared/this->mesh->zErrorSquared[1];
            if(drawError==2)
                c+=this->mesh->getA()[i].meanOErrorSquared/this->mesh->oErrorSquared[1];
            if(drawError==3)
                c+=this->mesh->getA()[i].meanZErrorAbs/this->mesh->zErrorAbs[1];
            if(drawError==4)
                c+=this->mesh->getA()[i].meanOErrorAbs/this->mesh->oErrorAbs[1];
            if(drawSmoothError==1)
                c+=this->mesh->fitthread->alpha*(this->mesh->getA()[i].smoothErrorSquared/(this->mesh->smoothSquared[1]));
            if(drawSmoothError==2)
                c+=this->mesh->fitthread->alpha*(this->mesh->getA()[i].smoothErrorAbs/(this->mesh->smoothAbs[1]));

            if(c<=0.01)
                glColor3f(0,c*100,0);
            else if(c<=0.1 && c>0.01)
                glColor3f(c*10-1,1,0);
            else if(c>0.1)
                glColor3f(1,1-c,c);

            /*if(c<1)
                glColor3f(0,c,0);
            if(c>1)
                glColor3f(1-c,c,0);
            if(c>2)
                glColor3f(1,2-c,0);*/

            glVertex3dv(this->mesh->getA()[i].v_w.data());
        }
        glEnd();
        glPointSize(1);
    }

    //if(frame)
	//drawFrame(*frame,0.2,5.0);

    //drawFrame(cml::identity_4x4(),0.5,3);

    //Stop timer
    t = fpsTimer.elapsed();

    this->mesh->basicsMutex.unlock();
}

void GLWidget::drawOverlay()
{

}

void GLWidget::initializeGL()
{
    glClearColor(0,0,0,1);
    //glShadeModel(GL_FLAT);
    glShadeModel(GL_FLAT);
    glEnable(GL_DEPTH_TEST);
    glFrontFace(GL_CW);
    glColor3f(1.f,1.f,1.f);
    //glEnable (GL_BLEND);
    //glBlendFunc(GL_SRC_ALPHA,GL_ONE);
}

QSize GLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize GLWidget::sizeHint() const
{
    return QSize(400,400);
}

void GLWidget::resizeGL(int width, int height)
{
    this->cam->initViewPort(width,height);
}

void GLWidget::keyPressEvent(QKeyEvent *e)
{
    //QMessageBox::information(this,QString("KEY"),(QString)e->key());
    switch(e->key())
    {
    case Qt::Key_W:
    	this->cam->moveForward(this->step);
    	break;
    case Qt::Key_S:
    	this->cam->moveForward(-this->step);
    	break;
    case Qt::Key_A:
    	this->cam->strafeRight(-this->step/1.75);
    	break;
    case Qt::Key_D:
    	this->cam->strafeRight(this->step/1.75);
    	break;
    case Qt::Key_Q:
    	this->cam->moveUpward(this->step);
	break;
    case Qt::Key_Y:
	this->cam->moveUpward(-this->step);
	break;
    case Qt::Key_1:
	this->cam->rotateLocalZ(deg2rad(5));
	break;
    case Qt::Key_3:
	this->cam->rotateLocalZ(deg2rad(-5));
	break;
    /*case Qt::Key_Space:
	this->cam->resetPosition();
	this->cam->resetRotation();
	break;*/
    //case Qt::Key_Escape:
    	//this->parentWidget()->close();
    	//break;
    case Qt::Key_Shift:
	this->mouseLook = !this->mouseLook;
	if(this->mouseLook)
	{
	    this->lastPoint = this->cursor().pos();
	    setCursor( QCursor( Qt::BlankCursor ) );
	}
	else
	    setCursor( QCursor( Qt::ArrowCursor ) );


	this->setMouseTracking(mouseLook);
	break;
    default:
	e->ignore();
	break;
    }
    updateGL();
}

void GLWidget::mouseMoveEvent(QMouseEvent *e)
{
	if ( (e->buttons() & Qt::LeftButton) || this->mouseLook )
	{
		dif_x = lastPoint.x() - e->globalX();
		dif_y = lastPoint.y() - e->globalY();

		cam->rotateLocalY(1.5f*(dif_x/640.0f));
		cam->rotateLocalX(1.5f*(dif_y/480.0f));
		
		updateGL();
	}
	lastPoint = e->globalPos();
}

void GLWidget::leaveEvent ( QEvent * event )
{
    if(this->mouseLook)
    {
	int x = this->cursor().pos().x();
	int y = this->cursor().pos().y();
	int widget_x = this->window()->geometry().x()+this->pos().x();
	int widget_y = this->window()->geometry().y()+this->pos().y();
	int widget_width = this->width();
	int widget_height=this->height();
	QPoint newPos;
	if( x<=widget_x)
	{
	    newPos = QPoint(widget_x+widget_width-2,this->cursor().pos().y());
	    this->lastPoint = newPos+QPoint(dif_x,0);
	}
	else if( y<=widget_y )
	{
	    newPos = QPoint(this->cursor().pos().x(), widget_y+widget_height-2);
	    this->lastPoint = newPos+QPoint(0,dif_y);
	}
	else if( x>= widget_x+widget_width )
	{
	    newPos = QPoint(widget_x+2,this->cursor().pos().y());
	    this->lastPoint = newPos+QPoint(dif_x,0);
	}
	else if( y>= widget_y+widget_height )
	{
	    newPos = QPoint(this->cursor().pos().x(), widget_y+2);
	    this->lastPoint = newPos+QPoint(0,dif_y);
	}

	this->cursor().setPos( newPos );
    }
}


void GLWidget::mousePressEvent(QMouseEvent *e)
{
	if (e->button() == Qt::LeftButton)
	    lastPoint = e->globalPos();
}

void GLWidget::highlightFan(int u, int v)
{
    this->highlightFan_u = u;
    this->highlightFan_v = v;
}

void GLWidget::highlightFace(int id)
{
    this->highlightFace_id = id;
}

void GLWidget::setCameraPose(double tx, double ty, double tz, double rx, double ry, double rz, bool isRadian)
{
    cam->resetRotation();
    cam->resetPosition();

    if(!isRadian)
    {
	if(rx) rx = deg2rad(rx);
	if(ry) ry = deg2rad(ry);
	if(rz) rz = deg2rad(rz);
    }

    if(rx) this->cam->rotateGlobalX(rx);
    if(ry) this->cam->rotateGlobalY(ry);
    if(rz) this->cam->rotateGlobalZ(rz);

    this->cam->move(cml::vector3d(tx,ty,tz));
}
