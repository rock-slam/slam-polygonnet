/*
 * TriangularMesh.cpp
 *
 *  Created on: Nov 13, 2009
 *      Author: malte
 */

#include "RegularTriangularMesh.h"

/*
 *
 * Constructors / Destructor
 *
 */

RTM::RegularTriangularMesh::RegularTriangularMesh()
{
    this->u = Vector3(3.2, 0.0, 0.0);
    this->v = Vector3(0.0, 0.0, -3.2);
    this->n_u = 30;
    this->n_v = 30;
    this->O = Vector3(-1.5, 0, 1.5);
    this->A = 0;
    this->F = 0;

    this->n_x=0;
    this->highlightFace(-1);
    this->highlightFan(-1,-1);

    this->max_std_dev=0;
    this->mean_std_dev=0;
    this->var=0;
    this->n_occupied=0;
    this->averageAnchorHeight=0;

    this->fitthread = 0;

    this->X_mutex.unlock();
    this->F_mutex.unlock();
    this->A_mutex.unlock();
    this->basicsMutex.unlock();

    this->addPointsThread = new RTM::AddPointsThread(this);

    update();
}

RTM::RegularTriangularMesh::~RegularTriangularMesh()
{
    if(this->addPointsThread)
    {
        if(this->addPointsThread->isRunning())
        {
            addPointsThread->stop();
            bool success = addPointsThread->wait(10000);
            if(!success)
            {
                addPointsThread->terminate();
                addPointsThread->wait(10000);
            }
        }
        delete addPointsThread;
    }

    if(this->fitthread)
    {
        fitthread->cancelFitting();
        bool success = fitthread->wait(10000);
        if(!success)
        {
            fitthread->terminate();
            fitthread->wait(10000);
        }
        delete fitthread;
    }
}

/*
 *
 * Private Methods
 *
 */

void RTM::RegularTriangularMesh::update()
{
    if(this->fitthread)
    {
        if(this->fitthread->isRunning())
        {
            std::clog << "WARNING: Fitting thread is currently running.. will try to quit."<<std::endl;
            this->fitthread->cancelFitting();
            bool finished = this->fitthread->wait(10000);
            std::clog << "quit fitting thread with success: "<<finished<<std::endl;
            if(!finished)
            {
                std::clog << "WARNING: Fitting thread is currently running.. will try to quit."<<std::endl;
                this->fitthread->terminate();
                finished = this->fitthread->wait(10000);
                std::clog << "terminated fitting thread with success: "<<finished<<std::endl;
            }
        }
        delete fitthread;
        fitthread = 0;
    }
    if(this->addPointsThread)
    {
        if(this->addPointsThread->isRunning())
        {
            std::clog << "WARNING: AddPoints thread is currently running.. will try to quit."<<std::endl;
            bool finished = this->addPointsThread->stop();
            this->addPointsThread->wait(10000);
            std::clog << "quit addponits thread with success: "<<finished<<std::endl;
            if(!finished)
            {
                std::clog << "WARNING: Addpoints thread is currently running.. will try to quit."<<std::endl;
                this->addPointsThread->terminate();
                finished = this->addPointsThread->wait(10000);
                std::clog << "terminated addPointsThread  with success: "<<finished<<std::endl;
            }
        }
        delete addPointsThread;
        addPointsThread=0;
    }

    this->basicsMutex.lock();

    this->dir_u = cml::normalize(u); //Direction of u: u / ||u||
    this->dir_v = cml::normalize(v); //Direction of v: v / ||v||
    this->dir_h = cml::normalize(cml::cross(this->u, this->v)); //Direction of h: ||u x v||

    this->Mp.set( dir_u[0], dir_v[0], dir_h[0], O[0],
                  dir_u[1], dir_v[1], dir_h[1], O[1],
                  dir_u[2], dir_v[2], dir_h[2], O[2],
                  0   ,    0    ,     0   ,   1 );

    this->len_u = cml::length(u); //Length of u: ||u||
    this->len_v = cml::length(v); //Length of v: ||v||

    this->delta_u = len_u / (n_u-1); //Distance between vertices along u: len_u / (n_u-1)
    this->delta_v = len_v / (n_v-1); //Distance between vertices along v: len_v / (n_v-1)

    this->n_k = 2*(n_u-1);
    this->n_l = n_v-1;

    this->n_x=0;
    this->highlightFace(-1);
    this->highlightFan(-1,-1);

    createA();
    createW2MP();

    this->basicsMutex.unlock();

    resetAnchorHeights();

    this->fitthread = new Fitting::FitThread(this,this->n_u,this->n_v);
}

void RTM::RegularTriangularMesh::createA()
{
    int _c=0;

    //Release old space if necessary
    if(this->A != 0)
        delete[] A;

    //Allocate space needed for A
    this->nA = this->n_u*this->n_v;
    this->A = new Anchor[this->nA];

    Vector3 _p;;// = this->O; //world coordinate of current anchor

    for(int j=0; j<this->n_v; j++)
    {
        for(int i=0; i<this->n_u; i++)
        {
            _p = this->O + ((i*this->delta_u)*this->dir_u) + ((j*this->delta_v)*this->dir_v);
            A[_c].v0_w = _p;
            A[_c].v_w = A[_c].v0_w; //No elevations, yet, so v=v0
            A[_c].v0_m = Vector3(i*this->delta_u,j*this->delta_v,0);
            A[_c].v_m = A[_c].v0_m; //No elevations, yet, so v=v0
            A[_c].dir_h = &this->dir_h;
            A[_c].idx = _c;
            A[_c].parent_mesh = this;
            A[_c].u = i;
            A[_c].v = j;
            _c++;
        }
    }

    createF();
}

void RTM::RegularTriangularMesh::createF()
{
    int _c=0;
    int _u=0;
    Anchor *_A_n;
    Anchor *_A_nPlus1;

    //Release old space if necessary
    if(this->F != 0)
        delete[] F;

    //Allocate space needed for F
    this->nF = this->n_k*this->n_l;
    this->F = new Face[this->nF];

    Anchor *a;
    float _tmp;

    //Link to anchors. Thereby is the OpenGL convention for Triangle strips used
    for(int j=0; j<this->n_l; j++)
    {
        _u=0;
        _A_n = &this->A[j*this->n_u + _u];
        _A_nPlus1 = &this->A[(j+1)*this->n_u];
        for(int i=0; i<this->n_k; i++)
        {
            F[_c].A_n = i==0 ? _A_n : F[_c-1].A_nPlus1;
            F[_c].A_nPlus1 = i==0 ? _A_nPlus1 : F[_c-1].A_nPlus2;

            _tmp=_c%2;	//Very strange tried without _tmp directly in if().. did not work properly
            if(!_tmp)	//odd triangle
            {
                ++_u;
                a = &this->A[j*this->n_u + _u];
                F[_c/*j*n_k+i*/].A_nPlus2 = a;
                F[_c].isOdd=false;
            }
            else	//even triangle
            {
                a =  &this->A[(j+1)*this->n_u + _u];
                F[_c/*j*n_k+i*/].A_nPlus2 = a;
                F[_c].isOdd=true;
            }

            //Set refernces from anchors to face..
            F[_c].A_n->F.push_back(&F[_c]);
            F[_c].A_nPlus1->F.push_back(&F[_c]);
            F[_c].A_nPlus2->F.push_back(&F[_c]);

            //.. and to parent mesh
            F[_c].parent_mesh = this;

            F[_c].id = _c;

            _c++;
        }
    }
}

void RTM::RegularTriangularMesh::createW2MP()
{
    this->W2MP = cml::inverse(this->Mp);
}

/*
 *
 * Public Methods
 *
 */

void RTM::RegularTriangularMesh::setU( double x, double y, double z, double len)
{
    if(len==0)
    {
        this->u = Vector3(x,y,z);
        this->dir_u = u;
        cml::normalize(dir_u);
    }
    else
    {
        this->dir_u = Vector3(x,y,z);
        cml::normalize(dir_u);
        this->u = dir_u*len;
    }
}

void RTM::RegularTriangularMesh::setV( double x, double y, double z, double len)
{
    if(len==0)
    {
        this->v = Vector3(x,y,z);
        this->dir_v = v;
        cml::normalize(dir_v);
    }
    else
    {
        this->dir_v = Vector3(x,y,z);
        cml::normalize(dir_v);
        this->v = dir_u*len;
    }
}

void RTM::RegularTriangularMesh::setNU( int n_u )
{
    this->n_u = n_u;
}

void RTM::RegularTriangularMesh::setNV( int n_v )
{
    this->n_v = n_v;
}

void RTM::RegularTriangularMesh::setOrigin( double x, double y, double z )
{
    this->O = Vector3(x,y,z);
}

void RTM::RegularTriangularMesh::setAnchorHeight( int i, int j, double h )
{
    int _idx = j*n_u+i;
    this->A[_idx].setHeight(h);
}

void RTM::RegularTriangularMesh::resetAnchorHeights()
{
    for(int i=0; i<this->n_u*n_v; i++)
    {
        A[i].setHeight(0);
    }
}


RTM::Face* RTM::RegularTriangularMesh::getFace(int k, int l)
{
    return &F[l*this->n_k+k];
}

RTM::Face* RTM::RegularTriangularMesh::getFace(int id)
{
    return &F[id];
}

RTM::Anchor* RTM::RegularTriangularMesh::getAnchor(int u, int v)
{
    return &A[v*this->n_u+u];
}

int RTM::RegularTriangularMesh::getNU()
{
    return this->n_u;
}
int RTM::RegularTriangularMesh::getNV()
{
    return this->n_v;
}
double RTM::RegularTriangularMesh::getLenU()
{
    return this->len_u;
}
double RTM::RegularTriangularMesh::getLenV()
{
    return this->len_v;
}
int RTM::RegularTriangularMesh::getNK()
{
    return this->n_k;
}
int RTM::RegularTriangularMesh::getNL()
{
    return this->n_l;
}
int RTM::RegularTriangularMesh::getNX()
{
    return this->n_x;
}
RTM::Anchor* RTM::RegularTriangularMesh::getA()
{
    return A;
}

/*
 * Parameter adjustColor:
 *   0 - keine Änderung
 *   1 - Mittlere Farbe der Punkte, die den Facetten des Ankers zugeordnet sind
 *   2 - Abhängig von Höhe
 */
void RTM::RegularTriangularMesh::draw(int adjustColor)
{    
    int _lineOffset=0;
    Vector3 _v;
    Vector3 _n;

    //if(this->F_mutex.tryLock(500))
    {
        for(int j=0; j<this->n_l; j++)
        {
            _lineOffset = j*n_k;

            glBegin(GL_TRIANGLE_STRIP);
            glNormal3dv( F[_lineOffset].A_n->n.data() );
            glVertex3dv( F[_lineOffset].A_n->v_w.data() );

            glNormal3dv( F[_lineOffset].A_nPlus1->v_w.data() );
            glVertex3dv( F[_lineOffset].A_nPlus1->v_w.data() );
            for(int i=0; i<this->n_k; i++)
            {
                if(adjustColor==1)
                    glColor3d(F[_lineOffset + i].meanC,F[_lineOffset + i].meanC,F[_lineOffset + i].meanC);
                if(adjustColor==2)
                {
                    float c=F[_lineOffset + i].height/1.65+0.05;
                    //glColor3d(c<=0 ? 1 : 1-c*1.2/*.05-F[_lineOffset + i].height/1.8+0.05*/,c<=0 ? 1 : 1-c*0.8,c<=0 ? 1 : 1-c*1.3/*1.05-F[_lineOffset + i].height/1.8*/);
                    glColor3f(1-c*1.15+0.1,1-c*0.8-0.05,1-c*1.15+0.1);
                }

                _v = this->F[_lineOffset + i].A_nPlus2->v_w;
                _n = this->F[_lineOffset + i].A_nPlus2->n;
                glNormal3dv( _n.data() );
                glVertex3dv( _v.data() );
            }
            glEnd();
        }
        //this->F_mutex.unlock();
    }
}

void RTM::RegularTriangularMesh::drawNormals()
{
    float len = (this->delta_u+this->delta_v)/4.0;

    glPushAttrib(GL_COLOR_BUFFER_BIT);
    glColor3f(0,.9f,1.f);
    glBegin(GL_LINES);
    //if(this->F_mutex.tryLock(500))
    {
        for(int i=0; i<this->n_k*this->n_l; i++)
            F[i].drawNormal(len,false);
        //this->F_mutex.unlock();
        for(int i=0; i<this->nA; i++)
        {
            A[i].drawNormal(len);
        }
    }

    glEnd();
    glPopAttrib();
}

void RTM::RegularTriangularMesh::drawOrigin()
{
    drawFrame(this->Mp,3.0,3.0);
}

void RTM::RegularTriangularMesh::drawPoints()
{
    glPushAttrib(GL_COLOR_BUFFER_BIT);
    glColor3f(1.f,0.0f,0.0f);
    glPushAttrib(GL_POINT_BIT);
    glPointSize(1);
    float c;

    glBegin(GL_POINTS);
    //if(this->X_mutex.tryLock(500))
    {
        for(int i=0; i<this->n_x; i++)
        {
            //glColor3f(X[i].c,X[i].c,X[i].c);
            glVertex3dv( X[i].Xw.data() );
        }
        //this->X_mutex.unlock();
    }
    glEnd();

    glPopAttrib();
    glPopAttrib();
}

void RTM::RegularTriangularMesh::highlightFan(int u, int v)
{
    if( u==-1 || v==-1 )
        return;

    float len = (this->delta_u+this->delta_v)/4.0;
    Anchor *a;
    a = getAnchor(u,v);

    glPushAttrib(GL_LINE_BIT);
    //if(this->F_mutex.tryLock(500))
    {
        for(size_t i=0; i<a->F.size(); i++)
        {
            glLineWidth(3.f);
            a->F[i]->draw( 0.4f, 0.2f, 0.9f );
            glLineWidth(1.f);
            a->F[i]->drawNormal( len );
        }
        //this->F_mutex.unlock();
    }
    glPopAttrib();
}

void RTM::RegularTriangularMesh::highlightFace(int id)
{
    if( id==-1 )
        return;

    glPushAttrib(GL_LINE_BIT);
    glLineWidth(3.f);
    //if(this->F_mutex.tryLock(500))
    {
        this->F[id].draw(0.8f,0.9f,0.f);
        //this->F_mutex.unlock();
    }
    glPopAttrib();
}

Vector4 RTM::RegularTriangularMesh::convertWorldToModel(Vector4 Xw)
{
    return this->W2MP*Xw;
}

Vector3 RTM::RegularTriangularMesh::convertWorldToModel(Vector3 Xw)
{
    Vector4 Xw4;
    Xw4[0] = Xw[0];
    Xw4[1] = Xw[1];
    Xw4[2] = Xw[2];
    Xw4[3]=1;

    Vector4 Xp4 = convertWorldToModel(Xw4);

    return Vector3(Xp4[0],Xp4[1],Xp4[2]);
}

Vector4 RTM::RegularTriangularMesh::convertModelToWorld(Vector4 Xp) //UNTESTED!!!
{
    return this->Mp*Xp;
}

Vector3 RTM::RegularTriangularMesh::convertModelToWorld(Vector3 Xp) //UNTESTED!!!!
{
    Vector4 Xp4;
    Xp4[0] = Xp[0];
    Xp4[1] = Xp[1];
    Xp4[2] = Xp[2];
    Xp4[3]=1;

    Vector4 Xw4 = convertModelToWorld(Xp4);

    return Vector3(Xw4[0],Xw4[1],Xw4[2]);
}

void RTM::RegularTriangularMesh::mapModelToFace(Vector4 Xp, int *k, int *l)
{
    *k = floor(Xp.data()[0])*2 + floor( Xp.data()[0]-floor(Xp.data()[0]) + Xp.data()[2]-floor(Xp.data()[2]) );
    *l = floor(Xp.data()[2]);
}

void RTM::RegularTriangularMesh::mapWorldToFace(Vector4 Xw, int *k, int *l)
{
    Vector4 Xp = convertWorldToModel(Xw);
    mapWorldToFace( Xw, k, l);
}

int RTM::RegularTriangularMesh::mapModelToFaceIndex(Vector4 Xp )
{
    double u = Xp[0];
    double v = Xp[1];

    if(u>=this->u.length() || u<0)
        return -1;
    if(v>=this->v.length() || v<0)
        return -1;

    double dk = u/this->delta_u;
    int ik = floor(dk);
    double rk = dk-ik;	//nachkommastellen
    double dl = v/this->delta_v;
    int il = floor(dl);
    double rl = dl - il;    //nachkommastellen

    int k = ik*2 + floor( rk + rl );
    int l = il;

    int idx = l*n_k+k;

    if( idx<0 || idx>=(this->n_k*this->n_l) )
        return -1;
    else
        return idx;
}

int RTM::RegularTriangularMesh::mapWorldToFaceIndex(Vector4 Xw )
{
    Vector4 Xp = convertWorldToModel(Xw);
    return mapModelToFaceIndex(Xp);
}

int RTM::RegularTriangularMesh::addPoint( Vector3 Xw, Vector3 Xm, double c )
{
    Vector4 Xm4 = Vector4(Xm[0],Xm[1],Xm[2],1);
    int idx=0;

    //Find index in F to corresponding face
    idx = this->mapModelToFaceIndex( Xm4 );
    if( idx < 0 )
    {
        //std::cerr << "Error: world point is not located in the World-Plane" << std::endl;
        return -1;
    }

    //Add point to mesh. Store it in X_w (world coordinates) and X_m (model coordinates)
    if(this->n_x < MAX_NX-1 )
    {
        //if(this->X_mutex.tryLock(500))
        {
            this->X[this->n_x].Xw = Xw;
            this->X[this->n_x].Xm = Xm;
            this->X[this->n_x].c = c;
            //this->X_mutex.unlock();
        }
    }
    else
    {
        std::cerr << "Error: maximum number of points (" << MAX_NX << ") already reached" << std::endl;
        return -2;
    }

    //Add reference to face
    //if(this->F_mutex.tryLock(500))
    {
        F[idx].addPoint( &this->X[this->n_x] );
        F[idx].A_n->setUpdated();
        F[idx].A_nPlus1->setUpdated();
        F[idx].A_nPlus2->setUpdated();
        //this->F_mutex.unlock();
    }

    //Increase number of current points
    this->n_x++;

    return idx;
}

int RTM::RegularTriangularMesh::addPointWorldCoordinates( double x, double y, double z, double c )
{
    Vector3 Xw = Vector3(x,y,z);
    Vector3 Xm = convertWorldToModel(Xw);

    return addPoint(Xw,Xm,c);
}

/*
 * THIS CAN BE OPTIMIZED! transformation is done twice. One PLane->World, and in addPointWolrdCorrdinates
 * again world->plane
 */
int RTM::RegularTriangularMesh::addPointModelCoordinates( double x, double y, double z, double c )
{
    Vector3 Xm = Vector3(x,y,z);
    Vector3 Xw = convertModelToWorld(Xm);

    return addPoint(Xw,Xm,c);
}

void RTM::RegularTriangularMesh::fitToPoints(FITTING_STRATEGY FS, double alpha)
{
    this->calculateStatistics();

    if(this->fitthread->isRunning())
        this->fitthread->iterations++;
    else
        this->fitthread->fit(0,FS,alpha);

    /*
    double newHeight=0;

    for(int j=0; j<this->n_v;j++)
    {
        for(int i=0; i<this->n_u;i++)
        {
            std::clog << "Adjusting anchor u,v: "<<i<<", "<<j<<std::endl;
            //
            // Calculate new height of anchor
            //
            switch(FS)
            {
            case FS_SQUARED_Z_DISTANCE:
                newHeight = Fitting::fitFanSquaredZDistance(&this->A[j*n_u+i], alpha);
                break;
            case FS_SIMPLE_FIT:
                newHeight = Fitting::verySimpleFitPlane(&this->A[j*n_u+i]);
                break;
            case FS_SQUARED_ORTHOGONAL_DISTANCE:
                newHeight = Fitting::fitFanSquaredOrthogonalDistance(&this->A[j*n_u+i], alpha);
                break;
            case FS_ABSOLUTE_ORTHOGONAL_DISTANCE:
                newHeight = Fitting::fitFanAbsoluteOrthogonalDistance(&this->A[j*n_u+i], alpha);
                break;
            case FS_ABSOLUTE_Z_DISTANCE:
                newHeight = Fitting::fitFanAbsoluteZDistance(&this->A[j*n_u+i], alpha);
                break;
            case FS_SQUARED_Z_DISTANCE_BRENT:
                newHeight = Fitting::fitFanSquaredZDistanceBrent(&this->A[j*n_u+i], alpha);
                break;
            default:
                break;
            }

            this->A[j*n_u+i].setHeight(newHeight);
           // Fitting::plotSmoothness(&this->A[j*n_u+i]);
        }
    }*/
}

void RTM::RegularTriangularMesh::addPointCloud(QList<Point1>points)
{
    if(!this->addPointsThread)
        addPointsThread = new RTM::AddPointsThread(this);

    if(!this->addPointsThread->isRunning())
    {
        this->addPointsThread->start();
    }
    this->addPointsThread->appendPoints(points);
}

void RTM::RegularTriangularMesh::addPointCloud(QString fileName)
{
    if(!this->addPointsThread)
        addPointsThread = new RTM::AddPointsThread(this);

    if(this->addPointsThread->isRunning())
    {
        this->addPointsThread->stop();
    }
    this->addPointsThread->appendPoints(fileName);

    addPointsThread->start();
}


/*
 * Return an array as follows:
 * [0]: Sum of errors at anchors
 * [1]: Mean error of all anchors
 * [2]: Minimal error
 * [3]: Maximal error
 * Return mean point distance at Anchor
 */
double RTM::RegularTriangularMesh::calculateDistanceErrorAtAnchor(RTM::Anchor* a, FITTING_STRATEGY FS, bool globalError)
{
    double e=0.0;
    Fitting::DistanceFunctionParams p(a,0,globalError);

    switch(FS)
    {
    case FS_SQUARED_Z_DISTANCE:
        p.absolute_error=false;
        e = Fitting::zDistance(a->getH(),&p);
        a->zErrorSquared = e;
        a->meanZErrorSquared = e/a->nX;
        break;
    case FS_SIMPLE_FIT:
    case FS_SQUARED_ORTHOGONAL_DISTANCE:
        p.absolute_error=false;
        e = Fitting::orthogonalDistance(a->getH(),&p);
        a->oErrorSquared = e;
        a->meanOErrorSquared = e/a->nX;
        break;
    case FS_ABSOLUTE_ORTHOGONAL_DISTANCE:
        p.absolute_error=true;
        e = Fitting::orthogonalDistance(a->getH(),&p);
        a->oErrorAbs = e;
        a->meanOErrorAbs = e/a->nX;
        break;
    case FS_ABSOLUTE_Z_DISTANCE:
        p.absolute_error=true;
        e = Fitting::zDistance(a->getH(),&p);
        a->zErrorAbs = e;
        a->meanZErrorAbs = e/a->nX;
        break;
    default:
        break;
    }
    if(a->nX)
        return e;
    else
        return 0.0;
}
void RTM::RegularTriangularMesh::calculateDistanceErrorAtMesh(FITTING_STRATEGY FS)
{
    long double e_sum=0.0;
    long double e_cur=0.0;
    double e_min=0.0;
    double e_max=0.0;
    double ret[4];


    for(int i=0; i<this->n_u*this->n_v; i++)
    {
        e_cur = calculateDistanceErrorAtAnchor( &A[i], FS, true );
        e_sum += e_cur;
        e_cur > e_max ? e_max=e_cur : e_max=e_max;
        if(i>0)
            e_cur < e_min ? e_min=e_cur : e_min=e_min;
        else
            e_min = e_cur;
    }

    switch(FS)
    {
    case FS_SQUARED_Z_DISTANCE:
        this->zErrorSquared[0] = e_sum;
        this->zErrorSquared[1] = e_sum/this->getNX();
        this->zErrorSquared[2] = e_min;
        this->zErrorSquared[3] = e_max;
        break;
    case FS_SQUARED_ORTHOGONAL_DISTANCE:
        this->oErrorSquared[0] = e_sum;
        this->oErrorSquared[1] = e_sum/this->getNX();
        this->oErrorSquared[2] = e_min;
        this->oErrorSquared[3] = e_max;
        break;
    case FS_ABSOLUTE_Z_DISTANCE:
        this->zErrorAbs[0] = e_sum;
        this->zErrorAbs[1] = e_sum/this->getNX();
        this->zErrorAbs[2] = e_min;
        this->zErrorAbs[3] = e_max;
        break;
    case FS_ABSOLUTE_ORTHOGONAL_DISTANCE:
        this->oErrorAbs[0] = e_sum;
        this->oErrorAbs[1] = e_sum/this->getNX();
        this->oErrorAbs[2] = e_min;
        this->oErrorAbs[3] = e_max;
        break;
    }
}

double RTM::RegularTriangularMesh::calculateSmoothnessErrorAtAnchor(RTM::Anchor* a, bool globalError, bool absoluteError)
{
    if(globalError)
        return Fitting::smoothnessError(a,a->getH(),globalError,absoluteError);
    else
        return Fitting::smoothnessError(a,a->getH(),globalError,absoluteError);
}
void RTM::RegularTriangularMesh::calculateSmoothnessErrorAtMesh(bool absoluteError)
{
    double e_sum=0.0;
    double e_cur=0.0;
    double e_min=0.0;
    double e_max=0.0;

    //if(this->A_mutex.tryLock(500))
    {
        for(int i=0; i<this->n_u*this->n_v; i++)
        {
            e_cur = calculateSmoothnessErrorAtAnchor( &A[i], true, absoluteError );
            e_sum += e_cur;
            e_cur > e_max ? e_max=e_cur : e_max=e_max;
            if(i>0)
                e_cur < e_min ? e_min=e_cur : e_min=e_min;
            else
                e_min = e_cur;
            if(absoluteError)
                A[i].smoothErrorAbs = e_cur;
            else
                A[i].smoothErrorSquared = e_cur;
        }
        //this->A_mutex.unlock();
    }

    if(absoluteError)
    {
        this->smoothAbs[0] = e_sum;
        this->smoothAbs[1] = (e_sum)/(this->n_u*this->n_v);
        this->smoothAbs[2] = e_min;
        this->smoothAbs[3] = e_max;
    }
    else
    {
        this->smoothSquared[0] = e_sum;
        this->smoothSquared[1] = (e_sum)/(this->n_u*this->n_v);
        this->smoothSquared[2] = e_min;
        this->smoothSquared[3] = e_max;
    }
}

void RTM::RegularTriangularMesh::calculateTotalErrorAtMesh(FITTING_STRATEGY FS,double alpha)
{
    long double e_sum=0.0;
    long double e_cur=0.0;
    double e_min=0.0;
    double e_max=0.0;
    double ret[4];

    Fitting::DistanceFunctionParams params(&A[0],alpha, true,
                                           (FS == FS_ABSOLUTE_Z_DISTANCE ||  FS == FS_ABSOLUTE_ORTHOGONAL_DISTANCE) ? true : false,
                                           false);

    for(int i=0; i<this->n_u*this->n_v; i++)
    {
        params.a = &A[i];

        switch(FS)
        {
        case FS_SQUARED_Z_DISTANCE:
        case FS_ABSOLUTE_Z_DISTANCE:
            e_cur = Fitting::zErrorAtAnchor( A[i].getH(), (void*)&params );
            break;
        case FS_SQUARED_ORTHOGONAL_DISTANCE:
             case FS_ABSOLUTE_ORTHOGONAL_DISTANCE:
            e_cur = Fitting::orthogonalErrorAtAnchor( A[i].getH(), (void*)&params );
            break;
        }

        e_sum += e_cur;
        e_cur > e_max ? e_max=e_cur : e_max=e_max;
        if(i>0)
            e_cur < e_min ? e_min=e_cur : e_min=e_min;
        else
            e_min = e_cur;
    }

    switch(FS)
    {
    case FS_SQUARED_Z_DISTANCE:
        this->totalSquaredZError[0] = e_sum;
        //this->totalSquaredZError[1] = e_sum/(this->getNA());
        this->totalSquaredZError[2] = e_min;
        this->totalSquaredZError[3] = e_max;
        break;
    case FS_SQUARED_ORTHOGONAL_DISTANCE:
        this->totalSquaredOError[0] = e_sum;
        //this->totalSquaredOError[1] = e_sum/(this->getNA());
        this->totalSquaredOError[2] = e_min;
        this->totalSquaredOError[3] = e_max;
        break;
    case FS_ABSOLUTE_Z_DISTANCE:
        this->totalAbsZError[0] = e_sum;
        //this->totalAbsZError[1] = e_sum/(this->getNA());
        this->totalAbsZError[2] = e_min;
        this->totalAbsZError[3] = e_max;
        break;
    case FS_ABSOLUTE_ORTHOGONAL_DISTANCE:
        this->totalAbsOError[0] = e_sum;
        //this->totalAbsOError[1] = e_sum/(this->getNA());
        this->totalAbsOError[2] = e_min;
        this->totalAbsOError[3] = e_max;
        break;
    }
}

void RTM::RegularTriangularMesh::calculateStatistics(bool measureOnlyCurrentFittingDistances)
{
    this->averageAnchorOccupancy = this->n_x / (double)this->nA;
    this->averageFaceOccupancy = this->n_x / (double)this->nF;
    this->divAverageAnchorOccupancy = 1/averageAnchorOccupancy;
    this->divAverageFaceOccupancy = 1/averageAnchorOccupancy;

    double sum_std_dev=0;
    double std_dev=0;
    this->max_std_dev=0;
    this->n_occupied=0;
    //if(this->F_mutex.tryLock(500))
    {
        for(int i=0; i<this->nF; i++)
        {
            this->F[i].calculateStatistics();

            if(this->F[i].X.size() > 0)
            {
                n_occupied++;

                std_dev = this->F[i].mean_std_dev;
                sum_std_dev += std_dev;

                if(std_dev > this->max_std_dev)
                {
                    this->max_std_dev = std_dev;
                }
            }
        }
        //this->F_mutex.unlock();
    }

    //std::clog<<"n_occupied: "<<n_occupied<<std::endl;
    this->mean_std_dev = sum_std_dev/(float)this->n_occupied;
    this->occupied_ratio = this->n_occupied/(float)(this->nF);

    //if(this->A_mutex.tryLock(500))
    float sumAnchorHeight=0;
    {
        for(int i=0; i<this->nA; i++)
        {
            A[i].calculateStatistics();
            sumAnchorHeight+=A[i].getH();
        }
        this->averageAnchorHeight = sumAnchorHeight / this->nA;
        //this->A_mutex.unlock();
    }

   /* if(!measureOnlyCurrentFittingDistances)
    {
        this->calculateDistanceErrorAtMesh(FS_SQUARED_Z_DISTANCE);
        this->calculateDistanceErrorAtMesh(FS_SQUARED_ORTHOGONAL_DISTANCE);
        this->calculateDistanceErrorAtMesh(FS_ABSOLUTE_Z_DISTANCE);
        this->calculateDistanceErrorAtMesh(FS_ABSOLUTE_ORTHOGONAL_DISTANCE);
        this->calculateSmoothnessErrorAtMesh(true);
        this->calculateSmoothnessErrorAtMesh(false);

        this->calculateTotalErrorAtMesh(FS_SQUARED_Z_DISTANCE,this->fitthread->alpha);
        this->calculateTotalErrorAtMesh(FS_SQUARED_ORTHOGONAL_DISTANCE,this->fitthread->alpha);
        this->calculateTotalErrorAtMesh(FS_ABSOLUTE_Z_DISTANCE,this->fitthread->alpha);
        this->calculateTotalErrorAtMesh(FS_ABSOLUTE_ORTHOGONAL_DISTANCE,this->fitthread->alpha);

        this->totalSquaredZError[1] = this->zErrorSquared[1]+this->fitthread->alpha*(this->smoothSquared[1]);
        this->totalSquaredOError[1] = this->oErrorSquared[1]+this->fitthread->alpha*(this->smoothSquared[1]);
        this->totalAbsZError[1] = this->zErrorAbs[1]+this->fitthread->alpha*(this->smoothAbs[1]);
        this->totalAbsOError[1] = this->oErrorAbs[1]+this->fitthread->alpha*(this->smoothAbs[1]);
    }
    else
    {
        this->calculateDistanceErrorAtMesh(this->fitthread->FS);
        this->calculateSmoothnessErrorAtMesh( (this->fitthread->FS == FS_ABSOLUTE_ORTHOGONAL_DISTANCE || this->fitthread->FS==FS_ABSOLUTE_Z_DISTANCE) ? true : false);
    }*/
/*    this->totalSquaredZError[0] = this->zErrorSquared[0]+this->fitthread->alpha*this->smoothSquared[0];
    this->totalSquaredZError[1] = this->zErrorSquared[1]+(this->fitthread->alpha*this->smoothSquared[0])/this->getNA();
    this->totalAbsZError[0] = this->zErrorAbs[0]+this->fitthread->alpha*this->smoothAbs[0];
    this->totalAbsZError[1] = this->zErrorAbs[1]+(this->fitthread->alpha*this->smoothAbs[0])/this->getNA();
    this->totalSquaredOError[0] = this->oErrorSquared[0]+this->fitthread->alpha*this->smoothSquared[0];
    this->totalSquaredOError[1] = this->oErrorSquared[1]+(this->fitthread->alpha*this->smoothSquared[0])/this->getNA();
    this->totalAbsOError[0] = this->oErrorAbs[0]+this->fitthread->alpha*this->smoothAbs[0];
    this->totalAbsOError[1] = this->oErrorAbs[1]+(this->fitthread->alpha*this->smoothAbs[0])/this->getNA();*/
}
