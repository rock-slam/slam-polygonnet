#include "face.h"

RTM::Face::Face()
{
    this->X.reserve(sizeof(Point*)*100);
    this->isOdd = false;
    k=-1;
    l=-1;

    this->mean_var=0;
    this->mean_std_dev=0;
    this->max_std_dev=0;
    this->sum_height=0;

    this->relativeHeight=0;

    this->meanC=0;

    this->nX = 0;
}

void RTM::Face::addPoint(Point* x)
{
    this->X.push_back(x);
    this->sum_height += x->Xm[2];
    this->nX++;
}

void RTM::Face::draw(float r, float g, float b, float a)
{
    glPushAttrib(GL_COLOR_BUFFER_BIT);
    glColor3f(r,g,b);

    glBegin(GL_TRIANGLES);
    if(!this->isOdd)
    {
	glVertex3dv(this->A_n->v_w.data());
	glVertex3dv(this->A_nPlus1->v_w.data());
	glVertex3dv(this->A_nPlus2->v_w.data());
    }
    else
    {
	glVertex3dv(this->A_n->v_w.data());
	glVertex3dv(this->A_nPlus2->v_w.data());
	glVertex3dv(this->A_nPlus1->v_w.data());
    }
    glEnd();

    glPopAttrib();
}

void RTM::Face::drawNormal(float len, bool update)
{
    Vector3 p;
    Vector3 n;

    glBegin(GL_LINES);
    p = this->getMean(true);

    glVertex3dv( p.data() );

    n = this->getNormal(update)*len;
    n = p+n;

    glVertex3dv( n.data() );
    glEnd();
}

Vector3 RTM::Face::getNormal(bool updateNormal, bool modelCoordinates)
{
    if( updateNormal )
    {
	if( this->isOdd )
	    this->n = cml::normalize( cml::cross( (this->A_nPlus1->v_w-this->A_n->v_w),  (this->A_nPlus2->v_w-this->A_n->v_w) ) );
	else
	    this->n = cml::normalize( cml::cross( (this->A_nPlus2->v_w-this->A_n->v_w), (this->A_nPlus1->v_w-this->A_n->v_w) ) );
    }

    if(modelCoordinates)
        return this->parent_mesh->convertWorldToModel(n);
    else
        return n;
}

Vector3 RTM::Face::getMean(bool updateMean)
{
    if( updateMean )
	this->M = (this->A_n->v_w + (this->A_nPlus1->v_w-this->A_n->v_w)/3.0) + (this->A_nPlus2->v_w-this->A_n->v_w)/3.0;

    return this->M;
}

int RTM::Face::getK()
{
    if(this->k==-1)
	this->k = this->id%this->parent_mesh->getNK();
    return this->k;
}

int RTM::Face::getL()
{
    if(this->l==-1)
	this->l = this->id/this->parent_mesh->getNK();
    return this->l;
}

void RTM::Face::calculateStatistics()
{
    this->statMutex.lock();
    if(this->nX)
	this->mean_height = this->sum_height / (float)this->X.size();
    else
	this->mean_height = 0;

    //Varianz und Standartabweichung berechnen
    this->sum_var=0.0;
    this->sum_std_dev=0.0;
    double var=0;
    double max_var=0;
    double std_dev=0;
    double c=0;

    for(int i=0; i<this->X.size(); i++)
    {

        var = pow( X[i]->Xm[2] - this->mean_height, 2 );
        sum_var += var;
        std_dev = sqrt(var);
        sum_std_dev += std_dev;

        X[i]->var = var;
        X[i]->std_dev = std_dev;

        if(var > max_var)
            max_var = var;

	//Mean confidence
	c+=X[i]->c;
    }

    height = (this->A_n->getH()+this->A_nPlus1->getH()+this->A_nPlus2->getH())/3.0;
    relativeHeight = (this->A_n->relativeHeight+this->A_nPlus1->relativeHeight+this->A_nPlus2->relativeHeight)/3.0;

    this->mean_var = sum_var / this->X.size();
    this->mean_std_dev = sum_std_dev/this->X.size();
    this->max_var = max_var;
    this->max_std_dev = sqrt(max_var);

    //Mean confidence
    this->meanC = c/this->X.size();

    //Occupancy berechnen
    //std::clog << this->nX<<","<<this->parent_mesh->nF<<","<<this->parent_mesh->divAverageFaceOccupancy<<std::endl;
    this->relativeOccupancy = (this->nX)*this->parent_mesh->divAverageFaceOccupancy;
    this->statMutex.unlock();
}
