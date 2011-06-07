#include "anchor.h"

RTM::Anchor::Anchor()
{
    this->F.reserve(sizeof(Face*)*6);
    this->u = -1;
    this->v = -1;
    this->nX = 0;
    this->meanC=0;
    this->relativeHeight=0;
}

void RTM::Anchor::setHeight( double h )
{
    if(fabs(this->h-h)<1e-3)
    {
        unsetUpdated();
        return;
    }
    this->h = h;
    this->v_w = this->v0_w + ( (*this->dir_h)*h );
    this->v_m[2] = h;

    //Calculate Normal
    this->n.set(0,0,0);
    for(size_t i=0; i<this->F.size();i++)
    {
        this->n += this->F[i]->getNormal(true);
    }
    this->n /= this->F.size();
    cml::normalize(n);

    // Set to updated.. and neighbours too
    this->setUpdated();
    Anchor* A = this->parent_mesh->getA();
    if(u<this->parent_mesh->getNU()-1)
    {
        A[idx+1].setUpdated();
    }
    if(v<this->parent_mesh->getNV()-1)
    {
        A[idx+(this->parent_mesh->getNU())].setUpdated();
    }
    if(u>0 && v<this->parent_mesh->getNV()-1)
        A[idx+(this->parent_mesh->getNU())-1].setUpdated();
    if(u>0)
        A[idx-1].setUpdated();
    if(v>0)
        A[idx-(this->parent_mesh->getNU())].setUpdated();
    if(u<this->parent_mesh->getNU()-1 && v>0)
        A[idx-(this->parent_mesh->getNU())+1].setUpdated();
}

void RTM::Anchor::drawNormal( double len )
{
    Vector3 _n = this->v_w+(cml::normalize(this->n)*len);
    glVertex3dv( this->v_w.data() );
    glVertex3dv( _n.data() );
}

int RTM::Anchor::getU()
{
    if(this->u==-1)
        this->u=this->idx%this->parent_mesh->getNU();
    return this->u;
}
int RTM::Anchor::getV()
{
    if(this->v==-1)
        this->v=this->idx/this->parent_mesh->getNU();
    return this->v;
}

void RTM::Anchor::calculateStatistics()
{
    //Calculate Occupancy
    int sumX=0;
    double sumC=0;
    this->relativeHeight = this->h / this->parent_mesh->averageAnchorHeight;
    for(size_t i=0; i<this->F.size(); i++)
    {
        sumX += F[i]->nX;
        sumC+=F[i]->meanC;
    }
    this->nX = sumX;
    this->relativeOccupancy = (this->nX) * this->parent_mesh->divAverageAnchorOccupancy;
    this->meanC=sumC/this->F.size();
}

void RTM::Anchor::setUpdated()
{
    updatedMutex.lock();
    this->updated=true;
    updatedMutex.unlock();
}

void RTM::Anchor::unsetUpdated()
{
    updatedMutex.lock();
    this->updated=false;
    updatedMutex.unlock();
}
bool RTM::Anchor::getUpdated()
{
    return updated;
}

