#include "Fitting.h"
#include <fstream>

Fitting::FitThread::FitThread(RTM::RegularTriangularMesh* mesh,int n_u,int n_v)
{
    this->mesh=mesh;
    this->n_u=n_u;
    this->n_v=n_v;
}

void Fitting::FitThread::fit(int iterations, RTM::FITTING_STRATEGY FS, double alpha)
{
    this->iterations = iterations;
    this->FS = FS;
    this->alpha = alpha;
    this->cancelF = false;
    this->start();
}

void Fitting::FitThread::run()
{
    double newHeight=0;
    double oldHeight=0;
    int c=0;
    RTM::Anchor *A;
    int idx;
    forceFit=true;
    QTime t_oneLap;
    QTime t_untilConvergence;
    int t1,t2;
    int anchorsInvolved_oneLap=0;
    int anchorsInvolved_untilConvergence=0;
    bool updated=true;
    long int sum_t1=0;
    long int max_t1=0;
    QTime t_stat;
    long int sum_tstat;

    t_untilConvergence.start();
    while ( c < this->iterations || this->iterations==0 )
    {
        t_stat.start();

        iterations_to_do = iterations-c-1;
        anchorsInvolved_oneLap=0;

        //std::clog << "Fitting ("<<c<<"/"<<iterations<<")"<<" Strategy: "<<FS<<std::endl;

        if(this->updateStatistics)
            this->mesh->calculateStatistics(true);

        sum_tstat += t_stat.elapsed();
        //std::clog<<"Stat calculation took (mean)"<<sum_tstat/(float)(c+1)<<std::endl;

        t_oneLap.start();
        for(int j=0; j<this->n_v;j++)
        {
            for(int i=0; i<this->n_u;i++)
            {
                if(cancelF)
                    break;

                //
                // Calculate new height of anchor
                //
                this->mesh->basicsMutex.lock();
                A = this->mesh->getA();
                idx = j*n_u+i;

                if(A[idx].getUpdated() || forceFit)
                {
                    anchorsInvolved_oneLap++;
                    anchorsInvolved_untilConvergence++;
                    updated=true;
                    oldHeight = A[idx].getH();
                    switch(FS)
                    {
                    case RTM::FS_SQUARED_Z_DISTANCE:
                        newHeight = Fitting::fitFanSquaredZDistance(&A[idx], alpha);
                        break;
                    case RTM::FS_SIMPLE_FIT:
                        newHeight = Fitting::verySimpleFitPlane(&A[idx],alpha);
                        break;
                    case RTM::FS_SQUARED_ORTHOGONAL_DISTANCE:
                        newHeight = Fitting::fitFanSquaredOrthogonalDistance(&A[idx], alpha, adaptiveSmoothness);
                        break;
                    case RTM::FS_ABSOLUTE_ORTHOGONAL_DISTANCE:
                        newHeight = Fitting::fitFanAbsoluteOrthogonalDistance(&A[idx], alpha, adaptiveSmoothness);
                        break;
                    case RTM::FS_ABSOLUTE_Z_DISTANCE:
                        newHeight = Fitting::fitFanAbsoluteZDistance(&A[idx], alpha, adaptiveSmoothness);
                        break;
                    case RTM::FS_SQUARED_Z_DISTANCE_BRENT:
                        newHeight = Fitting::fitFanSquaredZDistanceBrent(&A[idx], alpha, adaptiveSmoothness);
                        break;
                    default:
                        break;
                    }

                    A[idx].setHeight(newHeight);
                }
                this->mesh->basicsMutex.unlock();
                // Fitting::plotSmoothness(&this->A[j*n_u+i]);
            }
            if(cancelF)
                break;
        }
        if(cancelF)
            break;

         /*mesh->calculateStatistics(false);
        std::clog<<"zError2: "<<mesh->zErrorSquared[0]+alpha*mesh->smoothSquared[0]<<","<<mesh->zErrorSquared[1]+(alpha*mesh->smoothSquared[0])/mesh->getNA()<<std::endl;
        std::clog<<"oError2: "<<mesh->oErrorSquared[0]+alpha*mesh->smoothSquared[0]<<","<<mesh->oErrorSquared[1]+(alpha*mesh->smoothSquared[0])/mesh->getNA()<<std::endl;
        std::clog<<"zErrorAbs: "<<mesh->zErrorAbs[0]+alpha*mesh->smoothAbs[0]<<","<<mesh->zErrorAbs[1]+(alpha*mesh->smoothAbs[0])/mesh->getNA()<<std::endl;
        std::clog<<"oErrorAbs: "<<mesh->oErrorAbs[0]+alpha*mesh->smoothAbs[0]<<","<<mesh->oErrorAbs[1]+(alpha*mesh->smoothAbs[0])/mesh->getNA()<<std::endl;
        break;*/

        if(!anchorsInvolved_oneLap && updated) //First round where nothing was found to fit
        {
            t2 = t_untilConvergence.elapsed();
            this->currentSpeed_untilConvergence = t2;
            this->iterations_untilConvergence = ++c;
            updated = false;
            //c=0;
            emit convergenceSignal();
	    printf("Nothing to fit\n");
	    break;
        }
        else if(!updated)   //Nothing to fit and not the first round
        {
            msleep(200);
            continue;
        }
        else    //Normal case during fitting
        {
            this->currentSpeed_perAnchor = (float)t1/anchorsInvolved_oneLap;
            msleep(10);
            forceFit=false;
            c++;
        }

        // Read timers
        t1 = t_oneLap.elapsed();
        this->currentSpeed_oneLap = t1;
        sum_t1+=t1;
        if(t1>max_t1)
            max_t1=t1;
        std::clog<<"Iteration " <<c <<" took "<<t1<<"ms with " <<anchorsInvolved_oneLap<<" involved: "<<t1/(float)anchorsInvolved_oneLap<< "ms per Anchor"<< " Durchschnitt: "<<sum_t1/(float)c<<std::endl;


    }

    if(cancelF)
        std::clog << "Fitting canceled!"<<std::endl;
    else
        std::clog << "Fitting quit successfully"<<std::endl;

    std::clog<<"Sum T1: "<<sum_t1<<" Iterationen: "<<c<<" Mean: "<<sum_t1/(double)c<<std::endl;
    std::clog<<"Max T1: "<<max_t1<<std::endl;

    emit stoppedFitting(false);
}

void Fitting::FitThread::cancelFitting()
{
    this->cancelF=true;
}

void Fitting::FitThread::setStrategy(int fs)
{
    switch(fs)
    {
    case 0:
        FS = RTM::FS_SQUARED_Z_DISTANCE;
        break;
    case 1:
        FS = RTM::FS_SQUARED_ORTHOGONAL_DISTANCE;
        break;
    case 2:
        FS = RTM::FS_SIMPLE_FIT;
        break;
    case 3:
        FS = RTM::FS_ABSOLUTE_Z_DISTANCE;
        break;
    case 4:
        FS = RTM::FS_ABSOLUTE_ORTHOGONAL_DISTANCE;
        break;
    case 5:
        FS = RTM::FS_SQUARED_Z_DISTANCE_BRENT;
        break;
    }
    forceFit=true;
}












double Fitting::optimalAnchorHeight(RTM::Face* f, RTM::Anchor *a,base::Vector3d X_m)
{
    if(f->X.size()==0)
        return a->getH();

    base::Vector3d tmp = a->v_m;
    double Xx = tmp[0];
    double Xy = tmp[1];
    double Xz = 0;
    base::Vector3d Pe = X_m;
    double Pex = Pe[0];
    double Pey = Pe[1];
    double Pez = Pe[2];
    base::Vector3d A1;
    base::Vector3d A2;
    base::Vector3d n;
    double ret;

    if(a==f->A_n)
    {
        A1 = f->A_nPlus1->v_m;
        A2 = f->A_nPlus2->v_m;
    }
    if(a==f->A_nPlus1)
    {
        A1 = f->A_n->v_m;
        A2 = f->A_nPlus2->v_m;
    }
    if(a==f->A_nPlus2)
    {
        A1 = f->A_nPlus1->v_m;
        A2 = f->A_n->v_m;
    }

    //Diese Funktion benutzt NICHT die hessesche Entfernungsmessung, sondern lediglig die
    //Normalform. Diese misst nicht die entfernung, sondern sagt, ob ein Punkt auf der Ebene liegt,
    //oder davor oder dahinter. Für diese Gleichung wird keine normalisierte Normale benötigt.
    //Es gibt auch nicht den D-Parameter (Abstand der Ebene zum Ursprung?) wie in der Hesseschen Normalform.
    //
    //Unter der Benutzung der Normalform wird die A0z so variiert, dass Pe in der Ebene liegt. Die hessesche Normalenform
    //bringt hier keine Vorteile, ist aber teurer
    if(f->isOdd)
        n = (A1-Pe).cross(A2-Pe);
    else
        n = (A2-Pe).cross(A1-Pe);
    double Nx = n[0];
    double Ny = n[1];
    double Nz = n[2];

    if(fabs(Nz)>1e-8)
        Xz = (-Xx*Nx + Pex*Nx - Xy*Ny+Pey*Ny + Pez*Nz)/Nz;
    else
        Xz = Pez;

    if(fabs(Xz)>fabs(Pez))
        ret = Pez;
    else
        ret = Xz;
    //ret = Xz;

    return ret;
}

double Fitting::verySimpleFitPlane(RTM::Anchor* a, double alpha)
{
    double sum_h=0;
    double mean_h=0;
    double h;
    base::Vector3d mean_n=base::Vector3d(0,0,0);
    base::Vector3d sum_n=base::Vector3d(0,0,0);
    double c=0;
    base::Vector3d x;
    //double g=1.0;
    int samples[8];
    int sum_X=0;
    QList <double> h_list;

    for(unsigned int i=0; i<a->F.size(); i++)
    {
        sum_X+=a->F[i]->X.size();
        if( !a->F[i]->X.size() )
            continue;
        for(int j=0;j<8;j++)
            samples[j]=qrand()%a->F[i]->X.size();
        for( unsigned int j=0;j<8 && c<a->F[i]->X.size(); j++ )
        {
            x = a->F[i]->X[samples[j]]->Xm;
            h_list.append(optimalAnchorHeight(a->F[i],a,a->F[i]->X[samples[j]]->Xm));

            sum_h += /*pow(g,2)**/h;
            c += 1;//g;
        }
    }

    //geometric error
    if(c==0)//Keine Punkte
        mean_h =  a->getH();
    else
    {
        qSort(h_list);
        mean_h = h_list[c/2];
    }
    h_list.clear();

    //Calculate smoothness
    double sumH;
    int nNeighbours;
    //double s = smoothnessError(a,mean_h,false,false,&sumH,&nNeighbours);

    //Calculate new height
    double newHeight = (nNeighbours*alpha*sumH+mean_h*sum_X) / (sum_X+pow(nNeighbours,2)*alpha);

    return newHeight;
}


/*# Ich habe diese Basisfunktion aus http://people.sc.fsu.edu/~burkardt/presentations/cg_lab_fem_basis_triangle.pdf
# Das Dreieck ist nach folgender Konvention aufgebaut, die scheinbar auch einzuhalten ist:
#
#   b
#	|\
#	|  \
#	|    \
# 	|______\
#   c       a
#
# Die Basisfunktion hat die Eigenschaft, dass sie jeweils in einem der Punkte (a,b,c) 1 ist waehrend
# sie fuer die anderen Punkte 0 ist.
# ACHTUNG: Fuer Punkte (x,y) die ausserhalb des Dreiecks liegen werden Werte != 0 ausgegeben.

  FUNKTION IS NOT IN USE!
*/
double Fitting::triangleBaseFunction(RTM::Face *f, RTM::Anchor *a, double x, double y, int p)
{
    double ax;
    double ay;
    double bx;
    double by;
    double cx;
    double cy;

    int edge=0;	//Here it is stored wich anchor (A_n, A_nPlus1, A_nPlus2) corresponds to the given anchor a.
    //The function is to be evaluated for this anchor a. That is mapped to one of the edges (a,b,c)

    if(f->isOdd)
    {
        ax = f->A_n->v0_m[0];
        ay = f->A_n->v0_m[1];
        bx = f->A_nPlus1->v0_m[0];
        by = f->A_nPlus1->v0_m[1];
        cx = f->A_nPlus2->v0_m[0];
        cy = f->A_nPlus2->v0_m[1];

        if(a == f->A_n)
            edge = 0;
        else if(a == f->A_nPlus1)
            edge = 1;
        else if(a == f->A_nPlus2)
            edge = 2;
    }
    else
    {
        ax = f->A_nPlus2->v0_m[0];
        ay = f->A_nPlus2->v0_m[1];
        bx = f->A_nPlus1->v0_m[0];
        by = f->A_nPlus1->v0_m[1];
        cx = f->A_n->v0_m[0];
        cy = f->A_n->v0_m[1];

        if(a == f->A_n)
            edge = 2;
        else if(a == f->A_nPlus1)
            edge = 1;
        else if(a == f->A_nPlus2)
            edge = 0;
    }

    double fa = ((by-cy)*(x-cx) - (bx-cx)*(y-cy)) / ((ax-cx)*(by-cy) - (ay-cy)*(bx-cy));
    double fb = ( -(ay-cy)*(x-cx) + (ax-cx)*(y-cy) ) / ((ax-cx)*(by-cy) - (ay-cy)*(bx-cy));
    double fc = 1 - fa -fb;

    switch(edge)
    {
    case 0: return pow(fa,p); break;
    case 1: return pow(fb,p); break;
    case 2: return pow(fc,p); break;
    default: break;
    }
    return -1;
}

double Fitting::smoothnessError(RTM::Anchor* a, double h, bool globalError,
                                bool absoluteError,
                                double* _sumH, int* _nNeighbours)
{
    int i = a->idx;
    int n_u = a->parent_mesh->getNU();
    int n_v = a->parent_mesh->getNV();
    int v = a->v;
    int u = a->u;
    RTM::Anchor* Anchors=a->parent_mesh->getA();
    RTM::Anchor* neighbourAnchor;
    double sum_Dh = 0.0;
    int nNeighbours=0;
    double sumH=0;

    if(u<n_u-1)
    {
        //ACHTUNG: Momentan gehen nur die Anker in die Fehlerberchnung ein, die auch belegt sind.
        // Es sei denn der aktuelle anker ist selbst nicht belegt, dann einfach smoothen
        neighbourAnchor = &Anchors[i+1];
        //if( neighbourAnchor->occupancy > 0 || a->occupancy == 0 )
        sumH += neighbourAnchor->getH();
        sum_Dh += h-neighbourAnchor->getH();//absoluteError ? fabs(h - neighbourAnchor->h) : pow(h - neighbourAnchor->h, 2); // u+1
        nNeighbours++;
    }
    if(v<n_v-1)
    {
        neighbourAnchor = &Anchors[i+(n_u)];
        //if( neighbourAnchor->occupancy >0 || a->occupancy == 0 )
        sumH += neighbourAnchor->getH();
        sum_Dh += h-neighbourAnchor->getH();//absoluteError ? fabs(h - neighbourAnchor->h) : pow(h - neighbourAnchor->h, 2); // v+1
        nNeighbours++;
    }
    if(u>0 && v<n_v-1)
    {
        neighbourAnchor = &Anchors[i+(n_u)-1];
        //if( neighbourAnchor->occupancy >0 || a->occupancy == 0 )
        sumH += neighbourAnchor->getH();
        sum_Dh += h-neighbourAnchor->getH();//absoluteError ? fabs(h - neighbourAnchor->h) : pow(h - neighbourAnchor->h, 2); // u-1, v+1
        nNeighbours++;
    }
    //Für den Fehler des gesamten netzes dürfen die differenzen zwischen Ankerhöhen nicht doppelt gezählt werden.
    //Von daher soll in diesem fall jeweils nur die distanz zu den vorne liegenden nachbarn genommne werden.
    //if(!globalError)
    {
        if(u>0)
        {
            neighbourAnchor = &Anchors[i-1];
            //if( neighbourAnchor->occupancy >0 )
            sumH += neighbourAnchor->getH();
            sum_Dh += h-neighbourAnchor->getH();//absoluteError ? fabs(h - neighbourAnchor->h) :  pow(h - neighbourAnchor->h, 2); // u-1
            nNeighbours++;
        }
        if(v>0)
        {
            neighbourAnchor = &Anchors[i-(n_u)];
            //if( neighbourAnchor->occupancy >0 || a->occupancy == 0 )
            sumH += neighbourAnchor->getH();
            sum_Dh += h-neighbourAnchor->getH();//absoluteError ? fabs(h - neighbourAnchor->h) : pow(h - neighbourAnchor->h, 2); // v-1
            nNeighbours++;
        }
        if(u<n_u-1 && v>0)
        {
            neighbourAnchor = &Anchors[i-(n_u)+1];
            //if( neighbourAnchor->occupancy >0 || a->occupancy == 0 )
            sumH += neighbourAnchor->getH();
            sum_Dh += h-neighbourAnchor->getH();//absoluteError ? fabs(h -neighbourAnchor->h) : pow(h -neighbourAnchor->h, 2); // u+1, v-1
            nNeighbours++;
        }
    }

    if(_sumH)
        *_sumH=sumH;
    if(_nNeighbours)
        *_nNeighbours=nNeighbours;

    //return sum_Dh/(float)nNeighbours;
    //Vieleicht ista das hier richtiger? Hab es aber zum testen in obiges verändert
    return absoluteError ? fabs(sum_Dh):pow(sum_Dh,2);
}

double Fitting::diffSquaredSmoothnessError(RTM::Anchor* a, double h, bool globalError)
{
    int i = a->idx;
    int n_u = a->parent_mesh->getNU();
    int n_v = a->parent_mesh->getNV();
    int v = i/n_u;
    int u = i%n_u;
    RTM::Anchor* Anchors=a->parent_mesh->getA();
    long double sum_Dh = 0;

    //Für den Fehler des gesamten netzes dürfen die differenzen zwischen Ankerhöhen nicht doppelt gezählt werden.
    //Von daher soll in diesem fall jeweils nur die distanz zu den vorne liegenden nachbarn genommne werden.
    /*if(u<n_u-1) sum_Dh += h - Anchors[i+1].h; // u+1
    if(v<n_v-1) sum_Dh += h - Anchors[i+(n_u)].h; // v+1
    if(u>0 && v<n_v-1) sum_Dh += h - Anchors[i+(n_u)-1].h; // u-1, v+1
    if(!globalError)
    {
        if(u>0) sum_Dh += h - Anchors[i-1].h; // u-1
        if(v>0) sum_Dh += h - Anchors[i-(n_u)].h; // v-1
        if(u<n_u-1 && v>0) sum_Dh += h -Anchors[i-(n_u)+1].h; // u+1, v-1
    }

    return sum_Dh*2;*/


    //For the "new" smoothness term, that is alpha*((h-up)+(h-vp)+(h-umvp)+(h-um)+(h-vm)+(h-upvm))^2;
    //I get as ableitung 12*alpha*(-vp-vm-upvm-up-umvp-um+6*h).
    int n=0;
    if(u<n_u-1) {sum_Dh += -Anchors[i+1].getH(); n++;} // u+1
    if(v<n_v-1) {sum_Dh += -Anchors[i+(n_u)].getH(); n++;} // v+1
    if(u>0 && v<n_v-1) {sum_Dh += -Anchors[i+(n_u)-1].getH(); n++;} // u-1, v+1
    if(!globalError)
    {
        if(u>0){ sum_Dh += -Anchors[i-1].getH(); n++;} // u-1
        if(v>0){ sum_Dh += -Anchors[i-(n_u)].getH(); n++;} // v-1
        if(u<n_u-1 && v>0){ sum_Dh += -Anchors[i-(n_u)+1].getH(); n++;} // u+1, v-1
    }
    return 2*n*(sum_Dh+n*h);
}

double* Fitting::heightWithSmoothnessError(RTM::Anchor *a, double e_s, double alpha, bool absoluteError)
{
    int i = a->idx;
    int n_u = a->parent_mesh->getNU();
    int n_v = a->parent_mesh->getNV();
    int v = i/n_u;
    int u = i%n_u;
    RTM::Anchor* Anchors=a->parent_mesh->getA();
    //long double sum_Dh = 0;
    //double up=0,vp=0,umvp=0,um=0,vm=0,upvm=0;
    double h[2];
    int N=0;
    double Sh=0.;

    //Es müssen die tatsächlichen Nachbarn gezählt werden (N) um mit den Rand-Ankern umgehen zu können
    if(u<n_u-1) {Sh += Anchors[i+1].getH(); N++;} // u+1
    if(v<n_v-1) {Sh += Anchors[i+(n_u)].getH(); N++;} // v+1
    if(u>0 && v<n_v-1) {Sh += Anchors[i+(n_u)-1].getH(); N++;} // u-1, v+1
    if(u>0) {Sh += Anchors[i-1].getH(); N++;} // u-1
    if(v>0) {Sh += Anchors[i-(n_u)].getH(); N++;} // v-1
    if(u<n_u-1 && v>0) {Sh += Anchors[i-(n_u)+1].getH(); N++;} // u+1, v-1

    //int N1 = N-1;

    if(!absoluteError)
    {
        //e_s = alpha*((h-up)**2+(h-vp)**2+(h-umvp)**2+(h-um)**2+(h-vm)**2+(h-upvm)**2)
        //solved to h gives:
        /*double alpha2 = alpha*alpha;
        double g=(N*alpha);
        double e = (vp+vm+upvm+up+umvp+um)/N;
        double p = -N1*alpha2*pow(vp,2)+2*alpha2*vm*vp+2*alpha2*upvm*vp+2*alpha2*up*vp+2*alpha2*umvp*vp+2*alpha2*um*vp-N1*alpha2*pow(vm,2)+2*alpha2*upvm*vm+2*alpha2*up*vm+2*alpha2*umvp*vm
                   +2*alpha2*um*vm-N1*alpha2*pow(upvm,2)+2*alpha2*up*upvm+2*alpha2*umvp*upvm+2*alpha2*um*upvm-N1*alpha2*pow(up,2)+2*alpha2*umvp*up+2*alpha2*um*up-N1*alpha2*pow(umvp,2)+2*alpha2*um*umvp-N1*alpha2*pow(um,2)
                   +N*alpha*e_s;
        double f=(sqrt(fabs(p))); //SQRT of negative gives nan
        h[0]=-f/g+e;
        h[1]=f/g+e;*/

        //h[0]=-(-N*alpha*Sh+sqrt(alpha*e_s))/(N*alpha);
        //h[1]=(alpha*vp+alpha*vm+alpha*upvm+alpha*up+alpha*umvp+alpha*um+sqrt(alpha*e_s))/(6*alpha);
        h[0]=(alpha*Sh-sqrt(alpha*e_s)) / (alpha*N);
        h[1]=(alpha*Sh+sqrt(alpha*e_s)) / (alpha*N);
    }
    else
    {
        //e_abs = alpha*(abs(vp-h)+abs(vm-h)+abs(upvm-h)+abs(up-h)+abs(umvp-h)+abs(um-h))
        //solved is not possible accrding to Maxima. it gives:
        //abs(vp-h)=-(alpha*abs(vm-h)+alpha*abs(upvm-h)+alpha*abs(up-h)+alpha*abs(umvp-h)+alpha*abs(um-h)-e_s)/alpha
        //I thought it might be possible to write it as:
        //h=vp + (alpha*abs(vm-h)+alpha*abs(upvm-h)+alpha*abs(up-h)+alpha*abs(umvp-h)+alpha*abs(um-h)-e_s)/alpha
        //so here it is:
        //double l= (vp+(alpha*fabs(vm-h)+alpha*fabs(upvm-h)+alpha*fabs(up-h)+alpha*fabs(umvp-h)+alpha*fabs(um-h)-e_s)/alpha);

        //double l = (alpha*vp+alpha*vm+alpha*upvm+alpha*up+alpha*umvp+alpha*um+e_s)/(6*alpha);
        //h[0] = -l;
        //h[1] = l;
        //Keine Ahnung ob das richtig ist.. ist einfach die Gleiche Gleichung wie bei sq, nur dass nicht die Wurzel
        //genommen wird, sondern +/- vom abs
        h[0]=(alpha*Sh-fabs(alpha*e_s)) / (alpha*N);
        h[1]=(alpha*Sh+fabs(alpha*e_s)) / (alpha*N);
    }

    return h;
}


double Fitting::zDistance(double h, DistanceFunctionParams* params)
{
    RTM::Anchor* a = params->a;
    double globalError = params->globalError;
    bool absoluteError = params->absolute_error;

    RTM::Face *f=0;
    base::Vector3d a1, a2;
    base::Vector3d x;
    double A0x,A0y,A0z;
    double A1x,A1y,A1z;
    double A2x,A2y,A2z;
    double g=1.0;

    base::Vector3d a0 = a->v_m;
    A0x = a0[0]; A0y = a0[1]; A0z = h;

    double A,B,D;
    double dist=0.0;
    double Sdist=0.0;

    int k;
    int l;

    //For each face...
    for(size_t j=0; j<a->F.size(); j++)
    {
        f = a->F.at(j);

        //Für den Fehler des gesamten netzes dürfen die Entfernungsfehler pro Face nur einmal gezählt werden.
        //Von daher soll in diesem fall jeweils nur zwei faces pro anker gezählt, und zwar die mit v=k die genau
        //rechts und links vom anker liegen.
        if(globalError)
        {
            k = f->getK();
            l = f->getL();
            if(l!=a->getV())
                continue;
            if(k!=a->getU()*2 && k!=a->getU()*2-1)
                continue;
        }

        //Set anchor coordinates
        if(a==f->A_n)
        {
            a1 = f->A_nPlus1->v_m;
            a2 = f->A_nPlus2->v_m;
        }
        else if(a==f->A_nPlus1)
        {
            a1 = f->A_n->v_m;
            a2 = f->A_nPlus2->v_m;
        }
        else if(a==f->A_nPlus2)
        {
            a1 = f->A_nPlus1->v_m;
            a2 = f->A_n->v_m;
        }
        A1x=a1[0]; A1y=a1[1]; A1z=a1[2];
        A2x=a2[0]; A2y=a2[1]; A2z=a2[2];

        // Caluculate A, B and D: Anchors must be in plane, so:
        // A0x*A + A0y*B - A0z + D = 0
        // A1x*A + A1y*B - A1z + D = 0
        // A2x*A + A2y*B - A2z + D = 0
        // This solved to A,B and D gives
        A = -(A0y*(A2z-A1z)-A1y*A2z+A1z*A2y+A0z*(A1y-A2y))/(A0x*(A2y-A1y)-A1x*A2y+A1y*A2x+A0y*(A1x-A2x));
        B = (A0x*(A2z-A1z)-A1x*A2z+A1z*A2x+A0z*(A1x-A2x))/(A0x*(A2y-A1y)-A1x*A2y+A1y*A2x+A0y*(A1x-A2x));
        D = (A0x*(A1z*A2y-A1y*A2z)+A0y*(A1x*A2z-A1z*A2x)+A0z*(A1y*A2x-A1x*A2y))/(A0x*(A2y-A1y)-A1x*A2y+A1y*A2x+A0y*(A1x-A2x));

        //For each point
        for(size_t i=0; i<f->X.size(); i++)
        {
            x = f->X.at(i)->Xm;
            //Calculate distance
            dist = (A*x[0]+B*x[1]+D-x[2]);

            //Summing up
            Sdist += absoluteError ? fabs(g*dist) :  g*(dist*dist);
        }
    }

    return Sdist;
}

double Fitting::diffSquaredZDistance(double h, DistanceFunctionParams* params)
{
    RTM::Anchor* a = params->a;
    double globalError = params->globalError;

    RTM::Face *f=0;
    base::Vector3d a1, a2;
    base::Vector3d x;
    double A0x,A0y;
    double A0z=h;
    double A1x,A1y,A1z;
    double A2x,A2y,A2z;
    double g=1.0;

    base::Vector3d a0 = a->v_m;
    A0x = a0[0]; A0y = a0[1];

    double dA,dB,dD,A,B,D;

    long double df=0.0;
    int k,l;

    //For each face...
    for(size_t j=0; j<a->F.size(); j++)
    {
        f = a->F.at(j);

        //Für den Fehler des gesamten netzes dürfen die Entfernungsfehler pro Face nur einmal gezählt werden.
        //Von daher soll in diesem fall jeweils nur zwei faces pro anker gezählt, und zwar die mit v=k die genau
        //rechts und links vom anker liegen.
        if(globalError)
        {
            k = f->getK();
            l = f->getL();
            if(l!=a->getV())
                continue;
            if(k!=a->getU()*2 && k!=a->getU()*2-1)
                continue;
        }

        //Set anchor coordinates
        if(a==f->A_n)
        {
            a1 = f->A_nPlus1->v_m;
            a2 = f->A_nPlus2->v_m;
        }
        else if(a==f->A_nPlus1)
        {
            a1 = f->A_n->v_m;
            a2 = f->A_nPlus2->v_m;
        }
        else if(a==f->A_nPlus2)
        {
            a1 = f->A_nPlus1->v_m;
            a2 = f->A_n->v_m;
        }
        A1x=a1[0]; A1y=a1[1]; A1z=a1[2];
        A2x=a2[0]; A2y=a2[1]; A2z=a2[2];

        // Caluculate A, B and D: Anchors must be in plane, so:
        // A0x*A + A0y*B - A0z + D = 0
        // A1x*A + A1y*B - A1z + D = 0
        // A2x*A + A2y*B - A2z + D = 0
        // This solved to A,B and D gives
        A = -(A0y*(A2z-A1z)-A1y*A2z+A1z*A2y+A0z*(A1y-A2y))/(A0x*(A2y-A1y)-A1x*A2y+A1y*A2x+A0y*(A1x-A2x));
        B = (A0x*(A2z-A1z)-A1x*A2z+A1z*A2x+A0z*(A1x-A2x))/(A0x*(A2y-A1y)-A1x*A2y+A1y*A2x+A0y*(A1x-A2x));
        D = (A0x*(A1z*A2y-A1y*A2z)+A0y*(A1x*A2z-A1z*A2x)+A0z*(A1y*A2x-A1x*A2y))/(A0x*(A2y-A1y)-A1x*A2y+A1y*A2x+A0y*(A1x-A2x));

        dA = (A2y-A1y)/(A0x*(A2y-A1y)-A1x*A2y+A1y*A2x+A0y*(A1x-A2x));
        dB = (A1x-A2x)/(A0x*(A2y-A1y)-A1x*A2y+A1y*A2x+A0y*(A1x-A2x));
        dD = (A1y*A2x-A1x*A2y)/(A0x*(A2y-A1y)-A1x*A2y+A1y*A2x+A0y*(A1x-A2x));

        //For each point
        for(size_t i=0; i<f->X.size(); i++)
        {
            x = f->X.at(i)->Xm;
            df += g * (dA*x[0]+dB*x[1]+dD)*(A*x[0]+B*x[1]+D-x[2]);
        }
    }

    return 2*df;
}


double Fitting::orthogonalDistance(double h, Fitting::DistanceFunctionParams* params)
{
    RTM::Anchor* a = ((DistanceFunctionParams*)params)->a;
    bool globalError = ((DistanceFunctionParams*)params)->globalError;
    bool absoluteError = ((DistanceFunctionParams*)params)->absolute_error;

    RTM::Face *f=0;
    base::Vector3d a1, a2;
    base::Vector3d x;
    base::Vector3d n;
    double A0x,A0y,A0z;
    double A1x,A1y,A1z;
    double A2x,A2y,A2z;

    base::Vector3d a0 = a->v_m;
    A0x = a0[0]; A0y = a0[1]; A0z = h;

    double nx,ny,nz;
    double A,B,D;
    double dist=0.0;
    long double Sdist=0.0;

    double g=1.0;
    int k,l;

    //
    // Distance error
    //
    for(size_t j=0; j<a->F.size(); j++)
    {
        f = a->F.at(j);

        //Für den Fehler des gesamten netzes dürfen die Entfernungsfehler pro Face nur einmal gezählt werden.
        //Von daher soll in diesem fall jeweils nur zwei faces pro anker gezählt, und zwar die mit v=k die genau
        //rechts und links vom anker liegen.
        if(globalError)
        {
            k = f->getK();
            l = f->getL();
            if(l!=a->getV())
                continue;
            if(k!=a->getU()*2 && k!=a->getU()*2-1)
                continue;
        }

        //Set anchor coordinates
        if(a==f->A_n)
        {
            a1 = f->A_nPlus1->v_m;
            a2 = f->A_nPlus2->v_m;
        }
        else if(a==f->A_nPlus1)
        {
            a1 = f->A_n->v_m;
            a2 = f->A_nPlus2->v_m;
        }
        else if(a==f->A_nPlus2)
        {
            a1 = f->A_nPlus1->v_m;
            a2 = f->A_n->v_m;
        }
        A1x=a1[0]; A1y=a1[1]; A1z=a1[2];
        A2x=a2[0]; A2y=a2[1]; A2z=a2[2];

        //Calculate
        //n=f->getNormal(true,true);

   /*
        nx = (A1y-A0y)*(A2z-A0z)-(A1z-A0z)*(A2y-A0y);
        ny = (A1z-A0z)*(A2x-A0x)-(A1x-A0x)*(A2z-A0z);
        nz = (A1x-A0x)*(A2y-A0y)-(A1y-A0y)*(A2x-A0x);*/

        if( f->isOdd )
            n = (f->A_nPlus1->v_m-f->A_n->v_m).cross(f->A_nPlus2->v_m-f->A_n->v_m).normalized();
        else
            n = (f->A_nPlus2->v_m-f->A_n->v_m).cross(f->A_nPlus1->v_m-f->A_n->v_m).normalized();


        //double valn = sqrt(nx*nx+ny*ny+nz*nz);

        /*double n0x = nx/valn;
        double n0y = ny/valn;
        double n0z = nz/valn;*/

        //Caluculate a
        A =n[0]; //n0x;
        //Calculate b
        B = n[1];//n0y;
        //Calculate c
        double C = n[2];//n0z;
        //Caluculate d
        D = A0x*A+A0y*B+A0z*C;

        //For each point
        double c_dist=0;
        for (size_t i=0;i<f->X.size();i++)
        {
            x = f->X.at(i)->Xm;
            //Calculate distance
            c_dist = (A*x[0]+B*x[1]+C*x[2]-D);

            //g = f->X.at(i)->c; //Confidence.. experimental!!
            g=1.0;

            //Summing up
            dist += g*c_dist;
            Sdist += absoluteError ? g*fabs(c_dist) : g*(c_dist*c_dist);
        }
    }

    return Sdist;
}

double Fitting::orthogonalErrorAtAnchor(double h, void* params)
{
    DistanceFunctionParams *p = (DistanceFunctionParams*)params;

    double e_dist = orthogonalDistance(h,p);
    double e_smooth = smoothnessError(p->a,h,false,p->absolute_error);

    double e = e_dist + p->alpha*e_smooth;

    return e;
}


double Fitting::zErrorAtAnchor(double h, void* params)
{
    DistanceFunctionParams *p = (DistanceFunctionParams*)params;

    double e_dist = zDistance(h,p);
    double e_smooth = smoothnessError(p->a,h,false,p->absolute_error/*false*/);

    double e = e_dist + p->alpha*e_smooth;

    //std::clog<<e_dist<<"+"<<p->alpha<<"*"<<e_smooth<<"="<<e<<std::endl;
    return e;
}

/*
 * To calculate the smoothness wieght adaptively.. not tested and not in use!
 */
void Fitting::getWeights( RTM::Anchor* a, double e_res, double* alpha, double *beta )
{
    int n_FwithX = 0;
    int nX=0;

    for(int i=0; i<a->F.size(); i++)
    {
        nX += a->F[i]->X.size();
        if(a->F[i]->X.size()>0)
            n_FwithX++;
    }
    if(n_FwithX>0)
    {
        *beta = ( 1/(1+(e_res)) );
        *alpha = 1-*beta;
    }
    else
    {
        *alpha = 1;
        *beta = 0;
    }

}

//No adaptive smoothness possible here
double Fitting::fitFanSquaredZDistance(RTM::Anchor* a, double alpha)
{
    DistanceFunctionParams p(a,alpha,false,false,false);

    double df1 = diffSquaredZDistance(0, &p ) + alpha*diffSquaredSmoothnessError(a,0,false);
    double df2 = diffSquaredZDistance(1, &p ) + alpha*diffSquaredSmoothnessError(a,1,false);
    double df = df1-df2;

    double newA0z = 0+(df1/df);

    return newA0z;
}


double Fitting::fitFanSquaredOrthogonalDistance(RTM::Anchor* anchor, double alpha, bool adaptiveSmoothness)
{
    DistanceFunctionParams p(anchor,alpha,false, false, adaptiveSmoothness);

    //initial error
    //double e_in = orthogonalErrorAtAnchor( anchor->getH(), &p );

    //Optimize
    double m = fitFanBrent(anchor, p, &orthogonalErrorAtAnchor );

    //Error after optimization
    //double e_out = orthogonalErrorAtAnchor( anchor->getH(), &p );

    return m;
}

double Fitting::fitFanAbsoluteOrthogonalDistance(RTM::Anchor* anchor, double alpha, bool adaptiveSmootheness)
{
    DistanceFunctionParams p(anchor,alpha,false, true, adaptiveSmootheness);

    //initial error
    //double e_in = orthogonalErrorAtAnchor( anchor->getH(), &p );

    //Optimize
    double m = fitFanBrent(anchor, p, &orthogonalErrorAtAnchor );

    //Error after optimization
    //double e_out = orthogonalErrorAtAnchor( anchor->getH(), &p );

    return m;
}

double Fitting::fitFanAbsoluteZDistance(RTM::Anchor* anchor, double alpha, bool adaptiveSmoothness)
{
    DistanceFunctionParams p(anchor,alpha,false, true, adaptiveSmoothness);

    //initial error
    //double e_in = zErrorAtAnchor( anchor->getH(), &p );

    //Optimize
    double m = fitFanBrent(anchor, p, &zErrorAtAnchor );

    //Error after optimization
    //double e_out = zErrorAtAnchor( anchor->getH(), &p );

    return m;
}

double Fitting::fitFanSquaredZDistanceBrent(RTM::Anchor* anchor, double alpha, bool adaptiveSmoothness)
{
    DistanceFunctionParams p(anchor,alpha,false, false, adaptiveSmoothness);

    //initial error
    //double e_in = zErrorAtAnchor( anchor->getH(), &p );

    //Optimize
    double m = fitFanBrent(anchor, p, &zErrorAtAnchor );

    //Error after optimization
    //double e_out = zErrorAtAnchor( anchor->getH(), &p );

    return m;
}

double Fitting::fitFanBrent(RTM::Anchor* anchor, DistanceFunctionParams p, double (*geometricErrorFunction)(double, void*))
{
    //Ankerhöhe schätzen mit simpleFit
    double h = anchor->getH();
    double e_o = geometricErrorFunction( h, (void*)&p );

    //Initialize minimization algorithm
    int status;
    int iter = 0, max_iter = 100;
    const gsl_min_fminimizer_type *T;
    gsl_min_fminimizer *s;
    double m = h;
    double m_expected = m;

    //Schranken für brent algorithmus so mit den höhen initialisieren in denen
    //der smoothness-Fehler allein größer währe als der Entfernungsfehler
    //in der heuristik
    double* ab;
    double a;
    double b;
    if(p.alpha!=0)
    {
        //! TODO: FEHLERHAFT!!! Muss abgepasst werden für absoluten fehler! Berechnung stimmt nur für squared!
        ab=heightWithSmoothnessError(anchor, e_o, p.alpha,p.absolute_error);
        a = /*h-10;//*/ab[0]>ab[1] ? ab[1]:ab[0];
        b = /*h+10;//*/ab[1]>ab[1] ? ab[0]:ab[1];
    }
    else
    {
        //Nicht so gut.. bessere idee?
        ab=heightWithSmoothnessError(anchor, e_o, 0.0001,p.absolute_error);
        a = /*h-10;//*/ab[0]>ab[1] ? ab[1]:ab[0];
        b = /*h+10;//*/ab[1]>ab[1] ? ab[0]:ab[1];
    }

    gsl_function F;

    F.function = geometricErrorFunction;
    F.params =  (void*)&p;

    T = gsl_min_fminimizer_brent;
    s = gsl_min_fminimizer_alloc (T);

    gsl_set_error_handler_off();
    status = gsl_min_fminimizer_set (s, &F, m, a, b);

    if(status==GSL_FAILURE)
    {
        std::clog<<"ERROR: endpoints do not enclose a minimum. Taking verySimpleFit-value"<<std::endl;
        return h;
    }

    /*
    printf ("using %s method\n",
            gsl_min_fminimizer_name (s));

    printf ("%5s [%9s, %9s] %9s %10s %9s\n",
            "iter", "lower", "upper", "min",
            "err", "err(est)");

    printf ("%5d [%.7f, %.7f] %.7f %+.7f %.7f\n",
            iter, a, b,
            m, m - m_expected, b - a);*/

    do
    {
        iter++;
        status = gsl_min_fminimizer_iterate (s);

        m = gsl_min_fminimizer_x_minimum (s);
        a = gsl_min_fminimizer_x_lower (s);
        b = gsl_min_fminimizer_x_upper (s);

        status = gsl_min_test_interval (a, b, 0.0005, 0.0001); //Test for convergence

        /*if (status == GSL_SUCCESS)
            printf ("Converged:\n");

        printf ("%5d [%.7f, %.7f] "
                "%.7f %+.7f %.7f\n",
                iter, a, b,
                m, m - m_expected, b - a);*/
        //std::clog << "m: "<<m<<", a: "<<a<<", b: "<<b<<std::endl;

    }
    while (status == GSL_CONTINUE && iter < max_iter);

    gsl_min_fminimizer_free (s);


    return m;
}

void Fitting::plotSmoothness(RTM::Anchor* a)
{
    double e,de;
    std::fstream fstream;
    fstream.open("/Volumes/Data/Documents/uni/Diplomarbeit/Plots/e_smooth_abs.csv",std::fstream::out);
    for (float h=-1;h<2;h+=0.01)
    {
        e=smoothnessError(a,h,false,true);
        //de=diffSquaredSmoothnessError(a,h,false);
        fstream << h << "," << e << "," << de << std::endl;
    }
    fstream.close();
}
