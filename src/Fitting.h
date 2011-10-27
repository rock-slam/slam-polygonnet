#ifndef FITTING_H
#define FITTING_H

#include <polygonnet/RegularTriangularMesh.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_min.h>
#include <fstream>
#include <QThread>
#include <QList>

namespace Fitting
{
    /** fitting thread will be started to match the mesh to all the points that currently stored on the mesh using certain criteria (e.g. minimum z-distance errors)*/
    class FitThread : public QThread
    {
	Q_OBJECT
    public:
        FitThread(RTM::RegularTriangularMesh* mesh,int n_u,int n_v);
        inline ~FitThread(){}
	void run();
	void fit(int iterations, RTM::FITTING_STRATEGY FS, double alpha);

	RTM::RegularTriangularMesh* mesh;
	RTM::FITTING_STRATEGY FS;
	double alpha;
	int n_u;
	int n_v;
	int iterations;
	int iterations_to_do;
	bool adaptiveSmoothness;
        bool updateStatistics;

        int currentSpeed_untilConvergence;
        int iterations_untilConvergence;
        int currentSpeed_oneLap;
        float currentSpeed_perAnchor;

    private:
	bool cancelF;
        bool forceFit;

    public:
	void cancelFitting();
    public slots:
	inline void setStrategy(RTM::FITTING_STRATEGY FS){this->FS = FS;}
	void setStrategy(int fs);
        inline void setAlpha(double alpha){this->alpha = alpha; forceFit=true;}
        //inline void setAdaptiveSmoothness(bool adaptiveSmoothness){this->adaptiveSmoothness=adaptiveSmoothness;}
        inline void setUpdateStatistics(bool val){this->updateStatistics=val;}

    signals:
	void stoppedFitting(bool);
        void convergenceSignal();
    };

    struct DistanceFunctionParams
    {
	inline DistanceFunctionParams(RTM::Anchor* a, double alpha=0, bool globalError=false,
				      bool absolute_error=false, bool adaptiveSmoothness=true)
	{ this->a = a; this->alpha=alpha; this->globalError=globalError; this->absolute_error=absolute_error;
	  this->adaptiveSmoothness=adaptiveSmoothness;}

	RTM::Anchor* a;
	bool useBaseFunction;
	int p; //power for triangle base function
	bool globalError;
	double alpha;
	bool absolute_error; //Absolute error or squared (default)
	bool adaptiveSmoothness;
    };

    double verySimpleFitPlane(RTM::Anchor* a, double alpha);
    double optimalAnchorHeight(RTM::Face* f, RTM::Anchor *a, base::Vector3d X_m);

    void getWeights(RTM::Anchor* a, double e_res, double* alpha, double *beta);

    double triangleBaseFunction(RTM::Face *f, RTM::Anchor *a, double x, double y, int p=1);
    double smoothnessError(RTM::Anchor* a, double h, bool globalError=false,
                           bool absoluteError=false,double* _sumH=0,
                           int* _nNeighbours=0);
    double diffSquaredSmoothnessError(RTM::Anchor* a, double h, bool globalError=false);
    double* heightWithSmoothnessError(RTM::Anchor *a, double e_s, double alpha, bool absoluteError=false);

    double orthogonalDistance(double h, DistanceFunctionParams* params);
    double orthogonalErrorAtAnchor(double h, void* params);
    double fitFanSquaredOrthogonalDistance(RTM::Anchor* a, double alpha, bool adaptiveSmoothness);
    double fitFanAbsoluteOrthogonalDistance(RTM::Anchor* a, double alpha, bool adaptiveSmoothness);
    double fitFanAbsoluteZDistance(RTM::Anchor* a, double alpha, bool adaptiveSmoothness);

    double fitFanBrent(RTM::Anchor* anchor, DistanceFunctionParams p, double (*geometricErrorFunction)(double, void*));

    double zDistance(double h, DistanceFunctionParams* params);
    double diffSquaredZDistance(double h, DistanceFunctionParams* params);
    double zErrorAtAnchor(double h, void* params);
    double fitFanSquaredZDistance(RTM::Anchor* a, double alpha);
    double fitFanSquaredZDistanceBrent(RTM::Anchor* a, double alpha, bool adaptiveSmoothness);

    void plotOrthogonalDistance(RTM::Anchor *A, bool useBaseFunction, int p);
    void plotSmoothness(RTM::Anchor *A);
}

#endif // FITTING_H
