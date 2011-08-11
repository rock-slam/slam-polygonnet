#include "RegularTriangularMesh.h"

RTM::AddPointsThread::AddPointsThread(RTM::RegularTriangularMesh *mesh)
{
    this->mesh = mesh;
    this->pointsToAddMutex = new QMutex(QMutex::NonRecursive);
    pointsToAddMutex->unlock();
    this->loop = true;
}

RTM::AddPointsThread::~AddPointsThread()
{
    delete pointsToAddMutex;
}

bool RTM::AddPointsThread::appendPoints(QList<Point1>points)
{
    if(pointsToAddMutex->tryLock(-1))
    {
        t1.start();
	if(pointsToAdd.size()+points.size() < MAX_POINTS_BUFFER)
	{
	    this->pointsToAdd.append(points);

	    pointsToAddMutex->unlock();
	    return true;
	}
	pointsToAddMutex->unlock();
    }
    return false;
}

bool RTM::AddPointsThread::appendPoints(QString filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return false;

    QTextStream in(&file);
    Point1 P;
    bool ok;
    while (!in.atEnd())
    {
        ok=true;
        QString line = in.readLine();
        QStringList l = line.split(' ');
        P.x = l.at(0).toDouble(&ok);
        if(!ok)
            continue;
        P.y = l.at(1).toDouble(&ok);
        if(!ok)
            continue;
        P.z = l.at(2).toDouble(&ok);
        if(!ok)
            continue;
        P.c = l.at(3).toDouble(&ok);
        if(!ok)
            continue;

        pointsToAdd.append(P);
    }
    file.close();

    return true;
}

bool RTM::AddPointsThread::stop()
{
    this->loop = false;
    return true;
}

void RTM::AddPointsThread::run()
{
    loop=true;
    while(loop)
    {
	if( pointsToAddMutex->tryLock(-1) )
	{
	    if(pointsToAdd.size()>0)
	    {
		do
		{

		    mesh->addPointWorldCoordinates(pointsToAdd[0].x,
						   pointsToAdd[0].y,
						   pointsToAdd[0].z,
						   pointsToAdd[0].c);
		    pointsToAdd.pop_front();
		}while(pointsToAdd.size()>5000);
		pointsToAddMutex->unlock();
                usleep(10);
	    }
	    else
	    {
		pointsToAddMutex->unlock();
                msleep(500);
	    }

	}
	else
	{
	    std::clog<<"run: locked"<<std::endl;
            msleep(500);
	}
    }
}

