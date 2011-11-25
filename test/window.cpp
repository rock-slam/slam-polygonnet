#include "window.h"
#include <QtGui>

window::window()
{
    ui.setupUi(this);
    e_sum_z=e_sum_o=e_sum_s=e_total_z=e_total_o=-1;
    it_z=it_o=0;
    t_z=t_o=t_s=-1;

    RTM::RegularTriangularMesh *mesh = new RTM::RegularTriangularMesh();
    ui.wGLWidget->mesh = mesh;
    
    updateMeshValues();

    connect(ui.dsbAlpha,SIGNAL(valueChanged(double)),ui.wGLWidget->mesh->fitthread,SLOT(setAlpha(double)));
    connect(ui.cbFitStrategy,SIGNAL(currentIndexChanged(int)),ui.wGLWidget->mesh->fitthread,SLOT(setStrategy(int)));
    connect(ui.cbUpdateStatistics,SIGNAL(clicked(bool)),ui.wGLWidget->mesh->fitthread,SLOT(setUpdateStatistics(bool)));
    ui.wGLWidget->mesh->fitthread->setAlpha(ui.dsbAlpha->value());
    ui.wGLWidget->mesh->fitthread->setUpdateStatistics(ui.cbUpdateStatistics->checkState());
}

void window::updateGL()
{
    ui.wGLWidget->updateGL();
}

void window::updateMeshValues()
{
    ui.wGLWidget->mesh->setOrigin( ui.dspOriginX->value(), ui.dspOriginY->value(), ui.dspOriginZ->value() );
    ui.wGLWidget->mesh->setU( ui.dspUX->value(), ui.dspUY->value(), ui.dspUZ->value() );
    ui.wGLWidget->mesh->setV( ui.dspVX->value(), ui.dspVY->value(), ui.dspVZ->value() );
    ui.wGLWidget->mesh->setNU( ui.sbNu->value() );
    ui.wGLWidget->mesh->setNV( ui.sbNv->value() );
    
    ui.wGLWidget->mesh->update();
    updateGL();
    it_o=0;
    it_z=0;
}

void window::on_pbUpdate_clicked()
{
    updateMeshValues();
}

void window::on_pbSetHeightSet_clicked()
{
    ui.wGLWidget->mesh->setAnchorHeight( ui.sbSetHeightU->value(), ui.sbSetHeightV->value(), ui.dsbSetHeightH->value() );
    ui.wGLWidget->highlightFan( ui.sbSetHeightU->value(), ui.sbSetHeightV->value() );
    updateGL();
}

void window::on_pbAddPoint_clicked()
{
    int id;
    id = ui.wGLWidget->mesh->addPointModelCoordinates( ui.dsbAddPointX->value(), ui.dsbAddPointY->value(), ui.dsbAddPointZ->value() );
    
    if(id < 0)
    {
	QMessageBox msgBox;
	msgBox.setText("Point could not be added. Mesh reported an error");
	msgBox.exec();
    }
    else
    {
	ui.wGLWidget->highlightFace( id );
	
	//Calculate statistics
        //ui.wGLWidget->mesh->calculateStatistics();
	
	updateGL();
    }
    it_o=0;
    it_z=0;
}

void window::on_pbLoadPointCloud_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
						    tr("Open Point Cloud"),
						    "./",
						    tr("Point Clouds (*.txt *.3d *.ply)"));
    
    if(fileName.isEmpty() || fileName.isNull())
	return;
    
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	return;

    file.close();

    this->ui.wGLWidget->mesh->addPointCloud(fileName);
    
    //Draw
    updateGL();
    it_o=0;
    it_z=0;

  /*char buf[100];
  for(int img = 2; img < 6000; img +=5)
  {
    sprintf(buf, "/home/likewise-open/DFKI/shi/workspace/rimres/image_processing/visual_odometry/build/test/scan%4.4d.3d", img);
    QString fileName(buf);
    std::cout<<buf<<std::endl;
    if(fileName.isEmpty() || fileName.isNull())
        return;

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    file.close();

    this->ui.wGLWidget->mesh->addPointCloud(fileName);
    updateGL();
    sleep(3);
  }*/
}

void window::on_pbImportHM_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open Heightmap"),
                                                    ".",
                                                    tr("Point Clouds (*.png *.jpg) *.tif *.tiff"));
    if(fileName.isEmpty() || fileName.isNull())
        return;
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    file.close();
    this->ui.wGLWidget->mesh->importHeightmap((std::string)fileName.toAscii().data());
    updateGL();

}

void window::on_pbExportHM_clicked()
{
    this->ui.wGLWidget->mesh->exportHeightmap(512,512,"heightmap.jpg");
    updateGL();
}

void window::on_pbFit_clicked()
{
    
    if(ui.pbFit->isChecked())
    {
	RTM::FITTING_STRATEGY FS;
	
	switch(ui.cbFitStrategy->currentIndex())
	{
	case 0:
	    FS = RTM::FS_SQUARED_Z_DISTANCE;
	    it_z++;
	    break;
	case 1:
	    FS = RTM::FS_SQUARED_ORTHOGONAL_DISTANCE;
	    it_o++;
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
	
	ui.wGLWidget->mesh->fitToPoints(FS,ui.dsbAlpha->value());
    }
    else
    {
	ui.wGLWidget->mesh->fitthread->cancelFitting();
    }
    
    /*QTime t;
    t.start();
    ui.wGLWidget->mesh->fitToPoints( FS, ui.dspAlpha->value() );
    if(FS == RTM::FS_SQUARED_Z_DISTANCE) t_z = t.elapsed();
    if(FS == RTM::FS_SQUARED_ORTHOGONAL_DISTANCE) t_o = t.elapsed();
    if(FS == RTM::FS_SIMPLE_FIT) t_s = t.elapsed();*/
    
    updateGL();
}

void window::on_cbFitStrategy_currentIndexChanged(int idx)
{
    RTM::FITTING_STRATEGY FS;
    
    switch(idx)
    {
    case 0:
	FS = RTM::FS_SQUARED_Z_DISTANCE;
	it_z++;
	break;
    case 1:
	FS = RTM::FS_SQUARED_ORTHOGONAL_DISTANCE;
	it_o++;
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
    
    if(ui.wGLWidget->mesh->fitthread)
	ui.wGLWidget->mesh->fitthread->setStrategy(FS);
}

void window::on_pbCancelFit_clicked()
{
    if(ui.wGLWidget->mesh->fitthread)
	ui.wGLWidget->mesh->fitthread->cancelFitting();
}

void window::on_cbMesh_clicked()
{
    ui.wGLWidget->drawMesh=ui.cbMesh->isChecked();
    updateGL();
}

void window::on_cbNormals_clicked()
{
    ui.wGLWidget->drawNormals=ui.cbNormals->isChecked();
    updateGL();
}

void window::on_cbPointCloud_clicked()
{
    ui.wGLWidget->drawPointCloud=ui.cbPointCloud->isChecked();
    updateGL();
}

void window::on_cbFaces_clicked()
{
    ui.wGLWidget->drawFaces=ui.cbFaces->isChecked();
    updateGL();
}

void window::on_cbLight_clicked()
{
    ui.wGLWidget->light=ui.cbLight->isChecked();
    updateGL();
}

void window::on_cbOccupancy_clicked()
{
    ui.wGLWidget->drawOccupancy=ui.cbOccupancy->isChecked();
    updateGL();
}

void window::on_cbStdDev_clicked()
{
    ui.wGLWidget->drawStdDev=ui.cbStdDev->isChecked();
    updateGL();
}

void window::on_cbUpdated_clicked()
{
    ui.wGLWidget->drawUpdated=ui.cbUpdated->isChecked();
    updateGL();
}

void window::on_cbEGeo_clicked()
{
    if(ui.cbEGeo->isChecked())
    {
        if(ui.cbFitStrategy->currentIndex()==0)
            ui.wGLWidget->drawError=1;
        if(ui.cbFitStrategy->currentIndex()==1)
            ui.wGLWidget->drawError=2;
        if(ui.cbFitStrategy->currentIndex()==2)
            ui.wGLWidget->drawError=2;
        if(ui.cbFitStrategy->currentIndex()==3)
            ui.wGLWidget->drawError=3;
        if(ui.cbFitStrategy->currentIndex()==4)
            ui.wGLWidget->drawError=4;
    }
    else
        ui.wGLWidget->drawError=0;

    updateGL();
}
void window::on_cbESmooth_clicked()
{
    ui.wGLWidget->drawSmoothError=ui.cbESmooth->isChecked();
    updateGL();
}

void window::on_pbResetAnchorHeights_clicked()
{
    ui.wGLWidget->mesh->resetAnchorHeights();
    updateGL();
    it_o=0;
    it_z=0;
}

//
// Info
//

QString window::printInfo()
{
    QString text;
    
    RTM::RegularTriangularMesh* m = ui.wGLWidget->mesh;
    //double o;
    //double z;
    //double s;
    double *esq_z;
    double *esq_o;
    double *esq_s;
    double *eab_z;
    double *eab_o;
    double *eab_s;

    double alpha = m->fitthread->alpha;

    this->ui.wGLWidget->mesh->calculateStatistics(false);
    
    //
    // Error at anchor
    //
    /*if(ui.pbErrorAtAnchor->isChecked())
    {
	text.append(QString( "Error at anchor (%1,%2):" )
		    .arg( ui.sbInfoU->value() )
		    .arg( ui.sbInfoV->value() ));
	
        o = ui.wGLWidget->mesh->calculateDistanceErrorAtAnchor(ui.wGLWidget->mesh->getAnchor(ui.sbInfoU->value(),
										    ui.sbInfoV->value()),
						      RTM::FS_SQUARED_ORTHOGONAL_DISTANCE,false);
	text.append(QString("\n  Orthogonal distance: %1 ").arg(o));
        z = ui.wGLWidget->mesh->calculateDistanceErrorAtAnchor(ui.wGLWidget->mesh->getAnchor(ui.sbInfoU->value(),ui.sbInfoV->value()),
						      RTM::FS_SQUARED_Z_DISTANCE,false);
	text.append(QString("\n  z-Distance: %1 ").arg(z));
        s = ui.wGLWidget->mesh->calculateSmoothnessErrorAtAnchor(ui.wGLWidget->mesh->getAnchor(ui.sbInfoU->value(),ui.sbInfoV->value()),
							false);
	text.append(QString("\n  Smoothness error: %1 ").arg(s));
    }*/
    
    //
    // General Information
    //
    //if(ui.pbUpdateInfo->isChecked())
    {
        esq_z = m->zErrorSquared;
        esq_o = m->oErrorSquared;
        eab_z = m->zErrorAbs;
        eab_o = m->oErrorAbs;
        esq_s = m->smoothSquared;
        eab_s = m->smoothAbs;
	
	text.append( "General information:\n" );
	text.append(QString("  n_u: %1\n  n_v: %2\n  len_u: %3\n  len_v: %4\n  n_x: %5")
                    .arg(m->getNU()).arg(m->getNV()).arg(m->getLenU()).arg(m->getLenV()).arg(m->getNX()));
	
	//
	// Error
	//
        text.append("\n\n###########\n# Fitting #\n###########\n");
        text.append(QString("Currently selected Fitting Strategy: FS_SQAURED_Z_DISTANCE\n"));
        text.append(QString("\nSquared Error [e_sum, e_mean, e_min, e_max]:\n"));
        text.append(QString("  Sq. Z-Dist.: [%1, %2, %3, %4]\n")
                    .arg(esq_z[0],0,'f',3).arg(esq_z[1],0,'f',3).arg(esq_z[2],0,'f',3).arg(esq_z[3],0,'f',3));
        text.append(QString("  Sq. Orth.-D.: [%1, %2, %3, %4]\n")
                    .arg(esq_o[0],0,'f',3).arg(esq_o[1],0,'f',3).arg(esq_o[2],0,'f',3).arg(esq_o[3],0,'f',3));
        text.append(QString("  Sq. Smooth: [%1, %2, %3, %4]\n")
                    .arg(esq_s[0],0,'f',3).arg(esq_s[1],0,'f',3).arg(esq_s[2],0,'f',3).arg(esq_s[3],0,'f',3));
        text.append(QString("Total Sq.: [e_sum, e_mean]\n"));
        text.append(QString("  Sq. e_z+alpha*e_s: [%1, %2] (alpha=%3)\n")
                    .arg(m->totalSquaredZError[0],0,'f',3)
                    .arg(m->totalSquaredZError[1],0,'f',3)
                    .arg(alpha,0,'f',3));
        text.append(QString("  Sq. e_o+alpha*e_s: [%1, %2] (alpha=%3)\n")
                    .arg(m->totalSquaredOError[0],0,'f',3)
                    .arg(m->totalSquaredOError[1],0,'f',3)
                    .arg(alpha,0,'f',3));

        text.append(QString("\nAbsolute Error [e_sum, e_mean, e_min, e_max]:\n"));
        text.append(QString("  Abs. Z-Dist.: [%1, %2, %3, %4]\n")
                    .arg(eab_z[0],0,'f',3).arg(eab_z[1],0,'f',3).arg(eab_z[2],0,'f',3).arg(eab_z[3],0,'f',3));
        text.append(QString("  Abs. Orth.-D.: [%1, %2, %3, %4]\n")
                    .arg(eab_o[0],0,'f',3).arg(eab_o[1],0,'f',3).arg(eab_o[2],0,'f',3).arg(eab_o[3],0,'f',3));
        text.append(QString("  Abs. Smooth: [%1, %2, %3, %4]\n")
                    .arg(eab_s[0],0,'f',3).arg(eab_s[1],0,'f',3).arg(eab_s[2],0,'f',3).arg(eab_s[3],0,'f',3));
        text.append(QString("Total Abs.: [e_sum, e_mean]\n"));
        text.append(QString("  Abs. e_z+alpha*e_s: [%1, %2] (alpha=%3)\n")
                    .arg(m->totalAbsZError[0],0,'f',3)
                    .arg(m->totalAbsZError[1],0,'f',3)
                    .arg(alpha,0,'f',3));
        text.append(QString("  Abs. e_o+alpha*e_s: [%1, %2] (alpha=%3)\n")
                    .arg(m->totalAbsOError[0],0,'f',3)
                    .arg(m->totalAbsOError[1],0,'f',3)
                    .arg(alpha,0,'f',3));

        text.append(QString("\nFitting runtime information:\n  current Speed one lap: %1 ms\n  current Speed per Anchor: %2 ms\n  Time to converge: %3 sec\n  Iterations to converge: %4\n")
                    .arg(m->fitthread->currentSpeed_oneLap)
                    .arg(m->fitthread->currentSpeed_perAnchor)
                    .arg(m->fitthread->currentSpeed_untilConvergence/1000.0)
                    .arg(m->fitthread->iterations_untilConvergence));

	//
	// Statistics
	//
        text.append(QString("\n\n##############\n# Statistics #\n##############\n"));
	text.append(QString("  Time for last fit (Z-Distance): %1ms\n").arg(t_z));
	text.append(QString("  Time for last fit (Orthogonal-Distance): %1ms\n").arg(t_o));
	text.append(QString("  Time for last fit (SimpleFit): %1ms\n").arg(t_s));
	text.append(QString("  Time for last rendering: %1ms => %2 fps\n")
		    .arg(ui.wGLWidget->getT()).arg(1/(ui.wGLWidget->getT()/1000.0)));
	text.append(QString("  Iteration for Z-Distance fit: %1\n").arg(it_z));
	text.append(QString("  Iteration for Orthogonal-Distance fit: %1\n").arg(it_o));
	
	//
	// Mesh Statistics
	//
        text.append(QString("\nMesh Statistics:\n  std_dev: %1\n  max_dev: %2\n  occuied_ratio: %3\n  Avg. Anchor Occuoancy: %4\n  Avg. Face Occupany: %5\n  Avg. Anchor height: %6")
                    .arg(ui.wGLWidget->mesh->mean_std_dev).arg(ui.wGLWidget->mesh->max_std_dev)
                    .arg(ui.wGLWidget->mesh->occupied_ratio).arg(ui.wGLWidget->mesh->averageAnchorOccupancy)
                    .arg(m->averageFaceOccupancy).arg(m->averageAnchorHeight));
    }
    
    //
    // put text int text box
    //
    ui.teInfo->setText(text);
    
    return text;
}

void window::on_pbInfoFit_clicked()
{
    ui.pbFit->click();
}

void window::on_pbErrorAtAnchor_clicked()
{
    printInfo();
}

void window::on_pbUpdateInfo_clicked()
{
    printInfo();
}

void window::on_cbSignalConvergence_clicked()
{
    if(ui.cbSignalConvergence->checkState())
        connect(ui.wGLWidget->mesh->fitthread,SIGNAL(convergenceSignal()),this,SLOT(signalConvergence()));
    else
        disconnect(ui.wGLWidget->mesh->fitthread,SIGNAL(convergenceSignal()),this,SLOT(signalConvergence()));
}

void window::signalConvergence()
{
    QMessageBox msgBox;
    msgBox.setText(QString("Mesh converged after %1 seconds with %2 iterations (%3 seconds/iterations)")
                   .arg(ui.wGLWidget->mesh->fitthread->currentSpeed_untilConvergence/1000.0)
                   .arg(ui.wGLWidget->mesh->fitthread->iterations_untilConvergence)
                   .arg((ui.wGLWidget->mesh->fitthread->currentSpeed_untilConvergence/1000.0)/ui.wGLWidget->mesh->fitthread->iterations_untilConvergence));
    msgBox.exec();
}




//
// Camera
//
void window::on_pbLoadCamera_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
						    tr("Open camera calibration"),
						    "/Volumes/Data/Documents/uni/Diplomarbeit/workspace3/TestData",
						    tr("Camera calibration (*.txt *.cvMat)"));
    
    if(fileName.isEmpty() || fileName.isNull())
	return;
    
    CvMat *K = 0;
    K = (CvMat*)cvLoad(fileName.toLatin1().data());
    
    double *mat = K->data.db;
    delete ui.wGLWidget->cam;
    ui.wGLWidget->cam = new OGLCamera(ui.sbCamWidth->value(),ui.sbCamHeight->value(),mat[0], mat[4], mat[2], mat[5], 0.2, 1000);
    
    cvReleaseMat(&K);
    
    ui.wGLWidget->cam->initViewPort(ui.wGLWidget->size().width(),ui.wGLWidget->size().height());
    updateGL();
}

void window::on_pbSetCameraPose_clicked()
{
    double rx,ry,rz;
    
    ui.wGLWidget->cam->resetRotation();
    ui.wGLWidget->cam->resetPosition();
    
    rx = ui.dsbCamRx->value();
    ry = ui.dsbCamRy->value();
    rz = ui.dsbCamRz->value();
    if(!ui.cbCamRRadian->isChecked())
    {
	if(rx) rx = deg2rad(rx);
	if(ry) ry = deg2rad(ry);
	if(rz) rz = deg2rad(rz);
    }
    
    if(rx) ui.wGLWidget->cam->rotateGlobalX(rx);
    if(ry) ui.wGLWidget->cam->rotateGlobalY(ry);
    if(rz) ui.wGLWidget->cam->rotateGlobalZ(rz);
    
    ui.wGLWidget->cam->move(cml::vector3d(ui.dsbCamTx->value(),ui.dsbCamTy->value(),ui.dsbCamTz->value()));
    
    updateGL();
}

/*void window::on_pbCreateHeightMap_clicked()
{
    int nu = ui.sbHeightMapWidth->value();
    int nv = ui.sbHeightMapHeight->value();
    IplImage *heightmap16 = cvCreateImage(cvSize(nu,nv), IPL_DEPTH_16U, 1);
    double du = ui.wGLWidget->mesh->getLenU() / (double)nu;
    double dv = ui.wGLWidget->mesh->getLenV() / (double)nv;
    cml::vector3d udir = ui.wGLWidget->mesh->getU() / ui.wGLWidget->mesh->getLenU();
    cml::vector3d vdir = ui.wGLWidget->mesh->getV() / ui.wGLWidget->mesh->getLenV();

    double u,v;
    int idx;
    cml::vector3d dir(0,0,-1);
    cml::vector3d X, X_;
    for( int j=0; j<nv; j++)
    {
        v = (j*dv);
        for( int i=0; i<nu; i++)
        {
            u = (i*du);
            idx = ui.wGLWidget->mesh->mapModelToFaceIndex(cml::vector4d(u,v,5000,1));
            X[0]=u; X[1]=v; X[2]=5000;
            X_ = intersectLineTriangle( X, dir, ui.wGLWidget->mesh->F[idx].A_n->v_m,
                                   ui.wGLWidget->mesh->F[idx].A_nPlus1->v_m,
                                   ui.wGLWidget->mesh->F[idx].A_nPlus2->v_m );

            cvSet2D(heightmap16,j,nu-1-i,cvScalar((int)((X_[2]/ui.dsbHeightMapScale->value())*65536)));
        }
    }

    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Image File"),
                                                    "/Volumes/Data/Documents/uni/Diplomarbeit/workspace3/TestData",
                                                    tr("All files (*.*)"));

    if(fileName.isEmpty() || fileName.isNull())
        return;

    //write_png_file(fileName.toLatin1().data(),heightmap16);
    cvSaveImage(fileName.toLatin1().data(),heightmap16);

    cvReleaseImage(&heightmap16);
}*/







