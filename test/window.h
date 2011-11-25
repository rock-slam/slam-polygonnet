#ifndef WINDOW_H
#define WINDOW_H

#include <QWidget>
#include "ui_Design.h"
#include "glwidget.h"
#include <QPushButton>
#include <QMessageBox>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "tools.h"

QT_BEGIN_NAMESPACE
	class QSlider;
QT_END_NAMESPACE

	class GLWidget;

class window : public QWidget
{
    Q_OBJECT
public:
    window();

private:
    Ui::Form ui;
    void updateGL();
    void updateMeshValues();
    QString printInfo();
    double e_sum_z,e_sum_o,e_sum_s,e_total_z,e_total_o;
    int t_z,t_o,t_s;
    int it_z,it_o; //iteration z, iteration orthogonal

private slots:
    //Controls
    void on_pbUpdate_clicked();
    void on_pbSetHeightSet_clicked();
    void on_pbAddPoint_clicked();
    void on_pbLoadPointCloud_clicked();
    void on_pbFit_clicked();
    void on_cbFitStrategy_currentIndexChanged(int idx);
    void on_pbCancelFit_clicked();
    void on_cbMesh_clicked();
    void on_cbNormals_clicked();
    void on_cbPointCloud_clicked();
    void on_pbResetAnchorHeights_clicked();
    void on_cbFaces_clicked();
    void on_cbLight_clicked();
    void on_cbOccupancy_clicked();
    void on_cbStdDev_clicked();
    void on_cbUpdated_clicked();
    void on_cbEGeo_clicked();
    void on_cbESmooth_clicked();
    void on_cbSignalConvergence_clicked();

    void signalConvergence();

    //Info
    void on_pbInfoFit_clicked();
    void on_pbErrorAtAnchor_clicked();
    void on_pbUpdateInfo_clicked();
    void on_pbImportHM_clicked();
    void on_pbExportHM_clicked();

    //Camera
    void on_pbLoadCamera_clicked();
    void on_pbSetCameraPose_clicked();
};

#endif // WINDOW_H
