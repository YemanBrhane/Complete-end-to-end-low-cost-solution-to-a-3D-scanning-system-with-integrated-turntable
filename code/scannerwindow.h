#ifndef SCANNERWINDOW_H
#define SCANNERWINDOW_H

#include <QMainWindow>


// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
// Grabber
//#include "kinect2grabber.h"
#include <Windows.h>

//#include <pcl/point_types.h>
//#include <pcl/filters/passthrough.h>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>



typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


//typedef pcl::PointCloud<pcl::PointXY> PointCloudXYZ;

//pcl::PointCloud<pcl::PointXYZ>

namespace Ui {
class ScannerWindow;
}

class ScannerWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ScannerWindow(QWidget *parent = 0);
    ~ScannerWindow();
    bool scanStatrt;

private slots:

    void on_openPreviousScanWindowButton_clicked();

    void on_newScanWindowButton_clicked();

    //void on_pushButton_clicked();

    void on_previousScanWindowHomeButton_clicked();

    void on_openImageButton_clicked();

    void on_registerButon_clicked();

    void on_helpAndAboutUsButton_clicked();

    void on_helpButton_clicked();

    void on_aboutUsButton_clicked();

    void on_aboutUsBackButton_clicked();

    void on_hepBackButton_clicked();

    void on_nextCalibrationButton_clicked();

    void on_homeCalibrationButton_clicked();



    void on_newScanFrameHomeButton_clicked();

//    void on_xMinSlider_valueChanged(int value);



//    void on_xMaxSlider_valueChanged(int value);

//    void on_yMinSlider_valueChanged(int value);

//    void on_yMaxSlider_valueChanged(int value);

//    void on_zMinSlider_valueChanged(int value);

//    void on_zMaxSlider_valueChanged(int value);

    void on_helpAndAboutUsHomeButton_clicked();

    void on_filterButton_clicked();

    void on_recordButton_clicked();


private:
    Ui::ScannerWindow *ui;
    PointCloudT::Ptr cloud;


    // Registration output
//    pcl::PointCloud<PointT>::Ptr register_out;
    PointCloudT::Ptr pcdOpen;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
   // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3;
    int cmp = 0;

    unsigned int red;
    unsigned int green;
    unsigned int blue;
};

#endif // SCANNERWINDOW_H
