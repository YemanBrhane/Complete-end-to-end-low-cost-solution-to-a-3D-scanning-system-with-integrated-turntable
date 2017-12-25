
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include <iostream>
#include "scannerwindow.h"
#include "ui_scannerwindow.h"

#include <vtkAutoInit.h>
// registration and 3D view
#include "registration.h"

// Grabber
#include "kinect2grabber.h"
#include<QElapsedTimer>
#include <thread>
#include <vector>
#include <chrono>
#include  <QDebug>
#include <QString>

// For Arduino

#include <iostream>
#include <C:\Qt\Qt5.7.0\5.7\msvc2015_64\include\QtSerialPort\QSerialPort>
#include <QDebug>
#include <Windows.h>
#include <QElapsedTimer>
#include <dos.h>
#include <stdio.h>
#include <conio.h>

// Filter & Surface Libraries

#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply/io_operators.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/PolygonMesh.h>
#include <math.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/grid_projection.h>


using namespace std;
QSerialPort serial;
QElapsedTimer timer;
typedef pcl::PointXYZRGBA PointType;


int i = 1;

QByteArray DataOutToArduino;

// Grabber variables: global variables

double Xmin=0.0, Xmax=0.0, Ymin=0.0, Ymax=0.0,Zmin=0.0, Zmax=0.0;

#include <QString>
#include <QMessageBox>

// File access header file

#include<QFile>
#include<QDir>
#include<QFileDialog>

// this is for Warning Window
/*
 vtkOutputWindow *w = vtkFileOutputWindow::New();
w->SetFileName("vtk_errors.txt")
vtkOutputWindow::SetInstance(w);
w->Delete(); // now SetInstance owns the reference
*/
ScannerWindow::ScannerWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ScannerWindow)
{
    ui->setupUi(this);


    ui->homeFrame->setVisible(true);
    ui->helpAndAboutusFrame->setVisible(false);
    ui->CaliberationFrame->setVisible(false);
    ui->helpFrame->setVisible(false);
    ui->aboutUsFrame->setVisible(false);
    ui->viewPreviousScanWindowFrame->setVisible(false);
    ui->newScanFrame->setVisible(false);

    // set default directory
    // QDir dir('E:/MAIA Course/Software Engineering/Project/Scanner_3D_GUI');
    //QDir dir;
    //foreach (QFileInfo var, dir.drives()){};
}

ScannerWindow::~ScannerWindow()
{
    delete ui;
}

void ScannerWindow::on_openPreviousScanWindowButton_clicked()
{
    ui->homeFrame->setVisible(false);
    ui->helpAndAboutusFrame->setVisible(false);
    ui->CaliberationFrame->setVisible(false);
    ui->helpFrame->setVisible(false);
    ui->aboutUsFrame->setVisible(false);
    ui->viewPreviousScanWindowFrame->setVisible(true);
    ui->newScanFrame->setVisible(false);

    // Setup the clo ud pointer
    cloud.reset (new PointCloudT);
    // The number of points in the cloud
    cloud->points.resize (1000);

    // The default color
    red   = 128;
    green = 128;
    blue  = 128;

    // Fill the cloud with some points
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);

        cloud->points[i].r = red;
        cloud->points[i].g = green;
        cloud->points[i].b = blue;
    }


    // Set up the QVTK window
        viewer1.reset (new pcl::visualization::PCLVisualizer ("viewer1", false));
        ui->qvtkWidget->SetRenderWindow (viewer1->getRenderWindow ());
        viewer1->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());

        viewer1->addPointCloud (cloud, "cloud");
        viewer1->resetCamera ();

    //ui->qvtkWidget->update ();

//    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("test_pcd.pcd", *cloud2) == -1) //* load the file
//    {
//      PCL_ERROR ("Couldn't read file \n");
//      //return (-1);
//    }

//    viewer->updatePointCloud(cloud2, "cloud");

//    ui->qvtkWidget->update ();
}

void ScannerWindow::on_newScanWindowButton_clicked()
{
    ui->homeFrame->setVisible(false);
    ui->helpAndAboutusFrame->setVisible(false);
    ui->CaliberationFrame->setVisible(true);
    ui->helpFrame->setVisible(false);
    ui->aboutUsFrame->setVisible(false);
    ui->viewPreviousScanWindowFrame->setVisible(false);
    ui->newScanFrame->setVisible(false);

    // Update status
    //ui->scanStatusLabel->setText("Viewer Calibration");
    // don't start saving scan
    scanStatrt = false;

    // Gabber

    bool counter;
    QElapsedTimer timer;
    timer.start();
    vector<pcl::PointCloud<PointType>> filtered_data;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer( "Point Cloud Viewer" ) );
    viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0 , 0.0 , 0.0 );

    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;

    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;

    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =[&cloud, &mutex ]( const pcl::PointCloud<PointType>::ConstPtr& ptr )
    {

      boost::mutex::scoped_lock lock( mutex );
      cloud = ptr->makeShared();

    };

    // Kinect2Grabber
    boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );

    serial.setPortName("COM3");
    serial.open(QIODevice::ReadWrite);
    serial.setBaudRate(QSerialPort::Baud9600);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);

    grabber->start();

    while(!viewer->wasStopped())
    {
      counter =false;
        // Update Viewer


      viewer->spinOnce();

      boost::mutex::scoped_try_lock lock( mutex );
      if( lock.owns_lock() && cloud )
      {
        // Update Point Cloud
        pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr cloud_filtered1 (new pcl::PointCloud<PointType>);
        // Create the filtering object
        pcl::PassThrough<PointType> pass;

        pass.setFilterFieldName ("x");
        pass.setFilterLimits (ui->xMinSlider->value()/10.0, ui->xMaxSlider->value()/10.0);

        pass.setInputCloud (cloud);
        pass.filter (*cloud_filtered);

        pass.setFilterFieldName ("y");
        pass.setFilterLimits (ui->yMinSlider->value()/50.0, ui->yMaxSlider->value()/50.0);

        pass.setInputCloud (cloud_filtered);
        pass.filter (*cloud_filtered1);

        pass.setFilterFieldName ("z");
        pass.setFilterLimits (ui->yMinSlider->value()/10.0, ui->zMaxSlider->value()/10.0);

        pass.setInputCloud (cloud_filtered1);
        pass.filter (*cloud_filtered);

        if (timer.elapsed()>= 2500)
        {

            filtered_data.push_back(*cloud_filtered);
            counter = false;

            if (scanStatrt)
            {
                while(!serial.isOpen()) serial.open(QIODevice::ReadWrite);
                if (serial.isOpen() && serial.isWritable())
                {

                    DataOutToArduino[0]=0;
                    serial.write(DataOutToArduino);
                    serial.flush();
                    timer.start();
                    qDebug()<< "Saving Model, Halting Turntable";
                    serial.waitForBytesWritten(100);

                    stringstream ss;
                    ss << ".\\"<<ui->toBeScanPersonNameLineEdit->text().toStdString()<< i << ".pcd";
                    string filename = ss.str();
                    i++;
                    cmp++;
                    std::cout<<filename<<std::endl;

                    pcl::io::savePCDFile(filename, *cloud_filtered);

                    DataOutToArduino[0]=1;
                    serial.write(DataOutToArduino);
                    serial.flush();
                    serial.waitForBytesWritten(100);

                 }
           }

         }


            if( !viewer->updatePointCloud( cloud_filtered, "cloud" ) )
             {
                viewer->addPointCloud( cloud_filtered, "cloud" );
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
             }

        }
   }


    // Stop Grabber
    grabber->stop();

  if( connection.connected() )
  {

        connection.disconnect();

  }

   //Sending 2 via the serial bus
   DataOutToArduino[0]=2;
   serial.write(DataOutToArduino);
   serial.flush();
   serial.waitForBytesWritten(100);
   //This stops all the output from the arduino, hence this stops the turntable
}


void ScannerWindow::on_previousScanWindowHomeButton_clicked()
{
    ui->homeFrame->setVisible(true);
    ui->viewPreviousScanWindowFrame->setVisible(false);
    ui->newScanFrame->setVisible(false);
}

void ScannerWindow::on_openImageButton_clicked()
{
    QString filename= QFileDialog::getOpenFileName(this,"Open pcd File", QDir::homePath(),"ply Files (*.ply)");
    pcdOpen.reset (new PointCloudT);
    pcl::PolygonMesh mesh;
    std::string pathName(filename.toStdString());
    pcl::io::loadPLYFile(pathName, mesh);
    viewer1.reset (new pcl::visualization::PCLVisualizer ("viewer2", false));
    ui->qvtkWidget->SetRenderWindow (viewer1->getRenderWindow ());
    viewer1->setupInteractor (ui->qvtkWidget->GetInteractor (),
                              ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();
    viewer1->addPolygonMesh (mesh, pathName);
    viewer1->resetCamera ();

}

void ScannerWindow::on_registerButon_clicked()
{
    viewer2.reset(new pcl::visualization::PCLVisualizer("viewer" , false) );
    ui->qvtkWidgetNewScan->SetRenderWindow (viewer2->getRenderWindow ());
    viewer2->setupInteractor (ui->qvtkWidgetNewScan->GetInteractor (),
                              ui->qvtkWidgetNewScan->GetRenderWindow ());
    //Update status
    ui->regFilterStatusLabel->setText("Registering....");
    //New registration

    //remove this
    //int pcdNum=60;
    std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
    loadData (ui->scanNameLineEdit->text().toStdString(), data, cmp);

    pcl::PointCloud<PointT>::Ptr cloud_previous (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_current (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_merged (new pcl::PointCloud<PointT>);


    pcl::PointCloud<PointT>::Ptr result (new pcl::PointCloud<PointT>);
    pcl::IterativeClosestPoint<PointT, PointT> icp;


    //SMOOTHING
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_sor (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_downsampled (new pcl::PointCloud<PointT>);

    result = data[0].cloud;

    for (size_t i = 1; i < data.size () ; i++)
    {
           cloud_previous = result ;
           cloud_current = data[i].cloud;

           int iterations = 100 ;

           icp.setInputCloud (cloud_previous);
           icp.setInputTarget (cloud_current);
           icp.setMaxCorrespondenceDistance(0.015);
           icp.setTransformationEpsilon (1e-8);
           icp.setMaximumIterations (iterations);
           icp.setRANSACOutlierRejectionThreshold(0.03);
           icp.align (*cloud_merged);

           Eigen::Matrix4f transform = icp.getFinalTransformation();
           if (icp.hasConverged ())
           {
               pcl::transformPointCloud(*cloud_previous, *cloud_merged, transform);
               *cloud_merged += *cloud_current;
           }
           else
           {
             PCL_ERROR ("\n.Not Converged\n");

           }

           pcl::VoxelGrid<PointT> grid;
           grid.setInputCloud (cloud_merged);
           grid.setLeafSize (0.01f, 0.01f, 0.01f);
           grid.filter (*cloud_downsampled);

           result = cloud_downsampled;

      }// end of for loop

    stringstream fname;
    fname <<ui->scanNameLineEdit->text().toStdString()<< "3D" << ".ply";
    string filename = fname.str();

      pcl::io::savePLYFile(filename, *cloud_downsampled);

      viewer2->addPointCloud (cloud_downsampled, "cloud");
      viewer2->resetCamera ();

      ui->regFilterStatusLabel->setText("Registering Fnished");

}




void ScannerWindow::on_helpAndAboutUsButton_clicked()
{

    ui->helpAndAboutusFrame->setVisible(true);
    ui->CaliberationFrame->setVisible(false);
    ui->helpFrame->setVisible(false);
    ui->aboutUsFrame->setVisible(false);
    ui->viewPreviousScanWindowFrame->setVisible(false);
    ui->newScanFrame->setVisible(false);
    ui->homeFrame->setVisible(false);
}

void ScannerWindow::on_helpButton_clicked()
{
    ui->helpAndAboutusFrame->setVisible(false);
    ui->CaliberationFrame->setVisible(false);
    ui->helpFrame->setVisible(true);
    ui->aboutUsFrame->setVisible(false);
    ui->viewPreviousScanWindowFrame->setVisible(false);
    ui->newScanFrame->setVisible(false);
    ui->homeFrame->setVisible(false);
}

void ScannerWindow::on_aboutUsButton_clicked()
{
    ui->helpAndAboutusFrame->setVisible(false);
    ui->CaliberationFrame->setVisible(false);
    ui->helpFrame->setVisible(false);
    ui->aboutUsFrame->setVisible(true);
    ui->viewPreviousScanWindowFrame->setVisible(false);
    ui->newScanFrame->setVisible(false);
    ui->homeFrame->setVisible(false);
}

void ScannerWindow::on_aboutUsBackButton_clicked()
{
    ui->helpAndAboutusFrame->setVisible(true);
    ui->CaliberationFrame->setVisible(false);
    ui->helpFrame->setVisible(false);
    ui->aboutUsFrame->setVisible(false);
    ui->viewPreviousScanWindowFrame->setVisible(false);
    ui->newScanFrame->setVisible(false);
    ui->homeFrame->setVisible(false);
}

void ScannerWindow::on_hepBackButton_clicked()
{
    ui->helpAndAboutusFrame->setVisible(true);
    ui->CaliberationFrame->setVisible(false);
    ui->helpFrame->setVisible(false);
    ui->aboutUsFrame->setVisible(false);
    ui->viewPreviousScanWindowFrame->setVisible(false);
    ui->newScanFrame->setVisible(false);
    ui->homeFrame->setVisible(false);
}

void ScannerWindow::on_nextCalibrationButton_clicked()
{
    ui->helpAndAboutusFrame->setVisible(false);
    ui->CaliberationFrame->setVisible(false);
    ui->helpFrame->setVisible(false);
    ui->aboutUsFrame->setVisible(false);
    ui->viewPreviousScanWindowFrame->setVisible(false);
    ui->newScanFrame->setVisible(true);
    ui->homeFrame->setVisible(false);



    // Setup the clo ud pointer
    cloud.reset (new PointCloudT);
    // The number of points in the cloud
    cloud->points.resize (1000);

    // The default color
    red   = 128;
    green = 128;
    blue  = 128;

    // Fill the cloud with some points
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);

        cloud->points[i].r = red;
        cloud->points[i].g = green;
        cloud->points[i].b = blue;
    }


        // Set up the QVTK window
        viewer2.reset (new pcl::visualization::PCLVisualizer ("viewer2", false));
        ui->qvtkWidgetNewScan->SetRenderWindow (viewer2->getRenderWindow ());
        viewer2->setupInteractor (ui->qvtkWidgetNewScan->GetInteractor (), ui->qvtkWidgetNewScan->GetRenderWindow ());
        viewer2->resetCamera ();

}

void ScannerWindow::on_homeCalibrationButton_clicked()
{
    ui->helpAndAboutusFrame->setVisible(false);
    ui->CaliberationFrame->setVisible(false);
    ui->helpFrame->setVisible(false);
    ui->aboutUsFrame->setVisible(false);
    ui->viewPreviousScanWindowFrame->setVisible(false);
    ui->newScanFrame->setVisible(false);
    ui->homeFrame->setVisible(true);
}

void ScannerWindow::on_newScanFrameHomeButton_clicked()
{
    ui->helpAndAboutusFrame->setVisible(false);
    ui->CaliberationFrame->setVisible(false);
    ui->helpFrame->setVisible(false);
    ui->aboutUsFrame->setVisible(false);
    ui->viewPreviousScanWindowFrame->setVisible(false);
    ui->newScanFrame->setVisible(false);
    ui->homeFrame->setVisible(true);
}


void ScannerWindow::on_helpAndAboutUsHomeButton_clicked()
{
    ui->helpAndAboutusFrame->setVisible(false);
    ui->CaliberationFrame->setVisible(false);
    ui->helpFrame->setVisible(false);
    ui->aboutUsFrame->setVisible(false);
    ui->viewPreviousScanWindowFrame->setVisible(false);
    ui->newScanFrame->setVisible(false);
    ui->homeFrame->setVisible(true);
}

void ScannerWindow::on_filterButton_clicked()
{
    ui->regFilterStatusLabel->setText("Filtering Started....");
    // This part where they call algorithms (for connecting points, tree tracking, etc.) used in later part of the code

    pcl :: PointCloud<pcl :: PointXYZ>:: Ptr cloud (new pcl :: PointCloud<pcl::PointXYZ>) ;
    pcl :: PointCloud<pcl :: PointNormal >:: Ptr cloudwithnormals (new pcl :: PointCloud<pcl :: PointNormal>) ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers (new pcl::PointCloud<pcl::PointXYZ>);

    //Add Morpho.
    pcl :: search :: KdTree<pcl :: PointXYZ>:: Ptr tree ;
    pcl :: search :: KdTree<pcl :: PointNormal>:: Ptr tree2 ;
    std :: cerr << "Uploading" ;
    pcl :: PCLPointCloud2 cloudblob ;

    // Yeman...comment

   // meshname
    stringstream fname;
    fname <<ui->scanNameLineEdit->text().toStdString()<< "3D" << ".ply";
    string filename = fname.str();
    pcl :: io :: loadPLYFile ( filename , cloudblob ) ;
    pcl :: fromPCLPointCloud2 ( cloudblob ,*cloud ) ;
    // cloud = register_out ;
    std :: cerr << "Completed";

    // Removing the background of the object
//    pcl::PassThrough<pcl::PointXYZ> pass;
//    pass.setInputCloud (cloud);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (0.0, 3.0);
//    pass.filter (*cloud_filtered);

    // Remove outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);

     // Mean and STD user
     double meanK = ui->meandoubleSpinBox->value();
     double stdThesh = ui->stddoubleSpinBox->value();
     sor.setMeanK (meanK); // Setting a threshold for number of pcl; less than this amount remove it, more than this amount keep it
     sor.setStddevMulThresh (stdThesh);
     sor.filter (*cloud_outliers);

     pcl::io::savePCDFile("result.pcd", *cloud_outliers);       // End construction Zone

     std :: cerr << "KD Search Tree" ;
     tree.reset (new pcl :: search :: KdTree<pcl :: PointXYZ>( false ) ) ;
     tree->setInputCloud(cloud_outliers);
     std :: cerr << "Completed" ;



    // Estimation
    std :: cerr << "Normal Estimation" ;
    pcl :: NormalEstimation<pcl :: PointXYZ , pcl :: Normal> n ;
    pcl :: PointCloud<pcl :: Normal >:: Ptr normals (new pcl :: PointCloud<pcl ::Normal>() ) ;
    n.setInputCloud(cloud_outliers);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    std :: cerr << "Completed";

    std :: cerr << "Concatenation of Fields" ;
    pcl :: concatenateFields (*cloud_outliers,*normals,*cloudwithnormals);
    std :: cerr << "Completed" ;

    // From Sharon part
    std :: cerr << "Starting Moving Least Square" ;
    //Output has the PointNormal type in order to store the normals calculated by MLS
    pcl :: PointCloud<pcl :: PointNormal >:: Ptr mlspoints (new pcl :: PointCloud<pcl :: PointNormal >() ) ;
    // Init object. Second point type is for the normals
    pcl :: MovingLeastSquares<pcl :: PointXYZ , pcl :: PointNormal> mls ;

    mls.setInputCloud(cloud_outliers);
    mls.setComputeNormals ( true ) ;
    mls.setPolynomialFit ( true ) ;
    mls.setPolynomialOrder(2);
    mls.setSearchMethod ( tree ) ;
    mls.setSearchRadius ( 0.03 ) ; // try with 5cm or 3cm
    mls.setSqrGaussParam(0.0009);
    mls.process(*mlspoints); //reconstruct
    std :: cerr << "Completed" ;

// Mesh starts here
// Test which method is selected
  if (ui->triangradioButton->isChecked() == true)
  {

   boost :: shared_ptr<pcl :: PolygonMesh> triangles (new pcl :: PolygonMesh );
   std :: cerr << "Starting Greedy Projection Triangulation" ;

   pcl :: GreedyProjectionTriangulation<pcl :: PointNormal> gt;

   gt.setInputCloud(mlspoints); // Here either you use with MLS or withnormals
   gt.setSearchMethod ( tree2 ) ;
   gt.setSearchRadius (0.025) ; //R
   gt.setMu (5) ; //mu
   gt.setMaximumNearestNeighbors (50);
   gt.setMaximumSurfaceAngle (M_PI/4);
   gt.setMinimumAngle (M_PI/18);
   gt.setMaximumAngle(2*M_PI/3);
   gt.setNormalConsistency (false);
   gt.reconstruct(*triangles);
   std :: cerr << "Completed" ;

   std :: cerr << "Starting Laplacian Smoothing" ;
   pcl :: PolygonMesh output ;
   pcl :: MeshSmoothingLaplacianVTK vtk ;
   vtk.setInputMesh ( triangles ) ;
   vtk.setNumIter (20000) ;
   vtk.setConvergence ( 0.0002 ) ;
   vtk.setRelaxationFactor ( 0.0002 ) ;
   vtk.setFeatureEdgeSmoothing ( true ) ;
   vtk.setFeatureAngle (M_PI/5) ;
   vtk.setBoundarySmoothing ( true ) ;
   vtk.process ( output ) ;
   std :: cerr << "Completed" << std :: endl ;

   // Save and Display it

   stringstream fname;
   fname <<ui->scanNameLineEdit->text().toStdString()<< "Triangmesh" << ".ply";
   string filename = fname.str();

   pcl::io::savePLYFile(filename ,output);

   viewer2->addPolygonMesh(output, "cloud");
   //viewer2->addPointCloud((grid.cloud), "cloud");
   //viewer2->updateCamera();
   viewer2->resetCamera ();


   } // triangulation ends here

  // Grid Projection Starts here

    else
  {
        std :: cerr << "Applying Grid Projection" ;
        pcl :: PolygonMesh grid;
        pcl :: GridProjection <pcl :: PointNormal> gp;
        gp.setInputCloud (mlspoints);
        gp.setSearchMethod(tree2);
        gp.setResolution( 0.0025 ); //add as parameter
        gp.setPaddingSize (3); // add as parameter
        gp.reconstruct(grid); //
        std :: cerr << "Grid Projection Done";

        // Finish added part from Sharon

        stringstream fname;
        fname <<ui->scanNameLineEdit->text().toStdString()<< "gridmesh" << ".ply";
        string filename = fname.str();

        pcl::io::savePLYFile(filename ,grid);

        // pcl :: PointCloud<pcl :: PointXYZ>:: Ptr grid_out (new pcl :: PointCloud<pcl::PointXYZ>) ;
        //pcl :: PolygonMesh grid_out;
        //  pcl::io::loadPLYFile("Solen3D.ply" ,grid_out);

        viewer2->addPolygonMesh(grid, "cloud");
        //viewer2->addPointCloud((grid.cloud), "cloud");

        //viewer2->updateCamera();
        viewer2->resetCamera ();

     }
   ui->regFilterStatusLabel->setText("Filtering Finshed....");

}

void ScannerWindow::on_recordButton_clicked()
{
    ui->scanStatusLabel->setText("Scanning");
    scanStatrt = true;
}




