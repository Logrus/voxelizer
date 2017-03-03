#include "mainwindow.h"
#include "ui_mainwindow.h"

// QT
#include <QDropEvent>
#include <QDebug>
#include <QMimeData>
#include <QDragEnterEvent>
#include <QFileInfo>

// VTK
#include <vtkRenderWindow.h>

// PCL
#include <pcl/filters/voxel_grid_occlusion_estimation.h>

// Project
#include "voxelgrid.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    voxel_id(0)
{
    ui->setupUi(this);

    // Set up the QVTK window for Point Cloud
    viewer_.reset (new pcl::visualization::PCLVisualizer ("Voxelizer", false));
    viewer_->setBackgroundColor (0.2, 0.2, 0.2);
    ui->widget_main->SetRenderWindow (viewer_->getRenderWindow ());
    viewer_->setupInteractor (ui->widget_main->GetInteractor (), ui->widget_main->GetRenderWindow ());
    viewer_->setShowFPS(false);
    ui->widget_main->update ();

    // Initialize PointCloud pointer
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    // Set accept drops
    setAcceptDrops(true);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasUrls()) {
        event->acceptProposedAction();
    }
}

void MainWindow::dropEvent(QDropEvent *event)
{
    const QMimeData *mimeData = event->mimeData();

    SimpleCloudReader scr;

    foreach (const QUrl &url, mimeData->urls()) {
       QString fileName = url.toLocalFile();
       qDebug() << "[MainWindow] Dropped file:" << fileName;
       if(QFileInfo(fileName).completeSuffix()=="txt"){
           scr.accumulatePointsFromTxtPointCloud(fileName.toUtf8().constData());
           qDebug() << "[MainWindow] Accumulating a txt point cloud.";
           qApp->processEvents();
           ui->statusBar->showMessage("Loading " + QFileInfo(fileName).baseName());
       }
    }
    qDebug() << "[MainWindow] Finished processing files.";
    ui->statusBar->showMessage("Finished processing files.");
    scr.getDemeanedPointCloud(cloud_);
    shift_ = scr.getShift();


    updatePointCloudView();
    viewer_->resetCamera ();

}

void MainWindow::updatePointCloudView()
{
    viewer_->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler(cloud_,"intensity");
    viewer_->addPointCloud<pcl::PointXYZI>(cloud_, handler, "MergedCloud");
    //viewer_->resetCamera ();
    ui->widget_main->update ();
    qDebug() << "[MainWindow] Updated point cloud view.";
}

void MainWindow::on_btn_voxelize_clicked()
{
    viewer_->removeAllShapes();
    viewer_->removeAllPointClouds();
    qDebug() << "[MainWindow] Creating voxel grid.";
    VoxelGrid vg;
    vg.setInputCloud(cloud_);

    vg.setGridResolution(ui->spinBox_xdim->value(),ui->spinBox_ydim->value(),ui->spinBox_zdim->value());
    qDebug() << "[MainWindow] Initializing voxel grid.";
    vg.computeOccupancy();
    auto res = vg.getXYZResolution();
    auto ls  = vg.getLeafSize();

    for (int z = 0; z < res[2]; ++z){
        for (int y = 0; y < res[1]; ++y){
            for (int x = 0; x < res[0]; ++x){
                //qApp->processEvents();
                bool isoccupied;
                auto xyz = vg.getCentroid(isoccupied,x,y,z);
                if(isoccupied){
                    viewer_->addCube(xyz[0]-(ls[0]/2),xyz[0]+(ls[0]/2),xyz[1]-(ls[1]/2),xyz[1]+(ls[1]/2),xyz[2]-(ls[2]/2),xyz[2]+(ls[2]/2),1,1,1,"vox" + std::to_string(voxel_id));
                    voxel_id++;
                }
            }
        }
    }
    qDebug() << "[MainWindow] Finished adding voxels. ";
    //viewer_->setRepresentationToSurfaceForAllActors();
    ui->widget_main->update ();
}

void MainWindow::on_btn_reset_clicked()
{
    viewer_->removeAllShapes();
    updatePointCloudView();
}

void MainWindow::on_spinBox_xdim_valueChanged(int arg1)
{
    ui->spinBox_ydim->setValue(arg1);
    ui->spinBox_zdim->setValue(arg1);
}
