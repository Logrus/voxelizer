#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <ctime>

// QT
#include <QDropEvent>
#include <QDebug>
#include <QMimeData>
#include <QDragEnterEvent>
#include <QFileInfo>

// VTK
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkNamedColors.h>
#include <vtkConeSource.h>
#include <vtkCleanPolyData.h>
#include <vtkDecimatePro.h>
#include <vtkTriangleFilter.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

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
    pcl::PCLPointCloud2::Ptr tmp_cloud(new pcl::PCLPointCloud2);

    foreach (const QUrl &url, mimeData->urls()) {
       QString fileName = url.toLocalFile();
       qDebug() << "[MainWindow] Dropped file:" << fileName;
       if(QFileInfo(fileName).completeSuffix()=="txt"){
           scr.accumulatePointsFromTxtPointCloud(fileName.toUtf8().constData());
           qDebug() << "[MainWindow] Accumulating a txt point cloud.";
           qApp->processEvents();
           ui->statusBar->showMessage("Loading " + QFileInfo(fileName).baseName());
       }
       if(QFileInfo(fileName).completeSuffix()=="pcd"){
           if (pcl::io::loadPCDFile(fileName.toUtf8().constData(), *tmp_cloud) == -1) //* load the file
           {
               qWarning() << "Couldn't read file " << fileName;
               return;
           }
           pcl::fromPCLPointCloud2(*tmp_cloud, *cloud_);
       }

       if(QFileInfo(fileName).completeSuffix()=="ply"){
           if (pcl::io::loadPLYFile(fileName.toUtf8().constData(), *tmp_cloud) == -1) //* load the file
           {
               qWarning() << "Couldn't read file " << fileName;
               return;
           }
           pcl::fromPCLPointCloud2(*tmp_cloud, *cloud_);
       }

       if(QFileInfo(fileName).completeSuffix()=="labels"){
           //scr.loadLabels(fileName.toUtf8().constData());
       }
    }
    qDebug() << "[MainWindow] Finished processing files.";
    ui->statusBar->showMessage("Finished processing files.");

    if(!scr.cloud_.empty()){
        scr.getDemeanedPointCloud(cloud_);
        shift_ = scr.getShift();
    }

    updatePointCloudView();
    viewer_->resetCamera ();

}

void MainWindow::updatePointCloudView()
{
    viewer_->removeAllPointClouds();
    //pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler(cloud_,"intensity");
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZI> handler(cloud_);
    viewer_->addPointCloud<pcl::PointXYZI>(cloud_, handler, "MergedCloud");
    ui->widget_main->update ();
    qDebug() << "[MainWindow] Updated point cloud view.";
}

vtkSmartPointer<vtkPolyData>
getCuboid (double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
  vtkSmartPointer < vtkCubeSource > cube = vtkSmartPointer<vtkCubeSource>::New ();
  cube->SetBounds (minX, maxX, minY, maxY, minZ, maxZ);
  return cube->GetOutput ();
}

void MainWindow::on_btn_voxelize_clicked()
{
    viewer_->getRendererCollection()->GetFirstRenderer()->RemoveActor(treeActor);
    viewer_->removeAllShapes();
    viewer_->removeAllPointClouds();
    qDebug() << "[MainWindow] Creating voxel grid.";
    VoxelGrid vg;
    vg.setInputCloud(cloud_);

    vg.setGridResolution(ui->spinBox_xdim->value(),ui->spinBox_ydim->value(),ui->spinBox_zdim->value());
    qDebug() << "[MainWindow] Initializing voxel grid.";
    clock_t start = clock();
    vg.computeOccupancy();
    qDebug() << "[MainWindow] Computed occupancy in " << (clock()-start)/(double)CLOCKS_PER_SEC << " sec.";
    auto res = vg.getXYZResolution();
    auto ls  = vg.getLeafSize();
    vtkSmartPointer < vtkAppendPolyData > treeWireframe = vtkSmartPointer<vtkAppendPolyData>::New ();

    // Go through all occupied voxels
    start = clock();
    while(vg.hasOccupiedVoxels()){
        auto xyz = vg.getNextOccupiedCentroid();
        vtkSmartPointer < vtkCubeSource > cube = vtkSmartPointer<vtkCubeSource>::New ();
        cube->SetBounds (xyz[0]-(ls[0]/2),xyz[0]+(ls[0]/2),xyz[1]-(ls[1]/2),xyz[1]+(ls[1]/2),xyz[2]-(ls[2]/2),xyz[2]+(ls[2]/2));
        cube->Update();
        treeWireframe->AddInputData (cube->GetOutput ());
    }
    qDebug() << "[MainWindow] Went through occupied voxels " << (clock()-start)/(double)CLOCKS_PER_SEC << " sec.";

    start = clock();
    treeWireframe->Update();
    vtkSmartPointer<vtkPolyData> input =
       vtkSmartPointer<vtkPolyData>::New();
     input->ShallowCopy(treeWireframe->GetOutput());
     vtkSmartPointer<vtkTriangleFilter> triangleFilter =
         vtkSmartPointer<vtkTriangleFilter>::New();
     triangleFilter->SetInputData(input);
     triangleFilter->Update();

    vtkSmartPointer < vtkPolyDataMapper > mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
    mapper->SetInputConnection(triangleFilter->GetOutputPort());
    treeActor = vtkSmartPointer<vtkLODActor>::New ();
    treeActor->SetMapper(mapper);
    viewer_->getRendererCollection()->GetFirstRenderer()->AddActor (treeActor);
    qDebug() << "[MainWindow] For triangulation and adding new actor " << (clock()-start)/(double)CLOCKS_PER_SEC << " sec.";

    qDebug() << "[MainWindow] Finished adding voxels. ";
    viewer_->setRepresentationToSurfaceForAllActors();
    ui->widget_main->update ();
}

void MainWindow::on_btn_reset_clicked()
{
    viewer_->getRendererCollection()->GetFirstRenderer()->RemoveActor(treeActor);
    viewer_->removeAllShapes();
    updatePointCloudView();
}

void MainWindow::on_spinBox_xdim_valueChanged(int arg1)
{
    if(ui->checkBox_dynamicUpdate->isChecked()){
        ui->spinBox_ydim->setValue(arg1);
        ui->spinBox_zdim->setValue(arg1);
        on_btn_voxelize_clicked();
    }
}
