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


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Set up the QVTK window for Point Cloud
    viewer_.reset (new pcl::visualization::PCLVisualizer ("Voxelizer", false));
    viewer_->setBackgroundColor (0.4, 0.4, 0.4);
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
       qDebug() << "Dropped file:" << fileName;
       if(QFileInfo(fileName).completeSuffix()=="txt"){
           scr.accumulatePointsFromTxtPointCloud(fileName.toUtf8().constData());
           qDebug() << "Accumulating a point cloud.";
           qApp->processEvents();
           ui->statusBar->showMessage("Loading " + QFileInfo(fileName).baseName());
       }
    }
    ui->statusBar->showMessage("Finished processing files.");
    scr.getDemeanedPointCloud(cloud_);
    shift_ = scr.getShift();

    updatePointCloudView();

}

void MainWindow::updatePointCloudView()
{
    viewer_->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler(cloud_,"intensity");
    viewer_->addPointCloud<pcl::PointXYZI>(cloud_, handler, "MergedCloud");
    viewer_->resetCamera ();
    ui->widget_main->update ();
}
