#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

// PCL
#include <pcl/visualization/pcl_visualizer.h>

// Internal
#include "simplecloudreader.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dropEvent(QDropEvent *event) override;

private:
    Ui::MainWindow *ui;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > cloud_;
    PointXYZI shift_;

    void updatePointCloudView();
};

#endif // MAINWINDOW_H
