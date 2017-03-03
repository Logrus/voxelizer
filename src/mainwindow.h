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

private slots:
    void on_btn_voxelize_clicked();

    void on_btn_reset_clicked();

    void on_spinBox_xdim_valueChanged(int arg1);

private:
    Ui::MainWindow *ui;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > cloud_;
    PointXYZI shift_;

    int voxel_id;

    void updatePointCloudView();
};

#endif // MAINWINDOW_H
