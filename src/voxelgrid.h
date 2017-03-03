#ifndef VOXELGRID_H
#define VOXELGRID_H
#include <vector>
#include <iostream>
#include <math.h>
#include <pcl/point_cloud.h> // pcl::PointCloud
#include <pcl/point_types.h> // pcl::PointXYZI
#include <pcl/common/common.h> // pcl::getMinMax3D

class VoxelGrid
{
public:
    VoxelGrid();

    void setInputCloud(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > in_cloud);
    void setLeafSize(float xsize, float ysize, float zsize);
    void setGridResolution(int xres, int yres, int zres);

    /**
    * Computing occupancy from input point cloud for defined voxel grid
    */
    void computeOccupancy();

    void setBoundingBox(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);
    void computeBoundingBoxFromPointCloud();

    Eigen::Vector3f getCentroid(bool &is_occupied, int x, int y, int z);
    Eigen::Vector3i getXYZResolution();
    Eigen::Vector3f getLeafSize();

protected:
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > cloud_;
    std::vector<bool> occupancy_;
    int x_res_,y_res_,z_res_;
    float x_leafsize_, y_leafsize_, z_leafsize_;
    float x_min_, y_min_, z_min_;
    float x_max_, y_max_, z_max_;
    int idx3(int x, int y, int z);
};

#endif // VOXELGRID_H
