#include "voxelgrid.h"

VoxelGrid::VoxelGrid()
{

}

void VoxelGrid::setInputCloud(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > in_cloud)
{
    cloud_ = in_cloud;
    computeBoundingBoxFromPointCloud();
}

void VoxelGrid::setLeafSize(float xsize, float ysize, float zsize)
{
    x_leafsize_ = xsize;
    y_leafsize_ = ysize;
    z_leafsize_ = zsize;
    x_res_ = std::floor(static_cast<float>(x_max_-x_min_)/xsize);
    y_res_ = std::floor(static_cast<float>(y_max_-y_min_)/ysize);
    z_res_ = std::floor(static_cast<float>(z_max_-z_min_)/zsize);
    setGridResolution(x_res_, y_res_, z_res_);
}

void VoxelGrid::setGridResolution(int xres, int yres, int zres)
{
    x_res_ = xres;
    y_res_ = yres;
    z_res_ = zres;
    x_leafsize_ = static_cast<float>(x_max_-x_min_)/x_res_;
    y_leafsize_ = static_cast<float>(y_max_-y_min_)/y_res_;
    z_leafsize_ = static_cast<float>(z_max_-z_min_)/z_res_;
    std::cout << "[VoxelGrid] x_res_ " << x_res_ << " y_res_ " << y_res_ << " z_res_ " << z_res_ << std::endl;
    std::cout << "[VoxelGrid] x_leafsize_ " << x_leafsize_ << " y_leafsize_ " << y_leafsize_ << " z_leafsize_ " << z_leafsize_ << std::endl;
    //occupancy_.resize(x_res_*y_res_*z_res_, false);
}

void VoxelGrid::computeOccupancy()
{
    // FIXME: omp can write to the same memory, but that shouldn't be a huge problem, thougth it's not nice
//    omp_lock_t writelock;
//    omp_init_lock(&writelock);
//#pragma omp parallel for
    for(int i=0; i<cloud_->points.size(); ++i){
        pcl::PointXYZI pt = cloud_->points[i];
        //std::cout << "[VoxelGrid] pt " << pt << std::endl;
        int xcoord = static_cast<int>((pt.x - x_min_)/x_leafsize_);
        int ycoord = static_cast<int>((pt.y - y_min_)/y_leafsize_);
        int zcoord = static_cast<int>((pt.z - z_min_)/z_leafsize_);
        //std::cout << "[VoxelGrid] xcoord " << xcoord << " ycoord " << ycoord << " zcoord " << zcoord << std::endl;
        //omp_set_lock(&writelock);
        occupancy_[idx3(xcoord, ycoord, zcoord)] = true;
        //omp_unset_lock(&writelock);
    }
    //omp_destroy_lock(&writelock);
    it_ = occupancy_.begin();
}

void VoxelGrid::resetIterator()
{
    it_ = occupancy_.begin();
}

bool VoxelGrid::hasOccupiedVoxels()
{
    return (it_ != occupancy_.end());
}

Eigen::Vector3f VoxelGrid::getNextOccupiedCentroid()
{
    update3dIdx();
    Eigen::Vector3f voxel_center;
    voxel_center[0] = x_min_ + currx_ * x_leafsize_;
    voxel_center[1] = y_min_ + curry_ * y_leafsize_;
    voxel_center[2] = z_min_ + currz_ * z_leafsize_;
    ++it_;
    return voxel_center;
}

void VoxelGrid::setBoundingBox(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
{
    x_min_ = x_min;
    y_min_ = y_min;
    z_min_ = z_min;
    x_max_ = x_max;
    y_max_ = y_max;
    z_max_ = z_max;
}

void VoxelGrid::computeBoundingBoxFromPointCloud()
{
    Eigen::Vector4f min_p, max_p;
    pcl::getMinMax3D (*cloud_, min_p, max_p);
    x_min_ = min_p[0];
    y_min_ = min_p[1];
    z_min_ = min_p[2];
    x_max_ = max_p[0];
    y_max_ = max_p[1];
    z_max_ = max_p[2];
    std::cout << "[VoxelGrid] xmin " << x_min_ << " ymin " << y_min_ << " zmin " << z_min_ << std::endl;
    std::cout << "[VoxelGrid] xmax " << x_max_ << " ymax " << y_max_ << " zmax " << z_max_ << std::endl;
}

Eigen::Vector3f VoxelGrid::getCentroid(bool &is_occupied, int x, int y, int z)
{
    is_occupied = occupancy_[idx3(x,y,z)];
    Eigen::Vector3f voxel_center;
    voxel_center[0] = x_min_ + x * x_leafsize_;
    voxel_center[1] = y_min_ + y * y_leafsize_;
    voxel_center[2] = z_min_ + z * z_leafsize_;
    return voxel_center;
}

Eigen::Vector3i VoxelGrid::getXYZResolution()
{
    Eigen::Vector3i xyzres;
    xyzres[0] = x_res_;
    xyzres[1] = y_res_;
    xyzres[2] = z_res_;
    return xyzres;
}

Eigen::Vector3f VoxelGrid::getLeafSize()
{
    Eigen::Vector3f leaf_size;
    leaf_size[0] = x_leafsize_;
    leaf_size[1] = y_leafsize_;
    leaf_size[2] = z_leafsize_;
    return leaf_size;
}

int VoxelGrid::idx3(int x, int y, int z)
{
    return x+(y+z*y_res_)*x_res_;
}

void VoxelGrid::update3dIdx()
{
    int i = it_->first;
    currx_ = i % z_res_;
    curry_ = (i / z_res_) % y_res_;
    currz_ = i / (y_res_ * z_res_);
}

