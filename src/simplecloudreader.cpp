#include "simplecloudreader.h"
#include <fstream>
#include <limits>
//#include <fcntl.h>    /* For O_RDWR */
//#include <unistd.h>   /* For open(), creat() */

SimpleCloudReader::SimpleCloudReader()
{

}

bool SimpleCloudReader::accumulatePointsFromTxtPointCloud(const std::string &filename)
{
    // Open the file
    std::ifstream file;
    file.open(filename.c_str());
    if(!file.is_open()){
      std::cerr << "[SimpleCloudReader] Could not open the file " << filename.c_str() << std::endl;
      return false;
    }

    // Skip first line
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // Read points from file as doubles
    double x,y,z,i;
    while(file >> x >> y >> z >> i){
      PointXYZI p;
      p.x = x;
      p.y = y;
      p.z = z;
      p.i = i;
      cloud_.push_back(p);
      file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    file.close();

    return true;
}

PointXYZI SimpleCloudReader::getShift()
{
    return shift;
}

void SimpleCloudReader::getDemeanedPointCloud(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > cloud){

    // Find a mean value along every dimension
    double x = std::accumulate(cloud_.begin(), cloud_.end(), 0.0, [](const double acc, PointXYZI p){ return acc + p.x; }) / (double) cloud_.size();
    double y = std::accumulate(cloud_.begin(), cloud_.end(), 0.0, [](const double acc, PointXYZI p){ return acc + p.y; }) / (double) cloud_.size();
    double z = std::accumulate(cloud_.begin(), cloud_.end(), 0.0, [](const double acc, PointXYZI p){ return acc + p.z; }) / (double) cloud_.size();

    // Make sure the coud is empty
    cloud->clear();

    // Subtract mean values from every point
    // and save it to the point cloud
    for (int i=0; i<cloud_.size(); i++){
      pcl::PointXYZI pt;

      // All of the coordinates
      pt.x = static_cast<float>(cloud_[i].x - x);
      pt.y = static_cast<float>(cloud_[i].y - y);
      pt.z = static_cast<float>(cloud_[i].z - z);
      pt.intensity = cloud_[i].i;

      cloud->points.push_back(pt);
    }

    // Save shift
    shift.x = x;
    shift.y = y;
    shift.z = z;
    shift.i = 0;
  }
