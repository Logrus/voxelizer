#ifndef SIMPLECLOUDREADER_H
#define SIMPLECLOUDREADER_H
#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// POD type
struct PointXYZI {
    double x, y, z;
    int i;
};

class SimpleCloudReader
{
public:
    SimpleCloudReader();
    bool accumulatePointsFromTxtPointCloud(const std::string &filename);
    void getDemeanedPointCloud(const boost::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > cloud);
    PointXYZI getShift();

protected:
    std::vector<PointXYZI> cloud_;
    PointXYZI shift;
};

#endif // SIMPLECLOUDREADER_H
