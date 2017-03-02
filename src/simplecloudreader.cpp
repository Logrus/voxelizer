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

//bool SimpleCloudReader::accumulatePointsFromTxtPointCloudRapid(const std::string &filename)
//{
//    static const auto BUFFER_SIZE = 15;//16*1024;
//    int fd = open(filename.c_str(), O_RDONLY);
//    if(fd == -1){
//        std::cerr << "[SimpleCloudReader] Could not open the file " << filename << "." << std::endl;
//    }

//    /* Advise the kernel of our access pattern.  */
//    posix_fadvise(fd, 0, 0, 1);  // FDADVICE_SEQUENTIAL
//    char buf[BUFFER_SIZE + 1];

//    int cached = 0;
//    char cached_line[256];
//    while(size_t bytes_read = read(fd, buf, BUFFER_SIZE))
//    {
//        if(bytes_read == (size_t)-1){
//            std::cerr << "[SimpleCloudReader] Could not read the file " << filename << "." << std::endl;
//        }
//        if (!bytes_read) break;

//        int line_start = 0;
//        for(char *p = buf; (p = (char*) memchr(p, '\n', (buf + bytes_read) - p)); ++p){
//            int lastreadpos = bytes_read-((buf + bytes_read) - p)+1;
//            int haveread = lastreadpos-line_start;
//            memcpy(cached_line+cached, buf+line_start, haveread);
//            line_start += haveread;
//            cached = 0;
//            std::cout << "LINE: " << cached_line << std::endl;
//            double x,y,z,i;
//            std::stringstream ss(cached_line);
//            ss >> x >> y >> z >> i;
//            PointXYZI pt;
//            pt.x = x;
//            pt.y = y;
//            pt.z = z;
//            pt.i = i;
//            cloud_.push_back(pt);
//        }

//        // Cache unfinished line
//        memcpy(cached_line+cached, buf+line_start, bytes_read-line_start);
//        cached += bytes_read-line_start;
////        std::cout << "Cached: " << cached << " last read " << line_start << " bytes to read " << bytes_read-line_start << std::endl;
////        std::cout << "Cached line: " << cached_line << std::endl;
////        std::cin.get();

//    }
//}

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
