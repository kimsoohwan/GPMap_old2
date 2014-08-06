#ifndef _UTILITY_HPP_
#define _UTILITY_HPP_

// STL
#include <vector>

// PCL
#include <pcl/point_types.h>

namespace GPMap {

/** @brief A vector of pcl::PointXYZ points */
typedef std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > PointXYZVector;

}

#endif