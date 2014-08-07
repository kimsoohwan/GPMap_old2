#ifndef _GPMAP_UTILITY_HPP_
#define _GPMAP_UTILITY_HPP_

// STL
#include <algorithm>		// min, max

// PCL
#include <pcl/point_types.h>

namespace GPMap {

template <typename PointT1, typename PointT2>
inline pcl::PointXYZ minPoint(const PointT1 &p1, const PointT2 &p2)
{
	return pcl::PointXYZ(min<float>(p1.x, p2.x), min<float>(p1.y, p2.y), min<float>(p1.z, p2.z));
}

template <typename PointT1, typename PointT2>
inline pcl::PointXYZ maxPoint(const PointT1 &p1, const PointT2 &p2)
{
	return pcl::PointXYZ(max<float>(p1.x, p2.x), max<float>(p1.y, p2.y), max<float>(p1.z, p2.z));
}

}

#endif