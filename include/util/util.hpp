#ifndef _GPMAP_UTILITY_HPP_
#define _GPMAP_UTILITY_HPP_

// STL
#include <algorithm>		// min, max

// PCL
#include <pcl/point_types.h>

namespace GPMap {

/** @brief		Find the minimum of two pcl::PointXYZ */
template <typename PointT1, typename PointT2>
inline pcl::PointXYZ minPointXYZ(const PointT1 &p1, const PointT2 &p2)
{
	return pcl::PointXYZ(min<float>(p1.x, p2.x), min<float>(p1.y, p2.y), min<float>(p1.z, p2.z));
}

/** @brief		Find the maximum of two pcl::PointXYZ */
template <typename PointT1, typename PointT2>
inline pcl::PointXYZ maxPointXYZ(const PointT1 &p1, const PointT2 &p2)
{
	return pcl::PointXYZ(max<float>(p1.x, p2.x), max<float>(p1.y, p2.y), max<float>(p1.z, p2.z));
}

/** @brief		Fisher–Yates shuffle (select random m out of n) */
template<class bidiiter>
bidiiter random_unique(bidiiter begin, bidiiter end, size_t num_random)
{
	size_t left = std::distance(begin, end);
	while(num_random--)
	{
		bidiiter r = begin;
		std::advance(r, rand()%left);
		std::swap(*begin, *r);
		++begin;
		--left;
	}
	return begin;
}

}

#endif