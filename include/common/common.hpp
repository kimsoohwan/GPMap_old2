#ifndef _COMMON_HPP_
#define _COMMON_HPP_

// STL
#include <algorithm>		// min, max

// PCL
#include <pcl/point_types.h>		// pcl::PointXYZ, pcl::Normal, pcl::PointNormal
#include <pcl/point_cloud.h>		// pcl::PointCloud
#include <pcl/common/common.h>	// pcl::getMinMax3D

namespace GPMap {

template <typename PointT>
void getMinMax3DFromPointClouds(const std::vector<typename pcl::PointCloud<PointT>::Ptr>		&pPointClouds,
										  pcl::PointXYZ &min_pt, pcl::PointXYZ &max_pt)
{
	PointT min_pt_temp, max_pt_temp;

	// for each point cloud
	for(size_t i = 0; i < pPointClouds.size(); i++)
	{
		// get min max
		pcl::getMinMax3D(*pPointClouds[i], min_pt_temp, max_pt_temp);

		// compare
		if(i == 0)
		{
			min_pt.x = min_pt_temp.x;
			min_pt.y = min_pt_temp.y;
			min_pt.z = min_pt_temp.z;

			max_pt.x = max_pt_temp.x;
			max_pt.y = max_pt_temp.y;
			max_pt.z = max_pt_temp.z;
		}
		else
		{
			min_pt.x = min<float>(min_pt.x, min_pt_temp.x);
			min_pt.y = min<float>(min_pt.y, min_pt_temp.y);
			min_pt.z = min<float>(min_pt.z, min_pt_temp.z);

			max_pt.x = max<float>(max_pt.x, max_pt_temp.x);
			max_pt.y = max<float>(max_pt.y, max_pt_temp.y);
			max_pt.z = max<float>(max_pt.z, max_pt_temp.z);
		}
	}
}


}

#endif