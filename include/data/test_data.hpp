#ifndef _GPMAP_TEST_DATA_HPP_
#define _GPMAP_TEST_DATA_HPP_

// OpenGP
#include <GP.h>

namespace GPMap {

TYPE_DEFINE_MATRIX(float);

void meshGrid(const Eigen::Vector3f		&min,
				  const unsigned int			nGrids,
				  const float					gridSize,
				  MatrixPtr						&pXs)
{
	assert(nGrids > 0);

	// matrix size
	if(pXs && pXs->rows() != nGrids*nGrids*nGrids && pXs->cols() != 3)
		pXs.reset(new Matrix(nGrids*nGrids*nGrids, 3));

	// generate mesh grid in the order of x, y, z
	int row = 0;
	float x = min.x() + gridSize * static_cast<float>(0.5f);
	for(size_t ix = 0; ix < nGrids; ix++, x += gridSize)
	{
		float y = min.y() + gridSize * static_cast<float>(0.5f);
		for(size_t iy = 0; iy < nGrids; iy++, y += gridSize)
		{
			float z = min.z() + gridSize * static_cast<float>(0.5f);
			for(size_t iz = 0; iz < nGrids; iz++, z += gridSize)
			{
				(*pXs)(row, 0) = x;
				(*pXs)(row, 1) = y;
				(*pXs)(row, 2) = z;
				row++;
			}
		}
	}

	return;
}

}
#endif