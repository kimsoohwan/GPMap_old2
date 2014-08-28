#ifndef _BAYESIAN_COMMITTEE_MACHINE_HPP_
#define _BAYESIAN_COMMITTEE_MACHINE_HPP_

// STL
#include <string>
#include <fstream>

// GPMap
#include "util/data_types.hpp"	// MatrixPtr, VectorPtr
#include "serialization/eigen_serialization.hpp"	// serialize, deserialize

namespace GPMap {

/** @brief Bayesian Committee Machine */
class BCM
{
public:
	BCM()
		: m_fInitialized(false),
		  m_fSerialized(false)
	{}

	BCM(VectorPtr &pSumOfWeightedMeans, MatrixPtr &pSumOfInvCovs)
		: m_fInitialized(true),
		  m_fSerialized(false),
		  m_pSumOfWeightedMeans(pSumOfWeightedMeans),
		  m_pSumOfInvCovs(pSumOfInvCovs)
	{
	}

	// BCM& operator=(const BCM &rhs)

	bool get(VectorPtr &pMean, MatrixPtr &pCov)
	{
		// load if serialized
		load();

		// check 
		if(!m_fInitialized || !m_pSumOfInvCovs || !m_pSumOfWeightedMeans) return false;

		// constant 
		const bool fVarianceVector(m_pSumOfInvCovs->cols() == 1);

		// memory allocation
		if(!pMean || pMean->size() != m_pSumOfWeightedMeans->size())	
			pMean.reset(new Vector(m_pSumOfWeightedMeans->size()));

		if(!pCov  || pCov->rows()  != m_pSumOfInvCovs->rows()
					 || pCov->cols()  != m_pSumOfInvCovs->cols())			
			pCov.reset(new Matrix(m_pSumOfInvCovs->rows(), m_pSumOfInvCovs->cols()));

		// variance vector
		if(fVarianceVector)
		{
			// Sigma
			pCov->noalias() = m_pSumOfInvCovs->cwiseInverse();

			// mean
			pMean->noalias() = pCov->cwiseProduct(*m_pSumOfWeightedMeans);
		}

		// covariance matrix
		else
		{
			// check square matrix
			assert(m_pSumOfInvCovs->rows() == m_pSumOfInvCovs->cols());

			// cholesky factor of the covariance matrix
			CholeskyFactor L(*m_pSumOfInvCovs);
			if(L.info() != Eigen::/*ComputationInfo::*/Success)
			{
				GP::Exception e;
				switch(L.info())
				{
					case Eigen::/*ComputationInfo::*/NumericalIssue :
					{
						e = "NumericalIssue";
						break;
					}
					case Eigen::/*ComputationInfo::*/NoConvergence :
					{
						e = "NoConvergence";
						break;
					}
#if EIGEN_VERSION_AT_LEAST(3,2,0)
					case Eigen::/*ComputationInfo::*/InvalidInput :
					{
						e = "InvalidInput";
						break;
					}
#endif
				}
				throw e;
			}

			// Sigma
#if EIGEN_VERSION_AT_LEAST(3,2,0)
			pCov->noalias() = L.solve(Matrix::Identity(m_pSumOfInvCovs->rows(), m_pSumOfInvCovs->cols()));	// (LL')*inv(Cov) = I
#else
			(*pCov) = L.solve(Matrix::Identity(m_pSumOfInvCovs->rows(), m_pSumOfInvCovs->cols()));				// (LL')*inv(Cov) = I
#endif

			// mean
#if EIGEN_VERSION_AT_LEAST(3,2,0)
			pMean->noalias() = L.solve(*m_pSumOfWeightedMeans);	// (LL')*x = mean
#else
			(*pMean) = L.solve(*m_pSumOfWeightedMeans);				// (LL')*x = mean
#endif
		}

		return true;
	}

	bool update(const VectorPtr &pMean, const MatrixPtr &pCov)
	{
		// check
		if(!pMean || pMean->size() <= 0 || 
			!pCov  || pCov->rows() <= 0  || pCov->cols() <=0) return false;

		// load if serialized
		load();

		// constant 
		const bool fVarianceVector(pCov->cols() == 1);

		// memory allocation
		MatrixPtr pInvCov(new Matrix(pCov->rows(), pCov->cols()));	// inverted cov
		VectorPtr pWeightedMean(new Vector(pMean->size()));			// weighted mean

		// variance vector
		if(fVarianceVector)
		{
			// inv(Sigma)
			pInvCov->noalias() = pCov->cwiseInverse();

			// inv(Sigma)*mean
			pWeightedMean->noalias() = pInvCov->cwiseProduct(*pMean);
		}

		// covariance matrix
		else
		{
			// check square matrix
			assert(pCov->rows() == pCov->cols());

			// cholesky factor of the covariance matrix
			CholeskyFactor L(*pCov);
			if(L.info() != Eigen::/*ComputationInfo::*/Success)
			{
				GP::Exception e;
				switch(L.info())
				{
					case Eigen::/*ComputationInfo::*/NumericalIssue :
					{
						e = "NumericalIssue";
						break;
					}
					case Eigen::/*ComputationInfo::*/NoConvergence :
					{
						e = "NoConvergence";
						break;
					}
#if EIGEN_VERSION_AT_LEAST(3,2,0)
					case Eigen::/*ComputationInfo::*/InvalidInput :
					{
						e = "InvalidInput";
						break;
					}
#endif
				}
				throw e;
			}

			// inv(Sigma)
#if EIGEN_VERSION_AT_LEAST(3,2,0)
			pInvCov->noalias() = L.solve(Matrix::Identity(pCov->rows(), pCov->rows()));	// (LL')*inv(Cov) = I
#else
			(*pInvCov) = L.solve(Matrix::Identity(pCov->rows(), pCov->rows()));				// (LL')*inv(Cov) = I
#endif

			// inv(Sigma)*mean
#if EIGEN_VERSION_AT_LEAST(3,2,0)
			pWeightedMean->noalias() = L.solve(*pMean);	// (LL')x = b
#else
			(*pWeightedMean) = L.solve(*pMean);				// (LL')x = b
#endif
		}

		// set
		if(m_fInitialized)
		{
			// sum of inverted covariance matrices or variance vectors
			(*m_pSumOfInvCovs) += (*pInvCov);

			// sum of weighted means
			(*m_pSumOfWeightedMeans) += (*pWeightedMean);
		}
		else
		{
				m_pSumOfInvCovs			= pInvCov;
				m_pSumOfWeightedMeans	= pWeightedMean;
				m_fInitialized				= true;
		}

		return true;
	}

	VectorConstPtr getSumOfWeightedMeans() const
	{
		return m_pSumOfWeightedMeans;
	}

	MatrixConstPtr getSumOfInvCovs() const
	{
		return m_pSumOfInvCovs;
	}

protected:
	/** @brief	Flag for initialization */
	bool				m_fInitialized;

	/** @brief	Flag for serialization */
	bool				m_fSerialized;

	/** @brief	Serialization file name */
	std::string		m_strFileName;

	/** @brief	sum of weighted means with its inverse covariance matrix 
	  * @detail	\f$\mathbf\mu_* = \mathbf\Sigma_*\left(\sum_{k=1}^K \mathbf\Sigma_k^{-1}\mathbf\mu_k\right)\f$
	  */
	VectorPtr m_pSumOfWeightedMeans;

	/** @brief sum of inverse covariance matrices
	  * @detail	\f$\mathbf\Sigma_* = \left(\sum_{k=1}^K \mathbf\Sigma_k^{-1} - (K-1)\mathbf\Simga_0^{-1}\right)^{-1}\f$
	  */
	MatrixPtr m_pSumOfInvCovs;

	/** @brief	Serialization */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & m_fInitialized;
		if(m_fInitialized)
		{
			ar & (*m_pSumOfWeightedMeans);
			ar & (*m_pSumOfInvCovs);
		}
	}


public:
	inline bool isSerialized() const
	{
		return m_fSerialized;
	}

	bool save(const std::string &filename)
	{
		// check initialized
		if(!m_fInitialized) return false;

		// save
		if(!GPMap::serialize(*this, filename)) return false;

		// deallocate memories
		m_pSumOfWeightedMeans.reset();
		m_pSumOfInvCovs.reset();

		// filename and flag
		m_strFileName = filename;
		m_fSerialized = true;

		return true;
	}

	bool load()
	{
		// flag check
		if(!m_fSerialized) return false;

		// allocate memories
		m_pSumOfWeightedMeans.reset(new Vector());
		m_pSumOfInvCovs.reset(new Matrix());

		// load
		if(!GPMap::deserialize(*this, m_strFileName)) return false;

		// filename and flag
		m_strFileName.clear();
		m_fSerialized = false;

		return true;
	}

	bool save2pcd(const std::string &filename)
	{
		// load if necessary
		load();

		// 
	}
};

}


#endif