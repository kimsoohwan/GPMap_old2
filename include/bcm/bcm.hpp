#ifndef _BAYESIAN_COMMITTEE_MACHINE_HPP_
#define _BAYESIAN_COMMITTEE_MACHINE_HPP_

#include "utildata_types.hpp"

namespace GPMap {

/* @brief Bayesian Committee Machine */
class BCM
{
public:
	bool get(VectorPtr &pMean, MatrixPtr &pCov) const
	{
		// check 
		if(!m_pSumOfInvCovs || !m_pSumOfWeightedMeans) return;

		// constant 
		const bool fVarianceVector(m_pSumOfInvCovs->cols() == 1);

		// memory allocation
		if(!pMean || pMean->size() != m_pSumOfWeightedMeans->size())	pMean.reset(new Vector(m_pSumOfWeightedMeans->size()));
		if(!pCov  || pCov->rows()  != m_pSumOfInvCovs->rows() || 
						 pCov->cols()  != m_pSumOfInvCovs->cols())			pCov.reset(new Matrix(m_pSumOfInvCovs->rows(), m_pSumOfInvCovs->cols()));
		
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
			if(pL->info() != Eigen::ComputationInfo::Success)
			{
				GP::Exception e;
				switch(L.info())
				{
					case Eigen::ComputationInfo::NumericalIssue :
					{
						e = "NumericalIssue";
						break;
					}
					case Eigen::ComputationInfo::NoConvergence :
					{
						e = "NoConvergence";
						break;
					}
					case Eigen::ComputationInfo::InvalidInput :
					{
						e = "InvalidInput";
						break;
					}
				}
				throw e;
			}

			// Sigma
			pCov->noalias() = L.solve(Matrix::Identity(m_pSumOfInvCovs->rows(), m_pSumOfInvCovs->rows()));

			// mean
			pMean->noalias() = L.solve(*m_pSumOfWeightedMeans);
			//pMean->noalias() = L.matrixL().solve(*m_pSumOfWeightedMeans);
		}

		return true;
	}

	void update(const VectorPtr &pMean, const MatrixPtr &pCov)
	{
		// constant 
		const bool fVarianceVector(pCov->cols() == 1);

		// memory allocation
		MatrixPtr pInvCov(new Matrix(pCov->rows(), pCov->cols()));	// inverted cov
		VectorPtr pWeightedMean(new VectorPtr(pMean->size()));		// weighted mean

		// variance vector
		if(fVarianceVector)
		{
			// inv(Sigma)
			pInvCov->noalias() = pCov->cwiseInverse();

			// inv(Sigma)*mean
			pWeightedMean->noalias() = pCov->cwiseProduct(*pMean);
		}

		// covariance matrix
		else
		{
			// check square matrix
			assert(pCov->rows() == pCov->cols());

			// cholesky factor of the covariance matrix
			CholeskyFactor L(*pCov);
			if(pL->info() != Eigen::ComputationInfo::Success)
			{
				GP::Exception e;
				switch(L.info())
				{
					case Eigen::ComputationInfo::NumericalIssue :
					{
						e = "NumericalIssue";
						break;
					}
					case Eigen::ComputationInfo::NoConvergence :
					{
						e = "NoConvergence";
						break;
					}
					case Eigen::ComputationInfo::InvalidInput :
					{
						e = "InvalidInput";
						break;
					}
				}
				throw e;
			}

			// inv(Sigma)
			pInvCov->noalias() = L.solve(Matrix::Identity(pCov->rows(), pCov->rows()));

			// inv(Sigma)*mean
			pWeightedMean->noalias() = L.solve(*pMean);
			//pWeightedMean->noalias() = L.matrixL().solve(*pMean);
		}

		// sum of inverted covariance matrices or variance vectors
		if(m_pSumOfInvCovs) (*m_pSumOfInvCovs) += (*pInvCov);
		else					   m_pSumOfInvCovs = invCov;
		
		// sum of weighted means
		if(m_pSumOfWeightedMeans)	(*m_pSumOfWeightedMeans) += (*pWeightedMean);
		else								m_pSumOfWeightedMeans = pWeightedMean;
	}

protected:
	/* @brief	sum of weighted means with its inverse covariance matrix 
	 * @detail	\f$\mathbf\mu_* = \mathbf\Sigma_*\left(\sum_{k=1}^K \mathbf\Sigma_k^{-1}\mathbf\mu_k\right)\f$
	 */
	VectorPtr m_pSumOfWeightedMeans;

	/* @brief sum of inverse covariance matrices
	 * @detail	\f$\mathbf\Sigma_* = \left(\sum_{k=1}^K \mathbf\Sigma_k^{-1} - (K-1)\mathbf\Simga_0^{-1}\right)^{-1}\f$
	 */
	MatrixPtr m_pSumOfInvCovs;
}

template <>
class GaussianDistribution<1>
{
public:
	GaussianDistribution()
		: m_mean((Scalar) 0.f), 
			m_inverseVariance((Scalar) 0.f)
	{
	}

	GaussianDistribution(const Scalar mean, const Scalar variance)
		: m_mean(mean), 
			m_inverseVariance(((Scalar) 1.f) / variance)
	{
	}

	GaussianDistribution& operator=(const GaussianDistribution &rhs)
	{
		// Only do assignment if RHS is a different object from this.
		if (this != &rhs)
		{
			m_mean						= rhs.m_mean;
			m_inverseVariance	= rhs.m_inverseVariance;
		}
		return *this;
	}

	// merge
	GaussianDistribution& operator+=(const GaussianDistribution &rhs)
	{
		m_mean						= m_inverseVariance * m_mean + rhs.m_inverseVariance * rhs.m_mean;
		m_inverseVariance			+= rhs.m_inverseVariance;
		m_mean						/= m_inverseVariance;
		return *this;
	}

	/** \brief Return mean.
	*  \return mean
	* */
	Scalar
	getMean() const
	{
		return m_mean;
	}

	/** \brief Return variance.
	*  \return variance
	* */
	Scalar
	getVariance() const
	{
		return ((Scalar) 1.f) / m_inverseVariance;
	}

	/** \brief Return inverse variance.
	*  \return inverse variance
	* */
	Scalar
	getInverseVariance() const
	{
		return m_inverseVariance;
	}

	/** \brief reset */
	void
	reset ()
	{
		m_mean					= (Scalar) 0.f;
		m_inverseVariance		= (Scalar) 0.f;
	}

protected:
	Scalar			m_mean;
	Scalar			m_inverseVariance;
};

#endif