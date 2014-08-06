#ifndef _BAYESIAN_COMMITTEE_MACHINE_HPP_
#define _BAYESIAN_COMMITTEE_MACHINE_HPP_

namespace GPMap {

/* @brief Bayesian Committee Machine */
template <typename Dim>
class BCM
{
protected:
	typedef Matrix<float, Dim, 1>		VectorT;
	typedef Matrix<float, Dim, Dim>	MatrixT;

public:
	BCM()
	{
		m_sumWeightedMean.setZero();
		m_inverseVariance.setZero();
	}

	// assume cov is invertible
	BCM(const VectorT & mean, const MatrixT &cov)
		: m_sumInvCov(cov.inverse()),
		  m_sumWeightedMean(m_sumInvCov * mean)
	{
	}

protected:
	/* @brief	sum of weighted means with its inverse covariance matrix 
	 * @detail	\f$\mathbf\mu_* = \mathbf\Sigma_*\left(\sum_{k=1}^K \mathbf\Sigma_k^{-1}\mathbf\mu_k\right)\f$
	 */
	Matrix<float, Dim, 1> m_sumWeightedMean;

	/* @brief sum of inverse covariance matrices
	 * @detail	\f$\mathbf\Sigma_* = \left(\sum_{k=1}^K \mathbf\Sigma_k^{-1} - (K-1)\mathbf\Simga_0^{-1}\right)^{-1}\f$
	 */
	Matrix<float, Dim, Dim> m_sumInvCov;
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