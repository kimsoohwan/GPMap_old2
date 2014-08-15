#ifndef _BAYESIAN_COMMITTEE_MACHINE_SERIALIZABLE_HPP_
#define _BAYESIAN_COMMITTEE_MACHINE_SERIALIZABLE_HPP_

// STL
#include <string>
#include <fstream>

// Boost
#include <boost/serialization/split_member.hpp>

// GPMap
#include "bcm.hpp"
#include "serialization/eigen_serialization.hpp"	// serialize, deserialize

namespace GPMap {

class BCM_Serializable : public BCM
{
public:
	/** @brief Default Constructor */
	BCM_Serializable()
		: m_fDumped(false)
	{}

	/** @brief Get mean and [co]variance */
	inline void get(VectorPtr &pMean, MatrixPtr &pCov)
	{
		// load if necessary
		load();

		// get
		BCM::get(pMean, pCov);
	}

	/** @brief Update the mean and [co]variance */
	void update(const VectorConstPtr &pMean, const MatrixConstPtr &pCov)
	{
		// load if necessary
		load();

		// get
		BCM::update(pMean, pCov);
	}

	/** @brief Check if the data was dumpted into a file */
	inline bool isDumpted() const
	{
		return m_fDumped;
	}

	/** @brief Dump all the data */
	bool dump(const std::string &filename)
	{
		// check initialized
		if(!isInitialized()) return false;

		// save
		if(!GPMap::serialize(*this, filename)) return false;

		// deallocate memories
		m_pSumOfWeightedMeans.reset();
		m_pSumOfInvCovs.reset();

		// remember file name
		m_strDumpFileName = filename;

		// turn the flag on
		m_fDumped = true;

		return true;
	}

	/** @brief Load all the data */
	bool load()
	{
		// flag check
		if(!m_fDumped) return false;

		// load
		if(!GPMap::deserialize(*this, m_strDumpFileName)) return false;

		// clear file name
		m_strDumpFileName.clear();

		// turn the flag off
		m_fDumped = false;

		return true;
	}

	/** @brief Load all the data */
	bool isSameDumpFileName(const std::string &strDumpFileName) const
	{
		return (m_strDumpFileName.compare(strDumpFileName) == 0);
	}

protected:
	/** @brief	Boost Serialization */
	friend class boost::serialization::access;

	/** @brief	Save */
	template<class Archive>
	void save(Archive & ar, const unsigned int version) const
	{
		// initialization flag
		const bool fInitialized = isInitialized();
		ar & fInitialized;

		// data
		if(fInitialized)
		{
			// dimension
			const size_t dim = D();
			ar & dim;

			// independency flag
			const bool fIndependent = isIndependent();
			ar & fIndependent;

			// mean
			ar & (*m_pSumOfWeightedMeans);

			// cov
			ar & (*m_pSumOfInvCovs);
		}
	}

	/** @brief	Load */
	template<class Archive>
	void load(Archive & ar, const unsigned int version)
	{
		// initialization flag
		bool fInitialized;
		ar & fInitialized;

		// data
		if(fInitialized)
		{
			// dimension
			size_t dim;
			ar & dim;

			// independency flag
			bool fIndependent;
			ar & fIndependent;

			// memory allocation
			if(!m_pSumOfWeightedMeans || m_pSumOfWeightedMeans->size() != dim)
			{
				m_pSumOfWeightedMeans.reset(new Vector(dim));
			}
			if(fIndependent)
			{
				if(!m_pSumOfInvCovs || m_pSumOfInvCovs->rows() != dim || m_pSumOfInvCovs->cols() != 1)
					m_pSumOfInvCovs.reset(new Matrix(dim, 1));
			}
			else
			{
				if(!m_pSumOfInvCovs || m_pSumOfInvCovs->rows() != dim || m_pSumOfInvCovs->cols() != dim)
					m_pSumOfInvCovs.reset(new Matrix(dim, dim));
			}

			// mean
			ar & (*m_pSumOfWeightedMeans);

			// cov
			ar & (*m_pSumOfInvCovs);
		}
	}

	BOOST_SERIALIZATION_SPLIT_MEMBER()

protected:
	/** @brief	Flag for serialization */
	bool				m_fDumped;

	/** @brief	Dumped file name */
	std::string		m_strDumpFileName;
};


}

#endif