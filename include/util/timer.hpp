#ifndef _BOOST_TIMER_HELPER_HPP_
#define _BOOST_TIMER_HELPER_HPP_

// STL
#include <iostream>

// Boost
#include <boost/timer/timer.hpp>

namespace GPMap {

inline float ns2sec(const boost::timer::nanosecond_type ns)
{
	return static_cast<float>(ns)/static_cast<float>(1e9);
}

inline float ns2ms(const boost::timer::nanosecond_type ns)
{
	return static_cast<float>(ns)/static_cast<float>(1e6);
}

/** @class	CPU_Times
  * @brief	Wrapper for boost::timer::cpu_times
  */
class CPU_Times : public boost::timer::cpu_times
{
public:
	/** @brief Constructor */
	CPU_Times()
	{
		clear();
	}

	/** @brief Copy constructor */
	CPU_Times(const boost::timer::cpu_times &other)
		//: user	(other.user),
		//  system	(other.system),
		//  wall	(other.wall)
	{
		(*this) = other;
	}

	/** @brief Copy constructor */
	CPU_Times(const CPU_Times &other)
		//: user	(other.user),
		//  system	(other.system),
		//  wall	(other.wall)
	{
		(*this) = other;
	}

	/** @brief Operator= */
	inline CPU_Times& operator=(const boost::timer::cpu_times &other)
	{
		user		= other.user;
		system	= other.system;
		wall		= other.wall;
		return *this;
	}

	/** @brief Operator= */
	inline CPU_Times& operator=(const CPU_Times &other)
	{
		user		= other.user;
		system	= other.system;
		wall		= other.wall;
		return *this;
	}

	/** @brief Operator+= */
	inline CPU_Times& operator+=(const CPU_Times &other)
	{
		user		+= other.user;
		system	+= other.system;
		wall		+= other.wall;
		return *this;
	}

	/** @brief Operator+ */
	inline CPU_Times operator+(const CPU_Times &other)
	{
		CPU_Times ret(*this);
		ret += other;
		return ret;
	}

	/** @brief Operator<< */
	friend std::ostream& operator<< (std::ostream& out, const CPU_Times &elapsed)
	{
		return out << "User CPU time: "		<< ns2sec(elapsed.user)		<< " seconds\n"
					  << "System CPU time: "	<< ns2sec(elapsed.system)	<< " seconds\n"
					  << "Wall-clock time: "	<< ns2sec(elapsed.wall)		<< " seconds\n"
					  << "Total CPU time (user+system): " << ns2sec(elapsed.user + elapsed.system) << " seconds\n";
	}
};

/** @class	CPU_Timer
  * @brief	Wrapper for Boost Timer
  */
class CPU_Timer : public boost::timer::cpu_timer
{
};

}
#endif