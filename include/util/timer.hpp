#ifndef _BOOST_TIMER_HELPER_HPP_
#define _BOOST_TIMER_HELPER_HPP_

// STL
#include <iostream>

// Boost
#include <boost/timer/timer.hpp>

namespace GPMap {

///** @class	Timer
//  * @brief	Wrapper for Boost Timer
//  */
//class Timer : protected boost::timer::cpu_times
//{
//public:
//	Timer()
//	{
//		clear();
//	}
//};

inline boost::timer::cpu_times& operator+=(boost::timer::cpu_times &lhs, 
														 const boost::timer::cpu_times &rhs)
{
	lhs.user		+= rhs.user;		// User CPU time		in nano seconds
	lhs.system	+= rhs.system;		// System CPU time	in nano seconds
	lhs.wall		+= rhs.wall;		// Wall-clock time	in nano seconds
	return lhs;
}

inline boost::timer::cpu_times operator+(const boost::timer::cpu_times &lhs, 
													  const boost::timer::cpu_times &rhs)
{
	boost::timer::cpu_times ret;
	ret = lhs;
	ret += rhs;
	return ret;
}

inline float ns2sec(const boost::timer::nanosecond_type ns)
{
	return static_cast<float>(ns)/static_cast<float>(1e9);
}

inline float ns2ms(const boost::timer::nanosecond_type ns)
{
	return static_cast<float>(ns)/static_cast<float>(1e6);
}

std::ostream& operator<< (std::ostream& out, const boost::timer::cpu_times &elapsed)
{
   return out << "User CPU time: "		<< ns2sec(elapsed.user)		<< " seconds\n"
				  << "System CPU time: "	<< ns2sec(elapsed.system)	<< " seconds\n"
				  << "Wall-clock time: "	<< ns2sec(elapsed.wall)		<< " seconds\n"
				  << "Total CPU time (user+system): " << ns2sec(elapsed.user + elapsed.system) << " seconds\n";
}

}
#endif