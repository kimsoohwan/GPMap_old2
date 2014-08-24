#ifndef _BOOST_TIMER_HELPER_HPP_
#define _BOOST_TIMER_HELPER_HPP_

// STL
#include <iostream>

// Boost
#include <boost/timer/timer.hpp>			// boost::timer::cpu_times, 
													// boost::timer::cpu_timer
#include <boost/chrono/include.hpp>		// boost::chrono::process_user_cpu_clock
													// boost::chrono::process_system_cpu_clock
													// boost::chrono::process_real_cpu_clock
namespace GPMap {

/** @class	CPU_Times
  * @brief	Wrapper for Boost Timer and Chrono
  */
class CPU_Times : protected boost::timer::cpu_times
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
		// boost::timer
		user		= other.user;
		system	= other.system;
		wall		= other.wall;

		return *this;
	}

	/** @brief Operator= */
	inline CPU_Times& operator=(const CPU_Times &other)
	{
		// boost::timer
		user		= other.user;
		system	= other.system;
		wall		= other.wall;

		// boost::chrono
		m_user_sec		= other.m_user_sec;
		m_system_sec	= other.m_system_sec;
		m_real_sec		= other.m_real_sec;

		return *this;
	}

	/** @brief Operator+= */
	inline CPU_Times& operator+=(const CPU_Times &other)
	{
		// boost::timer
		user		+= other.user;
		system	+= other.system;
		wall		+= other.wall;

		// boost::chrono
		m_user_sec		+= other.m_user_sec;
		m_system_sec	+= other.m_system_sec;
		m_real_sec		+= other.m_real_sec;

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
		return out << "[boost::timer]\n"
					  << "User CPU time: "		<< ns2sec(elapsed.user)		<< " seconds\n"
					  << "System CPU time: "	<< ns2sec(elapsed.system)	<< " seconds\n"
					  << "Wall-clock time: "	<< ns2sec(elapsed.wall)		<< " seconds\n"
					  << "Total CPU time (user+system): " << ns2sec(elapsed.user + elapsed.system) << " seconds\n\n"
					  << "[boost::chrono]\n"
					  << "User CPU time: "		<< elapsed.m_user_sec		<< " seconds\n"
					  << "System CPU time: "	<< elapsed.m_system_sec	<< " seconds\n"
					  << "Wall-clock time: "	<< elapsed.m_real_sec		<< " seconds\n"
					  << "Total CPU time (user+system): " << elapsed.m_user_sec + elapsed.m_system_sec << " seconds\n\n";
	}

	void clear()
	{
		// boost::timer
		boost::timer::cpu_times::clear();

		// boost::chrono
		m_user_sec		= boost::chrono::duration<double>::zero();
		m_system_sec	= boost::chrono::duration<double>::zero();
		m_real_sec		= boost::chrono::duration<double>::zero();
	}

protected:
	static inline float ns2sec(const boost::timer::nanosecond_type ns)
	{
		return static_cast<float>(ns)/static_cast<float>(1e9);
	}

	static inline float ns2ms(const boost::timer::nanosecond_type ns)
	{
		return static_cast<float>(ns)/static_cast<float>(1e6);
	}

protected:
	friend class CPU_Timer;
	boost::chrono::duration<double> m_user_sec;		// User CPU time in sec
	boost::chrono::duration<double> m_system_sec;	// System CPU time in sec
	boost::chrono::duration<double> m_real_sec;		// Wall-clock time in sec
};

/** @class	CPU_Timer
  * @brief	Wrapper for Boost Timer and Chrono
  */
class CPU_Timer : public boost::timer::cpu_timer
{
public:
	CPU_Timer()
		: boost::timer::cpu_timer(),													// boost::timer
		m_start_user(boost::chrono::process_user_cpu_clock::now()),			// boost::chrono
		m_start_system(boost::chrono::process_system_cpu_clock::now()),
		m_start_real(boost::chrono::process_real_cpu_clock::now())
	{
	}

	CPU_Times elapsed()
	{
		CPU_Times t_elapsed;

		// boost::timer
		t_elapsed = boost::timer::cpu_timer::elapsed();

		// boost::chrono
		t_elapsed.m_user_sec		= boost::chrono::process_user_cpu_clock::now()		- m_start_user;
		t_elapsed.m_system_sec	= boost::chrono::process_system_cpu_clock::now()	- m_start_system;
		t_elapsed.m_real_sec		= boost::chrono::process_real_cpu_clock::now()		- m_start_real;

		return t_elapsed;
	}

protected:
	boost::chrono::process_user_cpu_clock::time_point		m_start_user;		// User CPU time
	boost::chrono::process_system_cpu_clock::time_point	m_start_system;	// System CPU time
	boost::chrono::process_real_cpu_clock::time_point		m_start_real;		// Wall-clock time
};

}
#endif