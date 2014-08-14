#ifndef _GPMAP_LOG_HPP_
#define _GPMAP_LOG_HPP_

#include <string>
#include <fstream>

namespace GPMap {

class Log
{
public:
	Log(const std::string &strFileName)
	{
		m_ofs.open(strFileName.c_str());
		if(!m_ofs.is_open())
		{
			std::cerr << "Error: Can't open a log file, " << strFileName << std::endl;
		}
	}
	virtual ~Log()
	{
		m_ofs.close();
	}
	
	template <typename T>
	Log& operator<< (const T &obj)
	{
		// print on screen
		std::cout << obj;

		// print in file
		m_ofs << obj;

		return *this;
	}

	// for std::endl which is a function template
	Log& operator<< (std::ostream& (*pfun)(std::ostream&))
	{
		// print on screen
		pfun(std::cout);

		// print in file
		pfun(m_ofs);

	  return *this;
	}

protected:
	std::ofstream m_ofs;
};

}

#endif