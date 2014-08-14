#ifndef _GPMAP_FILE_SYSTEM_HPP_
#define _GPMAP_FILE_SYSTEM_HPP_

// refer to http://www.boost.org/doc/libs/1_31_0/libs/filesystem/doc/index.htm

// STL
#include <string>
#include <vector>

// Boost
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
//#include <boost/regex.hpp>

namespace GPMap {

inline std::string fileExtension(const std::string &str_file_path)
{
	// boost path
	boost::filesystem::path p(str_file_path);

	// extension
	return p.extension().string();
}

inline std::string fileName(const std::string &str_file_path)
{
	// boost path
	boost::filesystem::path p(str_file_path);

	// extension
	return p.filename().string();
}


/** @brief Create a directory */
bool create_directory(const std::string dir_name)
{
	return boost::filesystem::create_directory(dir_name);
}

/** @brief Search a file from a directory and its subdirectories */
bool find_file_in_subdirectories(const std::string		&str_dir_path,		// in this directory,
											const std::string		&str_file_name,	// search for this name,
											std::string				&str_path_found)	// placing path here if found
{
	// current path
	boost::filesystem::path dir_path(str_dir_path);

	// check if there exists the path
	if(!boost::filesystem::exists(dir_path)) return false;

	// for all items in the directory - files and subdirectories
	boost::filesystem::directory_iterator end_iter; // default construction yields past-the-end
	for(boost::filesystem::directory_iterator iter(dir_path); iter != end_iter; ++iter)
	{
		// if it is a sub-directory
		if(boost::filesystem::is_directory(*iter))
		{
			// search the file recursively
			if(find_file_in_subdirectories((*iter).path().string(), str_file_name, str_path_found)) return true;
		}

		// if it is a file
		else if((*iter).path().filename() == str_file_name) // see below
		{
			str_path_found = (*iter).path().string();
			return true;
		}
	}
	
	return false;
}

bool search_files(const std::string				&str_dir_path,			// in this directory,
						const std::string				&str_ext_name,			// search for this extension,
						std::vector<std::string>	&str_file_name_list)	// placing file names here if found
{
	// clear
	str_file_name_list.clear();

	// filter
	//const boost::regex filter("somefiles.*\.txt");

	// current path
	boost::filesystem::path dir_path(str_dir_path);

	// for all items in the directory - files and subdirectories
	boost::filesystem::directory_iterator end_iter; // default construction yields past-the-end
	for(boost::filesystem::directory_iterator iter(dir_path); iter != end_iter; ++iter)
	{
		 // skip if not a file
		 if(!boost::filesystem::is_regular_file(iter->status())) continue;
		 
		 // skip if no match
		 //boost::smatch what;
		 //if(!boost::regex_match((*iter).path().filename(), what, filter)) continue;
		 
		 // if it is a file
		 if((*iter).path().extension().string() == str_ext_name) // see below
		 
		 // file matches, store it
		 str_file_name_list.push_back((*iter).path().filename().string());
	}

	return str_file_name_list.size() > 0;
}

}

#endif