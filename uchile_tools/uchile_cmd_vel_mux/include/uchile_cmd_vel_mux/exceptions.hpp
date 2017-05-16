/**
* @file /cmd_vel_mux/include/uchile_cmd_vel_mux/exceptions.hpp
*
* @brief Exception classes for cmd_vel_mux.
*
* License: BSD
* https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_cmd_vel_mux/LICENSE
**/
#ifndef BENDER_CMD_VEL_EXCEPTIONS_HPP_
#define BENDER_CMD_VEL_EXCEPTIONS_HPP_

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#include <exception>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace uchile_cmd_vel_mux {

/*****************************************************************************
** Exceptions
*****************************************************************************/

class FileNotFoundException: public std::runtime_error {
public:
	FileNotFoundException(const std::string& msg)
		: std::runtime_error(msg) {}
		virtual ~FileNotFoundException() throw() {}
};

class EmptyCfgException: public std::runtime_error {
public:
	EmptyCfgException()
		: std::runtime_error("") {}
		virtual ~EmptyCfgException() throw() {}
};

class YamlException: public std::runtime_error {
public:
	YamlException(const std::string& msg)
		: std::runtime_error(msg) {}
		virtual ~YamlException() throw() {}
};

} // namespace uchile_cmd_vel_mux

#endif /* BENDER_CMD_VEL_EXCEPTIONS_HPP_ */
