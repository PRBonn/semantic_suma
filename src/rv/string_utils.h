/**
 * \headerfile misc.h
 *
 * \brief some utility functions for std::strings.
 *
 * \author behley
 */

#ifndef MISC_H_
#define MISC_H_

#include <stdint.h>
#include <vector>
#include <string>
#include <map>
#include <list>
#include <sstream>

namespace rv
{

/** \brief remove whitespaces at the beginning and end of a string. **/
std::string trim(const std::string& str, const std::string& whitespaces =
    " \0\t\n\r\x0B");

/** \brief splits a string in tokens, which are seperated by delim. **/
std::vector<std::string> split(const std::string& line,
    const std::string& delim = " ", bool skipEmpty = false);

/** \brief generate a string from a map.
 *  \return stringified version of a map. **/
template<class K, class V>
std::string stringify(const std::map<K, V>& map)
{
  std::stringstream str;

  typename std::map<K, V>::const_iterator it;
  str << "{";
  for (it = map.begin(); it != map.end(); ++it)
    str << (it == map.begin() ? "" : ", ") << it->first << ": " << it->second;
  str << "}";

  return str.str();
}

/** \brief generate a string from a vector
 *  \return stringified version of a vector. **/
template<class T>
std::string stringify(const std::vector<T>& vec)
{
  std::stringstream str;

  str << "[";
  for (uint32_t i = 0; i < vec.size(); ++i)
    str << (i == 0 ? "" : ", ") << vec[i];
  str << "]";

  return str.str();
}

/** \brief generate a string from a list
 *  \return stringified version of a list. **/
template<class T>
std::string stringify(const std::list<T>& list)
{
  std::stringstream str;
  typename std::list<T>::const_iterator it;
  str << "[";
  for (it = list.begin(); it != list.end(); ++it)
    str << (it == list.begin() ? "" : ", ") << *it;
  str << "]";

  return str.str();
}

/** \brief generate a string from a value using stringstream
 *
 *  \param value stringifiable value (e.g., int, double, string,)
 *  \return stringified version of the value
 */
template<class T>
std::string stringify(const T& value)
{
  std::stringstream str;
  str << value;
  return str.str();
}

}

/** some useful concatenation operators for std::strings. **/
//std::string operator+(const std::string& string, const uint32_t& other);
//std::string operator+(const std::string& string, const int32_t& other);
//std::string operator+(const std::string& string, const float& other);
//std::string operator+(const std::string& string, const double& other);

#endif /* MISC_H_ */
