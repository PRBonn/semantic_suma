#include "rv/string_utils.h"

#include <boost/tokenizer.hpp>
#include <stdint.h>
#include <iostream>
#include <sstream>

namespace rv
{

std::string trim(const std::string& str, const std::string& whitespaces )
{
  int32_t beg = 0;
  int32_t end = 0;

  /** find the beginning **/
  for (beg = 0; beg < (int32_t) str.size(); ++beg)
  {
    bool found = false;
    for (uint32_t i = 0; i < whitespaces.size(); ++i)
    {
      if (str[beg] == whitespaces[i])
      {
        found = true;
        break;
      }
    }
    if (!found) break;
  }

  /** find the end **/
  for (end = int32_t(str.size()) - 1; end > beg; --end)
  {
    bool found = false;
    for (uint32_t i = 0; i < whitespaces.size(); ++i)
    {
      if (str[end] == whitespaces[i])
      {
        found = true;
        break;
      }
    }
    if (!found) break;
  }

  return str.substr(beg, end - beg + 1);
}

std::vector<std::string> split(const std::string& line, const std::string& delim, bool skipEmpty)
{
  std::vector<std::string> tokens;

  boost::char_separator<char> sep(delim.c_str(), "", (skipEmpty ? boost::drop_empty_tokens : boost::keep_empty_tokens));
  boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);

  for (auto it = tokenizer.begin(); it != tokenizer.end(); ++it)
    tokens.push_back(*it);

  return tokens;
}

}

//std::string operator+(const std::string& string, const uint32_t& other)
//{
//  std::stringstream stream;
//  stream << string << other;
//  return stream.str();
//}
//
//std::string operator+(const std::string& string, const int32_t& other)
//{
//  std::stringstream stream;
//  stream << string << other;
//  return stream.str();
//}
//
//std::string operator+(const std::string& string, const float& other)
//{
//  std::stringstream stream;
//  stream << string << other;
//  return stream.str();
//}
//
//std::string operator+(const std::string& string, const double& other)
//{
//  std::stringstream stream;
//  stream << string << other;
//  return stream.str();
//}

