#include "rv/FileUtil.h"

#include <boost/filesystem.hpp>
#include <iostream>

namespace rv
{

FileUtil::FileUtil()
{

}

std::string FileUtil::dirName(const std::string& path)
{
  boost::filesystem::path p(path);

  return p.parent_path().string();
}

std::string FileUtil::baseName(const std::string& path)
{
  boost::filesystem::path p(path);

  return p.filename().string();
}

std::string FileUtil::extension(const std::string& path, int32_t level)
{
  std::string filename = boost::filesystem::path(path).filename().string();
  if (filename == "" || filename == "." || filename == "..") return "";

  std::string ext;
  while (level-- > 0)
  {
    std::string::size_type idx = filename.rfind(".");
    if (idx == std::string::npos) break;
    ext.insert(0, filename.substr(idx));
    filename.resize(idx);
  }

  return ext;
}

std::string FileUtil::stripExtension(const std::string& path, int32_t level)
{
  std::string filename = boost::filesystem::path(path).filename().string();
  if (filename == "" || filename == "." || filename == "..") return path;

  std::string::size_type path_index = dirName(path).length() + 1;
  std::string complete_path = path;

  while (level-- > 0)
  {
    std::string::size_type idx = complete_path.rfind(".");
    if (idx == std::string::npos || idx < path_index) break;
    complete_path.resize(idx);
  }

  return complete_path;
}

bool FileUtil::exists(const std::string& filename)
{
  return boost::filesystem::exists(filename);
}

std::vector<std::string> FileUtil::getDirectoryListing(const std::string& directory)
{
  typedef boost::filesystem::directory_iterator diter;

  std::vector<std::string> listing;
  for (diter it = diter(directory); it != diter(); ++it)
    listing.push_back(it->path().string());

  return listing;
}

} /* namespace rv */
