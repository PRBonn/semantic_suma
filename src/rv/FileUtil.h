#ifndef CORE_FILEUTIL_H_
#define CORE_FILEUTIL_H_

#include <string>
#include <vector>
#include <stdint.h>

/** \brief Decorator for boost::path.
 *  
 * 
 *  \author behley
 */

namespace rv
{

class FileUtil
{
  public:

    /** \brief Returns the directory components of a file path. */
    static std::string dirName(const std::string& path);

    /** \brief Strips the directory components from a file path. */
    static std::string baseName(const std::string& path);

    /** \brief Returns the file extension.
     * \param[in] path the file name
     * \param[in] level maximum number of extension levels. For instance, ".tar.gz" has level 2. */
    static std::string extension(const std::string& path, int32_t level = 1);

    /** \brief Removes the file extension.
     * \param[in] path the file name
     * \param[in] level maximum number of extension levels to be stripped. For instance, ".tar.gz" has level 2. */
    static std::string stripExtension(const std::string& path, int32_t level = 1);

    /** \brief Checks if a file exists. */
    static bool exists(const std::string& filename);

    /** \brief Get directory listing. **/
    static std::vector<std::string> getDirectoryListing(const std::string& directory);

  protected:
    FileUtil();
};

} /* namespace rv */
#endif /* CORE_FILEUTIL_H_ */
