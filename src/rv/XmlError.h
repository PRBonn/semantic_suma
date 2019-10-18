#ifndef CORE_XMLERROR_H_
#define CORE_XMLERROR_H_

#include <stdexcept>

/** \brief Exception thrown in case of an error while parsing XML files.
 *  
 *  \author behley
 */

namespace rv
{

class XmlError: public std::runtime_error
{
  public:
    XmlError(const std::string& reason) :
        std::runtime_error(reason)
    {
    }
};

}
#endif /* SRC_CORE_RV_XMLERROR_H_ */
