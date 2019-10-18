#ifndef CORE_IOERROR_H_
#define CORE_IOERROR_H_

#include <stdexcept>

/** \brief Exception associated with I/O operations
 *
 *  \author behley
 */

namespace rv
{

class IOError: public std::runtime_error
{
  public:
    IOError(const std::string& reason)
        : std::runtime_error(reason)
    {
    }
};

} // namespace rv

#endif /* SRC_CORE_RV_IOERROR_H_ */
