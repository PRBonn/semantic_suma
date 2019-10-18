#ifndef RV_EXCEPTION_H_
#define RV_EXCEPTION_H_

#include <exception>
#include <string>

/** \brief Base class for exceptions
 *  \author behley
 */
namespace rv
{

class Exception: public std::exception
{
  public:
    Exception(const std::string& msg) :
        what_(msg)
    {
    }

    virtual ~Exception() throw ()
    {
    }

    /** \brief Returns associated message.
     *
     * This method is called by the C++ runtime system even if
     * the exception aborts the program.
     */
    virtual const char* what() const throw ()
    {
      return what_.c_str();
    }

  private:
    std::string what_;
};

}

#endif /* SRC_CORE_RV_EXCEPTION_H_ */
