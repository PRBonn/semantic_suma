#ifndef LOG_H_
#define LOG_H_

#include <rv/Exception.h>

#include <map>
#include <vector>
#include <fstream>
#include <rv/Parameter.h>

namespace rv {

/** \brief logfile for storing intermediate results of experiments/evaluations.
 *
 *  The idea is to have an XML representation of some interesting parameters, which
 *  is accessible globally through a static member function.
 *
 *  If no Log has been created with this specific name, we return a dummy logfile.
 *
 *  \author behley.
 *
 */
class Log {
 public:
  ~Log();

  /** non-copyable. **/
  Log(const Log& other) = delete;
  Log& operator=(const Log& other) = delete;

  /** \brief close the log stream, and remove Log from global map. **/
  void close();

  /**
   * \brief starts an event and adds an Entry with the start timestamp; every new element will be attached to this
   * Event,
   *  until the Event is ended.
   */
  void startEvent(const std::string& name);
  /**
   * \brief ends an event and adds an Entry with the end timestamp
   */
  void endEvent();

  /** \brief return the file stream of the logfile. **/
  std::ostream& stream();

  /** \brief writing a value to the logfile **/
  void appendParam(const Parameter& param);
  template <typename T>
  void appendParam(const std::string& name, const T& value);

  /** \brief generate a new log with given name, which is stored in the specific directory. **/
  static Log& createInstance(const std::string& name, const std::string& filename, bool append = false);

  /** \brief returns an Log instance with given name or a /dev/null-log if there is no such log **/
  static Log& getInstance(const std::string& name);

 protected:
  Log(const std::string& name, const std::string& filename, bool append = false);

  std::string name_;
  std::ofstream out_;
  std::string identation_;
  static Log dummy_;
  static std::map<std::string, Log*> instances_;

  std::vector<std::string> event_stack_;
};

class LogError : public Exception {
 public:
  LogError(const std::string& reason) : Exception(reason) {}
};
}

#endif /* LOG_H_ */
