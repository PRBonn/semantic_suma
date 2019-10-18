/**
 * \headerfile Stopwatch.h
 *
 * \brief simple timing of function calls.
 *
 * Mimics the behavior of tic/toc of Matlab and throws an error if toc() is called without starting a timer before.
 *
 * Example:
 *   Stopwatch::tic(); starts timer A
 *
 *   Stopwatch::tic(); starts timer B
 *   method1();
 *   double time1 = Stopwatch::toc(); stops timer B and returns time elapsed since timer B started
 *
 *   Stopwatch::tic(); starts timer C
 *   method2();
 *   double time2 = Stopwatch::toc(); stops  timer C
 *
 *   Stopwatch::toc("finished"); stops timer A, thus this is approx. time1 + time2. and outputs a message.
 *
 * \author behley
 */

#ifndef STOPWATCH_H_
#define STOPWATCH_H_

#include <chrono>
#include <string>
#include <vector>

namespace rv {

class Stopwatch {
 public:
  /** \brief starts a timer. **/
  static void tic();
  /** \brief stops the last timer started and outputs \a msg **/
  static float toc();
  /** \brief number of active stopwatches. **/
  static size_t active() { return stimes.size(); }

 protected:
  static std::vector<std::chrono::system_clock::time_point> stimes;
};
}

#endif /* STOPWATCH_H_ */
