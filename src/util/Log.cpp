#include "Log.h"
#include <sstream>
#include <cassert>
#include <chrono>
#include <rv/XmlNode.h>
#include <rv/XmlDocument.h>
#include <rv/XmlError.h>

namespace rv {

Log Log::dummy_("dummy", "/dev/null");
std::map<std::string, Log*> Log::instances_;

template <>
void Log::appendParam(const std::string& name, const double& value) {
  out_ << identation_ << "<param name=\"" << name << "\" type=\"float\">" << value << "</param>" << std::endl;
}

template <>
void Log::appendParam(const std::string& name, const float& value) {
  out_ << identation_ << "<param name=\"" << name << "\" type=\"float\">" << value << "</param>" << std::endl;
}

template <>
void Log::appendParam(const std::string& name, const int32_t& value) {
  out_ << identation_ << "<param name=\"" << name << "\" type=\"integer\">" << value << "</param>" << std::endl;
}

template <>
void Log::appendParam(const std::string& name, const int64_t& value) {
  out_ << identation_ << "<param name=\"" << name << "\" type=\"integer\">" << value << "</param>" << std::endl;
}

template <>
void Log::appendParam(const std::string& name, const uint32_t& value) {
  out_ << identation_ << "<param name=\"" << name << "\" type=\"integer\">" << value << "</param>" << std::endl;
}

template <>
void Log::appendParam(const std::string& name, const std::string& value) {
  out_ << identation_ << "<param name=\"" << name << "\" type=\"string\">" << value << "</param>" << std::endl;
}

template <>
void Log::appendParam(const std::string& name, const bool& value) {
  out_ << identation_ << "<param name=\"" << name << "\" type=\"boolean\">" << ((value) ? "true" : "false")
       << "</param>" << std::endl;
}

Log::Log(const std::string& name, const std::string& filename, bool append) : name_(name), identation_("  ") {
  std::ios_base::openmode mode = std::ios_base::out;
  /** todo: parse resulting file and restore file up to last unfinished event. **/
  if (append) mode |= std::ios_base::app;

  out_.open(filename.c_str(), mode);
  if (!out_.is_open()) throw LogError("Unable to open log file.");

  /** write logheader. **/
  if (!append) {
    std::time_t tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{0};
    localtime_r(&tt, &tm);

    char prev;
    out_ << "<log name = \"" << name_;
    out_ << "\" date = \"" << tm.tm_mday << "-" << (tm.tm_mon + 1) << "-" << (tm.tm_year + 1900);
    prev = out_.fill('0');
    out_ << "\" time=\"" << tm.tm_hour << ":" << tm.tm_min << ":" << tm.tm_sec << "\">" << std::endl;
    out_.fill(prev);
  }
}

Log::~Log() {
  if (!out_.is_open() && Log::instances_.find(name_) != Log::instances_.end()) {
    delete (Log::instances_.find(name_)->second);
    Log::instances_.erase(Log::instances_.find(name_));
  }
}

void Log::close() {
  if (this != &dummy_) {
    out_ << "</log>" << std::endl;
    out_.close();
  }
}

void Log::startEvent(const std::string& name) {
  out_ << identation_ << "<event name = \"" << name << "\">" << std::endl;
  identation_.append(2, ' ');  // increase indentation
  std::time_t tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  appendParam("starttime", tt);
  event_stack_.push_back(name);
}

void Log::endEvent() {
  if (event_stack_.size() == 0) {
    throw LogError("logic error: no event started before.");
  }
  std::time_t tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  appendParam("endtime", tt);
  identation_.erase(identation_.size() - 2);  // strip last 2 spaces
  out_ << identation_ << "</event>";
  out_ << "<!--" << event_stack_.back() << "-->";
  out_ << std::endl;
  event_stack_.pop_back();
}

void Log::appendParam(const Parameter& param) {
  out_ << identation_ << param << std::endl;
}

std::ostream& Log::stream() {
  return out_;
}

Log& Log::createInstance(const std::string& name, const std::string& filename, bool append) {
  Log* newLog = new Log(name, filename, append);

  if (instances_.find(name) != instances_.end()) delete instances_[name];
  instances_[name] = newLog;

  return *newLog;
}

Log& Log::getInstance(const std::string& name) {
  if (Log::instances_.find(name) == Log::instances_.end())
    return dummy_;
  else
    return *Log::instances_[name];
}
}
