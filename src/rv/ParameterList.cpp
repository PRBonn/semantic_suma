#include "rv/ParameterList.h"

#include "rv/XmlDocument.h"

namespace rv
{

ParameterList::ParameterList()
{

}

ParameterList::~ParameterList()
{
  for (std::map<std::string, Parameter*>::const_iterator it = params.begin(); it != params.end(); ++it)
    delete (it->second);
  params.clear();
}

ParameterList::ParameterList(const ParameterList& other)
{
  for (std::map<std::string, Parameter*>::const_iterator it = other.params.begin(); it != other.params.end(); ++it)
    params[it->first] = it->second->clone();
}

ParameterList& ParameterList::operator=(const ParameterList& other)
{
  if (&other == this) return *this;
  // remove old stuff.
  for (std::map<std::string, Parameter*>::const_iterator it = params.begin(); it != params.end(); ++it)
    delete it->second;
  params.clear();
  // copy new stuff!
  for (std::map<std::string, Parameter*>::const_iterator it = other.params.begin(); it != other.params.end(); ++it)
    params[it->first] = it->second->clone();

  return *this;
}

bool ParameterList::operator==(const ParameterList& other) const
{
  for (std::map<std::string, Parameter*>::const_iterator it = other.params.begin(); it != other.params.end(); ++it)
  {
    const Parameter& param1 = *(it->second);
    bool found = false;
    for (std::map<std::string, Parameter*>::const_iterator it2 = params.begin(); it2 != params.end(); ++it2)
      if (param1 == *(it2->second) && param1.name() == it2->second->name()) found = true;

    if (!found) return false;
  }

  return true;
}

bool ParameterList::operator!=(const ParameterList& other) const
{
  return !(*this == other);
}

void ParameterList::insert(const Parameter& param)
{
  std::map<std::string, Parameter*>::iterator el = params.find(param.name());
  if (el != params.end())
  {
    // delete old parameter.
    delete el->second;
    params.erase(el);
  }
  params[param.name()] = param.clone();
}

void ParameterList::insert(Parameter* param)
{
  if (param == 0) return;
  std::map<std::string, Parameter*>::iterator el = params.find(param->name());
  if (el != params.end())
  {
    // delete old parameter.
    delete el->second;
    params.erase(el);
  }
  /** no clone! **/
  params[param->name()] = param;
}

void ParameterList::erase(const std::string& name)
{
  std::map<std::string, Parameter*>::iterator el = params.find(name);
  if (el != params.end())
  {
    delete el->second;
    params.erase(el);
  }
}

bool ParameterList::hasParam(const std::string& name) const
{
  return (params.find(name) != params.end());
}

ParameterList::const_iterator ParameterList::begin() const
{
  return ParameterList::const_iterator(params.begin());
}

ParameterList::const_iterator ParameterList::end() const
{
  return ParameterList::const_iterator(params.end());
}

void ParameterList::checkParam(const std::string& name) const
{
  if (params.find(name) != params.end()) return;

  std::stringstream str;
  str << "no parameter with name " << name << " in parameter list";
  throw Exception(str.str());
}

void ParameterList::clear()
{
  for (std::map<std::string, Parameter*>::iterator it = params.begin(); it != params.end(); ++it)
    delete it->second;
  params.clear();
}

void parseXmlFile(const std::string& filename, ParameterList& params)
{
  XmlDocument document = XmlDocument::fromFile(filename);
  XmlNode root = document.root();

  if (!root.isValid()) throw XmlError("Invalid root node!");

  const XmlNode::List& children = root.getChildren();
  for (XmlNode::List::const_iterator it = children.begin(); it != children.end(); ++it)
  {
    const XmlNode& child = *it;

    if (!child.isValid()) throw XmlError("Invalid child node!");

    if (child.getName() == "param")
    {
      Parameter* param = Parameter::parseParameter(child);
      params.insert(*param);
//      assert(params.hasParam(child.getProperty("name")));
      delete param;
    }
  }
}

}
