#include "rv/CompositeParameter.h"
#include <sstream>

namespace rv
{

CompositeParameter::CompositeParameter(const std::string& name) :
    Parameter(name)
{

}

CompositeParameter::CompositeParameter(const std::string& name, const ParameterList& values) :
    Parameter(name), param_list(values)
{

}

CompositeParameter::CompositeParameter(const std::string& name, const XmlNode& node) :
    Parameter(name), val(node)
{
  const XmlNode::List& children = node.getChildren();

  for (XmlNode::List::const_iterator it = children.begin(); it != children.end(); ++it)
  {
    if (it->getName() == "param")
    {
      Parameter* p = parseParameter(*it);
      if (param_list.hasParam(p->name())) std::cout << "WARNING: (" << name << ") parameter with name = '" << p->name()
          << "' already inserted." << std::endl;
      param_list.insert(*p);
      delete p;
    }
  }
}

CompositeParameter::~CompositeParameter()
{

}

CompositeParameter::CompositeParameter(const CompositeParameter& other) :
    Parameter(other), param_list(other.param_list), val(other.val)
{

}

CompositeParameter& CompositeParameter::operator=(const CompositeParameter& other)
{
  if (&other == this) return *this;

  Parameter::operator=(other);

  param_list = other.param_list;

  return *this;
}

CompositeParameter* CompositeParameter::clone() const
{
  return new CompositeParameter(*this);
}

std::string CompositeParameter::toString() const
{
  std::stringstream out;

  out << "<param name=\"" << mName << "\" type=\"composite\">" << std::endl;
  for (ParameterList::const_iterator it = param_list.begin(); it != param_list.end(); ++it)
    out << "   " << *it << std::endl;
  out << "</param>";

  return out.str();
}

CompositeParameter::operator ParameterList() const
{
  return param_list;
}

CompositeParameter* parseCompositeParameter(const XmlNode& node)
{
  return new CompositeParameter(node.getAttribute("name"), node);
}

}
