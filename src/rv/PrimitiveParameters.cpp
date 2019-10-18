#include "rv/PrimitiveParameters.h"
#include <sstream>
#include <cstdlib>
#include <cstdio>

namespace rv
{
/******************************************************************************
 *                             BooleanParameter                               *
 ******************************************************************************/

BooleanParameter::BooleanParameter(const std::string& name, const bool value) :
	Parameter(name), val(value)
{
}

BooleanParameter* BooleanParameter::clone() const
{
	return new BooleanParameter(*this);
}

std::string BooleanParameter::toString() const
{
  std::stringstream out;
  out << "<param name=\"" << mName << "\" type=\"boolean\">" << (val ? "true" : "false") << "</param>";
  return out.str();
}

BooleanParameter* parseBooleanParameter(const XmlNode& node)
{
	if (node.getAttribute("type") == "boolean")
		return new BooleanParameter(node.getAttribute("name"), (node.getContent()
				== "true"));
	else
		throw XmlError("Error while parameter parsing: 'type' not 'boolean'");
}

/******************************************************************************
 *                             IntegerParameter                               *
 ******************************************************************************/

IntegerParameter::IntegerParameter(const std::string& name, const int value) :
	Parameter(name), val(value)
{
}

IntegerParameter* IntegerParameter::clone() const
{
	return new IntegerParameter(*this);
}

std::string IntegerParameter::toString() const
{
  std::stringstream out;
  out << "<param name=\"" << mName << "\" type=\"integer\">" << val << "</param>";
  return out.str();
}

IntegerParameter* parseIntegerParameter(const XmlNode& node)
{
	if (node.getAttribute("type") == "integer")
		return new IntegerParameter(node.getAttribute("name"), std::atoi(
				node.getContent().c_str()));
	else
		throw XmlError("Error while parameter parsing: 'type' not 'integer'");
}

/******************************************************************************
 *                             FloatParameter                                 *
 ******************************************************************************/

FloatParameter::FloatParameter(const std::string& name, const double value) :
	Parameter(name), val(value)
{
}

FloatParameter* FloatParameter::clone() const
{
	return new FloatParameter(*this);
}

std::string FloatParameter::toString() const
{
  std::stringstream out;
  out << "<param name=\"" << mName << "\" type=\"float\">" << val << "</param>";
  return out.str();
}

FloatParameter* parseFloatParameter(const XmlNode& node)
{
	if (node.getAttribute("type") == "float")
		return new FloatParameter(node.getAttribute("name"), std::atof(
				node.getContent().c_str()));
	else
		throw XmlError("Error while parameter parsing: 'type' not 'float'");
}

/******************************************************************************
 *                             StringParameter                                *
 ******************************************************************************/

StringParameter::StringParameter(const std::string& name,
		const std::string value) :
	Parameter(name), val(value)
{
}

StringParameter* StringParameter::clone() const
{
	return new StringParameter(*this);
}

std::string StringParameter::toString() const
{
	std::stringstream out;

	out << "<param name=\"" << mName << "\" type=\"string\">" << val << "</param>";

	return out.str();
}

StringParameter* parseStringParameter(const XmlNode& node)
{
	if (node.getAttribute("type") == "string")
		return new StringParameter(node.getAttribute("name"), node.getContent());
	else
		throw XmlError("Error while parameter parsing: 'type' not 'string'");
}

}
